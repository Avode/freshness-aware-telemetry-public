"""Health envelopes and expiring, idempotent commands through the faultable telemetry link."""
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String
from std_srvs.srv import SetBool
from autonomy.unit import NavigationTelemetryAgent
from milestone.unit import RUN
from .protocol import CommandLedger, validate_command


class HealthTelemetryAgent(NavigationTelemetryAgent):
    heartbeat = True

    def __init__(self):
        super().__init__()
        self.health = None; self.health_arrival = 0.; self.safety = None; self.safety_arrival = 0.
        self.ledger = CommandLedger(RUN/'edge-commands.sqlite')
        self.commands = self.create_publisher(String, '/inspection/command', 10)
        self.hold_client = self.create_client(SetBool, '/husky/set_hold')
        self.create_subscription(String, '/unit/husky/health', self.receive_health, 10)
        self.create_subscription(String, '/husky/safety', self.receive_safety, 10)
        self.create_subscription(String, '/unit/husky/command', self.command, 10)
        self.create_subscription(String, '/inspection/ack', self.ack_command, 10)
        # This wall-timed heartbeat belongs to the robot edge. It contains only
        # session/clock/map metadata, never simulator pose or fault flags.
        if os.environ.get('FLEETSCOPE_SITE_CONFIG'):
            from network.protocol import SessionHeartbeat
            site = json.loads(Path(os.environ['FLEETSCOPE_SITE_CONFIG']).read_text())
            self.session = SessionHeartbeat(self.run_id, site)
            self.session_pub = self.create_publisher(String, '/unit/husky/session', 10)
            self.create_timer(1., self.session_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def session_tick(self):
        if not self.enabled:
            return
        row = self.outbox.db.execute("SELECT seq FROM sqlite_sequence WHERE name='outbox'").fetchone()
        try:
            # The watermark includes already captured observations even if this
            # node's /clock subscription has not caught up with their callbacks.
            capture_time = max(self.get_clock().now().nanoseconds*1e-9, self.last_sample)
            value = self.session.sample(capture_time, row[0] if row else 0)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self.session_pub.publish(String(data=json.dumps(value, allow_nan=False)))

    def receive_health(self, msg):
        self.health = json.loads(msg.data); self.health_arrival = time.monotonic()

    def receive_safety(self, msg):
        self.safety = json.loads(msg.data); self.safety_arrival = time.monotonic()

    def ack_command(self, msg):
        self.ledger.finish(json.loads(msg.data))

    def command(self, msg):
        # Link outages affect both telemetry and mission-command delivery.
        if not self.enabled: return
        try:
            cmd = json.loads(msg.data)
            validate_command(cmd, self.run_id, time.time())
        except (ValueError, KeyError, TypeError, AttributeError): return
        if not self.ledger.reserve(cmd): return
        def finish(status, reason):
            self.ledger.finish(dict(request_id=cmd['request_id'], status=status, reason=reason))
        if cmd['action'] == 'start':
            if (not self.safety or self.safety['held'] or time.monotonic()-self.safety_arrival>1.
                    or not self.health or self.health['status']=='CRITICAL' or time.monotonic()-self.health_arrival>1.):
                finish('REJECTED', 'Robot is held or critical sensors are unavailable'); return
        if cmd['action'] in ('start', 'cancel'):
            if not self.commands.get_subscription_count(): finish('REJECTED', 'Mission server unavailable'); return
            self.commands.publish(String(data=json.dumps(cmd)))
        else:
            if not self.hold_client.service_is_ready(): finish('REJECTED', 'Safety supervisor unavailable'); return
            future = self.hold_client.call_async(SetBool.Request(data=cmd['action']=='hold'))
            def complete(f):
                try:
                    result = f.result(); finish('ACKNOWLEDGED' if result.success else 'REJECTED', result.message)
                except Exception: finish('REJECTED', 'Safety supervisor call failed')
            future.add_done_callback(complete)

    def enrich_packet(self, packet):
        super().enrich_packet(packet)
        packet['health'] = self.health if time.monotonic()-self.health_arrival <= 1. else dict(
            status='CRITICAL', sensors={}, issues=[dict(code='monitor_stale', severity='critical', message='Health monitor unavailable')])
        packet['safety'] = self.safety if time.monotonic()-self.safety_arrival <= 1. else dict(held=True, reason='Safety supervisor unavailable', release_ready=False)
        packet['transport'] = dict(queued=self.outbox.count(), dropped=self.outbox.dropped, capacity=self.outbox.limit)
        packet['command_results'] = self.ledger.results()


def main():
    rclpy.init(); node = HealthTelemetryAgent()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.ledger.db.close(); node.outbox.db.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
