"""Local latched motion hold, separate from observational diagnostics."""
import json
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String
from std_srvs.srv import SetBool
from milestone.unit import RUN, atomic_json


class Supervisor(Node):
    def __init__(self):
        super().__init__('husky_safety_supervisor')
        self.health = None; self.arrival = 0.; self.good_since = None
        self.armed = False; self.held = True; self.reason = 'Initializing sensors'
        self.pub = self.create_publisher(String, '/husky/safety', 10)
        self.cancel = self.create_publisher(String, '/inspection/command', 10)
        self.create_subscription(String, '/unit/husky/health', self.receive, 10)
        self.create_service(SetBool, '/husky/set_hold', self.set_hold)
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        self.health = json.loads(msg.data); self.arrival = time.monotonic()

    def healthy(self):
        return self.health is not None and self.health['status'] != 'CRITICAL' and time.monotonic()-self.arrival < 1.

    def hold(self, reason):
        self.held = True; self.reason = reason
        self.cancel.publish(String(data=json.dumps(dict(action='cancel', request_id=str(uuid.uuid4()), reason=reason))))

    def set_hold(self, request, response):
        if request.data:
            self.armed = True; self.hold('Operator hold'); response.success = True
        elif not self.healthy() or self.good_since is None or time.monotonic()-self.good_since < 2.:
            response.success = False; response.message = 'Critical sensors must recover for two wall seconds before release'
            return response
        else:
            self.held = False; self.armed = True; self.reason = 'Ready'; response.success = True
        response.message = self.reason
        return response

    def tick(self):
        now = time.monotonic()
        if self.healthy():
            if self.good_since is None: self.good_since = now
            if not self.armed and now-self.good_since >= 2.:
                self.armed = True; self.held = False; self.reason = 'Ready'
        else:
            self.good_since = None
            if self.armed and not self.held:
                codes = [i['code'] for i in (self.health or {}).get('issues', []) if i['severity'] == 'critical']
                self.hold(', '.join(codes) or 'Health monitor unavailable')
        value = dict(held=self.held, reason=self.reason, armed=self.armed,
                     release_ready=self.healthy() and self.good_since is not None and now-self.good_since >= 2.,
                     stamp=self.get_clock().now().nanoseconds/1e9)
        self.pub.publish(String(data=json.dumps(value)))
        atomic_json(RUN/'safety-status.json', dict(value, updated_wall=time.time()))


def main():
    rclpy.init(); node = Supervisor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
