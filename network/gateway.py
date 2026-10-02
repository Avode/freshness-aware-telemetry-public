"""Allowlisted ROS/MQTT transport; only a Mac durable ACK clears the edge outbox."""
from collections import OrderedDict
import json
import os
from pathlib import Path
import queue
import signal
import threading
import time
import uuid

import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.subscribeoptions import SubscribeOptions
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String
from milestone.unit import RUN, atomic_json
from . import protocol


class Gateway(Node):
    def __init__(self):
        super().__init__('telemetry_mqtt_gateway')
        self.run_id = os.environ.get('FLEETSCOPE_RUN_ID', RUN.name)
        self.gateway_session_id = str(uuid.uuid4())
        self.config = json.loads(Path(os.environ['FLEETSCOPE_MQTT_CONFIG']).read_text())
        self.pending = queue.Queue(maxsize=256)
        self.forwarded = OrderedDict()
        self.last_heartbeat = 0.
        self.heartbeat = None
        self.stale_published = False
        self.connected = threading.Event()
        self.stats = dict(telemetry_published=0, mac_acks=0, commands_forwarded=0,
                          rejected=0, queue_overflow=0, last_mac_ack_wall=None)
        self.ack_pub = self.create_publisher(String, '/unit/husky/ack', 100)
        self.command_pub = self.create_publisher(String, '/unit/husky/command', 10)
        self.create_subscription(String, '/unit/husky/telemetry', self.receive_telemetry, 100)
        self.create_subscription(String, '/unit/husky/session', self.receive_heartbeat, 10)
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id=f'fleetscope-edge-{self.run_id}', protocol=mqtt.MQTTv5)
        self.client.username_pw_set(self.config['username'], self.config['password'])
        self.client.tls_set(ca_certs=self.config['ca_file'])
        self.client.max_inflight_messages_set(32)
        self.client.max_queued_messages_set(64)
        self.client.reconnect_delay_set(1, 10)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.will_set(protocol.PREFIX+'status', json.dumps(self.offline('gateway_disconnected')), qos=1, retain=True)
        self.client.connect_async(self.config['host'], self.config.get('port', 8883),
                                  keepalive=10, clean_start=True)
        self.client.loop_start()
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self.drain, clock=steady)
        self.create_timer(1., self.report, clock=steady)

    def offline(self, reason):
        return dict(schema=1, protocol_version=1, site_id=protocol.SITE,
                    robot_id=protocol.ROBOT, run_id=self.run_id, online=False,
                    gateway_session_id=self.gateway_session_id, reason=reason)

    def on_connect(self, client, userdata, flags, reason, properties):
        if reason.is_failure:
            self.get_logger().error(f'MQTT connection rejected: {reason}')
            return
        # Preserve RETAIN for live publications too, so retained commands cannot
        # slip through merely because the gateway was already subscribed.
        options = SubscribeOptions(qos=1, retainAsPublished=True)
        client.subscribe([(protocol.PREFIX+'commands', options),
                          (protocol.PREFIX+'telemetry_ack', options)])
        self.connected.set()
        self.get_logger().info('MQTT TLS connected; waiting for Mac durable receipt ACKs')

    def on_disconnect(self, client, userdata, flags, reason, properties):
        self.connected.clear()

    def on_message(self, client, userdata, message):
        if len(message.payload) > 4096:
            self.stats['rejected'] += 1
            return
        try:
            self.pending.put_nowait((message.topic, bytes(message.payload), message.retain))
        except queue.Full:
            self.stats['queue_overflow'] += 1

    def receive_telemetry(self, message):
        if not self.connected.is_set():
            return  # The durable edge outbox owns retries, not an unbounded RAM queue.
        try:
            value = protocol.telemetry(protocol.decode(message.data), self.run_id)
            seq = value['seq']
            now = time.monotonic()
            if now-self.forwarded.get(seq, -100.) < 1.:
                return
            info = self.client.publish(protocol.PREFIX+'telemetry', message.data, qos=1, retain=False)
            if info.rc == mqtt.MQTT_ERR_SUCCESS:
                self.forwarded[seq] = now
                self.forwarded.move_to_end(seq)
                while len(self.forwarded) > 4096:
                    self.forwarded.popitem(last=False)
                self.stats['telemetry_published'] += 1
        except (ValueError, TypeError, KeyError, AttributeError):
            self.stats['rejected'] += 1

    def receive_heartbeat(self, message):
        try:
            value = protocol.decode(message.data, 8192)
            protocol.identity(value, self.run_id)
            if value.get('schema') != 1 or not value.get('session_id'):
                raise ValueError('Invalid session')
            self.heartbeat = value
            self.last_heartbeat = time.monotonic()
            self.stale_published = False
            if self.connected.is_set():
                props = Properties(PacketTypes.PUBLISH)
                props.MessageExpiryInterval = 3
                payload = dict(value, gateway_session_id=self.gateway_session_id)
                self.client.publish(protocol.PREFIX+'status', json.dumps(payload), qos=1, retain=True, properties=props)
        except (ValueError, TypeError, KeyError):
            self.stats['rejected'] += 1

    def drain(self):
        for _ in range(100):
            try:
                topic, raw, retained = self.pending.get_nowait()
            except queue.Empty:
                break
            try:
                value = protocol.decode(raw, 4096)
                if topic == protocol.PREFIX+'telemetry_ack':
                    ack = protocol.acknowledgement(value, self.run_id, retained)
                    if ack['seq'] not in self.forwarded:
                        raise ValueError('ACK for an envelope not forwarded by this gateway')
                    self.ack_pub.publish(String(data=json.dumps(ack)))
                    self.stats['mac_acks'] += 1
                    self.stats['last_mac_ack_wall'] = time.time()
                elif topic == protocol.PREFIX+'commands':
                    if not self.connected.is_set() or time.monotonic()-self.last_heartbeat > 2.5:
                        raise ValueError('Edge session stale')
                    cmd = protocol.command(value, self.run_id, time.time(), retained)
                    self.command_pub.publish(String(data=json.dumps(cmd, allow_nan=False)))
                    self.stats['commands_forwarded'] += 1
                else:
                    raise ValueError('Unknown topic')
            except (ValueError, TypeError, KeyError, AttributeError):
                self.stats['rejected'] += 1

    def report(self):
        stale = time.monotonic()-self.last_heartbeat > 2.5
        if stale and self.connected.is_set() and not self.stale_published:
            self.client.publish(protocol.PREFIX+'status', json.dumps(self.offline('edge_heartbeat_stale')), qos=1, retain=True)
            self.stale_published = True
        atomic_json(RUN/'mqtt-gateway-status.json', dict(self.stats, run_id=self.run_id,
            updated_wall=time.time(), broker_connected=self.connected.is_set(),
            edge_session_fresh=not stale, gateway_session_id=self.gateway_session_id))

    def close(self):
        if self.connected.is_set():
            try:
                info = self.client.publish(protocol.PREFIX+'status', json.dumps(self.offline('gateway_stopped')), qos=1, retain=True)
                info.wait_for_publish(timeout=1.)
            except (RuntimeError, ValueError):
                pass
        self.client.disconnect()
        self.client.loop_stop()


def main():
    rclpy.init()
    node = Gateway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
