"""Passive robot-side monitoring of normalized sensors and localized estimates."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, Image
from std_msgs.msg import String
from milestone.unit import stamp, yaw, atomic_json, RUN
from .health import HealthTracker


class Observer(Node):
    def __init__(self):
        super().__init__('telemetry_health_observer')
        self.tracker = HealthTracker()
        self.motion_samples = {'localized': {}, 'odometry': {}}
        self.pub = self.create_publisher(String, '/unit/husky/health', 10)
        for name, typ, topic in [('imu', Imu, '/husky/imu/data'), ('lidar', LaserScan, '/husky/scan'),
                                 ('camera', Image, '/husky/image'), ('encoders', Odometry, '/husky/wheel_odometry'),
                                 ('position', Odometry, '/husky/odometry/localized')]:
            self.create_subscription(typ, topic, lambda msg, name=name: self.observe(name, msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/husky/odometry/filtered', lambda msg:self.motion_sample('odometry', msg), qos_profile_sensor_data)
        self.create_timer(.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def observe(self, name, msg):
        self.tracker.observe(name, stamp(msg), time.monotonic())
        if name == 'position':
            self.motion_sample('localized', msg)
            cov = msg.pose.covariance
            a, b, d = cov[0], (cov[1]+cov[6])/2, cov[7]
            largest = (a+d+math.sqrt((a-d)**2+4*b*b))/2
            self.tracker.sigma = math.sqrt(max(0., largest)) if math.isfinite(largest) else None

    def motion_sample(self, source, msg):
        t = round(stamp(msg), 6); pose = msg.pose.pose
        values = [pose.position.x, pose.position.y, yaw(pose.orientation)]
        if not all(math.isfinite(v) for v in values): return
        self.motion_samples[source][t] = values
        if len(self.motion_samples[source])>300: del self.motion_samples[source][next(iter(self.motion_samples[source]))]
        if all(t in samples for samples in self.motion_samples.values()):
            self.tracker.compare_motion(t, self.motion_samples['localized'][t], self.motion_samples['odometry'][t])

    def tick(self):
        value = self.tracker.snapshot(self.get_clock().now().nanoseconds/1e9, time.monotonic())
        self.pub.publish(String(data=json.dumps(value, allow_nan=False)))
        atomic_json(RUN/'health-status.json', dict(value, updated_wall=time.time()))


def main():
    rclpy.init(); node = Observer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
