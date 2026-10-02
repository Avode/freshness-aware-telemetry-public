"""Sensor normalization and local velocity guard; no world pose subscription."""
import copy
import math
import random
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, Image
from std_msgs.msg import String

class Adapter(Node):
    def __init__(self):
        super().__init__('husky_adapter')
        self.rng = random.Random(2026)
        self.odom_pub = self.create_publisher(Odometry, '/husky/wheel_odometry', 10)
        self.imu_pub = self.create_publisher(Imu, '/husky/imu/data', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/husky/scan', 10)
        self.image_pub = self.create_publisher(Image, '/husky/image', qos_profile_sensor_data)
        self.drive_pub = self.create_publisher(Twist, '/husky/drive', 10)
        self.guard_pub = self.create_publisher(String, '/husky/drive_status', 10)
        for typ, topic, cb in [(Odometry,'wheel_odometry',self.odom),(Imu,'imu',self.imu),
                               (LaserScan,'scan',self.scan),(Image,'image',self.image)]:
            self.create_subscription(typ, '/husky/raw/'+topic, cb, qos_profile_sensor_data)
        self.create_subscription(Twist, '/husky/cmd_vel', self.command, 10)
        self.last_command, self.last_scan, self.front = 0., 0., float('inf')
        self.target = Twist()
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def odom(self, msg):
        msg.header.frame_id = 'husky/odom'; msg.child_frame_id = 'husky/base_link'
        # Encoder calibration error and noise. Position from this message is not fused.
        msg.twist.twist.linear.x = msg.twist.twist.linear.x*1.005 + self.rng.gauss(0,.004)
        msg.twist.twist.linear.y = 0.
        msg.twist.covariance[0] = .01**2
        msg.twist.covariance[7] = .02**2
        msg.twist.covariance[35] = .02**2
        self.odom_pub.publish(msg)

    def imu(self, msg):
        msg.header.frame_id = 'husky/base_link'
        # Gazebo orientation is not an independent compass measurement. Exclude it.
        msg.orientation_covariance[0] = -1.
        for i in (0,4,8):
            msg.angular_velocity_covariance[i] = .003**2
            msg.linear_acceleration_covariance[i] = .03**2
        self.imu_pub.publish(msg)

    def scan(self, msg):
        msg.header.frame_id = 'husky/lidar'
        front = [r for i,r in enumerate(msg.ranges) if abs(msg.angle_min+i*msg.angle_increment)<.4 and math.isfinite(r) and r>=msg.range_min]
        self.front = min(front, default=float('inf')); self.last_scan = time.monotonic()
        self.scan_pub.publish(msg)

    def image(self, msg):
        msg.header.frame_id = 'husky/camera_optical'; self.image_pub.publish(msg)

    def command(self, msg):
        if not all(math.isfinite(v) for v in (msg.linear.x,msg.angular.z)): return
        self.target.linear.x = max(-.4,min(.7,msg.linear.x))
        self.target.angular.z = max(-.6,min(.6,msg.angular.z))
        self.last_command = time.monotonic()

    def tick(self):
        now = time.monotonic(); cmd = copy.deepcopy(self.target); status = 'ready'
        if now-self.last_command>.6:
            cmd = Twist(); status = 'command_timeout'
        elif now-self.last_scan>1.:
            cmd = Twist(); status = 'scan_stale'
        elif cmd.linear.x>0 and self.front<1.2:
            cmd.linear.x = 0.; cmd.angular.z = 0.; status = 'obstacle_stop'
        self.drive_pub.publish(cmd); self.guard_pub.publish(String(data=status))

def main():
    rclpy.init(); node = Adapter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.drive_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
