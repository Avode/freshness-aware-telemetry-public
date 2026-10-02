"""Robot-side known-site localization, frame graph, prior and velocity control."""
import copy
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped,Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer,TransformListener,StaticTransformBroadcaster,TransformException
from milestone.core import ANCHOR,wrap
from milestone.unit import yaw,atomic_json,RUN

class Navigation(Node):
    def __init__(self):
        super().__init__('husky_navigation_interface')
        self.require_safety = self.declare_parameter('require_safety', False).value
        self.safety_held = True; self.last_safety = 0.; self.localization_enabled = True
        self.reported_position_bias = 0.
        if self.require_safety:
            self.create_subscription(String, '/husky/safety', self.safety, 10)
            self.create_service(SetBool, '/simulation/localization_enabled', self.set_localization)
            self.create_service(SetBool, '/simulation/position_jump', self.set_position_jump)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.static=StaticTransformBroadcaster(self)
        transforms=[]
        for parent,child,xyz,a in [('estate','slam_map',(*ANCHOR[:2],0.),ANCHOR[2]),
                                  ('husky/base_link','husky/lidar',(.48,0.,.42),0.)]:
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=xyz
            t.transform.rotation.z=math.sin(a/2);t.transform.rotation.w=math.cos(a/2);transforms.append(t)
        self.static.sendTransform(transforms)
        self.localized=self.create_publisher(Odometry,'/husky/odometry/localized',30)
        self.drive=self.create_publisher(Twist,'/husky/drive',10)
        self.guard=self.create_publisher(String,'/husky/drive_status',10)
        self.prior_pub=self.create_publisher(OccupancyGrid,'/navigation/prior_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        prior_path=RUN/'config/survey-prior.npz'
        prior=np.load(prior_path if prior_path.exists() else Path(__file__).parent/'generated/survey-prior.npz')
        msg=OccupancyGrid();msg.header.frame_id='estate';msg.info.resolution=float(prior['resolution'])
        msg.info.height,msg.info.width=prior['data'].shape;msg.info.origin.position.x,msg.info.origin.position.y=map(float,prior['origin'])
        msg.info.origin.orientation.w=1.;msg.data=prior['data'].astype(np.int8).ravel().tolist();self.prior_pub.publish(msg)
        self.create_subscription(Odometry,'/husky/odometry/filtered',self.odom,30)
        self.amcl_cov=None
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:setattr(self,'amcl_cov',m.pose.covariance),10)
        self.create_subscription(Imu,'/husky/imu/data',self.imu,30)
        self.create_subscription(LaserScan,'/husky/scan',self.scan,10)
        self.create_subscription(Twist,'/navigation/cmd_vel',self.command,10)
        self.request=Twist();self.last_cmd=0.;self.last_imu=0.;self.last_scan=0.;self.last_localization=0.
        self.rate=0.;self.front=float('inf');self.integral=0.;self.odom_count=0
        self.create_timer(.05,self.control,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def odom(self,msg):
        if not self.localization_enabled: return
        try:tf=self.buffer.lookup_transform('slam_map','husky/odom',Time())
        except TransformException:return
        correction_time=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
        if self.get_clock().now().nanoseconds*1e-9-correction_time>1.:return
        out=copy.deepcopy(msg);p=msg.pose.pose.position;a=yaw(tf.transform.rotation);c,s=math.cos(a),math.sin(a)
        out.header.frame_id='slam_map'
        out.pose.pose.position.x=tf.transform.translation.x+c*p.x-s*p.y
        out.pose.pose.position.x += self.reported_position_bias
        out.pose.pose.position.y=tf.transform.translation.y+s*p.x+c*p.y
        heading=wrap(a+yaw(msg.pose.pose.orientation));out.pose.pose.orientation.z=math.sin(heading/2);out.pose.pose.orientation.w=math.cos(heading/2)
        cov=np.array([[msg.pose.covariance[0],msg.pose.covariance[1]],[msg.pose.covariance[6],msg.pose.covariance[7]]])
        rot=np.array([[c,-s],[s,c]]);cov=rot@cov@rot.T
        if self.amcl_cov is not None:
            ac=self.amcl_cov;cov=np.array([[ac[0],ac[1]],[ac[6],ac[7]]])
            ca,sa=math.cos(-ANCHOR[2]),math.sin(-ANCHOR[2]);rotation=np.array([[ca,-sa],[sa,ca]])
            cov=rotation@cov@rotation.T;out.pose.covariance[35]=ac[35]
        for index,val in zip((0,1,6,7),cov.ravel()):out.pose.covariance[index]=float(val)
        self.localized.publish(out);self.last_localization=time.monotonic();self.odom_count+=1

    def imu(self,msg):self.rate=msg.angular_velocity.z;self.last_imu=time.monotonic()

    def scan(self,msg):
        values=[r for i,r in enumerate(msg.ranges) if abs(msg.angle_min+i*msg.angle_increment)<.33 and math.isfinite(r) and r>msg.range_min]
        self.front=min(values,default=float('inf'));self.last_scan=time.monotonic()

    def command(self,msg):
        if not all(math.isfinite(v) for v in (msg.linear.x,msg.angular.z)):return
        self.request=msg;self.last_cmd=time.monotonic()

    def safety(self, msg):
        import json
        self.safety_held = json.loads(msg.data)['held']; self.last_safety = time.monotonic()

    def set_localization(self, request, response):
        self.localization_enabled = request.data
        response.success = True; response.message = 'Position reports enabled' if request.data else 'Position reports interrupted'
        return response

    def set_position_jump(self, request, response):
        self.reported_position_bias = 2.5 if request.data else 0.
        response.success = True; response.message = 'Reported position bias enabled' if request.data else 'Reported position bias cleared'
        return response

    def control(self):
        now=time.monotonic();out=Twist();status='ready'
        if getattr(self, 'require_safety', False) and (self.safety_held or now-self.last_safety>1.):status='safety_hold'
        elif now-self.last_cmd>.6:status='command_timeout'
        elif now-self.last_scan>1.:status='scan_stale'
        elif now-self.last_imu>1.:status='imu_stale'
        elif now-self.last_localization>1.:status='localization_stale'
        else:
            desired=max(-.5,min(.5,self.request.angular.z));error=desired-self.rate
            self.integral=max(-.6,min(.6,self.integral+error*.05))
            # Close the yaw-rate loop using measured IMU motion, not encoder yaw.
            out.angular.z=max(-1.8,min(1.8,desired*1.5+2.*error+self.integral))
            out.linear.x=max(0.,min(.65,self.request.linear.x))
            if out.linear.x>0 and self.front<.95:out.linear.x=0.;status='obstacle_stop'
            if abs(desired)<.015 and abs(self.request.linear.x)<.01:
                out=Twist();self.integral=0.
        if status in ('command_timeout','scan_stale','imu_stale','localization_stale','safety_hold'):self.integral=0.
        self.drive.publish(out);self.guard.publish(String(data=status))
        atomic_json(RUN/'navigation-status.json',dict(status=status,front_range=self.front if math.isfinite(self.front) else None,
            desired_yaw_rate=self.request.angular.z,measured_yaw_rate=self.rate,localized_samples=self.odom_count,updated_wall=time.time()))

def main():
    rclpy.init();n=Navigation()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():n.drive.publish(Twist())
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
