"""Edge telemetry agent: timestamped observations, bounded disk outbox and ACK delivery."""
import base64
from collections import deque
import json
import math
import os
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
from .core import Outbox, PoseHistory, estate_pose, capture_through

RUN = Path(os.environ.get('FLEETSCOPE_RUN', Path(os.environ.get('FLEETSCOPE_RUNS', Path(__file__).resolve().parents[1]/'runs'))/'milestone-dev')).expanduser()

def stamp(msg): return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def atomic_json(path, data):
    tmp = path.with_suffix('.tmp'); tmp.write_text(json.dumps(data, indent=2, allow_nan=False)); tmp.replace(path)

class EdgeTelemetryAgent(Node):
    def __init__(self):
        super().__init__('edge_telemetry_husky')
        RUN.mkdir(parents=True, exist_ok=True)
        self.run_id = os.environ.get('FLEETSCOPE_RUN_ID', RUN.name)
        self.outbox = Outbox(RUN / 'edge-outbox.sqlite')
        self.pub = self.create_publisher(String, '/unit/husky/telemetry', 50)
        self.create_subscription(String, '/unit/husky/ack', self.ack, 50)
        self.create_service(SetBool, '/unit/husky/set_link', self.link)
        self.create_subscription(Odometry, '/husky/odometry/filtered', self.odom, 30)
        self.create_subscription(Imu, '/husky/imu/data', self.imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/husky/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Image, '/husky/image', self.image, qos_profile_sensor_data)
        self.create_subscription(String, '/husky/drive_status', lambda m:setattr(self,'drive_status',m.data), 10)
        self.history = PoseHistory(); self.latest = None; self.scans = deque(maxlen=20)
        self.last_imu = None; self.camera = None; self.last_image_encoded = -1.
        self.sensor_stamps = {}; self.enabled = True; self.drive_status = 'waiting'
        self.last_sample = -1.; self.last_camera_sent = -1.; self.last_scan_sent = -1.
        self.delivered = 0; self.sent = 0; self.unaligned_scans = 0
        self.create_timer(.2, self.sample)
        self.create_timer(.2, self.transmit, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def odom(self, msg):
        p = msg.pose.pose; value = estate_pose(p.position.x,p.position.y,yaw(p.orientation))
        t = stamp(msg)
        if not all(math.isfinite(v) for v in value): return
        self.history.add(t,value)
        # Rotate the EKF XY covariance into the surveyed estate frame (yaw -pi/2).
        c = msg.pose.covariance
        self.latest = {'stamp':t,'pose':value,'covariance_xy':[c[7]+.01,-c[6],-c[1],c[0]+.01],
                       'yaw_variance':c[35]+.02**2,
                       'speed':msg.twist.twist.linear.x,'yaw_rate':msg.twist.twist.angular.z}
        self.sensor_stamps['estimate'] = t

    def imu(self, msg):
        self.sensor_stamps['imu'] = stamp(msg)
        self.last_imu = {'stamp':stamp(msg),'gyro':[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],
                         'acceleration':[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]}

    def scan(self, msg):
        self.sensor_stamps['lidar'] = stamp(msg); self.scans.append(msg)

    def image(self, msg):
        t = stamp(msg); self.sensor_stamps['camera'] = t
        if t-self.last_image_encoded<.5: return
        channels = {'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4}.get(msg.encoding)
        if channels is None: return
        pixels = np.frombuffer(msg.data, np.uint8).reshape(msg.height,msg.step)[:, :msg.width*channels].reshape(msg.height,msg.width,channels)
        if msg.encoding=='rgb8': pixels = cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR)
        elif msg.encoding=='rgba8': pixels = cv2.cvtColor(pixels,cv2.COLOR_RGBA2BGR)
        elif msg.encoding=='bgra8': pixels = cv2.cvtColor(pixels,cv2.COLOR_BGRA2BGR)
        ok, jpeg = cv2.imencode('.jpg',pixels,[cv2.IMWRITE_JPEG_QUALITY,65])
        if ok:
            self.camera = {'stamp':t,'format':'jpeg','data':base64.b64encode(jpeg).decode()}
            self.last_image_encoded = t

    def link(self, request, response):
        self.enabled = request.data
        response.success = True; response.message = 'UP' if self.enabled else 'DOWN: recording to edge outbox'
        return response

    def ack(self, msg):
        try:
            ack = json.loads(msg.data)
            if ack['run_id']==self.run_id:
                self.outbox.ack(int(ack['seq'])); self.delivered += 1
        except (ValueError,KeyError,TypeError): self.get_logger().warning('Ignored malformed ACK')

    def sample(self):
        now = self.get_clock().now().nanoseconds*1e-9
        # The health milestone emits an envelope even when positioning stops.
        # Preserve the original measurement stamp inside state for freshness checks.
        envelope_stamp = now if getattr(self, 'heartbeat', False) else (self.latest or {}).get('stamp', -1.)
        if not self.latest or envelope_stamp<=self.last_sample: return
        packet = {'schema':1,'robot_id':'husky_01','run_id':self.run_id,
                  'stamp':envelope_stamp,'state':self.latest,
                  'imu':self.last_imu,'sensor_age':{k:max(0.,now-v) for k,v in self.sensor_stamps.items()},
                  'drive_status':self.drive_status,'edge_dropped':self.outbox.dropped}
        # Choose the newest scan for which estimates bracket its capture time.
        for scan in reversed(self.scans):
            t = stamp(scan)
            if t<=self.last_scan_sent: break
            pose = self.history.at(t)
            if pose is None: continue
            ranges = [round(float(r),3) if math.isfinite(r) else (None if r>0 else 0.) for r in scan.ranges[::2]]
            packet['scan'] = {'stamp':t,'pose':pose,'ranges':ranges,'min':scan.range_min,'max':scan.range_max,
                              'angle_min':scan.angle_min,'angle_increment':scan.angle_increment*2}
            self.last_scan_sent = t; break
        if self.camera and self.camera['stamp']>self.last_camera_sent:
            packet['camera'] = self.camera; self.last_camera_sent = self.camera['stamp']
        self.enrich_packet(packet)
        # A source message can precede its /clock update in the executor. Cover
        # every included source timestamp while preserving each original stamp.
        packet['stamp'] = capture_through(packet)
        packet['sensor_age'] = {k:max(0.,packet['stamp']-v) for k,v in self.sensor_stamps.items()}
        self.last_sample = packet['stamp']
        self.outbox.put(packet)

    def enrich_packet(self, packet):
        """Optional robot-side payloads; delivery still uses the same durable outbox."""
        pass

    def transmit(self):
        if self.enabled:
            for seq, packet in self.outbox.scheduled_batch():
                packet['seq'] = seq
                self.pub.publish(String(data=json.dumps(packet,allow_nan=False,separators=(',',':')))); self.sent += 1
        atomic_json(RUN/'edge-status.json',{'run_id':self.run_id,'link_up':self.enabled,
                    'queued':self.outbox.count(),'dropped':self.outbox.dropped,'acked':self.delivered,
                    'sent_including_retries':self.sent,'sensor_stamps':self.sensor_stamps,
                    'updated_wall':time.time()})

def main():
    rclpy.init(); node = EdgeTelemetryAgent()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.outbox.db.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
