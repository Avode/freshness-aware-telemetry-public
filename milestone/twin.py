"""Command-center state: consumes received telemetry only, never raw sensors/truth."""
import base64
import json
import os
from pathlib import Path
import sqlite3
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import MarkerArray, Marker
from .core import Occupancy
from .unit import RUN, atomic_json
from . import visuals

def ros_stamp(t):
    from builtin_interfaces.msg import Time
    sec=int(t); return Time(sec=sec,nanosec=int((t-sec)*1e9))

class Twin(Node):
    def __init__(self):
        super().__init__('fleetscope_twin')
        RUN.mkdir(parents=True,exist_ok=True)
        self.run_id=os.environ.get('FLEETSCOPE_RUN_ID',RUN.name)
        self.reference=StaticTransformBroadcaster(self)
        reference=TransformStamped(); reference.header.frame_id='site_reference'; reference.child_frame_id='estate'
        reference.transform.rotation.w=1.; self.reference.sendTransform(reference)
        self.db=sqlite3.connect(RUN/'twin-history.sqlite')
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('CREATE TABLE IF NOT EXISTS packets (run_id TEXT, seq INTEGER, stamp REAL, received REAL, payload TEXT, PRIMARY KEY(run_id,seq))')
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.ack_pub=self.create_publisher(String,'/unit/husky/ack',50)
        self.state_pub=self.create_publisher(String,'/twin/husky/state',10)
        self.prior_pub=self.create_publisher(MarkerArray,'/twin/estate',latched)
        self.robot_pub=self.create_publisher(MarkerArray,'/twin/husky/model',latched)
        self.map_pub=self.create_publisher(OccupancyGrid,'/twin/observed_map',latched)
        self.points_pub=self.create_publisher(Marker,'/twin/husky/returns',latched)
        self.status_pub=self.create_publisher(Marker,'/twin/husky/status',latched)
        self.cov_pub=self.create_publisher(Marker,'/twin/husky/uncertainty',latched)
        self.path_pub=self.create_publisher(RosPath,'/twin/husky/path',latched)
        self.image_pub=self.create_publisher(Image,'/twin/husky/image',10)
        self.create_subscription(String,'/unit/husky/telemetry',self.receive,50)
        self.create_subscription(Clock,'/clock',self.clock,10)
        self.sim_time=0.; self.last_clock_wall=0.; self.last_arrival=0.
        self.latest=None; self.latest_stamp=-1.; self.camera_stamp=-1.; self.map_stamp=-1.
        self.accepted=0; self.duplicates=0; self.replayed=0; self.rejected=0
        self.map=Occupancy(); self.template=visuals.robot_template()
        self.path=RosPath(); self.path.header.frame_id='estate'
        self.prior_pub.publish(visuals.estate())
        # Rebuild observation history after a center restart before accepting retries.
        for (payload,) in self.db.execute('SELECT payload FROM packets WHERE run_id=? ORDER BY stamp,seq',(self.run_id,)):
            self.apply(json.loads(payload),recovery=True)
        self.create_timer(.2,self.status)
        self.create_timer(1.,self.publish_map)

    def clock(self,msg):
        value=msg.clock.sec+msg.clock.nanosec*1e-9
        if value!=self.sim_time: self.last_clock_wall=time.monotonic()
        self.sim_time=value

    def receive(self,msg):
        try:
            packet=json.loads(msg.data)
            assert packet['schema']==1 and packet['robot_id']=='husky_01' and packet['run_id']==self.run_id
            assert isinstance(packet['seq'],int) and packet['seq']>0
            assert all(np.isfinite(v) for v in packet['state']['pose'])
        except (ValueError,KeyError,TypeError,AssertionError):
            self.rejected+=1; return
        now=time.time()
        cur=self.db.execute('INSERT OR IGNORE INTO packets VALUES (?,?,?,?,?)',
            (packet['run_id'],packet['seq'],packet['stamp'],now,msg.data))
        self.db.commit()  # ACK only after durable storage.
        self.ack_pub.publish(String(data=json.dumps({'run_id':packet['run_id'],'seq':packet['seq']})))
        self.last_arrival=time.monotonic()
        if not cur.rowcount: self.duplicates+=1; return
        self.accepted+=1
        if self.sim_time-packet['stamp']>1.: self.replayed+=1
        self.apply(packet)

    def apply(self,packet,recovery=False):
        scan=packet.get('scan')
        if scan and scan['stamp']>self.map_stamp:
            points=self.map.integrate(scan); self.map_stamp=scan['stamp']
            if not recovery: self.points_pub.publish(visuals.point_marker(points))
        if packet['stamp']>self.latest_stamp:
            self.latest=packet; self.latest_stamp=packet['stamp']
            state=packet['state']
            p=PoseStamped(); p.header.frame_id='estate'; p.header.stamp=ros_stamp(packet['stamp'])
            p.pose.position.x,p.pose.position.y=map(float,state['pose'][:2]); p.pose.position.z=.22
            p.pose.orientation.w=1.; self.path.poses.append(p)
            self.path.poses=self.path.poses[-2500:]
            if not recovery:
                self.robot_pub.publish(visuals.robot(self.template,state['pose']))
                self.cov_pub.publish(visuals.uncertainty(state)); self.path_pub.publish(self.path)
                self.state_pub.publish(String(data=json.dumps(packet['state'],allow_nan=False)))
        cam=packet.get('camera')
        if cam and cam['stamp']>self.camera_stamp:
            self.camera_stamp=cam['stamp']
            if not recovery:
                bgr=cv2.imdecode(np.frombuffer(base64.b64decode(cam['data']),np.uint8),cv2.IMREAD_COLOR)
                if bgr is not None:
                    rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
                    im=Image(); im.header.stamp=ros_stamp(cam['stamp']); im.header.frame_id='twin/husky/camera_optical'
                    im.height,im.width=rgb.shape[:2]; im.encoding='rgb8'; im.step=im.width*3; im.data=rgb.tobytes()
                    self.image_pub.publish(im)
                    (RUN/'latest-camera.jpg').write_bytes(base64.b64decode(cam['data']))

    def status(self):
        age=time.monotonic()-self.last_arrival if self.last_arrival else None
        data_age=max(0.,self.sim_time-self.latest['state']['stamp']) if self.latest else None
        stale=age is None or age>2. or data_age>2.
        state=self.latest['state'] if self.latest else None
        status={'run_id':self.run_id,'robot':'husky_01','status':'STALE' if stale else 'LIVE',
                'sim_time':self.sim_time,
                'transport_age_wall_s':age,'measurement_age_sim_s':data_age,
                'simulation_paused_or_stalled':time.monotonic()-self.last_clock_wall>2.,
                'state':state,'accepted':self.accepted,'duplicates':self.duplicates,'replayed':self.replayed,
                'rejected':self.rejected,'scans_integrated':self.map.scans,
                'camera_stamp':self.camera_stamp,'map_stamp':self.map_stamp,
                'sensor_age_at_edge':self.latest.get('sensor_age',{}) if self.latest else {},
                'drive_status':self.latest.get('drive_status','waiting') if self.latest else 'waiting',
                'updated_wall':time.time()}
        atomic_json(RUN/'twin-status.json',status)
        if state:
            label=f"HUSKY 01 | {'STALE — last known pose' if stale else 'LIVE — estimated pose'}\n"
            label+=f"{state['speed']:.2f} m/s | data age {data_age:.1f}s | {self.map.scans} scans\n"
            drive_label={'ready':'Drive active','command_timeout':'Stopped: no drive request',
                         'obstacle_stop':'Stopped: obstacle ahead','scan_stale':'Stopped: LiDAR missing'}.get(status['drive_status'],'Waiting')
            label+=f"{drive_label} | camera age {max(0,self.sim_time-self.camera_stamp):.1f}s"
            label+=self.status_suffix()
            self.status_pub.publish(visuals.text_marker(label,state['pose'],stale))

    def status_suffix(self):
        return ''

    def publish_map(self):
        if not self.map.scans: return
        msg=OccupancyGrid(); msg.header.frame_id='estate'; msg.header.stamp=ros_stamp(max(0.,self.map_stamp))
        msg.info.resolution=self.map.resolution; msg.info.width=self.map.width; msg.info.height=self.map.height
        msg.info.origin.position.x,msg.info.origin.position.y=self.map.origin
        msg.info.origin.position.z=.10; msg.info.origin.orientation.w=1.
        msg.data=self.map.data().ravel().tolist(); self.map_pub.publish(msg)

def main():
    rclpy.init(); node=Twin()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        np.savez_compressed(RUN/'observed-map.npz',logodds=node.map.logodds,seen=node.map.seen)
        node.db.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
