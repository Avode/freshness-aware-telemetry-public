"""Received SLAM map, route and inspection progress, never live robot-side feeds."""
import base64
import json
import math
import time
import zlib
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import String
from milestone.twin import Twin,ros_stamp
from milestone.core import estate_pose
from milestone.unit import RUN,atomic_json

class NavigationTwin(Twin):
    def __init__(self):
        self.slam=None;self.plan=None;self.mission=None;self.extra_stamp=-1.
        super().__init__()
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.slam_pub=self.create_publisher(OccupancyGrid,'/twin/slam_map',qos)
        self.plan_pub=self.create_publisher(Path,'/twin/planned_path',qos)
        self.checkpoints_pub=self.create_publisher(MarkerArray,'/twin/inspection_points',qos)
        self.mission_pub=self.create_publisher(String,'/twin/mission_status',qos)
        self.create_timer(1.,self.publish_navigation)

    def apply(self,packet,recovery=False):
        super().apply(packet,recovery)
        if packet.get('slam') and (self.slam is None or packet['slam']['stamp']>self.slam['stamp']):self.slam=packet['slam']
        if packet['stamp']>self.extra_stamp:
            self.extra_stamp=packet['stamp'];self.plan=packet.get('planned_path',self.plan);self.mission=packet.get('mission',self.mission)

    def status_suffix(self):
        if not self.mission:return '\nInspection: waiting for telemetry'
        mission=self.mission;points=mission.get('checkpoints',[]);index=mission.get('index',0)
        target=points[index]['name'] if index<len(points) else 'route complete'
        return f"\nInspection: {mission['status']} | {target}"

    def publish_navigation(self):
        if self.slam:
            s=self.slam;msg=OccupancyGrid();msg.header.frame_id='estate';msg.header.stamp=ros_stamp(s['stamp'])
            msg.info.width=s['width'];msg.info.height=s['height'];msg.info.resolution=s['resolution']
            x,y,a=estate_pose(*s['origin']);msg.info.origin.position.x=x;msg.info.origin.position.y=y;msg.info.origin.position.z=.08
            msg.info.origin.orientation.z=math.sin(a/2);msg.info.origin.orientation.w=math.cos(a/2)
            data=np.frombuffer(zlib.decompress(base64.b64decode(s['data'])),np.int8)
            if data.size==s['width']*s['height']:msg.data=data.tolist();self.slam_pub.publish(msg)
        if self.plan:
            msg=Path();msg.header.frame_id='estate';msg.header.stamp=ros_stamp(self.plan['stamp'])
            for x,y in self.plan['points']:
                p=PoseStamped();p.header=msg.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=.18;p.pose.orientation.w=1.;msg.poses.append(p)
            self.plan_pub.publish(msg)
        stale=not self.latest or time.monotonic()-self.last_arrival>2. or self.sim_time-self.latest_stamp>2.
        data=dict(received_mission=self.mission,stale=stale,received_through_stamp=self.latest_stamp,
            slam_stamp=self.slam['stamp'] if self.slam else None,updated_wall=time.time())
        self.mission_pub.publish(String(data=json.dumps(data)));atomic_json(RUN/'twin-navigation.json',data)
        markers=MarkerArray()
        for i,point in enumerate((self.mission or {}).get('checkpoints',[])):
            m=Marker();m.header.frame_id='estate';m.ns='inspection';m.id=i;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
            m.pose.position.x=float(point['pose'][0]);m.pose.position.y=float(point['pose'][1]);m.pose.position.z=2.
            m.pose.orientation.w=1.;m.scale.z=.8;m.color.a=1.;m.color.r=1.;m.color.g=.85
            m.text=f"{i+1}. {point['name']} [{point['status']}]";markers.markers.append(m)
        clear=Marker();clear.action=Marker.DELETEALL;markers.markers.insert(0,clear);self.checkpoints_pub.publish(markers)

def main():
    rclpy.init();n=NavigationTwin()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if n.slam:atomic_json(RUN/'received-slam-map.json',n.slam)
        np.savez_compressed(RUN/'observed-map.npz',logodds=n.map.logodds,seen=n.map.seen)
        n.db.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
