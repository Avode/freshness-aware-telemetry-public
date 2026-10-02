"""SLAM and navigation observations share the durable, faultable edge transport."""
import base64
import json
import zlib
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from milestone.unit import EdgeTelemetryAgent, stamp, yaw

class NavigationTelemetryAgent(EdgeTelemetryAgent):
    def __init__(self):
        super().__init__()
        self.slam=None;self.route=None;self.mission=None;self.last_map_sent=-1.;self.last_map_envelope=-10.
        self.create_subscription(OccupancyGrid,'/map',self.map_received,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Path,'/plan',self.path_received,10)
        self.create_subscription(String,'/inspection/status',lambda m:setattr(self,'mission',json.loads(m.data)),10)

    def map_received(self,msg):
        if msg.header.frame_id!='slam_map':return
        p=msg.info.origin
        self.slam=dict(stamp=stamp(msg),frame=msg.header.frame_id,resolution=msg.info.resolution,
            width=msg.info.width,height=msg.info.height,origin=[p.position.x,p.position.y,yaw(p.orientation)],
            data=base64.b64encode(zlib.compress(np.asarray(msg.data,dtype=np.int8).tobytes(),3)).decode())

    def path_received(self,msg):
        if msg.header.frame_id!='estate':return
        self.route=dict(stamp=stamp(msg),points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses])

    def enrich_packet(self,packet):
        packet['mission']=self.mission
        # A complete compressed snapshot permits recovery even after queue overflow.
        if self.slam and (self.slam['stamp']>self.last_map_sent or packet['stamp']-self.last_map_envelope>=5.):
            packet['slam']=self.slam;self.last_map_sent=self.slam['stamp'];self.last_map_envelope=packet['stamp']
        if self.route:packet['planned_path']=self.route

def main():
    rclpy.init();n=NavigationTelemetryAgent()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.outbox.db.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
