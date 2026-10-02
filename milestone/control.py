"""Local experiment controls. No mission planner or remote actuation gateway."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import SetBool
from .core import wrap

ROOT=Path(__file__).resolve().parents[1]
LATEST=Path(os.environ.get('FLEETSCOPE_RUNS', ROOT/'runs')).expanduser()/'milestone-latest'

def read(name):
    try: return json.loads((LATEST/name).read_text())
    except (OSError,ValueError): return {}

class Control(Node):
    def __init__(self):
        super().__init__('fleetscope_controls')
        self.pub=self.create_publisher(Twist,'/husky/cmd_vel',10)
        self.client=self.create_client(SetBool,'/unit/husky/set_link')
        self.grid=None
        self.create_subscription(OccupancyGrid,'/twin/observed_map',lambda m:setattr(self,'grid',m),10)

    def wait(self,seconds):
        until=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<until:
            rclpy.spin_once(self,timeout_sec=min(.1,max(0,until-time.monotonic())))

    def link(self,up):
        if not self.client.wait_for_service(timeout_sec=5): raise RuntimeError('Edge telemetry agent is not running')
        future=self.client.call_async(SetBool.Request(data=up))
        rclpy.spin_until_future_complete(self,future,timeout_sec=5)
        if not future.done() or not future.result().success: raise RuntimeError('Link control failed')
        print(future.result().message,flush=True)

    def drive(self,linear,angular,seconds):
        if not 0<seconds<=30: raise ValueError('Choose a duration between 0 and 30 seconds')
        cmd=Twist(); cmd.linear.x=float(linear); cmd.angular.z=float(angular)
        until=time.monotonic()+seconds
        try:
            while time.monotonic()<until:
                self.pub.publish(cmd); self.wait(.1)
        finally:
            for _ in range(4): self.pub.publish(Twist()); self.wait(.05)

    def block(self,add=True):
        if add:
            service='/world/fleetscope_milestone/create'; typ='ignition.msgs.EntityFactory'
            req='sdf_filename: '+json.dumps(str(ROOT/'milestone/generated/road_block.sdf'))+' allow_renaming: false'
        else:
            service='/world/fleetscope_milestone/remove'; typ='ignition.msgs.Entity'; req='name: "road_block" type: MODEL'
        result=subprocess.run(['ign','service','-s',service,'--reqtype',typ,'--reptype','ignition.msgs.Boolean',
                                '--timeout','5000','--req',req],capture_output=True,text=True,timeout=8)
        if result.returncode or 'true' not in result.stdout: raise RuntimeError('Obstacle operation failed: '+result.stdout+result.stderr)

    def occupancy_near_block(self):
        if self.grid is None: return -1
        g=self.grid; values=[]
        # The face visible from the starting forecourt, not a hidden truth feed.
        for x in (-12.5,-12.25,-12.,-11.75,-11.5):
            for y in (-40.25,-40.5,-40.75):
                ix=int((x-g.info.origin.position.x)/g.info.resolution)
                iy=int((y-g.info.origin.position.y)/g.info.resolution)
                if 0<=ix<g.info.width and 0<=iy<g.info.height: values.append(g.data[iy*g.info.width+ix])
        return max(values,default=-1)

    def demo(self):
        report={'checks':{},'started_wall':time.time()}
        def check(name,condition,details=None):
            report['checks'][name]={'passed':bool(condition),'details':details}
            print(f"{'PASS' if condition else 'FAIL'} {name}: {details}",flush=True)
            (LATEST/'demo-report.json').write_text(json.dumps(report,indent=2))
            if not condition: raise AssertionError(name)
        print('Waiting for live sensors and estimated state…',flush=True)
        for _ in range(60):
            self.wait(.5); t=read('twin-status.json')
            if t.get('status')=='LIVE' and time.time()-t.get('updated_wall',0)<2 and t.get('scans_integrated',0)>8 and t.get('camera_stamp',-1)>0: break
        check('live_sensor_pipeline',t.get('status')=='LIVE' and t.get('camera_stamp',-1)>0,t.get('sensor_age_at_edge'))
        check('no_obstacle_in_prior',self.occupancy_near_block()<60,self.occupancy_near_block())
        self.drive(.45,0,4); self.wait(1)
        ev=read('evaluation.json')
        check('physical_motion',ev.get('truth_distance_m',0)>.8,ev.get('truth_distance_m'))
        self.link(False); self.wait(.8)
        before=read('twin-status.json'); frozen=before['state']['stamp']
        self.block(True)
        self.wait(4.)
        during=read('twin-status.json'); edge=read('edge-status.json')
        check('stale_during_outage',during['status']=='STALE',during['status'])
        check('no_truth_leak_during_outage',during['state']['stamp']==frozen and self.occupancy_near_block()<60,
              {'stamp_unchanged':during['state']['stamp']==frozen,'obstacle_cell':self.occupancy_near_block()})
        check('edge_buffered',edge.get('queued',0)>5,edge.get('queued'))
        self.link(True)
        for _ in range(60):
            self.wait(.3); after=read('twin-status.json'); edge=read('edge-status.json')
            if after.get('status')=='LIVE' and edge.get('queued',100)<3 and self.occupancy_near_block()>65: break
        check('reconnected_and_replayed',after['status']=='LIVE' and after.get('replayed',0)>0 and edge['queued']<3,
              {'status':after['status'],'replayed':after.get('replayed'),'queued':edge.get('queued')})
        check('obstacle_observed_after_delivery',self.occupancy_near_block()>65,self.occupancy_near_block())
        self.drive(.6,0,12)
        self.wait(.3); final=read('twin-status.json'); ev=read('evaluation.json')
        check('estimated_pose_evaluated',ev.get('samples',0)>20 and ev.get('position_rmse_m',999)<.65,ev)
        # Forward motion stops before the crate; test the robot's local guard directly
        # while publishing a continuing forward command (not after watchdog timeout).
        cmd=Twist(); cmd.linear.x=.4
        for _ in range(12): self.pub.publish(cmd); self.wait(.1)
        final=read('twin-status.json')
        check('local_obstacle_stop',final.get('drive_status')=='obstacle_stop',final.get('drive_status'))
        self.pub.publish(Twist())
        self.wait(.8)
        before_turn=read('evaluation.json')['last_truth_pose'][2]
        self.drive(0,.5,4); self.wait(1.)
        turned=read('evaluation.json'); state=read('twin-status.json')['state']
        turn=abs(wrap(turned['last_truth_pose'][2]-before_turn))
        yaw_error=abs(wrap(state['pose'][2]-turned['last_truth_pose'][2]))
        check('physical_steering_and_imu_tracking',turn>.15 and yaw_error<.10,
              {'rotation_rad':turn,'estimated_yaw_error_rad':yaw_error})
        report['passed']=True; report['finished_wall']=time.time()
        (LATEST/'demo-report.json').write_text(json.dumps(report,indent=2))
        print('Milestone demonstration passed. Report: '+str(LATEST/'demo-report.json'),flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__); sub=p.add_subparsers(dest='action',required=True)
    sub.add_parser('status'); d=sub.add_parser('drive'); d.add_argument('--linear',type=float,default=0.); d.add_argument('--angular',type=float,default=0.); d.add_argument('--seconds',type=float,default=3.)
    l=sub.add_parser('link'); l.add_argument('state',choices=['up','down'])
    b=sub.add_parser('block'); b.add_argument('state',choices=['add','remove'])
    sub.add_parser('demo')
    args=p.parse_args()
    if args.action=='status':
        print(json.dumps({name:read(name+'.json') for name in ('twin-status','edge-status','evaluation')},indent=2)); return
    rclpy.init(); node=Control(); node.wait(.5)
    try:
        if args.action=='drive': node.drive(args.linear,args.angular,args.seconds)
        elif args.action=='link': node.link(args.state=='up')
        elif args.action=='block': node.block(args.state=='add')
        elif args.action=='demo': node.demo()
    finally:
        node.pub.publish(Twist()); node.wait(.2); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
