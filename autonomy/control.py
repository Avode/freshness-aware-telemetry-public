"""Start/cancel inspection jobs, interrupt telemetry, insert obstacles and save SLAM."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from slam_toolbox.srv import SerializePoseGraph

ROOT=Path(__file__).resolve().parents[1]
LATEST=Path(os.environ.get('FLEETSCOPE_RUNS', ROOT/'runs')).expanduser()/'autonomy-latest'

def read(name):
    try:return json.loads((LATEST/name).read_text())
    except (OSError,ValueError):return {}

class Controls(Node):
    def __init__(self):
        super().__init__('inspection_controls')
        self.pub=self.create_publisher(String,'/inspection/command',10)

    def wait(self,seconds):
        end=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.1)

    def call(self,typ,service,request):
        client=self.create_client(typ,service)
        if not client.wait_for_service(timeout_sec=8):raise RuntimeError(f'{service} is unavailable')
        future=client.call_async(request);rclpy.spin_until_future_complete(self,future,timeout_sec=15)
        if not future.done():raise RuntimeError(f'{service} timed out')
        return future.result()

    def command(self,action,route=None):
        deadline=time.monotonic()+8
        while not self.pub.get_subscription_count() and time.monotonic()<deadline:self.wait(.1)
        if not self.pub.get_subscription_count():raise RuntimeError('Inspection server is unavailable')
        request=dict(action=action,route=route,request_id=str(uuid.uuid4()))
        self.pub.publish(String(data=json.dumps(request)))
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            self.wait(.1);state=read('mission-status.json')
            if action=='start' and state.get('request_id')==request['request_id']:break
            if action=='cancel' and state.get('status') not in ('RUNNING','WAITING_FOR_NAV2'):break
        if action=='start' and state.get('request_id')!=request['request_id']:raise RuntimeError('Job was not accepted; cancel the active job first')
        if action=='cancel' and state.get('status') in ('RUNNING','WAITING_FOR_NAV2'):raise RuntimeError('Cancellation was not acknowledged')
        print(json.dumps(state,indent=2))

    def block(self,add,x=-12.,y=-40.,width=3.):
        if add:
            filename=LATEST.resolve()/'navigation-block.sdf'
            filename.write_text(f'''<sdf version="1.9"><model name="navigation_block"><static>true</static><pose>{x} {y} 0.65 0 0 0</pose><link name="barrier"><collision name="body"><geometry><box><size>{width} 1 1.3</size></box></geometry></collision><visual name="body"><geometry><box><size>{width} 1 1.3</size></box></geometry><material><diffuse>0.95 0.22 0.08 1</diffuse></material></visual></link></model></sdf>''')
            service='/world/fleetscope_autonomy/create';typ='ignition.msgs.EntityFactory'
            request='sdf_filename: '+json.dumps(str(filename))+' allow_renaming: false'
        else:service='/world/fleetscope_autonomy/remove';typ='ignition.msgs.Entity';request='name: "navigation_block" type: MODEL'
        result=subprocess.run(['ign','service','-s',service,'--reqtype',typ,'--reptype','ignition.msgs.Boolean','--timeout','5000','--req',request],capture_output=True,text=True,timeout=8)
        if result.returncode or 'true' not in result.stdout:raise RuntimeError(result.stdout+result.stderr)
        print('Obstacle '+('inserted; robot must discover it through LiDAR' if add else 'removed'))

    def save(self):
        directory=LATEST.resolve()/'maps';directory.mkdir(exist_ok=True)
        request=SerializePoseGraph.Request();request.filename=str(directory/'inspection')
        result=self.call(SerializePoseGraph,'/slam_toolbox/serialize_map',request)
        if result.result!=0:raise RuntimeError(f'Pose graph save failed: {result.result}')
        # SLAM's save_map service writes the occupancy map via its map saver.
        from slam_toolbox.srv import SaveMap as SlamSaveMap
        request=SlamSaveMap.Request();request.name.data=str(directory/'inspection')
        result=self.call(SlamSaveMap,'/slam_toolbox/save_map',request)
        if result.result!=0:raise RuntimeError(f'Occupancy save failed: {result.result}')
        print(f'Saved occupancy map and reusable pose graph: {directory}')

def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    sub.add_parser('status');s=sub.add_parser('start');s.add_argument('route',choices=['quick','inspection'],default='inspection',nargs='?')
    sub.add_parser('cancel');sub.add_parser('save-map');sub.add_parser('demo')
    l=sub.add_parser('link');l.add_argument('state',choices=['up','down'])
    b=sub.add_parser('block');b.add_argument('state',choices=['add','remove']);b.add_argument('--x',type=float,default=-12.);b.add_argument('--y',type=float,default=-40.);b.add_argument('--width',type=float,default=3.)
    args=p.parse_args()
    if args.action=='demo':
        from .demo import main as demo
        demo();return
    if args.action=='status':
        data={name:read(name+'.json') for name in ['mission-status','navigation-status','twin-navigation','edge-status','evaluation','dead-reckoning-evaluation']}
        data['run_directory']=str(LATEST.resolve());data['snapshots_are_current']=time.time()-data['edge-status'].get('updated_wall',0)<3.
        print(json.dumps(data,indent=2));return
    rclpy.init();n=Controls()
    try:
        if args.action in ('start','cancel'):n.command(args.action,getattr(args,'route',None))
        elif args.action=='link':print(n.call(SetBool,'/unit/husky/set_link',SetBool.Request(data=args.state=='up')).message)
        elif args.action=='block':n.block(args.state=='add',args.x,args.y,args.width)
        elif args.action=='save-map':n.save()
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
