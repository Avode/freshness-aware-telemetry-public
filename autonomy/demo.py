"""Repeatable physical navigation and telemetry-outage acceptance scenario."""
import json
import math
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
from .control import Controls,LATEST,read

def run(node):
    report=dict(checks={},started_wall=time.time(),run=str(LATEST.resolve()))
    route_seen={'detour':False}
    def plan(msg):
        points=[p.pose.position for p in msg.poses if abs(p.pose.position.y+40)<.6]
        if points and all(abs(p.x+12)>2.1 for p in points):route_seen['detour']=True
    sub=node.create_subscription(Path,'/twin/planned_path',plan,10)
    def check(name,passed,details):
        report['checks'][name]=dict(passed=bool(passed),details=details)
        (LATEST/'autonomy-demo-report.json').write_text(json.dumps(report,indent=2))
        print(f"{'PASS' if passed else 'FAIL'} {name}: {details}",flush=True)
        if not passed:raise AssertionError(name)
    def until(predicate,seconds,message):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            node.wait(.3)
            if predicate():return
        raise TimeoutError(message)
    until(lambda:read('twin-status.json').get('status')=='LIVE' and time.time()-read('twin-status.json').get('updated_wall',0)<2. and read('twin-navigation.json').get('slam_stamp'),60,'Sensors / SLAM did not become live')
    check('fresh_start',read('mission-status.json').get('status')=='IDLE' and read('evaluation.json').get('truth_distance_m',999)<.5,'Run this scenario once per fresh stack')
    node.command('start','quick')
    until(lambda:read('evaluation.json').get('truth_distance_m',0)>.4,60,'Robot did not move')
    node.call(SetBool,'/unit/husky/set_link',SetBool.Request(data=False))
    def drained():
        twin=read('twin-status.json');navigation=read('twin-navigation.json')
        return (twin.get('status')=='STALE' and (twin.get('transport_age_wall_s') or 0)>2.5
                and navigation.get('received_through_stamp')==twin.get('state',{}).get('stamp'))
    until(drained,20,'In-flight telemetry did not settle after disabling the link')
    frozen=read('twin-status.json')['state']['stamp'];map_stamp=read('twin-navigation.json')['slam_stamp']
    before=read('evaluation.json')['truth_distance_m']
    node.block(True);node.wait(8.)
    twin=read('twin-status.json');edge=read('edge-status.json');nav=read('twin-navigation.json')
    check('center_stays_stale_and_frozen',twin['status']=='STALE' and twin['state']['stamp']==frozen and nav['slam_stamp']==map_stamp,
          dict(status=twin['status'],pose_stamp=twin['state']['stamp'],slam_stamp=nav['slam_stamp'],
               frozen_pose_stamp=frozen,frozen_slam_stamp=map_stamp))
    check('onboard_navigation_continues',read('evaluation.json')['truth_distance_m']-before>.5,read('evaluation.json')['truth_distance_m']-before)
    check('durable_buffer_during_outage',edge['queued']>8 and edge['dropped']==0,edge['queued'])
    node.call(SetBool,'/unit/husky/set_link',SetBool.Request(data=True))
    until(lambda:read('twin-status.json').get('status')=='LIVE' and read('edge-status.json').get('queued',999)<3,40,'Replay did not catch up')
    check('replayed_without_drops',read('twin-status.json')['replayed']>0 and read('edge-status.json')['dropped']==0,read('twin-status.json')['replayed'])
    last_print=time.monotonic();end=time.monotonic()+900
    while time.monotonic()<end:
        node.wait(.4);mission=read('mission-status.json')
        if time.time()-read('edge-status.json').get('updated_wall',0)>5:raise RuntimeError('The simulation stack stopped or became unresponsive')
        if mission.get('status') in ('SUCCEEDED','FAILED','CANCELED'):break
        if time.monotonic()-last_print>20:
            print(f"Checkpoint {mission.get('index',0)+1}/3, remaining {mission.get('distance_remaining_m',0):.1f} m",flush=True);last_print=time.monotonic()
    check('received_detour_around_new_obstacle',route_seen['detour'],route_seen)
    check('all_checkpoints_reached',mission.get('status')=='SUCCEEDED',mission)
    rows=[json.loads(line) for line in (LATEST/'pose-errors.jsonl').read_text().splitlines()]
    # Privileged truth is used only here for scoring, never for robot decisions.
    distances=[math.hypot(max(abs(r['truth'][0]+12)-1.5,0),max(abs(r['truth'][1]+40)-.5,0)) for r in rows]
    check('physical_barrier_clearance',min(distances)>.72,dict(min_center_to_barrier_m=min(distances),required_m=.72))
    ev=read('evaluation.json')
    check('localization_accuracy',ev['position_rmse_m']<.5 and ev['position_max_m']<1.,ev)
    node.save()
    report['passed']=True;report['finished_wall']=time.time()
    (LATEST/'autonomy-demo-report.json').write_text(json.dumps(report,indent=2))
    print('Autonomous inspection demo passed: '+str(LATEST/'autonomy-demo-report.json'),flush=True)

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Controls()
    try:run(node)
    except (Exception,KeyboardInterrupt) as error:
        report=read('autonomy-demo-report.json');report.update(passed=False,error=str(error) or 'Interrupted',finished_wall=time.time())
        (LATEST/'autonomy-demo-report.json').write_text(json.dumps(report,indent=2))
        raise
    finally:
        # On interrupted verification restore delivery and cancel any remaining job.
        if rclpy.ok():
            try:
                if read('mission-status.json').get('status') in ('RUNNING','WAITING_FOR_NAV2'):node.command('cancel')
                node.call(SetBool,'/unit/husky/set_link',SetBool.Request(data=True))
            except Exception as error:print(f'Cleanup could not contact the stack: {error}',flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
