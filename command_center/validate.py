"""Live HTTP acceptance tests. Fault scoring may inspect privileged evaluator logs."""
import argparse
import json
import math
import os
import sqlite3
from pathlib import Path
import time
import urllib.error
import urllib.request
import uuid


class Demo:
    def __init__(self, url):
        self.url = url
        self.token = self.get('/api/session')['token']
        self.run = Path(os.environ.get('FLEETSCOPE_RUNS', Path(__file__).resolve().parents[1]/'runs')).expanduser()/self.state()['run_id']
        self.report = dict(run=str(self.run), started_wall=time.time(), checks={})

    def get(self, path):
        with urllib.request.urlopen(self.url+path, timeout=10) as response: return json.load(response)

    def state(self): return self.get('/api/state')

    def post(self, path, value):
        request = urllib.request.Request(self.url+path, data=json.dumps(value).encode(),
            headers={'Content-Type':'application/json', 'X-FleetScope-Token':self.token})
        with urllib.request.urlopen(request, timeout=10) as response: return json.load(response)

    def wait(self, predicate, timeout=25):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            state=self.state()
            if predicate(state): return state
            time.sleep(.3)
        raise TimeoutError('Condition did not become true before deadline')

    def check(self, name, passed, details):
        self.report['checks'][name]=dict(passed=bool(passed), details=details)
        self.save(); print(('PASS ' if passed else 'FAIL ')+name, flush=True)
        if not passed: raise AssertionError(name)

    def save(self):
        (self.run/self.filename).write_text(json.dumps(self.report, indent=2))

    def command(self, action, route='quick', expect='ACKNOWLEDGED'):
        value=dict(action=action, route=route, request_id=str(uuid.uuid4()))
        self.post('/api/commands', value)
        result=self.wait(lambda s:any(c['id']==value['request_id'] and c['status']!='PENDING' for c in s['commands']))
        command=next(c for c in result['commands'] if c['id']==value['request_id'])
        if command['status']!=expect: raise AssertionError(command)
        return value

    def fault(self, name, active): return self.post('/api/simulation', dict(fault=name, active=active))

    def baseline(self):
        self.filename='command-center-baseline.json'
        state=self.wait(lambda s:s['live'] and s['robot']['health']['status']=='HEALTHY')
        if state['robot']['safety']['held']: self.command('resume')
        if (state['robot'].get('mission') or {}).get('status') not in ('RUNNING','WAITING_FOR_NAV2'):
            self.command('start','inspection')
        mission=self.state()['robot']['mission']; request_id=mission['request_id']
        events=[json.loads(line) for line in (self.run/'mission-events.jsonl').read_text().splitlines()]
        start=next(e['stamp'] for e in events if e.get('request_id')==request_id and e['event']=='requested')
        self.report['request_id']=request_id;self.report['start_stamp']=start;self.save()
        deadline=time.monotonic()+2400;last_log=0
        while time.monotonic()<deadline:
            state=self.state();mission=state['robot']['mission']
            if mission['request_id']!=request_id: raise AssertionError('Baseline mission changed')
            if mission['status'] in ('SUCCEEDED','FAILED','CANCELED'):break
            if time.monotonic()-last_log>30:
                print(f"Baseline: {mission['status']}, checkpoint {mission['index']+1}, {mission.get('distance_remaining_m',0):.1f} m remaining",flush=True);last_log=time.monotonic()
            time.sleep(1)
        self.check('complete_route',mission['status']=='SUCCEEDED',mission)
        events=[json.loads(line) for line in (self.run/'mission-events.jsonl').read_text().splitlines()]
        events=[e for e in events if e.get('request_id')==request_id]
        end=next(e['stamp'] for e in events if e['event']=='completed')
        samples=[json.loads(line) for line in (self.run/'pose-errors.jsonl').read_text().splitlines()]
        samples=[s for s in samples if start<=s['stamp']<=end]
        errors=sorted(s['position_error_m'] for s in samples)
        arrivals=[]
        for e in events:
            if e['event']=='checkpoint_reached':
                sample=min(samples,key=lambda s:abs(s['stamp']-e['stamp']))
                goal=e['checkpoints'][e['index']]
                arrivals.append(dict(name=goal['name'], physical_error_m=math.dist(goal['pose'][:2],sample['truth'][:2]),stamp_gap_s=abs(sample['stamp']-e['stamp'])))
        self.report['accuracy']=dict(samples=len(errors), rmse_m=math.sqrt(sum(e*e for e in errors)/len(errors)),
            max_m=max(errors), p95_m=errors[int(.95*(len(errors)-1))], arrivals=arrivals,
            note='Measured by separate ground-truth evaluator; completion is not a precision-positioning claim')
        import yaml
        config=yaml.safe_load((self.run/'config/nav2.yaml').read_text())
        self.report['amcl_max_beams']=config['amcl']['ros__parameters']['max_beams']
        self.report['passed']=True

    def faults(self):
        self.filename='command-center-validation.json'
        state=self.wait(lambda s:s['live'] and s['robot']['health']['status']=='HEALTHY')
        self.check('fresh_telemetry',True,dict(seq=state['robot']['seq'],health=state['robot']['health']['status']))
        if state['robot']['safety']['held']:self.command('resume')
        self.fault('camera_freeze',True); start=time.monotonic()
        state=self.wait(lambda s:s['robot']['health']['sensors']['camera']['status']=='STALE')
        frozen=state['robot']['camera_stamp']
        self.check('camera_freeze_is_warning',state['robot']['health']['status']=='DEGRADED' and not state['robot']['safety']['held'],dict(detection_wall_s=time.monotonic()-start,camera_stamp=frozen))
        time.sleep(1.);self.check('camera_capture_does_not_advance',self.state()['robot']['camera_stamp']==frozen,frozen)
        replay_seq=state['robot']['seq']
        self.fault('camera_freeze',False);self.wait(lambda s:s['robot']['health']['status']=='HEALTHY')
        self.wait(lambda s:s['robot']['health'].get('motion_disagreement_m') is not None,35)
        self.fault('position_jump',True)
        jumped=self.wait(lambda s:any(i['code']=='position_motion_disagreement' for i in s['robot']['health']['issues']))
        self.check('pose_jump_detected_without_ground_truth',jumped['robot']['health']['status']=='DEGRADED' and all(v['status']=='OK' for v in jumped['robot']['health']['sensors'].values()),jumped['robot']['health']['motion_disagreement_m'])
        self.fault('position_jump',False);self.wait(lambda s:s['robot']['health']['status']=='HEALTHY',45)
        self.command('start')
        moving=self.wait(lambda s:abs(s['robot']['state']['speed'])>.12,timeout=90)
        self.check('dashboard_dispatch_moves_robot',True,moving['robot']['mission']['status'])
        self.fault('imu_dropout',True);start=time.monotonic()
        held=self.wait(lambda s:s['robot']['safety']['held'] and abs(s['robot']['state']['speed'])<.03)
        self.check('imu_loss_holds_locally',held['robot']['health']['status']=='CRITICAL',dict(detection_and_stop_wall_s=time.monotonic()-start,reason=held['robot']['safety']['reason']))
        self.command('resume',expect='REJECTED')
        self.check('unsafe_resume_rejected',True,'IMU still absent')
        self.fault('imu_dropout',False)
        recovered=self.wait(lambda s:s['robot']['health']['status']=='HEALTHY' and s['robot']['safety']['release_ready'])
        self.check('recovery_keeps_hold_latched',recovered['robot']['safety']['held'],recovered['robot']['safety'])
        self.command('resume');self.wait(lambda s:not s['robot']['safety']['held'])
        self.wait(lambda s:s['robot']['mission']['status'] not in ('RUNNING','CANCELING','WAITING_FOR_NAV2'))
        self.command('start');self.wait(lambda s:abs(s['robot']['state']['speed'])>.12,90)
        self.fault('link_down',True)
        stale=self.wait(lambda s:not s['live']); seq=stale['robot']['seq'];pose=stale['robot']['state']['pose'];cam=stale['robot']['camera_stamp']
        start=time.monotonic();self.fault('imu_dropout',True)
        # Independent validation is allowed to inspect the onboard/evaluator artifacts.
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            local=json.loads((self.run/'safety-status.json').read_text())
            if local['held']:break
            time.sleep(.1)
        self.check('offline_robot_holds_without_dashboard',local['held'],dict(reason=local['reason'],wall_s=time.monotonic()-start))
        # Gazebo decelerates in simulation time. Establish rest from timestamped
        # truth samples instead of assuming a wall-second is a simulated second.
        def truth():return json.loads((self.run/'evaluation.json').read_text())
        first=previous=truth();stable_since=None;settled=False;max_distance=0.;speed=None
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            time.sleep(.2);current=truth()
            dt=current['last_truth_stamp']-previous['last_truth_stamp']
            if dt<.2:continue
            speed=math.dist(current['last_truth_pose'][:2],previous['last_truth_pose'][:2])/dt
            max_distance=max(max_distance,math.dist(current['last_truth_pose'][:2],first['last_truth_pose'][:2]))
            if speed<.03:
                if stable_since is None:stable_since=current['last_truth_stamp']
                if current['last_truth_stamp']-stable_since>=.5:settled=True;break
            else:stable_since=None
            if current['last_truth_stamp']-first['last_truth_stamp']>3.:break
            previous=current
        self.check('offline_hold_physically_stops_robot',settled and max_distance<.7,
            dict(settled=settled,settling_sim_s=current['last_truth_stamp']-first['last_truth_stamp'],
                 max_distance_after_hold_m=max_distance,final_measured_speed_m_s=speed))
        state=self.state()
        self.check('disconnected_view_does_not_leak_local_state',state['robot']['seq']==seq and state['robot']['state']['pose']==pose and state['robot']['camera_stamp']==cam,dict(seq=seq))
        try:self.post('/api/commands',dict(action='start',route='quick'));offline_rejected=False
        except urllib.error.HTTPError as error:offline_rejected=error.code==409
        self.check('stale_dashboard_blocks_commands',offline_rejected,'HTTP 409')
        self.fault('imu_dropout',False);self.fault('link_down',False)
        state=self.wait(lambda s:s['live'] and s['robot']['health']['status']=='HEALTHY' and s['robot']['safety']['release_ready'],45)
        state=self.wait(lambda s:s['robot']['transport']['queued']<3,45)
        self.check('replay_drains_without_drops',state['robot']['transport']['dropped']==0,dict(transport=state['robot']['transport'],replayed=state['center']['replayed']))
        cutoff=state['robot']['seq'];deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            with sqlite3.connect((self.run/'twin-history.sqlite').as_uri()+'?mode=ro',uri=True) as db:
                received=db.execute('SELECT COUNT(*) FROM packets WHERE seq<=?',(cutoff,)).fetchone()[0]
            with sqlite3.connect((self.run/'dashboard.sqlite').as_uri()+'?mode=ro',uri=True) as db:
                projected=db.execute('SELECT COUNT(*) FROM snapshots WHERE seq<=?',(cutoff,)).fetchone()[0]
            if received==projected==cutoff:break
            time.sleep(.2)
        self.check('all_delivered_packets_survive_web_projection',received==projected==cutoff,dict(through_seq=cutoff,received=received,projected=projected))
        self.check('offline_fault_history_delivered',any(e['kind']=='critical' and e['seq']>seq for e in state['events']),len(state['events']))
        replay=self.get('/api/state?seq='+str(replay_seq))
        self.check('historical_camera_and_fault_are_isolated',replay['replay'] and not replay['live'] and replay['robot']['camera_stamp']==frozen and replay['robot']['health']['status']=='DEGRADED',dict(seq=replay_seq,camera_stamp=frozen))
        self.command('resume');self.wait(lambda s:not s['robot']['safety']['held'])
        self.fault('position_dropout',True)
        state=self.wait(lambda s:s['robot']['health']['sensors']['position']['status']=='STALE' and s['robot']['safety']['held'])
        old_seq=state['robot']['seq'];old_envelope=state['robot']['stamp'];old_pose=state['robot']['state']['stamp']
        advanced=self.wait(lambda s:s['robot']['seq']>old_seq+1)
        self.check('health_heartbeat_survives_position_loss',advanced['live'] and advanced['robot']['stamp']>old_envelope and advanced['robot']['state']['stamp']==old_pose,
                   dict(first_seq=old_seq,later_seq=advanced['robot']['seq'],envelope_stamp=advanced['robot']['stamp'],unchanged_pose_stamp=old_pose))
        self.fault('position_dropout',False);self.wait(lambda s:s['robot']['safety']['release_ready'])
        self.command('resume');self.wait(lambda s:not s['robot']['safety']['held'])
        self.fault('lidar_dropout',True)
        state=self.wait(lambda s:s['robot']['health']['sensors']['lidar']['status']=='STALE' and s['robot']['safety']['held'])
        self.check('lidar_loss_holds_robot',True,state['robot']['safety']['reason'])
        self.fault('lidar_dropout',False);self.wait(lambda s:s['robot']['safety']['release_ready'])
        self.command('resume');self.wait(lambda s:not s['robot']['safety']['held'])
        self.report['passed']=True


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--url',default='http://127.0.0.1:8765');parser.add_argument('--baseline',action='store_true')
    args=parser.parse_args();demo=Demo(args.url)
    try:
        if args.baseline: demo.baseline()
        else: demo.faults()
    except (Exception,KeyboardInterrupt) as error:
        demo.report['passed']=False;demo.report['error']=str(error);raise
    finally:
        if not args.baseline:
            for fault in ('camera_freeze','imu_dropout','lidar_dropout','position_dropout','position_jump','link_down'):
                try:demo.fault(fault,False)
                except Exception:pass
        demo.report['finished_wall']=time.time();demo.save()


if __name__=='__main__':main()
