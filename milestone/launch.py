"""Own and supervise just this milestone's processes; Ctrl-C stops the stack."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
BASE=Path(os.environ.get('FLEETSCOPE_RUNS', ROOT/'runs')).expanduser()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--no-rviz',action='store_true',help='Run the backend and sensors without the command-center window')
    p.add_argument('--duration',type=float,default=0,help='Stop automatically after this many wall seconds; 0 runs until Ctrl-C')
    args=p.parse_args()
    BASE.mkdir(parents=True,exist_ok=True)
    lock=(BASE/'milestone.lock').open('w')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise SystemExit('A milestone stack is already running. Stop it before starting another.')
    subprocess.run([sys.executable,str(ROOT/'milestone/build.py')],check=True)
    run=BASE/datetime.now().strftime('milestone-%Y%m%d-%H%M%S')
    run.mkdir(); env=dict(os.environ,FLEETSCOPE_RUN=str(run),FLEETSCOPE_RUN_ID=run.name)
    latest=BASE/'milestone-latest'
    if latest.is_symlink(): latest.unlink()
    latest.symlink_to(run,target_is_directory=True)
    overlay=Path(env['FLEETSCOPE_OVERLAY'])
    ekf=overlay/'opt/ros/humble/lib/robot_localization/ekf_node'
    if not ekf.exists(): raise SystemExit('Run milestone/setup-deps.sh once to provision the local ROS overlay.')
    commands=[
        ('bridge',['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge','--ros-args','-p',f'config_file:={ROOT}/milestone/bridge.yaml']),
        ('adapter',[sys.executable,'-m','milestone.adapter','--ros-args','-p','use_sim_time:=true']),
        ('ekf',[str(ekf),'--ros-args','--params-file',str(ROOT/'milestone/generated/ekf.yaml'),'-r','odometry/filtered:=/husky/odometry/filtered']),
        ('edge_telemetry',[sys.executable,'-m','milestone.unit','--ros-args','-p','use_sim_time:=true']),
        ('twin',[sys.executable,'-m','milestone.twin']),
        ('evaluator',[sys.executable,'-m','milestone.evaluator']),
        ('gazebo',['ign','gazebo','-s','-r',str(ROOT/'milestone/generated/world.sdf'),'-v','2']),
    ]
    if not args.no_rviz:
        commands.append(('rviz',['rviz2','-d',str(ROOT/'milestone/command-center.rviz'),'--ros-args','-p','use_sim_time:=true']))
    processes=[]; stopping=False
    def stop(signum,frame):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    try:
        for name,cmd in commands:
            log=(run/f'{name}.log').open('w')
            proc=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append((name,proc,log))
        (run/'processes.json').write_text(json.dumps({name:proc.pid for name,proc,_ in processes},indent=2))
        print(f'FleetScope started. Logs: {run}',flush=True)
        print('Controls: ./milestone/control.sh status | drive --linear 0.4 --seconds 5 | demo',flush=True)
        start=time.monotonic()
        while not stopping and (not args.duration or time.monotonic()-start<args.duration):
            for name,proc,_ in processes:
                if proc.poll() is not None:
                    if name=='rviz' and proc.returncode==0:
                        stopping=True; break
                    raise RuntimeError(f'{name} exited with {proc.returncode}; see {run}/{name}.log')
            time.sleep(.25)
    finally:
        # Shut down whole owned process groups, including Gazebo's child server.
        for name,proc,_ in processes:
            if proc.poll() is None:
                try: os.killpg(proc.pid,signal.SIGINT)
                except ProcessLookupError: pass
        deadline=time.monotonic()+8
        for name,proc,log in processes:
            try: proc.wait(timeout=max(.1,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                try: os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError: pass
                proc.wait()
            log.close()
        print(f'Stopped. Data preserved in {run}',flush=True)

if __name__=='__main__': main()
