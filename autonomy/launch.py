"""Own and supervise just this milestone's processes; Ctrl-C stops the stack."""
import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[1]
BASE=Path(os.environ.get('FLEETSCOPE_RUNS', ROOT/'runs')).expanduser()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--no-rviz',action='store_true',help='Run the backend and sensors without the command-center window')
    p.add_argument('--duration',type=float,default=0,help='Stop automatically after this many wall seconds; 0 runs until Ctrl-C')
    p.add_argument('--map',type=Path,help='Saved SLAM pose-graph prefix; start again at the surveyed depot pose')
    p.add_argument('--command-center',action='store_true',help='Enable edge health monitoring, local safety supervisor and web dashboard')
    p.add_argument('--remote-center',action='store_true',help='MQTT to a remote command center; no local dashboard or receipt ACKs')
    p.add_argument('--port',type=int,default=8765,help='Loopback web dashboard port')
    args=p.parse_args()
    if args.remote_center:
        if not os.environ.get('FLEETSCOPE_MQTT_CONFIG') or not os.environ.get('FLEETSCOPE_SITE_CONFIG'):
            raise SystemExit('Use run-remote-center.sh with broker and site configuration.')
        args.command_center=True
        args.no_rviz=True
    required=['slam_toolbox/async_slam_toolbox_node','nav2_amcl/amcl','nav2_controller/controller_server',
              'nav2_planner/planner_server','nav2_behaviors/behavior_server','nav2_bt_navigator/bt_navigator',
              'nav2_lifecycle_manager/lifecycle_manager','ros_gz_bridge/parameter_bridge']
    missing=[name for name in required if not (Path('/opt/ros/humble/lib')/name).is_file()]
    if missing:raise SystemExit('Missing ROS Humble executables: '+', '.join(missing))
    BASE.mkdir(parents=True,exist_ok=True)
    lock=(BASE/'milestone.lock').open('w')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise SystemExit('A milestone stack is already running. Stop it before starting another.')
    subprocess.run([sys.executable,str(ROOT/'autonomy/build.py')],check=True)
    if args.map:
        args.map=args.map.expanduser().resolve()
        if not all(Path(str(args.map)+suffix).is_file() for suffix in ('.posegraph','.data')):
            raise SystemExit('--map needs a prefix with both .posegraph and .data files')
    run=BASE/datetime.now().strftime('autonomy-%Y%m%d-%H%M%S')
    run.mkdir(); env=dict(os.environ,FLEETSCOPE_RUN=str(run),FLEETSCOPE_RUN_ID=run.name)
    config=run/'config';shutil.copytree(ROOT/'autonomy/generated',config)
    shutil.copy2(ROOT/'milestone/bridge.yaml',config/'bridge.yaml')
    shutil.copy2(ROOT/'config/scene.json',config/'scene.json')
    shutil.copy2(ROOT/'autonomy/navigate.xml',config/'navigate.xml')
    nav=yaml.safe_load((config/'nav2.yaml').read_text())
    nav['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(config/'navigate.xml')
    (config/'nav2.yaml').write_text(yaml.safe_dump(nav))
    sources={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
             for folder in ('autonomy','milestone','command_center') for path in (ROOT/folder).rglob('*') if path.is_file() and path.suffix in ('.py','.js','.css','.html')}
    (run/'run-metadata.json').write_text(json.dumps(dict(sources_sha256=sources,ros_domain=env.get('ROS_DOMAIN_ID'),
        ignition_partition=env.get('IGN_PARTITION'),command_center=args.command_center,
        remote_center=args.remote_center,
        loaded_slam_graph=str(args.map) if args.map else None),indent=2))
    latest=BASE/'autonomy-latest'
    if latest.is_symlink(): latest.unlink()
    latest.symlink_to(run,target_is_directory=True)
    overlay=Path(env['FLEETSCOPE_OVERLAY'])
    ekf=overlay/'opt/ros/humble/lib/robot_localization/ekf_node'
    if not ekf.exists(): raise SystemExit('Run milestone/setup-deps.sh once to provision the local ROS overlay.')
    commands=[
        ('bridge',['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge','--ros-args','-p',f'config_file:={config}/bridge.yaml']),
        ('adapter',[sys.executable,'-m','milestone.adapter','--ros-args','-p','use_sim_time:=true','-r','/husky/drive:=/unused/adapter_drive','-r','/husky/drive_status:=/unused/adapter_status']),
        ('ekf',[str(ekf),'--ros-args','--params-file',str(config/'ekf.yaml'),'-r','odometry/filtered:=/husky/odometry/filtered']),
        ('edge_telemetry',[sys.executable,'-m','autonomy.unit','--ros-args','-p','use_sim_time:=true','-r','/husky/odometry/filtered:=/husky/odometry/localized']),
        ('twin',[sys.executable,'-m','autonomy.twin']),
        ('evaluator',[sys.executable,'-m','autonomy.evaluator']),
        ('gazebo',['ign','gazebo','-s','-r',str(config/'world.sdf'),'-v','2']),
    ]
    commands += [
        ('navigation',[sys.executable,'-m','autonomy.navigation','--ros-args','-p','use_sim_time:=true']),
        ('slam',['/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node','--ros-args','--params-file',str(config/'slam.yaml')]),
        ('mission',[sys.executable,'-m','autonomy.mission','--ros-args','-p','use_sim_time:=true']),
    ]
    commands.append(('amcl',['/opt/ros/humble/lib/nav2_amcl/amcl','--ros-args','--params-file',str(config/'nav2.yaml'),'-r','map:=/navigation/prior_map']))
    if args.map:
        next(cmd for name,cmd in commands if name=='slam').extend(['-p',f'map_file_name:={args.map}','-p','map_start_at_dock:=true'])
    for name,package in [('controller_server','nav2_controller'),('planner_server','nav2_planner'),('behavior_server','nav2_behaviors'),('bt_navigator','nav2_bt_navigator')]:
        commands.append((name,[f'/opt/ros/humble/lib/{package}/{name}','--ros-args','--params-file',str(config/'nav2.yaml'),'-r','cmd_vel:=/navigation/cmd_vel']))
    commands.append(('lifecycle',['/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager','--ros-args','-r','__node:=lifecycle_manager_navigation','-p','use_sim_time:=true','-p','autostart:=true','-p','bond_timeout:=30.0','-p','node_names:=[amcl, controller_server, planner_server, behavior_server, bt_navigator]']))
    if args.command_center:
        next(cmd for name,cmd in commands if name=='adapter')[2]='command_center.fault_adapter'
        next(cmd for name,cmd in commands if name=='edge_telemetry')[2]='command_center.unit'
        next(cmd for name,cmd in commands if name=='navigation').extend(['-p','require_safety:=true'])
        commands += [('health',[sys.executable,'-m','command_center.observer','--ros-args','-p','use_sim_time:=true']),
                     ('safety',[sys.executable,'-m','command_center.safety','--ros-args','-p','use_sim_time:=true']),
                     ('dashboard',[sys.executable,'-m','command_center.server','--port',str(args.port)])]
    if not args.no_rviz:
        commands.append(('rviz',['rviz2','-d',str(config/'command-center.rviz'),'--ros-args','-p','use_sim_time:=true']))
    if args.remote_center:
        # The Ubuntu twin must not consume receipts on behalf of the Mac.
        commands=[(name,cmd) for name,cmd in commands if name not in ('twin','dashboard')]
        commands.append(('mqtt-gateway',[sys.executable,'-m','network.gateway']))
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
        print('Controls: ./autonomy/control.sh start quick | start inspection | status | cancel',flush=True)
        if args.remote_center: print('Remote command center: MQTT gateway active; dashboard runs on the Mac.',flush=True)
        elif args.command_center: print(f'Web dashboard: http://127.0.0.1:{args.port}',flush=True)
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
