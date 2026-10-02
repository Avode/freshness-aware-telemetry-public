#!/usr/bin/env python3
"""Move the Gazebo viewer camera only; does not command any robot."""
import argparse
import math
import os
import subprocess

VIEWS={
 'overview':((85,-110,125),(15,-35,0)),
 'depot':((-2,-53,13),(-14,-34,.5)),
 'go2':((-15.7,-34.7,1.1),(-17,-33,.28)),
 'husky':((-10.2,-35,1.5),(-12,-33,.55)),
 'drone':((10.1,-34,1),(9,-33,.24)),
 'plantation':((12,-6,18),(-38,30,1)),
}
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('view',choices=VIEWS)
args=parser.parse_args()
pos,target=VIEWS[args.view]
dx,dy,dz=[t-p for t,p in zip(target,pos)]
yaw=math.atan2(dy,dx);pitch=math.atan2(-dz,math.hypot(dx,dy))
cp,sp=math.cos(pitch/2),math.sin(pitch/2)
cy,sy=math.cos(yaw/2),math.sin(yaw/2)
qw,qx,qy,qz=cp*cy,-sp*sy,sp*cy,cp*sy
request=f'pose: {{position: {{x: {pos[0]} y: {pos[1]} z: {pos[2]}}} orientation: {{w: {qw} x: {qx} y: {qy} z: {qz}}}}}'
env=dict(os.environ,IGN_PARTITION=f'fleetscope_estate_{os.getuid()}',IGN_IP='127.0.0.1')
subprocess.run(['ign','service','-s','/gui/move_to/pose','--reqtype','ignition.msgs.GUICamera','--reptype','ignition.msgs.Boolean','--timeout','5000','--req',request],env=env,check=True)
