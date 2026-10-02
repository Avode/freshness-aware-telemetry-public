#!/usr/bin/env python3
"""Derive a mobile Husky from the original URDF, keeping the exhibit intact."""
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as E
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from import_robots import origin, pose, resolve_uri, sub, transform, write_xml
from scipy.spatial.transform import Rotation
import numpy as np

OUT = ROOT / 'milestone' / 'generated'
OUT.mkdir(parents=True, exist_ok=True)

def plugin(parent, library, name):
    return sub(parent, 'plugin', filename=f'ignition-gazebo-{library}-system',
               name=f'ignition::gazebo::systems::{name}')

def world_system(world, library, name):
    """Reuse a shared world's system; never start two sensor render threads."""
    aliases = {f'ignition::gazebo::systems::{name}', f'gz::sim::systems::{name}'}
    matches = [p for p in world.findall('plugin') if p.get('name') in aliases]
    if not matches:
        return plugin(world, library, name)
    for duplicate in matches[1:]:
        world.remove(duplicate)
    return matches[0]

def prepare_world(world):
    """Configure the derived robotics world without changing the shared estate."""
    world.set('name', 'fleetscope_milestone')
    world.find('physics/max_step_size').text = '.002'
    sensors = world_system(world, 'sensors', 'Sensors')
    engine = sensors.find('render_engine')
    if engine is None:
        engine = sub(sensors, 'render_engine')
    engine.text = 'ogre2'
    world_system(world, 'imu', 'Imu')
    for include in world.findall('include'):
        uri = (include.findtext('uri') or '').rstrip('/')
        if (include.findtext('name') in ('husky_mobile_manipulator', 'fleetscope_player')
                or uri == 'model://fleetscope_player'):
            world.remove(include)
    for model in world.findall('model'):
        if model.get('name') == 'fleetscope_player':
            world.remove(model)

def main():
    # Use canonical joint references from upstream, never unlock the parked SDF.
    robot = E.parse(ROOT / 'models/fleetscope_husky_kinova/source_geometry.urdf').getroot()
    robot.set('name', 'husky_mobile_manipulator')
    q = json.loads((ROOT / 'models/fleetscope_husky_kinova/display_joint_positions.json').read_text())
    for mesh in robot.findall('.//mesh'):
        mesh.set('filename', str(resolve_uri(mesh.get('filename'))))
    for joint in robot.findall('joint'):
        if '_wheel_joint' in joint.get('name'):
            if joint.find('limit') is None:
                sub(joint, 'limit', effort='120', velocity='12')
            continue
        # Lock the arm in its deliberately chosen transport pose; no grasping yet.
        t = origin(joint.find('origin'))
        angle = q.get(joint.get('name'), 0.)
        if angle:
            motion = np.eye(4)
            motion[:3, :3] = Rotation.from_rotvec(np.array([float(v) for v in joint.find('axis').get('xyz').split()]) * angle).as_matrix()
            t = t @ motion
        for el in list(joint):
            if el.tag in ('origin', 'axis', 'limit', 'mimic', 'dynamics'):
                joint.remove(el)
        xyzrpy = pose(t).split()
        sub(joint, 'origin', xyz=' '.join(xyzrpy[:3]), rpy=' '.join(xyzrpy[3:]))
        joint.set('type', 'fixed')
    write_xml(robot, OUT / 'husky.urdf')
    converted = subprocess.run(['ign', 'sdf', '-p', str(OUT / 'husky.urdf')], capture_output=True, text=True, check=True)
    sdf = E.fromstring(converted.stdout)
    model = sdf.find('model')
    model.set('name', 'husky_mobile_manipulator')
    sub(model, 'static', 'false')
    base = model.find("link[@name='base_link']")
    assert base is not None and float(base.findtext('inertial/mass')) > 40
    # Contact friction permits skid steering; retain upstream masses and wheel size.
    for link in model.findall('link'):
        if '_wheel' in link.get('name'):
            for collision in link.findall('collision'):
                surface = sub(collision, 'surface'); ode = sub(sub(surface, 'friction'), 'ode')
                sub(ode, 'mu', '0.8'); sub(ode, 'mu2', '0.6')
                # WheelSlip's primary friction direction must be the axle.
                # The collision cylinder is rotated pi/2 around X: local Z
                # is parallel to the wheel joint's Y axis, and does not spin.
                sub(ode, 'fdir1', '0 0 1')
    slip = plugin(model, 'wheel-slip', 'WheelSlip')
    for end in ('front', 'rear'):
        for side in ('left', 'right'):
            wheel = sub(slip, 'wheel', link_name=f'{end}_{side}_wheel')
            sub(wheel, 'slip_compliance_lateral', .5)
            sub(wheel, 'slip_compliance_longitudinal', .01)
            sub(wheel, 'wheel_normal_force', 150)
            sub(wheel, 'wheel_radius', .1651)
    drive = plugin(model, 'diff-drive', 'DiffDrive')
    for side in ('left', 'right'):
        for end in ('front', 'rear'):
            sub(drive, f'{side}_joint', f'{end}_{side}_wheel_joint')
    for tag, value in {'wheel_separation': .555, 'wheel_radius': .1651,
                       'odom_publish_frequency': 30, 'topic': '/husky/drive',
                       'odom_topic': '/husky/encoder_odometry', 'tf_topic': '/unused/encoder_tf',
                       'frame_id': 'husky/odom', 'child_frame_id': 'husky/base_link',
                       'max_linear_velocity': .8, 'min_linear_velocity': -.5,
                       'max_angular_velocity': .7, 'min_angular_velocity': -.7,
                       'max_linear_acceleration': .5, 'min_linear_acceleration': -.5,
                       'max_angular_acceleration': .6, 'min_angular_acceleration': -.6}.items():
        sub(drive, tag, value)
    jointpub = plugin(model, 'joint-state-publisher', 'JointStatePublisher')
    sub(jointpub, 'topic', '/husky/joint_states')
    truth = plugin(model, 'pose-publisher', 'PosePublisher')
    for tag, value in {'publish_model_pose': 'true', 'publish_link_pose': 'false',
                       'publish_nested_model_pose': 'true', 'use_pose_vector_msg': 'true',
                       'update_frequency': 20}.items(): sub(truth, tag, value)
    imu = sub(base, 'sensor', name='imu', type='imu')
    sub(imu, 'topic', '/husky/imu'); sub(imu, 'update_rate', 100); sub(imu, 'always_on', 'true')
    im = sub(imu, 'imu')
    for kind, std in [('angular_velocity', .003), ('linear_acceleration', .03)]:
        section = sub(im, kind)
        for axis in 'xyz':
            noise = sub(sub(section, axis), 'noise', type='gaussian')
            sub(noise, 'mean', 0); sub(noise, 'stddev', std)
    lidar = sub(base, 'sensor', name='lidar', type='gpu_lidar')
    sub(lidar, 'pose', '.48 0 .42 0 0 0'); sub(lidar, 'topic', '/husky/scan')
    sub(lidar, 'update_rate', 8); sub(lidar, 'always_on', 'true')
    ray = sub(lidar, 'lidar'); horizontal = sub(sub(ray, 'scan'), 'horizontal')
    for tag, val in [('samples', 361), ('resolution', 1), ('min_angle', -math.pi * .75), ('max_angle', math.pi * .75)]: sub(horizontal, tag, val)
    ran = sub(ray, 'range')
    for tag, val in [('min', .18), ('max', 18), ('resolution', .01)]: sub(ran, tag, val)
    noise = sub(ray, 'noise'); sub(noise, 'type', 'gaussian'); sub(noise, 'mean', 0); sub(noise, 'stddev', .02)
    sensor = sub(base, 'sensor', name='front_camera', type='camera')
    sub(sensor, 'pose', '.49 0 .34 0 .10 0'); sub(sensor, 'topic', '/husky/camera')
    sub(sensor, 'update_rate', 8); sub(sensor, 'always_on', 'true')
    camera = sub(sensor, 'camera'); sub(camera, 'horizontal_fov', 1.15)
    image = sub(camera, 'image'); sub(image, 'width', 320); sub(image, 'height', 240); sub(image, 'format', 'R8G8B8')
    clip = sub(camera, 'clip'); sub(clip, 'near', .08); sub(clip, 'far', 100)
    noise = sub(camera, 'noise'); sub(noise, 'type', 'gaussian'); sub(noise, 'mean', 0); sub(noise, 'stddev', .003)
    for name, xyz, size, color in [('lidar_mount', '.48 0 .42', '.09 .09 .10', '.1 .15 .18 1'), ('camera_mount', '.47 0 .34', '.07 .14 .06', '.15 .18 .2 1')]:
        visual = sub(base, 'visual', name=name); sub(visual, 'pose', xyz + ' 0 0 0')
        sub(sub(sub(visual, 'geometry'), 'box'), 'size', size)
        mat = sub(visual, 'material'); sub(mat, 'diffuse', color); sub(mat, 'ambient', color)
    write_xml(sdf, OUT / 'husky.sdf')
    worldroot = E.parse(ROOT / 'worlds/plantation.sdf').getroot()
    world = worldroot.find('world')
    prepare_world(world)
    spawned = copy.deepcopy(model)
    sub(spawned, 'pose', '-12 -33 .18 0 0 -1.5707963267948966')
    world.append(spawned)
    for el in world.findall('.//gui//service'):
        if el.text: el.text = el.text.replace('fleetscope_estate', 'fleetscope_milestone')
    for el in world.findall('.//gui//stats_topic') + world.findall('.//gui//topic'):
        if el.text: el.text = el.text.replace('fleetscope_estate', 'fleetscope_milestone')
    for el in world.findall('.//gui//start_paused'): el.text = 'false'
    write_xml(worldroot, OUT / 'world.sdf')
    block = E.Element('sdf', version='1.9'); m = sub(block, 'model', name='road_block')
    sub(m, 'static', 'true'); sub(m, 'pose', '-12 -41 .6 0 0 0'); link = sub(m, 'link', name='crate')
    for kind in ('visual', 'collision'):
        g = sub(link, kind, name='crate'); sub(sub(sub(g, 'geometry'), 'box'), 'size', '2 1.2 1.2')
        if kind == 'visual':
            mat = sub(g, 'material'); sub(mat, 'ambient', '.95 .45 .06 1'); sub(mat, 'diffuse', '.95 .45 .06 1')
    write_xml(block, OUT / 'road_block.sdf')
    config=yaml.safe_load((ROOT/'milestone/ekf.yaml').read_text())
    params=config['ekf_filter_node']['ros__parameters']
    # Conservative demonstration covariances, not hardware-calibrated values.
    diagonal=[.0001,.0001,.0001,.0001,.0001,.00005,.02,.02,.02,.003,.003,.003,.05,.05,.05]
    params['process_noise_covariance']=np.diag(diagonal).ravel().tolist()
    params['initial_estimate_covariance']=np.diag([.0001]*15).ravel().tolist()
    (OUT/'ekf.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    print('Built dynamic Husky, sensor world, and optional experiment obstacle.')

if __name__ == '__main__': main()
