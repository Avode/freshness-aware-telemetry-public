#!/usr/bin/env python3
"""Import pinned manufacturer geometry into controller-free, parked SDF models.

The original descriptions are kept separately. Parked SDF joint origins encode
the display pose; use the original descriptions when adding controllers later.
"""
from pathlib import Path
import copy
import json
import os
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
ASSETS = Path(os.environ.get('FLEETSCOPE_ASSETS', ROOT/'assets')).expanduser()
UP = ASSETS / 'upstream'
PACKAGES = {
    'go2_description': UP / 'unitree_ros/robots/go2_description',
    'kortex_description': UP / 'kortex/kortex_description',
    'clearpath_platform_description': UP / 'clearpath_common/clearpath_platform_description',
    'clearpath_mounts_description': UP / 'clearpath_common/clearpath_mounts_description',
}

def numbers(text, default='0 0 0'):
    return np.array([float(x) for x in (text or default).split()])

def transform(xyz=None, rpy=None):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('xyz', numbers(rpy)).as_matrix()
    t[:3, 3] = numbers(xyz)
    return t

def origin(el):
    return transform(el.get('xyz'), el.get('rpy')) if el is not None else np.eye(4)

def pose(t):
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        return ' '.join(f'{x:.10g}' for x in [*t[:3, 3], *Rotation.from_matrix(t[:3, :3]).as_euler('xyz')])

def sub(el, tag, text=None, **attrs):
    child = ET.SubElement(el, tag, attrs)
    if text is not None:
        child.text = str(text)
    return child

def write_xml(el, path):
    ET.indent(el)
    ET.ElementTree(el).write(path, encoding='utf-8', xml_declaration=True)

def model_dir(name):
    d = ASSETS / 'models' / name
    d.mkdir(parents=True, exist_ok=True)
    target = ROOT / 'models' / name
    if not target.exists():
        target.symlink_to(d, target_is_directory=True)
    cfg = ET.Element('model')
    sub(cfg, 'name', name)
    sub(cfg, 'version', '1.0')
    sub(cfg, 'sdf', 'model.sdf', version='1.9')
    sub(cfg, 'description', 'Manufacturer geometry, parked scene import. No controllers or telemetry.')
    write_xml(cfg, d / 'model.config')
    return d

def resolve_uri(uri):
    if uri.startswith('package://'):
        package, tail = uri[10:].split('/', 1)
        return PACKAGES[package] / tail
    return Path(uri)

def imported_uri(uri, directory):
    source = resolve_uri(uri)
    if not source.exists():
        raise FileNotFoundError(source)
    # Keep the complete package subtree so DAE relative texture paths survive.
    if uri.startswith('package://'):
        package, tail = uri[10:].split('/', 1)
        link = directory / package
        if not link.exists():
            link.symlink_to(PACKAGES[package], target_is_directory=True)
        return f'model://{directory.name}/{package}/{tail}'
    return str(source)

def urdf_model(urdf, name, joints=None):
    directory = model_dir(name)
    robot = ET.parse(urdf).getroot()
    original = copy.deepcopy(robot)
    for el in list(original):
        if el.tag not in ('link', 'joint', 'material'):
            original.remove(el)
    write_xml(original, directory / 'source_geometry.urdf')
    links = {el.get('name'): el for el in robot.findall('link')}
    joint_list = robot.findall('joint')
    children = {el.find('child').get('link') for el in joint_list}
    transforms = {n: np.eye(4) for n in links if n not in children}
    pending = list(joint_list)
    joints = joints or {}
    while pending:
        progress = False
        for joint in pending[:]:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in transforms:
                continue
            t = origin(joint.find('origin'))
            q = joints.get(joint.get('name'), 0.0)
            axis = joint.find('axis')
            a = numbers(axis.get('xyz') if axis is not None else '1 0 0')
            motion = np.eye(4)
            if joint.get('type') in ('revolute', 'continuous'):
                motion[:3, :3] = Rotation.from_rotvec(a * q).as_matrix()
            elif joint.get('type') == 'prismatic':
                motion[:3, 3] = a * q
            transforms[child] = transforms[parent] @ t @ motion
            pending.remove(joint)
            progress = True
        if not progress:
            raise ValueError('Disconnected or cyclic URDF')
    sdf = ET.Element('sdf', version='1.9')
    model = sub(sdf, 'model', name=name)
    sub(model, 'static', 'true')
    for n, link in links.items():
        out = sub(model, 'link', name=n)
        sub(out, 'pose', pose(transforms[n]))
        inertial = link.find('inertial')
        if inertial is not None:
            ine = sub(out, 'inertial')
            sub(ine, 'pose', pose(origin(inertial.find('origin'))))
            mass = float(inertial.find('mass').get('value'))
            # Upstream Go2 IMU/radar marker frames have zero mass. Give only
            # those markers tiny positive inertias to satisfy DART when parked.
            sub(ine, 'mass', max(mass, 1e-6))
            tensor = sub(ine, 'inertia')
            for key, value in inertial.find('inertia').attrib.items():
                sub(tensor, key, 1e-9 if mass <= 0 and key in ('ixx','iyy','izz') else value)
        for tag in ('visual', 'collision'):
            for i, geom in enumerate(link.findall(tag)):
                g = sub(out, tag, name=f'{n}_{tag}_{i}')
                sub(g, 'pose', pose(origin(geom.find('origin'))))
                geo = sub(g, 'geometry')
                shape = geom.find('geometry')[0]
                shape_out = sub(geo, shape.tag)
                if shape.tag == 'mesh':
                    sub(shape_out, 'uri', imported_uri(shape.get('filename'), directory))
                    sub(shape_out, 'scale', shape.get('scale', '1 1 1'))
                else:
                    for key, value in shape.attrib.items():
                        sub(shape_out, key, value)
                color = geom.find('material/color')
                # Preserve material groups authored inside Collada files.
                if tag == 'visual' and color is not None and not (shape.tag == 'mesh' and shape.get('filename').lower().endswith('.dae')):
                    mat = sub(g, 'material')
                    sub(mat, 'ambient', color.get('rgba'))
                    sub(mat, 'diffuse', color.get('rgba'))
    for j in joint_list:
        out = sub(model, 'joint', name=j.get('name'), type='revolute' if j.get('type') == 'continuous' else j.get('type'))
        parent, child = j.find('parent').get('link'), j.find('child').get('link')
        sub(out, 'parent', parent)
        sub(out, 'child', child)
        sub(out, 'pose', '0 0 0 0 0 0', relative_to=child)
        if j.get('type') != 'fixed':
            ax = sub(out, 'axis')
            sub(ax, 'xyz', j.find('axis').get('xyz') if j.find('axis') is not None else '1 0 0')
            limit = j.find('limit')
            if limit is not None:
                lim = sub(ax, 'limit')
                for key in ('lower', 'upper', 'effort', 'velocity'):
                    if key in limit.attrib:
                        val = float(limit.get(key))
                        if key in ('lower', 'upper'):
                            val -= joints.get(j.get('name'), 0)
                        sub(lim, key, val)
    write_xml(sdf, directory / 'model.sdf')
    (directory / 'display_joint_positions.json').write_text(json.dumps(joints, indent=2)+'\n')
    return {'name': name, 'links': len(links), 'joints': len(joint_list), 'static': True}

def husky_description():
    import xacro
    # A local, generated copy resolves package includes without installing ROS packages.
    tmp = ASSETS / 'derived_xacro'
    tmp.mkdir(exist_ok=True)
    for package, package_dir in PACKAGES.items():
        if not package.startswith('clearpath'):
            continue
        for f in package_dir.rglob('*.xacro'):
            dest = tmp / package / f.relative_to(package_dir)
            dest.parent.mkdir(parents=True, exist_ok=True)
            text = f.read_text()
            text = re.sub(r'\$\(find ([^)]+)\)', lambda m: str(tmp / m.group(1)), text)
            dest.write_text(text)
    p = tmp / 'clearpath_platform_description/urdf/a200'
    wrapper = f'''<robot name="husky_kinova" xmlns:xacro="http://ros.org/wiki/xacro">
      <xacro:include filename="{p}/a200.urdf.xacro"/>
      <xacro:include filename="{p}/attachments/top_plate.urdf.xacro"/>
      <xacro:include filename="{p}/attachments/bumper.urdf.xacro"/>
      <xacro:a200/>
      <xacro:top_plate name="top_plate"><origin xyz="0 0 0"/></xacro:top_plate>
      <xacro:bumper name="front_bumper" parent_link="front_bumper_mount"><origin xyz="0 0 0"/></xacro:bumper>
      <xacro:bumper name="rear_bumper" parent_link="rear_bumper_mount"><origin xyz="0 0 0"/></xacro:bumper>
    </robot>'''
    wf = tmp / 'husky.xacro'
    wf.write_text(wrapper)
    doc = xacro.process_file(str(wf), mappings={'use_platform_controllers': 'false', 'is_sim': 'false'})
    base = ET.fromstring(doc.toxml())
    for item in list(base):
        if item.tag not in ('link', 'joint', 'material'):
            base.remove(item)
    arm = ET.parse(PACKAGES['kortex_description'] / 'robots/gen3_lite.urdf').getroot()
    for item in arm:
        if item.tag not in ('link', 'joint') or item.get('name') in ('world', 'base_joint'):
            continue
        item = copy.deepcopy(item)
        item.set('name', 'kinova_' + item.get('name'))
        for ref in item.findall('parent') + item.findall('child'):
            ref.set('link', 'kinova_' + ref.get('link'))
        mimic = item.find('mimic')
        if mimic is not None:
            mimic.set('joint', 'kinova_' + mimic.get('joint'))
        base.append(item)
    joint = sub(base, 'joint', name='kinova_mount_joint', type='fixed')
    sub(joint, 'parent', link='top_plate_default_mount')
    sub(joint, 'child', link='kinova_base_link')
    sub(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
    dest = tmp / 'husky_kinova.urdf'
    write_xml(base, dest)
    return dest

def x500_model():
    directory = model_dir('fleetscope_x500')
    source = UP / 'px4_models/models/x500_base'
    for folder in ('meshes', 'materials'):
        link = directory / folder
        if not link.exists():
            link.symlink_to(source / folder, target_is_directory=True)
    root = ET.parse(source / 'model.sdf').getroot()
    model = root.find('model')
    model.set('name', directory.name)
    model.find('static').text = 'true'
    if model.find('pose') is not None:
        model.remove(model.find('pose'))
    for parent in root.iter():
        for child in list(parent):
            if child.tag in ('sensor', 'plugin'):
                parent.remove(child)
    for el in root.iter():
        if el.text and 'model://x500_base/' in el.text:
            el.text = el.text.replace('model://x500_base/', 'model://fleetscope_x500/')
    write_xml(root, directory / 'model.sdf')
    return {'name': directory.name, 'links': len(model.findall('link')), 'joints': len(model.findall('joint')), 'static': True}

def main():
    specs = {}
    for repo in UP.iterdir():
        if not (repo / '.git').exists():
            continue
        def git(*args):
            return subprocess.check_output(['git', '-C', str(repo), *args], text=True).strip()
        specs[repo.name] = {'url': git('remote', 'get-url', 'origin'), 'commit': git('rev-parse', 'HEAD')}
    (ROOT / 'config/robot_sources.json').write_text(json.dumps(specs, indent=2)+'\n')
    results = []
    stance = {f'{leg}_{part}_joint': value for leg in ('FL', 'FR', 'RL', 'RR') for part, value in [('hip', 0), ('thigh', .72), ('calf', -1.44)]}
    results.append(urdf_model(PACKAGES['go2_description'] / 'urdf/go2_description.urdf', 'fleetscope_go2', stance))
    results.append(urdf_model(husky_description(), 'fleetscope_husky_kinova', {'kinova_joint_2': -.45, 'kinova_joint_3': .9, 'kinova_joint_5': -.45}))
    results.append(x500_model())
    (ROOT / 'config/robot_imports.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results, indent=2))

if __name__ == '__main__':
    main()
