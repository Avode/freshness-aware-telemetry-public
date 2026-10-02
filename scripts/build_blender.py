"""Run with blender --background --python scripts/build_blender.py.

Build an editable .blend from the same manifest and parked SDF robot geometry.
No mission logic, robot control, or telemetry is created by this script.
"""
import bpy
import json
import os
import math
import hashlib
import subprocess
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from mathutils import Matrix, Vector, Euler

ROOT=Path(__file__).resolve().parents[1]
ASSETS=Path(os.environ.get('FLEETSCOPE_ASSETS', ROOT/'assets')).expanduser()
OUT=Path(os.environ.get('FLEETSCOPE_BLENDER_OUT', ROOT/'output/blender')).expanduser()
CACHE=ASSETS/'blender_meshes'
OUT.mkdir(parents=True,exist_ok=True);CACHE.mkdir(parents=True,exist_ok=True)
DATA=json.loads((ROOT/'config/scene.json').read_text())
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
for c in list(bpy.data.collections):
    if not c.objects:bpy.data.collections.remove(c)
MAT={};MESH={}

def material(color):
    key=tuple(round(float(v),4) for v in color)
    if key not in MAT:
        m=bpy.data.materials.new('Estate '+str(key));m.diffuse_color=key
        m.use_nodes=True
        p=m.node_tree.nodes.get('Principled BSDF')
        p.inputs['Base Color'].default_value=key;p.inputs['Roughness'].default_value=.76
        MAT[key]=m
    return MAT[key]

def collection(name):
    c=bpy.data.collections.new(name);bpy.context.scene.collection.children.link(c);return c

def matrix(pos=(0,0,0),rot=(0,0,0),scale=(1,1,1)):
    return Matrix.Translation(pos)@Euler(rot,'XYZ').to_matrix().to_4x4()@Matrix.Diagonal((*scale,1))

def vals(text,n=6):
    return [float(x) for x in text.split()] if text else [0.]*n

def sdf_pose(el):
    p=vals(el.text if el is not None else None)
    return matrix(p[:3],p[3:])

def relink(obj,col):
    for c in list(obj.users_collection):c.objects.unlink(obj)
    col.objects.link(obj)

def load_mesh(path):
    path=path.resolve()
    if str(path) in MESH:return MESH[str(path)]
    old=set(bpy.data.objects)
    ext=path.suffix.lower()
    if ext=='.dae':
        dest=CACHE/(hashlib.sha256(str(path).encode()).hexdigest()[:16]+'.glb')
        # Assimp keeps external texture URIs when writing GLB.
        for texture in path.parent.iterdir():
            if texture.suffix.lower() in ('.png','.jpg','.jpeg'):
                shutil.copy2(texture,CACHE/texture.name)
        if not dest.exists():subprocess.run([str(ROOT/'scripts/mesh_converter'),str(path),str(dest)],check=True)
        bpy.ops.import_scene.gltf(filepath=str(dest))
    elif ext=='.stl':bpy.ops.wm.stl_import(filepath=str(path))
    elif ext=='.obj':bpy.ops.wm.obj_import(filepath=str(path),forward_axis='Y',up_axis='Z')
    else:raise ValueError(path)
    bpy.context.view_layer.update()
    created=set(bpy.data.objects)-old
    parts=[(o.data,o.matrix_world.copy()) for o in created if o.type=='MESH']
    for o in created:bpy.data.objects.remove(o,do_unlink=True)
    MESH[str(path)]=parts
    return parts

def resolve(uri):
    if uri.startswith('model://'):return ROOT/'models'/uri[8:]
    return Path(uri)

def add_shape(name,kind,size,pose,col,color=None,uri=None):
    objects=[]
    if kind=='mesh':
        for i,(data,local) in enumerate(load_mesh(resolve(uri))):
            obj=bpy.data.objects.new(name+f'_{i}',data);col.objects.link(obj)
            obj.matrix_world=pose@Matrix.Diagonal((*size,1))@local
            objects.append(obj)
    else:
        if kind=='box':bpy.ops.mesh.primitive_cube_add(size=1);scale=size
        elif kind=='cylinder':bpy.ops.mesh.primitive_cylinder_add(vertices=16,radius=1,depth=1);scale=(size[0],size[0],size[1])
        elif kind=='sphere':bpy.ops.mesh.primitive_uv_sphere_add(segments=12,ring_count=6,radius=1);scale=(size[0],)*3
        elif kind=='plane':bpy.ops.mesh.primitive_plane_add(size=1);scale=(*size,1)
        else:raise ValueError(kind)
        obj=bpy.context.object;obj.name=name;relink(obj,col);obj.matrix_world=pose@Matrix.Diagonal((*scale,1));objects=[obj]
    if color is not None:
        mat=material(color)
        for o in objects:
            if not o.data.materials:o.data.materials.append(mat)
            for slot in o.material_slots:slot.link='OBJECT';slot.material=mat
    return objects

for ent in DATA['entities']:
    col=collection(ent['name'])
    parent=matrix(ent['pos'],(0,0,ent['yaw']))
    for i,s in enumerate(ent['shapes']):
        if s.get('hidden'):continue
        add_shape(ent['name']+f'_{i}',s['kind'],s['size'],parent@matrix(s['pos'],s['rotation']),col,s['color'],s.get('uri'))

for robot in DATA['robots']:
    col=collection('ROBOT | '+robot['name'])
    m=ET.parse(ROOT/'models'/robot['model']/'model.sdf').getroot().find('model')
    base=matrix(robot['pose'][:3],robot['pose'][3:])
    for link in m.findall('link'):
        lp=base@sdf_pose(link.find('pose'))
        for i,v in enumerate(link.findall('visual')):
            g=v.find('geometry')[0];kind=g.tag
            if kind=='mesh':size=vals(g.findtext('scale','1 1 1'));uri=g.findtext('uri')
            elif kind in ('box','plane'):size=vals(g.findtext('size'));uri=None
            elif kind=='cylinder':size=[float(g.findtext('radius')),float(g.findtext('length'))];uri=None
            elif kind=='sphere':size=[float(g.findtext('radius'))];uri=None
            else:continue
            color=v.findtext('material/diffuse')
            color=vals(color) if color else None
            if color and len(color)==3:color.append(1)
            add_shape(robot['name']+'/'+link.get('name')+f'/{i}',kind,size,lp@sdf_pose(v.find('pose')),col,color,uri)

signs=collection('Estate signage');text_objects=[]
for i,s in enumerate(DATA['labels']):
    font=bpy.data.curves.new('Sign','FONT');font.body=s['text'];font.align_x='CENTER';font.size=s['size'];font.extrude=.004
    obj=bpy.data.objects.new(f'Sign {i}',font);signs.objects.link(obj);obj.matrix_world=matrix(s['pos'],s['rotation']);obj.data.materials.append(material((.93,.93,.85,1)))
    text_objects.append(obj)
# Export the same lettering for Gazebo. Blender retains editable text objects.
bpy.ops.object.select_all(action='DESELECT')
for o in text_objects:o.select_set(True)
sign_dir=ROOT/'models/estate_signage/meshes';sign_dir.mkdir(parents=True,exist_ok=True)
bpy.ops.wm.obj_export(filepath=str(sign_dir/'signage.obj'),export_selected_objects=True,forward_axis='Y',up_axis='Z',export_materials=True)

scene=bpy.context.scene
scene.unit_settings.system='METRIC';scene.unit_settings.scale_length=1
scene.world.color=(.4,.5,.6)
scene.world.use_nodes=True
scene.world.node_tree.nodes['Background'].inputs['Color'].default_value=(.65,.76,.85,1)
scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value=.45
lighting=collection('Lighting and cameras')
sun=bpy.data.lights.new('Afternoon sun','SUN');sun.energy=2.5;sun.angle=.12
sun_obj=bpy.data.objects.new('Afternoon sun',sun);lighting.objects.link(sun_obj);sun_obj.rotation_euler=(.4,-.5,-.4)

def camera(name,loc,target,ortho):
    c=bpy.data.cameras.new(name);o=bpy.data.objects.new(name,c);lighting.objects.link(o)
    o.location=loc;o.rotation_euler=(Vector(target)-o.location).to_track_quat('-Z','Y').to_euler()
    c.type='ORTHO';c.ortho_scale=ortho;c.clip_end=1000
    return o

overview=camera('01 Estate overview',(150,-190,175),(0,0,0),235)
depot=camera('02 Robot forecourt',(-1,-55,15),(-13.8,-33,.4),13)
go2=camera('03 Go2 closeup',(-15.7,-34.7,1.1),(-17,-33,.28),1.5)
husky=camera('04 Husky and Kinova',(-10.2,-35,1.5),(-12,-33,.55),2.5)
drone=camera('05 X500 closeup',(10.1,-34,1),(9,-33,.24),1.5)
scene.camera=overview
scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=12
scene.cycles.use_denoising=True
scene.render.resolution_x=1400;scene.render.resolution_y=1100;scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG'
scene.view_settings.view_transform='AgX'
for area in bpy.context.screen.areas:
    if area.type=='VIEW_3D':
        area.spaces.active.region_3d.view_distance=170
        area.spaces.active.region_3d.view_location=Vector((0,0,0))
        area.spaces.active.clip_end=1500
bpy.ops.object.select_all(action='DESELECT')
blend=OUT/'plantation_estate.blend'
bpy.ops.file.pack_all()
bpy.ops.wm.save_as_mainfile(filepath=str(blend))
link=ROOT/'plantation_estate.blend'
if not link.exists():link.symlink_to(blend)
for name,cam in [('estate-overview',overview),('go2',go2),('husky-kinova',husky),('x500',drone)]:
    scene.camera=cam
    if cam!=overview:scene.render.resolution_x=1000;scene.render.resolution_y=750
    scene.render.filepath=str(ROOT/'previews'/f'{name}.png')
    bpy.ops.render.render(write_still=True)
print('SAVED',blend)
