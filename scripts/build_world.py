#!/usr/bin/env python3
"""Deterministic plantation authoring: shared scene manifest and Gazebo SDF."""
from pathlib import Path
import json
import math
import random
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SCENE = []
LABELS = []
PALETTE = {
    'grass': [.24,.36,.16,1], 'field': [.34,.43,.21,1],
    'soil': [.40,.30,.18,1], 'road': [.16,.19,.20,1],
    'concrete': [.63,.65,.60,1], 'cream': [.80,.78,.66,1],
    'roof': [.16,.26,.27,1], 'teal': [.12,.38,.36,1],
    'glass': [.19,.35,.40,1], 'white': [.90,.90,.81,1],
    'yellow': [.95,.64,.13,1], 'wood': [.40,.24,.11,1],
    'metal': [.25,.29,.30,1], 'water': [.12,.39,.43,1],
    'leaf': [.17,.36,.12,1], 'leaf_light': [.29,.46,.12,1],
}

def shape(kind, pos, size, color, rotation=(0,0,0), collision=True, **kwargs):
    return dict(kind=kind, pos=list(pos), size=list(size), color=PALETTE[color] if isinstance(color,str) else color,
                rotation=list(rotation), collision=collision, **kwargs)

def box(pos,size,color,rotation=(0,0,0),collision=True):
    return shape('box',pos,size,color,rotation,collision)

def cylinder(pos,radius,length,color,rotation=(0,0,0),collision=True):
    return shape('cylinder',pos,(radius,length),color,rotation,collision)

def entity(name, shapes, pos=(0,0,0), yaw=0):
    SCENE.append(dict(name=name, pos=list(pos), yaw=yaw, shapes=shapes))

def label(text,pos,size=1.0,rotation=(math.pi/2,0,0)):
    LABELS.append(dict(text=text,pos=list(pos),size=size,rotation=list(rotation)))

def building(name,x,y,w,d,h,open_front=False,solar=False):
    a=[box((0,0,.05),(w+.8,d+.8,.1),'concrete')]
    if open_front:
        a += [box((0,d/2,h/2),(w,.25,h),'cream'),
              box((-w/2,0,h/2),(.25,d,h),'cream'),
              box((w/2,0,h/2),(.25,d,h),'cream')]
        for px in (-w/2,0,w/2):
            a.append(box((px,-d/2,h/2),(.26,.26,h),'teal'))
    else:
        a += [box((0,0,h/2),(w,d,h),'cream')]
        for px in range(-int(w/2)+2,int(w/2)-1,3):
            a.append(box((px,-d/2-.035,h*.58),(1.7,.08,1.4),'glass',collision=False))
            a.append(box((px,d/2+.035,h*.58),(1.7,.08,1.4),'glass',collision=False))
        a.append(box((0,-d/2-.065,1.3),(1.6,.1,2.6),'teal'))
    # Two pitched roof panels, with shallow eaves and raised seam strips.
    angle=.18
    for sign in (-1,1):
        a.append(box((0,sign*d/4,h+.28),(w+1,d/2+.8,.16),'roof',(sign*-angle,0,0)))
    for px in range(-int(w/2),int(w/2)+1,2):
        for sign in (-1,1):
            a.append(box((px,sign*d/4,h+.38),(.045,d/2+.8,.05),'metal',(sign*-angle,0,0),False))
    a.append(box((0,-d/2-.14,h-.55),(w,.16,.65),'teal',collision=False))
    if solar:
        for px in (-w*.26,0,w*.26):
            a.append(box((px,-d*.25,h+.52),(w*.21,d*.32,.06),'glass',(.18,0,0),False))
    entity(name,a,(x,y,0))
    label(name.replace('_',' ').upper(),(x,y-d/2-.245,h-.75),.57)

def shelter(name,x,y):
    a=[box((0,0,.04),(7,5,.08),'concrete')]
    for px in (-3,3):
        for py in (-2,2):
            a.append(box((px,py,1.55),(.18,.18,3.1),'wood'))
    a.append(box((0,0,3.2),(7.2,5.2,.2),'teal'))
    a += [box((0,.8,.85),(3.6,1.2,.12),'wood')]
    for px in (-1.5,1.5):
        a.append(box((px,.8,.4),(.12,1,.8),'metal'))
    for px in (-1,0,1):
        a.append(box((px,.8,1.12),(.7,.7,.45),'yellow'))
    entity(name,a,(x,y,0))
    label(name.replace('_',' ').upper(),(x,y-2.63,2.75),.4)

def palm_meshes():
    """Two reusable low-poly meshes; curved feather fronds and tapered ring trunk."""
    directory=ROOT/'models/estate_palm/meshes'
    directory.mkdir(parents=True,exist_ok=True)
    def save(name,vertices,faces):
        path=directory/(name+'.obj')
        with path.open('w') as f:
            f.write('# Original FleetScope procedural palm, metres, Z up\n')
            f.write(f'mtllib {name}.mtl\no {name}\nusemtl surface\n')
            for v in vertices:f.write('v '+' '.join(f'{x:.5f}' for x in v)+'\n')
            for face in faces:
                a,b,c=[vertices[i] for i in face]
                u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
                n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
                length=math.sqrt(sum(x*x for x in n)) or 1
                f.write('vn '+' '.join(f'{x/length:.7f}' for x in n)+'\n')
            for j,face in enumerate(faces):f.write('f '+' '.join(f'{i+1}//{j+1}' for i in face)+'\n')
        rgb=PALETTE['wood' if name=='trunk' else 'leaf'][:3]
        (directory/(name+'.mtl')).write_text('newmtl surface\nKa '+seq(rgb)+'\nKd '+seq(rgb)+'\nKs 0.03 0.03 0.03\nNs 4\nd 1\nillum 2\n')
        return f'model://estate_palm/meshes/{name}.obj'
    verts=[];faces=[]
    for j in range(19):
        z=j*.25;radius=.26-.08*j/18+(.025 if j%2==0 else 0)
        for i in range(10):
            theta=i*math.tau/10
            verts.append((radius*math.cos(theta)+.10*(z/4.5)**2,radius*math.sin(theta),z))
    for j in range(18):
        for i in range(10):
            a=j*10+i;b=j*10+(i+1)%10;c=b+10;d=a+10
            faces.extend([(a,b,c),(a,c,d)])
    trunk=save('trunk',verts,faces)
    verts=[];faces=[]
    def triangle(a,b,c):
        idx=len(verts);verts.extend((a,b,c));faces.append((idx,idx+1,idx+2))
    for k in range(11):
        angle=k*math.tau/11;length=3.3+(k%3)*.22
        def point(t,side=0):
            r=length*t;z=4.6+1.6*t-2.2*t*t
            return (.1+r*math.cos(angle)-side*math.sin(angle),r*math.sin(angle)+side*math.cos(angle),z)
        for j in range(1,13):
            t=j/14;spread=.62*math.sin(math.pi*t)**.6
            for s in (-1,1):
                a=point(t-.08);b=point(t+.025)
                tip=list(point(t+.17,s*spread));tip[2]-=.18
                triangle(a,b,tip)
        for j in range(12):
            t=j/12
            triangle(point(t,.035),point(t,-.035),point((j+1)/12))
    leaves=save('fronds',verts,faces)
    return trunk,leaves

def build_scene():
    rng=random.Random(2026)
    trunk,leaves=palm_meshes()
    entity('estate_ground',[box((0,0,-.3),(200,190,.6),'grass')])
    entity('campus_apron',[box((0,-43,.015),(158,48,.03),'concrete')])
    road=[]
    road += [box((0,-3,.035),(7,170,.05),'road'),box((0,-18,.036),(178,7,.05),'road'),
             box((0,-64,.036),(156,6,.05),'road'),box((0,37.5,.025),(145,4,.03),'soil')]
    for y in range(-81,79,7):
        if -23<y<-13 or -68<y<-60:continue
        road.append(box((0,y,.065),(.15,3,.009),'yellow',collision=False))
    for x in range(-82,85,7):
        if abs(x)<7:continue
        road.append(box((x,-18,.07),(3,.15,.009),'white',collision=False))
    entity('service_roads',road)
    # A planted grid with open rows and service tracks.
    count=0
    for bx in (-44,44):
        for by in (20,55):
            entity(f'plot_{bx}_{by}',[box((0,0,.008),(39,32,.016),'field')],(bx,by,0))
            for row in range(4):
                py=by+(row-1.5)*9
                entity(f'irrigation_{bx}_{by}_{row}',[cylinder((0,0,.04),.035,35,'metal',(0,math.pi/2,0))],(bx,py,0))
                for col in range(4):
                    x=bx+(col-1.5)*9;y=py
                    scale=rng.uniform(.90,1.12)
                    a=[shape('mesh',(0,0,0),(scale,)*3,'wood',uri=trunk,collision=False),
                       shape('mesh',(0,0,0),(scale,)*3,'leaf' if count%3 else 'leaf_light',uri=leaves,collision=False),
                       cylinder((0,0,2.25*scale),.26*scale,4.5*scale,'wood')]
                    # The primitive trunk provides contact geometry, hidden inside the mesh.
                    a[-1]['hidden']=True
                    entity(f'palm_{count:03}',a,(x,y,0),rng.uniform(0,math.tau));count+=1
    # Reservoir, pump and an explicit shallow drainage channel.
    entity('reservoir',[box((0,0,.02),(12,24,.04),'water'),
                        box((-6.3,0,.25),(.6,25,.5),'concrete'),box((6.3,0,.25),(.6,25,.5),'concrete'),
                        box((0,-12.3,.25),(13.2,.6,.5),'concrete'),box((0,12.3,.25),(13.2,.6,.5),'concrete')],(-79,53,0))
    entity('drainage_channel',[box((0,0,.025),(1.8,154,.05),'water'),box((-1.1,0,.15),(.4,154,.3),'concrete'),box((1.1,0,.15),(.4,154,.3),'concrete')],(79,5,0))
    entity('culvert_crossing',[box((0,0,.1),(5,8,.2),'concrete')],(79,-18,0))
    building('maintenance_depot',-32,-49,22,15,4.2,True,True)
    building('logistics_warehouse',36,-49,28,17,5.3,True,True)
    building('field_office',-66,-29,18,10,3.8,False,True)
    building('pump_house',-77,32,8,7,3.0)
    building('utility_store',64,-29,12,10,3.5)
    building('gatehouse',-10,-78,6,5,3.0)
    shelter('collection_west',-18,7)
    shelter('collection_east',18,57)
    # Warehouse racks and lightweight containers; no pick/place behaviour yet.
    for x in (27,36,45):
        a=[]
        for px in (-2,2):
            for py in (-.8,.8):a.append(box((px,py,1.5),(.12,.12,3),'teal'))
        for z in (.4,1.5,2.6):
            a.append(box((0,0,z),(4.3,1.9,.1),'metal'))
            for px in (-1.4,0,1.4):a.append(box((px,0,z+.4),(1.1,1.2,.7),'wood'))
        entity(f'warehouse_rack_{x}',a,(x,-43,0))
    for x,y in [(-46,-45),(-46,-49),(54,-51),(54,-55)]:
        entity(f'pallet_{x}_{y}',[box((0,0,.08),(1.2,1,.16),'wood'),box((0,0,.53),(1.05,.9,.74),'yellow')],(x,y,0))
    # Parked robots grouped in a clean forecourt immediately outside the depot.
    for x,name in [(-17,'GO2'),(-12,'HUSKY')]:
        a=[box((0,0,.021),(3.4,5,.025),'teal')]
        for px in (-1.6,1.6):a.append(box((px,0,.044),(.08,4.8,.012),'white',collision=False))
        a.append(box((0,2.15,.48),(.6,.35,.95),'metal'))
        a.append(box((0,1.96,.62),(.38,.04,.35),'glass',collision=False))
        entity(name.lower()+'_parking_bay',a,(x,-33,0))
        label(name,(x,-35.3,.057),.48,(0,0,0))
    pad=[cylinder((0,0,.025),4.2,.05,'teal')]
    for i in range(40):
        t=i*math.tau/40
        pad.append(box((3.75*math.cos(t),3.75*math.sin(t),.058),(.55,.12,.01),'white',(0,0,t+math.pi/2),False))
    pad.extend([box((-.8,0,.06),(.25,2.5,.01),'white',collision=False),box((.8,0,.06),(.25,2.5,.01),'white',collision=False),box((0,0,.06),(1.35,.25,.01),'white',collision=False)])
    entity('drone_landing_pad',pad,(9,-33,0))
    # Perimeter fence, gate, lamps and small roadside details.
    fence=[]
    for y in (-86,84):
        for x in range(-92,93,6):
            if y==-86 and abs(x)<7:continue
            fence.append(box((x,y,1),(.14,.14,2),'wood'))
        for x in (-50,50):
            for z in (.65,1.45):fence.append(box((x,y,z),(84,.08,.1),'wood'))
    for x in (-94,94):
        for y in range(-84,85,6):fence.append(box((x,y,1),(.14,.14,2),'wood'))
        for z in (.65,1.45):fence.append(box((x,0,z),(.08,168,.1),'wood'))
    entity('estate_fence',fence)
    for x in (-5,5):
        entity(f'entrance_pillar_{x}',[box((0,0,1.6),(.7,.7,3.2),'cream')],(x,-83,0))
    label('FLEETSCOPE  /  ESTATE 01',(0,-83.4,3.3),.48)
    for x in (-72,-52,-6,18,56,72):
        a=[cylinder((0,0,2.6),.09,5.2,'metal'),box((.6,0,5.1),(1.3,.15,.12),'metal'),box((1.1,0,5.03),(.5,.35,.08),'white',collision=False)]
        entity(f'road_light_{x}',a,(x,-23,0))
    for x,y in [(-8,-70),(8,-70),(-7,-12),(7,-12),(58,-60)]:
        entity(f'bollard_{x}_{y}',[cylinder((0,0,.4),.14,.8,'yellow')],(x,y,0))
    entity('water_tanks',[cylinder((0,0,1.6),1.7,3.2,'teal'),cylinder((4,0,1.6),1.7,3.2,'teal')],(62,-43,0))
    for x,y in [(-85,-50),(-84,-10),(84,-49),(85,14)]:
        a=[shape('sphere',(0,0,.6),(.65,),[.35,.37,.29,1])]
        entity(f'landscape_rock_{x}_{y}',a,(x,y,0))
    return count

def sub(el,tag,text=None,**attrs):
    c=ET.SubElement(el,tag,attrs)
    if text is not None:c.text=str(text)
    return c

def seq(a):return ' '.join(f'{x:.7g}' for x in a)

def geometry(parent,s):
    g=sub(parent,'geometry');el=sub(g,s['kind'])
    if s['kind']=='box':sub(el,'size',seq(s['size']))
    elif s['kind']=='cylinder':sub(el,'radius',s['size'][0]);sub(el,'length',s['size'][1])
    elif s['kind']=='sphere':sub(el,'radius',s['size'][0])
    elif s['kind']=='mesh':sub(el,'uri',s['uri']);sub(el,'scale',seq(s['size']))

def main():
    count=build_scene()
    robots=[dict(name='go2_inspector',model='fleetscope_go2',pose=[-17,-33,.342,0,0,-math.pi/2]),
            dict(name='husky_mobile_manipulator',model='fleetscope_husky_kinova',pose=[-12,-33,.18,0,0,-math.pi/2]),
            dict(name='x500_survey_drone',model='fleetscope_x500',pose=[9,-33,.295,0,0,0])]
    manifest=dict(name='FleetScope Plantation Estate',units='metres',seed=2026,extent=[200,190],
                  palm_count=count,entities=SCENE,labels=LABELS,robots=robots)
    (ROOT/'config/scene.json').write_text(json.dumps(manifest,indent=2)+'\n')
    root=ET.Element('sdf',version='1.9');world=sub(root,'world',name='fleetscope_estate')
    sub(world,'gravity','0 0 -9.81')
    physics=sub(world,'physics',name='estate_physics',type='ignored')
    sub(physics,'max_step_size','.004');sub(physics,'real_time_factor','1')
    for lib,cls in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster')]:
        sub(world,'plugin',filename=f'libignition-gazebo-{lib}-system.so',name=f'ignition::gazebo::systems::{cls}')
    sensors=sub(world,'plugin',filename='libignition-gazebo-sensors-system.so',
                name='ignition::gazebo::systems::Sensors')
    sub(sensors,'render_engine','ogre2')
    scene=sub(world,'scene');sub(scene,'ambient','.65 .65 .65 1');sub(scene,'background','.70 .82 .89 1');sub(scene,'shadows','true')
    sun=sub(world,'light',name='estate_sun',type='directional')
    sub(sun,'pose','0 0 100 0 0 0');sub(sun,'diffuse','.95 .91 .81 1');sub(sun,'specular','.2 .2 .2 1');sub(sun,'direction','-.4 .3 -1');sub(sun,'cast_shadows','true')
    gui=sub(world,'gui',fullscreen='false')
    for filename,title in [('MinimalScene','3D View'),('GzSceneManager','Scene Manager'),('InteractiveViewControl','View control'),('CameraTracking','Camera tracking'),('SelectEntities','Select Entities'),('EntityContextMenuPlugin','Entity context menu'),('EntityTree','Entity tree'),('TransformControl','Transform control'),('WorldControl','World control'),('WorldStats','World statistics')]:
        pl=sub(gui,'plugin',filename=filename,name=title)
        ui=sub(pl,'ignition-gui')
        sub(ui,'title',title)
        sub(ui,'property','false',type='bool',key='showTitleBar')
        floating=filename not in ('MinimalScene','EntityTree')
        sub(ui,'property','floating' if floating else 'docked',type='string',key='state')
        if floating:
            sub(ui,'property','false',type='bool',key='resizable')
            sub(ui,'property',250 if filename=='TransformControl' else 290 if filename=='WorldStats' else 121 if filename=='WorldControl' else 5,type='double',key='width')
            sub(ui,'property',50 if filename=='TransformControl' else 72 if filename=='WorldControl' else 110 if filename=='WorldStats' else 5,type='double',key='height')
        if filename in ('WorldControl','WorldStats'):
            anchors=sub(ui,'anchors',target='3D View')
            side='left' if filename=='WorldControl' else 'right'
            sub(anchors,'line',own=side,target=side);sub(anchors,'line',own='bottom',target='bottom')
        if filename=='MinimalScene':
            sub(pl,'engine','ogre2');sub(pl,'scene','scene');sub(pl,'ambient_light','.65 .65 .65');sub(pl,'background_color','.70 .82 .89');sub(pl,'camera_pose','85 -110 125 0 .88354151 2.32172539')
            clip=sub(pl,'camera_clip');sub(clip,'near','.05');sub(clip,'far','800')
        if filename=='TransformControl':sub(pl,'legacy','false')
        if filename=='WorldControl':
            sub(pl,'play_pause','true');sub(pl,'step','true');sub(pl,'start_paused','true');sub(pl,'service','/world/fleetscope_estate/control');sub(pl,'stats_topic','/world/fleetscope_estate/stats')
        if filename=='WorldStats':
            for key in ('sim_time','real_time','real_time_factor','iterations'):sub(pl,key,'true')
            sub(pl,'topic','/world/fleetscope_estate/stats')
    for e in SCENE:
        m=sub(world,'model',name=e['name']);sub(m,'static','true');sub(m,'pose',seq([*e['pos'],0,0,e['yaw']]))
        link=sub(m,'link',name='structure')
        for i,s in enumerate(e['shapes']):
            if not s.get('hidden'):
                v=sub(link,'visual',name=f'visual_{i}');sub(v,'pose',seq(s['pos']+s['rotation']));geometry(v,s)
                mat=sub(v,'material');sub(mat,'ambient',seq(s['color']));sub(mat,'diffuse',seq(s['color']));sub(mat,'specular','.08 .08 .08 1')
                if s['kind']=='box' and s['size'][2]<.12:sub(v,'cast_shadows','false')
            if s['collision']:
                c=sub(link,'collision',name=f'collision_{i}');sub(c,'pose',seq(s['pos']+s['rotation']));geometry(c,s)
    for robot in robots:
        inc=sub(world,'include');sub(inc,'uri','model://'+robot['model']);sub(inc,'name',robot['name']);sub(inc,'pose',seq(robot['pose']));sub(inc,'static','true')
    player=sub(world,'include')
    sub(player,'uri','model://fleetscope_player')
    sub(player,'name','fleetscope_player')
    sub(player,'pose','0 -43 0.75 0 0 0')
    if (ROOT/'models/estate_signage/meshes/signage.obj').exists():
        m=sub(world,'model',name='estate_signage');sub(m,'static','true')
        link=sub(m,'link',name='signs');v=sub(link,'visual',name='lettering')
        geo=sub(v,'geometry');mesh=sub(geo,'mesh');sub(mesh,'uri','model://estate_signage/meshes/signage.obj')
        sub(v,'cast_shadows','false')
    ET.indent(root)
    ET.ElementTree(root).write(ROOT/'worlds/plantation.sdf',encoding='utf-8',xml_declaration=True)
    print(f'Built {len(SCENE)} estate models, {count} palms, {len(robots)} imported robots.')

if __name__=='__main__':main()
