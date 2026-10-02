"""Render the surveyed estate and telemetry-positioned robot in RViz."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parents[1]

def matrix(pose):
    t=np.eye(4); t[:3,:3]=Rotation.from_euler('xyz',pose[3:]).as_matrix(); t[:3,3]=pose[:3]; return t

def xmlpose(el):
    return matrix([float(v) for v in el.text.split()]) if el is not None else np.eye(4)

def marker(idx, ns, typ, transform=None, scale=(1.,1.,1.), color=(1.,1.,1.,1.)):
    m=Marker(); m.header.frame_id='estate'; m.ns=ns; m.id=idx; m.type=typ; m.action=Marker.ADD
    t=transform if transform is not None else np.eye(4)
    m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,t[:3,3])
    q=Rotation.from_matrix(t[:3,:3]).as_quat()
    m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=map(float,q)
    m.scale.x,m.scale.y,m.scale.z=map(float,scale)
    m.color.r,m.color.g,m.color.b,m.color.a=map(float,color)
    return m

def shape(idx, ns, kind, size, t, color, uri=None):
    if kind=='mesh':
        m=marker(idx,ns,Marker.MESH_RESOURCE,t,size,color)
        path=ROOT/'models'/uri[8:] if uri.startswith('model://') else Path(uri)
        m.mesh_resource=path.resolve().as_uri(); m.mesh_use_embedded_materials=True
    else:
        typ={'box':Marker.CUBE,'plane':Marker.CUBE,'cylinder':Marker.CYLINDER,'sphere':Marker.SPHERE}[kind]
        dimensions=size
        if kind=='cylinder': dimensions=(size[0]*2,size[0]*2,size[1])
        elif kind=='sphere': dimensions=(size[0]*2,)*3
        elif kind=='plane': dimensions=(*size,.01)
        m=marker(idx,ns,typ,t,dimensions,color)
    return m

def estate():
    result=MarkerArray(); data=json.loads((ROOT/'config/scene.json').read_text())
    # Only surveyed static geometry, never live Gazebo entities or experiment props.
    for ent in data['entities']:
        parent=matrix([*ent['pos'],0,0,ent['yaw']])
        for s in ent['shapes']:
            if s.get('hidden'): continue
            result.markers.append(shape(len(result.markers),'surveyed_estate',s['kind'],s['size'],
                parent@matrix([*s['pos'],*s['rotation']]),s['color'],s.get('uri')))
    return result

def robot_template():
    model=E.parse(ROOT/'models/fleetscope_husky_kinova/model.sdf').getroot().find('model')
    result=[]
    for link in model.findall('link'):
        for v in link.findall('visual'):
            g=v.find('geometry')[0]; kind=g.tag; uri=None
            if kind=='mesh': size=[float(x) for x in g.findtext('scale','1 1 1').split()]; uri=g.findtext('uri')
            elif kind=='box': size=[float(x) for x in g.findtext('size').split()]
            elif kind=='cylinder': size=[float(g.findtext('radius')),float(g.findtext('length'))]
            elif kind=='sphere': size=[float(g.findtext('radius'))]
            else: continue
            color=[float(x) for x in v.findtext('material/diffuse','1 1 1 1').split()]
            result.append((kind,size,xmlpose(link.find('pose'))@xmlpose(v.find('pose')),color,uri))
    return result

def robot(template, pose):
    x,y,a=pose; parent=matrix([x,y,.166,0,0,a]); out=MarkerArray()
    for i,(kind,size,t,color,uri) in enumerate(template):
        out.markers.append(shape(i,'estimated_husky',kind,size,parent@t,color,uri))
    return out

def text_marker(text, pose, stale=False):
    t=matrix([pose[0],pose[1],2.7,0,0,0])
    m=marker(0,'telemetry_status',Marker.TEXT_VIEW_FACING,t,(1,1,.42),(1.,.35,.18,1.) if stale else (.12,1.,.8,1.))
    m.text=text; return m

def point_marker(points):
    m=marker(0,'lidar_returns',Marker.POINTS,scale=(.12,.12,.12),color=(1.,.3,.06,1.))
    m.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in points]
    return m

def uncertainty(state):
    cov=np.array(state['covariance_xy']).reshape(2,2)
    eig,vec=np.linalg.eigh(cov)
    m=marker(0,'position_uncertainty',Marker.LINE_STRIP,scale=(.045,1,1),color=(1.,.75,.1,1.))
    x,y,_=state['pose']
    for a in np.linspace(0,math.tau,65):
        p=vec@(2*np.sqrt(np.maximum(eig,0))*np.array([math.cos(a),math.sin(a)]))
        m.points.append(Point(x=float(x+p[0]),y=float(y+p[1]),z=.25))
    return m
