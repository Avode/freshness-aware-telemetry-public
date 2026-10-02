"""Build navigation configuration and a declared surveyed site prior."""
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as E
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'autonomy/generated'

def prior():
    resolution=.2; width,height=1000,950
    grid=np.zeros((height,width),np.uint8)
    def polygon(points):
        pixels=np.round((np.asarray(points)+[100,95])/resolution).astype(np.int32)
        cv2.fillPoly(grid,[pixels],100)
    def circle(x,y,r):
        cv2.circle(grid,(round((x+100)/resolution),round((y+95)/resolution)),max(1,math.ceil(r/resolution)),100,-1)
    scene=json.loads((ROOT/'config/scene.json').read_text())
    for e in scene['entities']:
        a=e['yaw']; rot=Rotation.from_euler('z',a).as_matrix()
        for s in e['shapes']:
            if not s['collision']: continue
            c=np.asarray(e['pos'])+rot@np.asarray(s['pos'])
            r=rot@Rotation.from_euler('xyz',s['rotation']).as_matrix()
            if s['kind']=='box':
                size=np.array(s['size']); corners=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*size
                corners=c+corners@r.T
                # Exclude traversable ground, roofs, and overhead structures.
                if corners[:,2].max()<.18 or corners[:,2].min()>1.5: continue
                hull=cv2.convexHull(corners[:,:2].astype(np.float32)).reshape(-1,2)
                polygon(hull)
            elif s['kind']=='cylinder':
                radius,length=s['size']
                if abs(r[2,2])>.8 and c[2]+length/2>.18 and c[2]-length/2<1.5:
                    circle(c[0],c[1],radius)
    # Surveyed no-go water and reserved parking areas, independent of live poses.
    for x,y,w,h in [(-79,53,13,25),(79,34,2.6,96),(79,-47,2.6,50),(-17,-33,3.6,5.4)]:
        polygon([(x-w/2,y-h/2),(x+w/2,y-h/2),(x+w/2,y+h/2),(x-w/2,y+h/2)])
    circle(9,-33,4.3)
    grid[:10]=100;grid[-10:]=100;grid[:,:10]=100;grid[:,-10:]=100
    np.savez_compressed(OUT/'survey-prior.npz',data=grid,resolution=resolution,origin=[-100.,-95.])

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    subprocess.run([sys.executable,str(ROOT/'milestone/build.py')],check=True)
    prior()
    xml=E.parse(ROOT/'milestone/generated/world.sdf')
    world=xml.getroot().find('world'); world.set('name','fleetscope_autonomy')
    for el in world.findall('.//gui//service')+world.findall('.//gui//topic')+world.findall('.//gui//stats_topic'):
        if el.text: el.text=el.text.replace('fleetscope_milestone','fleetscope_autonomy')
    model=world.find("model[@name='husky_mobile_manipulator']")
    lidar=model.find(".//sensor[@name='lidar']/lidar")
    lidar.find('range/max').text='60'
    lidar.find('scan/horizontal/samples').text='1081'
    lidar.find('scan/horizontal/min_angle').text=str(-math.pi)
    lidar.find('scan/horizontal/max_angle').text=str(math.pi)
    drive=model.find("plugin[@name='ignition::gazebo::systems::DiffDrive']")
    for key,value in [('max_angular_velocity',1.8),('min_angular_velocity',-1.8),('max_angular_acceleration',2.),('min_angular_acceleration',-2.)]:
        drive.find(key).text=str(value)
    xml.write(OUT/'world.sdf',encoding='utf-8',xml_declaration=True)
    ekf=yaml.safe_load((ROOT/'milestone/generated/ekf.yaml').read_text())
    ekf['ekf_filter_node']['ros__parameters']['publish_tf']=True
    (OUT/'ekf.yaml').write_text(yaml.safe_dump(ekf))
    slam=yaml.safe_load(Path('/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml').read_text())
    p=slam['slam_toolbox']['ros__parameters']
    p.update(use_sim_time=True,odom_frame='husky/odom',map_frame='slam_map',base_frame='husky/base_link',
             scan_topic='/husky/scan',resolution=.15,map_update_interval=1.,minimum_time_interval=.15,
             minimum_travel_distance=.15,minimum_travel_heading=.12,transform_timeout=.4,
             transform_publish_period=0.,enable_interactive_mode=False,min_laser_range=.2,max_laser_range=60.,
             correlation_search_space_dimension=1.,correlation_search_space_resolution=.02,
             loop_search_space_resolution=.05,fine_search_angle_offset=.001,do_loop_closing=True,
             distance_variance_penalty=.1,angle_variance_penalty=.01,
             minimum_distance_penalty=.1,minimum_angle_penalty=.1)
    (OUT/'slam.yaml').write_text(yaml.safe_dump(slam))
    base=yaml.safe_load(Path('/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml').read_text())
    nav={}
    amcl=copy.deepcopy(base['amcl']['ros__parameters'])
    amcl.update(global_frame_id='estate',odom_frame_id='husky/odom',base_frame_id='husky/base_link',
        scan_topic='/husky/scan',laser_max_range=60.,laser_min_range=.2,max_beams=1081,
        update_min_d=.1,update_min_a=.08,transform_tolerance=.5,
        alpha1=.1,alpha2=.1,alpha3=.05,alpha4=.05,set_initial_pose=True,
        initial_pose=dict(x=-12.,y=-33.,z=0.,yaw=-math.pi/2),always_reset_initial_pose=True)
    nav['amcl']={'ros__parameters':amcl}
    bt=copy.deepcopy(base['bt_navigator']['ros__parameters'])
    bt.update(global_frame='estate',robot_base_frame='husky/base_link',odom_topic='/husky/odometry/filtered',
              default_nav_to_pose_bt_xml=str(ROOT/'autonomy/navigate.xml'),transform_tolerance=.5)
    nav['bt_navigator']={'ros__parameters':bt}
    controller=dict(use_sim_time=True,odom_topic='/husky/odometry/filtered',controller_frequency=12.,min_x_velocity_threshold=.001,
        min_y_velocity_threshold=.001,min_theta_velocity_threshold=.001,failure_tolerance=2.,
        progress_checker_plugin='progress_checker',goal_checker_plugins=['goal_checker'],controller_plugins=['FollowPath'],
        progress_checker=dict(plugin='nav2_controller::SimpleProgressChecker',required_movement_radius=.15,movement_time_allowance=45.),
        goal_checker=dict(plugin='nav2_controller::SimpleGoalChecker',xy_goal_tolerance=.45,yaw_goal_tolerance=.3,stateful=True),
        FollowPath=dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            desired_linear_vel=.6,lookahead_dist=1.1,min_lookahead_dist=.6,max_lookahead_dist=1.8,
            lookahead_time=1.8,rotate_to_heading_angular_vel=.35,transform_tolerance=.4,
            use_velocity_scaled_lookahead_dist=True,min_approach_linear_velocity=.12,approach_velocity_scaling_dist=1.5,
            use_collision_detection=True,max_allowed_time_to_collision_up_to_carrot=1.5,
            use_regulated_linear_velocity_scaling=True,use_cost_regulated_linear_velocity_scaling=True,
            cost_scaling_dist=1.3,cost_scaling_gain=.7,inflation_cost_scaling_factor=2.5,
            regulated_linear_scaling_min_radius=1.5,regulated_linear_scaling_min_speed=.12,
            use_rotate_to_heading=True,allow_reversing=False,rotate_to_heading_min_angle=.65,max_angular_accel=.6,
            max_robot_pose_search_dist=15.))
    nav['controller_server']={'ros__parameters':controller}
    for name,global_frame,rolling in [('local_costmap','husky/odom',True),('global_costmap','estate',False)]:
        p=dict(use_sim_time=True,global_frame=global_frame,robot_base_frame='husky/base_link',transform_tolerance=.5,
            update_frequency=5. if rolling else 2.,publish_frequency=1.,resolution=.15 if rolling else .2,
            robot_radius=.72,footprint_padding=.04,rolling_window=rolling,track_unknown_space=False,
            plugins=['obstacle_layer','inflation_layer'] if rolling else ['static_layer','obstacle_layer','inflation_layer'],
            always_send_full_costmap=True,
            obstacle_layer=dict(plugin='nav2_costmap_2d::ObstacleLayer',enabled=True,observation_sources='scan',
                scan=dict(topic='/husky/scan',data_type='LaserScan',clearing=True,marking=True,
                    max_obstacle_height=2.,obstacle_max_range=58.,obstacle_min_range=.2,
                    raytrace_max_range=60.,raytrace_min_range=.2,inf_is_valid=True)),
            inflation_layer=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=1.7,cost_scaling_factor=2.5))
        if rolling:p.update(width=12,height=12)
        else:p['static_layer']=dict(plugin='nav2_costmap_2d::StaticLayer',map_topic='/navigation/prior_map',map_subscribe_transient_local=True)
        nav[name]={name:{'ros__parameters':p}}
    nav['planner_server']={'ros__parameters':dict(use_sim_time=True,expected_planner_frequency=2.,planner_plugins=['GridBased'],
        GridBased=dict(plugin='nav2_navfn_planner/NavfnPlanner',tolerance=.4,use_astar=True,allow_unknown=False))}
    behavior=copy.deepcopy(base['behavior_server']['ros__parameters'])
    behavior.update(robot_base_frame='husky/base_link',global_frame='husky/odom',transform_tolerance=.5,
                    max_rotational_vel=.35,min_rotational_vel=.1,rotational_acc_lim=.6)
    nav['behavior_server']={'ros__parameters':behavior}
    (OUT/'nav2.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
    rviz=yaml.safe_load((ROOT/'milestone/command-center.rviz').read_text())
    for d in rviz['Visualization Manager']['Displays']:
        if 'Observed occupancy' in d['Name']:
            d['Name']='Observed occupancy (localized scans)';d['Alpha']=.45
    topic=lambda value:dict(Value=value,Depth=1,**{'Reliability Policy':'Reliable','Durability Policy':'Transient Local'})
    rviz['Visualization Manager']['Displays'] += [
        {'Class':'rviz_default_plugins/Map','Name':'Independent SLAM map (optional)','Enabled':False,'Value':False,'Alpha':.4,'Topic':topic('/twin/slam_map')},
        {'Class':'rviz_default_plugins/Path','Name':'Planned route (received)','Enabled':True,'Value':True,'Color':'255; 200; 60','Line Style':'Lines','Line Width':.10,'Topic':topic('/twin/planned_path')},
        {'Class':'rviz_default_plugins/MarkerArray','Name':'Inspection checkpoints','Enabled':True,'Value':True,'Topic':topic('/twin/inspection_points')}]
    rviz['Visualization Manager']['Views']['Current'].update(Distance=65,**{'Focal Point':{'X':-5,'Y':-25,'Z':0}})
    (OUT/'command-center.rviz').write_text(yaml.safe_dump(rviz,sort_keys=False))
    print('Built autonomous world, surveyed prior, AMCL, SLAM and Nav2 configurations.')

if __name__=='__main__':main()
