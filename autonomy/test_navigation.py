"""Regression checks for data ordering, recoverable map transport and site clearance."""
import base64
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib
import xml.etree.ElementTree as E
from scipy.spatial.transform import Rotation
import numpy as np
from scipy.ndimage import distance_transform_edt
from nav_msgs.msg import OccupancyGrid
from autonomy.unit import NavigationTelemetryAgent
from autonomy.twin import NavigationTwin
from autonomy.mission import Inspection,ROUTES
from autonomy.navigation import Navigation
from geometry_msgs.msg import Twist
import time
from unittest.mock import Mock

class MapDelivery(unittest.TestCase):
    def test_map_roundtrip_and_periodic_resend_after_a_dropped_envelope(self):
        agent=SimpleNamespace(slam=None,route=None,mission={'status':'RUNNING'},last_map_sent=-1.,last_map_envelope=-10.)
        msg=OccupancyGrid();msg.header.frame_id='slam_map';msg.header.stamp.sec=3
        msg.info.width=3;msg.info.height=2;msg.info.resolution=.15;msg.info.origin.position.x=-2.;msg.info.origin.orientation.w=1.
        msg.data=[-1,0,100,50,-1,0]
        NavigationTelemetryAgent.map_received(agent,msg)
        first={'stamp':4.};NavigationTelemetryAgent.enrich_packet(agent,first)
        self.assertEqual(list(np.frombuffer(zlib.decompress(base64.b64decode(first['slam']['data'])),np.int8)),list(msg.data))
        # Even if first is dropped on outbox overflow, a stationary map is resent.
        later={'stamp':10.};NavigationTelemetryAgent.enrich_packet(agent,later)
        self.assertEqual(later['slam'],first['slam'])

    def test_late_packets_cannot_roll_back_map_or_mission(self):
        twin=NavigationTwin.__new__(NavigationTwin)
        twin.slam=None;twin.plan=None;twin.mission=None;twin.extra_stamp=-1.
        with patch('milestone.twin.Twin.apply'):
            NavigationTwin.apply(twin,{'stamp':20.,'slam':{'stamp':18.},'mission':{'status':'SUCCEEDED'},'planned_path':{'stamp':19.}})
            NavigationTwin.apply(twin,{'stamp':10.,'slam':{'stamp':8.},'mission':{'status':'RUNNING'},'planned_path':{'stamp':9.}})
        self.assertEqual(twin.mission['status'],'SUCCEEDED');self.assertEqual(twin.slam['stamp'],18.);self.assertEqual(twin.plan['stamp'],19.)

class MissionResults(unittest.TestCase):
    def test_nav2_abort_is_explicit_failure(self):
        node=SimpleNamespace(state=dict(status='RUNNING',index=0,checkpoints=[{'status':'ACTIVE'}]),goal_handle=object())
        node.event=lambda name:None
        node.finish=lambda success,reason:Inspection.finish(node,success,reason)
        future=Future();future.set_result(SimpleNamespace(status=6));Inspection.result(node,future)
        self.assertEqual(node.state['status'],'FAILED');self.assertEqual(node.state['checkpoints'][0]['status'],'UNREACHABLE')

    def test_timeout_is_not_reported_as_user_cancellation(self):
        node=SimpleNamespace(state=dict(status='CANCELING',reason='Checkpoint deadline exceeded',index=0,checkpoints=[{'status':'ACTIVE'}]),goal_handle=object(),event=lambda name:None)
        future=Future();future.set_result(SimpleNamespace(status=5));Inspection.result(node,future)
        self.assertEqual(node.state['status'],'FAILED')

class Survey(unittest.TestCase):
    def test_slip_axis_is_parallel_to_axle_after_collision_rotation(self):
        model=E.parse(Path(__file__).parent/'generated/world.sdf').find(".//model[@name='husky_mobile_manipulator']")
        for joint in model.findall("joint[@type='revolute']"):
            axle=np.fromstring(joint.findtext('axis/xyz'),sep=' ')
            link=model.find(f"link[@name='{joint.findtext('child')}']")
            collision=link.find('collision');pose=np.fromstring(collision.findtext('pose'),sep=' ')
            direction=np.fromstring(collision.findtext('surface/friction/ode/fdir1'),sep=' ')
            direction=Rotation.from_euler('xyz',pose[3:]).apply(direction)
            self.assertGreater(abs(float(np.dot(direction,axle))),.999,joint.get('name'))

    def test_checkpoints_have_robot_clearance_and_barrier_is_absent(self):
        prior=np.load(Path(__file__).parent/'generated/survey-prior.npz')
        data=prior['data'];distance=distance_transform_edt(data==0)*float(prior['resolution'])
        def cell(x,y):return round((y+95)/.2),round((x+100)/.2)
        for route in ROUTES.values():
            for name,(x,y,_) in route:self.assertGreater(distance[cell(x,y)],.9,name)
        self.assertEqual(data[cell(-12,-40)],0)
        self.assertEqual(data[cell(79,-18)],100,'The raised culvert is conservatively excluded from this flat-ground controller')

class MotionGuard(unittest.TestCase):
    def node(self):
        now=time.monotonic();cmd=Twist();cmd.linear.x=.5;cmd.angular.z=.2
        return SimpleNamespace(request=cmd,last_cmd=now,last_scan=now,last_imu=now,last_localization=now,
            rate=0.,front=10.,integral=0.,drive=Mock(),guard=Mock(),odom_count=1)

    def test_missing_commands_or_sensors_stop_wheels(self):
        for field in ('last_cmd','last_scan','last_imu','last_localization'):
            node=self.node();setattr(node,field,0.)
            with patch('autonomy.navigation.atomic_json'):Navigation.control(node)
            command=node.drive.publish.call_args.args[0]
            self.assertEqual(command.linear.x,0.,field);self.assertEqual(command.angular.z,0.,field)

    def test_front_obstacle_stops_forward_motion(self):
        node=self.node();node.front=.8
        with patch('autonomy.navigation.atomic_json'):Navigation.control(node)
        self.assertEqual(node.drive.publish.call_args.args[0].linear.x,0.)
        self.assertEqual(node.guard.publish.call_args.args[0].data,'obstacle_stop')

if __name__=='__main__':unittest.main()
