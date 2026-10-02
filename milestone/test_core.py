"""Regression checks for failure-prone geometry and delivery behavior."""
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as E
import yaml
from .core import Occupancy, Outbox, PoseHistory, estate_pose, capture_through
from .build import prepare_world

class DeliveryTests(unittest.TestCase):
    def test_unacknowledged_prefix_cannot_block_new_observations(self):
        with tempfile.TemporaryDirectory() as d:
            q=Outbox(Path(d)/'queue.sqlite')
            stuck={q.put({'stamp':i}) for i in range(12)}
            self.assertEqual({seq for seq,_ in q.scheduled_batch(now=0.)},stuck)
            fresh=q.put({'stamp':99.})
            sent=q.scheduled_batch(now=.2)
            self.assertIn(fresh,[seq for seq,_ in sent])
            self.assertTrue(stuck.isdisjoint(seq for seq,_ in sent))
            q.ack(fresh)
            self.assertEqual(q.count(),12)
            self.assertEqual({seq for seq,_ in q.scheduled_batch(now=1.)},stuck)
            self.assertEqual(q.scheduled_batch(now=2.),[])
            self.assertEqual({seq for seq,_ in q.scheduled_batch(now=3.)},stuck)
            q.db.close()

    def test_history_progresses_while_new_observations_keep_arriving(self):
        with tempfile.TemporaryDirectory() as d:
            q=Outbox(Path(d)/'queue.sqlite')
            initial={q.put({'stamp':i}) for i in range(100)}
            seen=set()
            for tick in range(30):
                newest=q.put({'stamp':100+tick})
                batch=q.scheduled_batch(now=tick*.2)
                ids=[seq for seq,_ in batch]
                self.assertIn(newest,ids)
                self.assertEqual(len(ids),len(set(ids)))
                self.assertLessEqual(len(ids),12)
                seen.update(ids)
                for seq in ids:
                    if seq not in initial:q.ack(seq)
            self.assertTrue(initial <= seen)
            self.assertEqual(q.count(),100)
            q.db.close()

    def test_restart_retains_retry_records_and_does_not_require_old_ram_timers(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'queue.sqlite';q=Outbox(path)
            seq=q.put({'stamp':10.});q.scheduled_batch(now=10.);q.db.close()
            q=Outbox(path)
            self.assertEqual(q.scheduled_batch(now=0.),[(seq,{'stamp':10.})])
            q.db.close()

    def test_restart_and_out_of_order_ack_do_not_lose_unsent_records(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'queue.sqlite'; q=Outbox(path)
            first=q.put({'stamp':1,'value':'first'}); second=q.put({'stamp':2,'value':'second'})
            q.db.close(); q=Outbox(path)
            self.assertEqual([row[0] for row in q.batch()],[first,second])
            q.ack(second)
            self.assertEqual(q.batch(),[(first,{'stamp':1,'value':'first'})])
            q.ack(second)  # Repeated ACK is harmless.
            self.assertEqual(q.count(),1)
            q.ack(first); self.assertEqual(q.count(),0); q.db.close()

    def test_bounded_outbox_reports_drops(self):
        with tempfile.TemporaryDirectory() as d:
            q=Outbox(Path(d)/'queue.sqlite',limit=3)
            for i in range(8): q.put({'stamp':i})
            self.assertEqual(q.count(),3); self.assertEqual(q.dropped,5)
            self.assertEqual([p['stamp'] for _,p in q.batch()],[5,6,7]); q.db.close()

class GeometryTests(unittest.TestCase):
    def test_envelope_covers_sensor_callbacks_ahead_of_cached_clock(self):
        packet={'stamp':100.,'state':{'stamp':98.,'pose':[1,2,0]},
                'imu':{'stamp':100.05},'camera':{'stamp':99.5},'health':{'stamp':100.02}}
        self.assertAlmostEqual(capture_through(packet),100.05)
        self.assertEqual(packet['state']['stamp'],98.)
        self.assertEqual(packet['camera']['stamp'],99.5)
        self.assertEqual(packet['imu']['stamp'],100.05)

    def test_scan_alignment_across_heading_wrap_and_timestamp_gap(self):
        h=PoseHistory(); h.add(1,[0,0,math.radians(179)]); h.add(1.1,[1,0,math.radians(-179)])
        mid=h.at(1.05)
        self.assertAlmostEqual(mid[0],.5); self.assertAlmostEqual(abs(mid[2]),math.pi)
        self.assertIsNone(h.at(3)); h.add(2,[1,0,0]); self.assertIsNone(h.at(1.5))

    def test_surveyed_heading_rotates_odometry(self):
        self.assertEqual(estate_pose(2,0,0)[:2],[-12.,-35.])

    def test_only_observations_create_occupied_cells_and_free_rays_clear_them(self):
        m=Occupancy(resolution=.1,width=100,height=100,origin=(-5.,-5.))
        self.assertTrue((m.data()==-1).all())
        scan={'pose':[0.,0.,0.],'ranges':[2.], 'min':.18,'max':4.,'angle_min':0.,'angle_increment':.1}
        m.integrate(scan); x,y=m.cell(2.48,0.)
        self.assertGreater(m.data()[y,x],65)
        fx,fy=m.cell(1,0); self.assertLess(m.data()[fy,fx],50)
        self.assertEqual(m.data()[0,0],-1)
        scan['ranges']=[None]
        for _ in range(10): m.integrate(scan)
        self.assertLess(m.data()[y,x],50)

class ConfigurationTests(unittest.TestCase):
    def test_shared_fps_world_has_one_renderer_and_no_player_in_milestone(self):
        # Both plugin filename spellings and namespaces resolve to the same system.
        w=E.fromstring('''<world name="shared">
          <physics><max_step_size>.004</max_step_size></physics>
          <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
            <render_engine>ogre2</render_engine><custom_setting>preserve</custom_setting>
          </plugin>
          <plugin filename="ignition-gazebo-sensors-system" name="gz::sim::systems::Sensors"/>
          <include><uri>model://fleetscope_player</uri><name>renamed_player</name></include>
          <include><uri>model://fleetscope_husky_kinova</uri><name>husky_mobile_manipulator</name></include>
          <include><uri>model://fleetscope_go2</uri><name>go2_inspector</name></include>
          <model name="estate_building"/>
        </world>''')
        prepare_world(w)
        first=E.tostring(w)
        prepare_world(w)
        self.assertEqual(E.tostring(w),first)
        systems=[p.get('name').split('::')[-1] for p in w.findall('plugin')]
        self.assertEqual(systems.count('Sensors'),1)
        self.assertEqual(systems.count('Imu'),1)
        self.assertEqual(w.findtext('plugin/custom_setting'),'preserve')
        self.assertEqual([i.findtext('name') for i in w.findall('include')],['go2_inspector'])
        self.assertIsNotNone(w.find("model[@name='estate_building']"))

    def test_generated_world_contains_no_fps_dependencies_or_duplicate_systems(self):
        root=Path(__file__).resolve().parent
        w=E.parse(root/'generated/world.sdf').getroot().find('world')
        systems=[p.get('name').split('::')[-1] for p in w.findall('plugin')]
        self.assertEqual(systems.count('Sensors'),1)
        self.assertEqual(systems.count('Imu'),1)
        self.assertNotIn('fleetscope_player',E.tostring(w,encoding='unicode'))

    def test_only_husky_is_dynamic_and_truth_is_not_an_estimator_input(self):
        root=Path(__file__).resolve().parent
        w=E.parse(root/'generated/world.sdf').getroot().find('world')
        dynamic=[m for m in w.findall('model') if m.findtext('static')=='false']
        self.assertEqual([m.get('name') for m in dynamic],['husky_mobile_manipulator'])
        joints=dynamic[0].findall('joint')
        self.assertEqual(len([j for j in joints if j.get('type')=='revolute']),4)
        params=yaml.safe_load((root/'generated/ekf.yaml').read_text())['ekf_filter_node']['ros__parameters']
        self.assertFalse(any(params['odom0_config'][:6]))
        self.assertFalse(any(params['imu0_config'][:6]))
        self.assertTrue(params['imu0_config'][11])
        self.assertEqual({params['odom0'],params['imu0']},{'/husky/wheel_odometry','/husky/imu/data'})

if __name__=='__main__': unittest.main()
