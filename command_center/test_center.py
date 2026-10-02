"""Fault, stale-data, replay, idempotency and local hold regression tests."""
import base64
from concurrent.futures import Future
from http.server import ThreadingHTTPServer
import json
import math
from pathlib import Path
import sqlite3
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import uuid
import urllib.request
import urllib.error
from .health import HealthTracker
from .protocol import CommandLedger, validate_command
from .store import Store


class HealthTests(unittest.TestCase):
    def ready(self):
        tracker = HealthTracker()
        for name in tracker.LIMITS:
            for i in range(11): tracker.observe(name, i*.1, 100+i*.1)
        return tracker

    def test_normal_measurements_are_healthy_and_rate_uses_capture_time(self):
        result = self.ready().snapshot(1., 101.)
        self.assertEqual(result['status'], 'HEALTHY')
        self.assertEqual(result['sensors']['imu']['rate_hz'], 10.)

    def test_frozen_camera_timestamp_is_detected_despite_continued_delivery(self):
        tracker = self.ready()
        for name in tracker.LIMITS: tracker.observe(name, 4. if name!='camera' else 1., 104.)
        value = tracker.snapshot(4., 104.)
        self.assertEqual(value['status'], 'DEGRADED')
        self.assertEqual([i['code'] for i in value['issues']], ['camera_stale'])

    def test_missing_critical_sensor_is_detected_even_if_clock_stops(self):
        self.assertEqual(self.ready().snapshot(1., 103.)['status'], 'CRITICAL')

    def test_capture_age_detects_delayed_measurements_with_recent_arrival(self):
        tracker = self.ready(); tracker.observe('imu', 1.1, 110.)
        self.assertEqual(tracker.snapshot(8., 110.)['sensors']['imu']['status'], 'STALE')

    def test_uncertainty_is_an_explicit_warning_not_a_claim_of_actual_error(self):
        tracker = self.ready(); tracker.sigma = 2.
        self.assertEqual(tracker.snapshot(1., 101.)['status'], 'DEGRADED')
        self.assertEqual(tracker.snapshot(1., 101.)['issues'][0]['code'], 'position_uncertainty')

    def test_relative_motion_comparison_ignores_different_frame_origins_and_rotations(self):
        tracker=self.ready()
        for i in range(41):
            tracker.compare_motion(i*.1, [100., -50.+i*.05, math.pi/2], [i*.05,0.,0.])
        self.assertAlmostEqual(tracker.motion_disagreement,0.)

    def test_pose_jump_warns_but_constant_bias_is_not_claimed_detectable(self):
        tracker=self.ready()
        for i in range(41):tracker.compare_motion(i*.1,[0.,0.,0.],[0.,0.,0.])
        tracker.compare_motion(4.1,[2.5,0.,0.],[0.,0.,0.])
        for name in tracker.LIMITS:tracker.observe(name,4.1,104.1)
        self.assertEqual(tracker.snapshot(4.1,104.1)['status'],'DEGRADED')
        self.assertEqual(tracker.snapshot(4.1,104.1)['issues'][0]['code'],'position_motion_disagreement')
        for i in range(42,91):tracker.compare_motion(i*.1,[2.5,0.,0.],[0.,0.,0.])
        self.assertAlmostEqual(tracker.motion_disagreement,0.)


class ProtocolTests(unittest.TestCase):
    def command(self):
        return dict(request_id=str(uuid.uuid4()), action='start', route='quick', robot_id='husky_01', run_id='test', expires_wall=108.)

    def test_expired_cross_run_and_unknown_commands_are_rejected(self):
        command = self.command(); validate_command(command, 'test', 100.)
        for change in [dict(expires_wall=99), dict(expires_wall=float('nan')), dict(run_id='old'), dict(robot_id='go2'), dict(action='shell'), dict(route='nowhere')]:
            with self.assertRaises(ValueError): validate_command(dict(command, **change), 'test', 100.)

    def test_command_id_survives_restart_without_reexecution(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'commands.sqlite'; cmd = self.command()
            ledger = CommandLedger(path); self.assertTrue(ledger.reserve(cmd))
            ledger.finish(dict(request_id=cmd['request_id'], status='ACKNOWLEDGED', reason='accepted'))
            ledger.db.close(); ledger = CommandLedger(path)
            self.assertFalse(ledger.reserve(cmd)); self.assertEqual(ledger.results()[0]['status'], 'ACKNOWLEDGED')
            ledger.db.close()


class ProjectionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.run = Path(self.temp.name)
        self.source = sqlite3.connect(self.run/'twin-history.sqlite')
        self.source.execute('CREATE TABLE packets(run_id TEXT,seq INTEGER,stamp REAL,received REAL,payload TEXT,PRIMARY KEY(run_id,seq))')
        self.store = Store(self.run)

    def tearDown(self):
        self.store.db.close(); self.source.close(); self.temp.cleanup()

    def packet(self, seq, **extras):
        packet = dict(run_id=self.run.name, seq=seq, stamp=float(seq), robot_id='husky_01',
            state=dict(pose=[float(seq), 0., 0.], stamp=float(seq)),
            health=dict(status='HEALTHY', sensors={}, issues=[]), safety=dict(held=False), mission=dict(status='IDLE'))
        packet.update(extras)
        self.source.execute('INSERT INTO packets VALUES (?,?,?,?,?)', (self.run.name, seq, packet['stamp'], time.time(), json.dumps(packet)))
        self.source.commit(); self.store.ingest()

    def test_replay_camera_state_and_trail_never_read_future_observations(self):
        self.packet(1, camera=dict(stamp=1., data=base64.b64encode(b'early-image').decode()))
        self.packet(2)
        self.packet(3, camera=dict(stamp=3., data=base64.b64encode(b'later-image').decode()))
        replay = self.store.snapshot(2)
        self.assertEqual(replay['robot']['state']['pose'][0], 2.)
        self.assertEqual(replay['robot']['camera_stamp'], 1.)
        self.assertEqual(self.store.camera(2), b'early-image')
        self.assertEqual(replay['trail'], [[1., 0.], [2., 0.]])
        self.assertTrue(replay['replay']); self.assertFalse(replay['live']); self.assertEqual(replay['commands'], [])

    def test_incidents_deduplicate_and_record_recovery(self):
        faulty = dict(status='CRITICAL', sensors={}, issues=[dict(code='imu_stale', severity='critical', message='Missing IMU')])
        self.packet(1); self.packet(2, health=faulty); self.packet(3, health=faulty); self.packet(4)
        events = self.store.snapshot()['events']
        self.assertEqual(len([e for e in events if e['kind']=='critical']), 1)
        self.assertEqual(len([e for e in events if e['kind']=='recovered']), 1)

    def test_out_of_order_payload_does_not_roll_back_state(self):
        self.packet(1); self.packet(3); self.packet(2)
        self.assertEqual(self.store.snapshot()['robot']['seq'], 3)
        self.assertEqual(self.store.snapshot()['bounds']['count'], 3)

    def test_late_incident_and_camera_fill_history_without_rewinding_live_state(self):
        self.packet(1)
        self.packet(3, camera=dict(stamp=3., data=base64.b64encode(b'future').decode()))
        fault=dict(status='CRITICAL', sensors={}, issues=[dict(code='imu_stale', severity='critical', message='Missing IMU')])
        self.packet(2, health=fault, camera=dict(stamp=2., data=base64.b64encode(b'late').decode()))
        current=self.store.snapshot(); past=self.store.snapshot(2)
        self.assertEqual(current['robot']['seq'],3);self.assertEqual(current['robot']['camera_stamp'],3.)
        self.assertEqual(past['robot']['health']['status'],'CRITICAL');self.assertEqual(self.store.camera(2),b'late')
        self.assertEqual([(e['seq'],e['kind']) for e in current['events'] if e['kind'] in ('critical','recovered')],[(3,'recovered'),(2,'critical')])

    def test_restarting_projection_is_idempotent(self):
        self.packet(1); self.packet(2)
        self.store.db.close(); self.store = Store(self.run); self.store.ingest()
        self.assertEqual(self.store.snapshot()['bounds']['count'], 2)

    def test_fresh_clock_does_not_make_stale_telemetry_live(self):
        self.packet(1)
        path = self.run/'twin-status.json'
        path.write_text(json.dumps(dict(updated_wall=time.time(), transport_age_wall_s=.1, sim_time=1.1, simulation_paused_or_stalled=False)))
        self.assertTrue(self.store.connected()[0])
        path.write_text(json.dumps(dict(updated_wall=time.time(), transport_age_wall_s=8., sim_time=9., simulation_paused_or_stalled=False)))
        self.assertFalse(self.store.connected()[0]); self.assertEqual(self.store.snapshot()['robot']['state']['pose'][0], 1.)

    def test_unacknowledged_command_is_uncertain_and_late_ack_reconciles(self):
        command = dict(request_id=str(uuid.uuid4()), action='cancel', run_id=self.run.name, robot_id='husky_01', expires_wall=time.time()-1.)
        self.assertTrue(self.store.add_command(command)); self.assertFalse(self.store.add_command(command))
        self.store.ingest(); self.assertEqual(self.store.snapshot()['commands'][0]['status'], 'UNCONFIRMED')
        result = dict(request_id=command['request_id'], status='ACKNOWLEDGED', reason='canceled')
        self.packet(1, command_results=[result])
        self.assertEqual(self.store.snapshot()['commands'][0]['status'], 'ACKNOWLEDGED')


class SafetyTests(unittest.TestCase):
    def test_sensor_clock_lead_is_covered_without_making_old_position_fresh(self):
        from milestone.unit import EdgeTelemetryAgent
        node=SimpleNamespace(heartbeat=True, latest=dict(stamp=42.,pose=[1.,2.,0.]), last_sample=64.,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=64200000000)),
            run_id='run',last_imu=dict(stamp=64.25,gyro=[0.,0.,0.]),sensor_stamps={'estimate':42.,'imu':64.25},
            drive_status='ready',outbox=Mock(dropped=0),scans=[],camera=None,enrich_packet=lambda p:None)
        EdgeTelemetryAgent.sample(node)
        packet=node.outbox.put.call_args.args[0]
        self.assertEqual(packet['stamp'],64.25)
        self.assertEqual(packet['imu']['stamp'],64.25)
        self.assertEqual(packet['state']['stamp'],42.)
        self.assertAlmostEqual(packet['sensor_age']['estimate'],22.25)
        self.assertEqual(node.last_sample,64.25)

    def test_health_envelope_advances_without_relabelling_old_pose_as_fresh(self):
        from milestone.unit import EdgeTelemetryAgent
        node=SimpleNamespace(heartbeat=True, latest=dict(stamp=42., pose=[1.,2.,0.]), last_sample=64.,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=64200000000)),
            run_id='run',last_imu=None,sensor_stamps={'estimate':42.},drive_status='safety_hold',
            outbox=Mock(dropped=0),scans=[],camera=None,enrich_packet=lambda p:None)
        EdgeTelemetryAgent.sample(node)
        packet=node.outbox.put.call_args.args[0]
        self.assertAlmostEqual(packet['stamp'],64.2)
        self.assertEqual(packet['state']['stamp'],42.)
        self.assertAlmostEqual(packet['sensor_age']['estimate'],22.2)

    def test_critical_fault_latches_hold_and_recovery_does_not_auto_resume(self):
        from .safety import Supervisor
        good = [False]
        node = SimpleNamespace(health=dict(issues=[dict(code='imu_stale', severity='critical')]),
            armed=True, held=False, reason='Ready', good_since=None, pub=Mock(), cancel=Mock(),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000)), healthy=lambda:good[0])
        node.hold = lambda reason:Supervisor.hold(node, reason)
        with patch('command_center.safety.atomic_json'):
            Supervisor.tick(node); self.assertTrue(node.held); node.cancel.publish.assert_called_once()
            good[0]=True; node.good_since=time.monotonic()-3.; Supervisor.tick(node); self.assertTrue(node.held)
        response=Supervisor.set_hold(node, SimpleNamespace(data=False), SimpleNamespace())
        self.assertTrue(response.success); self.assertFalse(node.held)

    def test_missing_supervisor_report_blocks_both_velocity_axes(self):
        from autonomy.navigation import Navigation
        from geometry_msgs.msg import Twist
        now = time.monotonic(); cmd=Twist(); cmd.linear.x=.6; cmd.angular.z=.3
        node=SimpleNamespace(require_safety=True, safety_held=False, last_safety=0., request=cmd,
            last_cmd=now, last_scan=now, last_imu=now, last_localization=now, integral=.2,
            front=10., rate=0., drive=Mock(), guard=Mock(), odom_count=1)
        with patch('autonomy.navigation.atomic_json'): Navigation.control(node)
        output=node.drive.publish.call_args.args[0]
        self.assertEqual(output.linear.x,0.); self.assertEqual(output.angular.z,0.)


class HTTPBoundaryTests(unittest.TestCase):
    def setUp(self):
        from .server import handler_type
        self.temp=tempfile.TemporaryDirectory(); self.store=Store(Path(self.temp.name))
        self.server=ThreadingHTTPServer(('127.0.0.1',0),handler_type(self.store,None,'test-token',0))
        port=self.server.server_address[1]
        self.server.RequestHandlerClass=handler_type(self.store,None,'test-token',port)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.url=f'http://127.0.0.1:{port}'

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join()
        self.store.db.close();self.temp.cleanup()

    def post(self,headers):
        req=urllib.request.Request(self.url+'/api/commands',data=b'{"action":"hold"}',headers=headers)
        try:
            with urllib.request.urlopen(req) as response:return response.status
        except urllib.error.HTTPError as error:return error.code

    def test_write_requires_command_token(self):
        self.assertEqual(self.post({}),403)

    def test_recorded_run_rejects_even_authorized_control(self):
        self.assertEqual(self.post({'X-FleetScope-Token':'test-token'}),409)

    def test_foreign_origin_cannot_use_command_token(self):
        self.assertEqual(self.post({'X-FleetScope-Token':'test-token','Origin':'https://foreign.example'}),403)

    def test_read_only_state_is_available_without_ros(self):
        with urllib.request.urlopen(self.url+'/api/state') as response:value=json.load(response)
        self.assertFalse(value['live']); self.assertIsNone(value['robot'])


if __name__=='__main__': unittest.main()
