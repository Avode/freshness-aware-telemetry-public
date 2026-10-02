import json
from pathlib import Path
import unittest
import uuid
from . import protocol


class BoundaryTests(unittest.TestCase):
    def test_decoder_rejects_non_object_nonfinite_and_oversized_messages(self):
        for value in ['[]', '{"n":NaN}', '{"n":Infinity}', '{"n":1e400}', 'null', '['*1100+']'*1100]:
            with self.assertRaises(ValueError): protocol.decode(value)
        with self.assertRaises(ValueError): protocol.decode('{}', 1)

    def test_ack_is_per_sequence_and_checks_identity_schema_and_retention(self):
        ack = dict(schema=1, robot_id=protocol.ROBOT, run_id='run', seq=3)
        self.assertEqual(protocol.acknowledgement(ack, 'run'), dict(run_id='run', seq=3))
        for change in [dict(seq=True), dict(seq=0), dict(seq=2.5), dict(robot_id='go2_01'), dict(run_id='old'), dict(schema=2)]:
            with self.assertRaises(ValueError): protocol.acknowledgement(dict(ack, **change), 'run')
        with self.assertRaises(ValueError): protocol.acknowledgement(ack, 'run', True)

    def test_commands_reject_retained_expired_wrong_run_and_clock_skew(self):
        cmd = dict(robot_id=protocol.ROBOT, run_id='run', request_id=str(uuid.uuid4()),
                   action='start', route='quick', issued_wall=100., expires_wall=108.)
        protocol.command(cmd, 'run', 100.)
        with self.assertRaises(ValueError): protocol.command(cmd, 'run', 100., True)
        with self.assertRaises(ValueError): protocol.command(cmd, 'old', 100.)
        with self.assertRaises(ValueError): protocol.command(cmd, 'run', 109.)
        with self.assertRaises(ValueError): protocol.command(cmd, 'run', 103.)
        for change in [dict(issued_wall=True), dict(expires_wall=True), dict(issued_wall=104.), dict(action='velocity')]:
            with self.assertRaises(ValueError): protocol.command(dict(cmd, **change), 'run', 100.)

    def test_legacy_command_contract_remains_compatible(self):
        cmd = dict(robot_id=protocol.ROBOT, run_id='run', request_id=str(uuid.uuid4()), action='hold', expires_wall=108.)
        protocol.command(cmd, 'run', 100.)

    def test_heartbeat_distinguishes_fresh_arrival_from_advancing_capture_clock(self):
        site = json.loads(Path(__file__).with_name('site.json').read_text())
        session = protocol.SessionHeartbeat('run', site)
        first = session.sample(10., 1, wall=100., monotonic=50.)
        self.assertFalse(first['clock_advancing'])
        value = session.sample(10.5, 2, wall=101., monotonic=51.)
        self.assertTrue(value['clock_advancing'])
        stale = session.sample(10.5, 2, wall=104., monotonic=54.)
        self.assertFalse(stale['clock_advancing'])
        self.assertEqual(stale['session_id'], first['session_id'])
        self.assertGreater(stale['heartbeat_seq'], first['heartbeat_seq'])
        self.assertNotIn('pose', stale)
        with self.assertRaises(ValueError): session.sample(0., 1, wall=105., monotonic=55.)

    def test_telemetry_requires_finite_pose_and_real_integer_sequence(self):
        value = dict(schema=1, robot_id=protocol.ROBOT, run_id='run', seq=1, stamp=2., state=dict(pose=[1.,2.,0.]))
        protocol.telemetry(value, 'run')
        for change in [dict(seq=True), dict(seq=-1), dict(state={'pose':[0.,float('nan'),0.]}), dict(state={'pose':[0,0]})]:
            with self.assertRaises(ValueError): protocol.telemetry(dict(value, **change), 'run')


if __name__ == '__main__':
    unittest.main()
