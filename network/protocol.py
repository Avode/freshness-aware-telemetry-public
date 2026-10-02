"""Small, ROS-independent checks for the robot telemetry MQTT boundary."""
import json
import math
import time
import uuid
from command_center.protocol import validate_command

SITE = 'plantation_estate'
ROBOT = 'husky_01'
PREFIX = f'fleetscope/v1/{SITE}/{ROBOT}/'
MAX_PAYLOAD = 8 * 1024 * 1024


def decode(raw, maximum=MAX_PAYLOAD):
    if len(raw) > maximum:
        raise ValueError('Payload too large')
    def invalid(value):
        raise ValueError('Non-finite JSON number')
    def finite_float(raw_number):
        number = float(raw_number)
        if not math.isfinite(number):
            raise ValueError('Non-finite JSON number')
        return number
    try:
        value = json.loads(raw, parse_constant=invalid, parse_float=finite_float)
    except RecursionError as exc:
        raise ValueError('JSON nesting too deep') from exc
    if not isinstance(value, dict):
        raise ValueError('Expected JSON object')
    return value


def identity(value, run_id):
    if value.get('run_id') != run_id or value.get('robot_id') != ROBOT:
        raise ValueError('Wrong run or robot')
    if 'site_id' in value and value['site_id'] != SITE:
        raise ValueError('Wrong site')


def acknowledgement(value, run_id, retained=False):
    identity(value, run_id)
    if retained or type(value.get('schema')) is not int or value['schema'] != 1:
        raise ValueError('Invalid ACK schema or retained ACK')
    if type(value.get('seq')) is not int or value['seq'] <= 0:
        raise ValueError('Invalid ACK sequence')
    return {'run_id': run_id, 'seq': value['seq']}


def command(value, run_id, now, retained=False):
    identity(value, run_id)
    if retained:
        raise ValueError('Retained commands are forbidden')
    if type(value.get('expires_wall')) not in (int, float):
        raise ValueError('Invalid expiry')
    validate_command(value, run_id, now)
    if 'issued_wall' in value:
        issued = value['issued_wall']
        if (type(issued) not in (int, float) or not math.isfinite(issued)
                or abs(now-issued) > 2. or value['expires_wall'] <= issued):
            raise ValueError('Command clock skew or age exceeds two seconds')
    return value


def telemetry(value, run_id):
    identity(value, run_id)
    if type(value.get('schema')) is not int or value['schema'] != 1:
        raise ValueError('Unknown telemetry schema')
    if type(value.get('seq')) is not int or value['seq'] <= 0:
        raise ValueError('Invalid sequence')
    if type(value.get('stamp')) not in (int, float) or not math.isfinite(value['stamp']):
        raise ValueError('Invalid capture time')
    pose = value.get('state', {}).get('pose')
    if not isinstance(pose, list) or len(pose) != 3 or any(type(n) not in (int, float) or not math.isfinite(n) for n in pose):
        raise ValueError('Invalid reported pose')
    return value


class SessionHeartbeat:
    def __init__(self, run_id, site):
        self.run_id = run_id
        self.site = site
        self.session_id = str(uuid.uuid4())
        self.seq = 0
        self.previous_clock = None
        self.last_advance = None

    def sample(self, capture_time, latest_seq, wall=None, monotonic=None):
        wall = time.time() if wall is None else wall
        monotonic = time.monotonic() if monotonic is None else monotonic
        if self.previous_clock is not None and capture_time < self.previous_clock:
            # A clock reset requires a new run, not a silent replay of old state.
            raise ValueError('Capture clock moved backwards; start a new run')
        if self.previous_clock is not None and capture_time > self.previous_clock:
            self.last_advance = monotonic
        self.previous_clock = capture_time
        self.seq += 1
        robot = next(r for r in self.site['robots'] if r['robot_id'] == ROBOT)
        return dict(schema=1, protocol_version=1, site_id=SITE, robot_id=ROBOT,
                    run_id=self.run_id, session_id=self.session_id,
                    heartbeat_seq=self.seq, online=True, edge_wall_time=wall,
                    capture_time=capture_time,
                    clock_advancing=self.last_advance is not None and monotonic-self.last_advance < 2.,
                    latest_seq=latest_seq, map_id=self.site['map_id'],
                    map_sha256=self.site['map_sha256'], frame_id='estate',
                    calibration_id=robot['calibration_id'],
                    lidar_from_reported_base=robot['lidar_from_reported_base'])
