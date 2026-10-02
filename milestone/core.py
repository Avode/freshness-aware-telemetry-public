"""ROS-independent geometry, occupancy mapping and durable delivery primitives."""
from collections import deque
import json
import math
import sqlite3
import time
import numpy as np

ANCHOR = (-12., -33., -math.pi / 2)  # Declared surveyed start, never live truth.
LIDAR_OFFSET = (.48, 0., .42)

def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def estate_pose(x, y, yaw):
    ax, ay, a = ANCHOR
    return [ax + math.cos(a)*x - math.sin(a)*y,
            ay + math.sin(a)*x + math.cos(a)*y, wrap(a + yaw)]


def capture_through(packet):
    """Envelope watermark covering its observations, without changing their stamps.

    ROS sensor callbacks and /clock can be serviced in different orders. A sensor
    from the same simulation clock may therefore be ahead of the local /clock
    cache when this envelope is assembled.
    """
    stamps = [packet['stamp']]
    for key in ('state', 'imu', 'scan', 'camera', 'slam', 'planned_path', 'mission', 'health', 'safety'):
        value = packet.get(key)
        if isinstance(value, dict) and 'stamp' in value:
            stamp = value['stamp']
            if type(stamp) not in (int, float) or not math.isfinite(stamp):
                raise ValueError('Invalid observation capture timestamp')
            stamps.append(stamp)
    return max(stamps)

class PoseHistory:
    def __init__(self): self.items = deque(maxlen=300)

    def add(self, stamp, value):
        if self.items and stamp < self.items[-1][0]: self.items.clear()
        self.items.append((stamp, value))

    def at(self, stamp, max_gap=.2):
        for (ta, a), (tb, b) in zip(self.items, list(self.items)[1:]):
            if ta <= stamp <= tb and tb - ta <= max_gap:
                f = (stamp-ta)/(tb-ta) if tb > ta else 0.
                return [a[0]+f*(b[0]-a[0]), a[1]+f*(b[1]-a[1]), wrap(a[2]+f*wrap(b[2]-a[2]))]
        return None  # Never silently attach a scan to an unrelated latest pose.

class Outbox:
    def __init__(self, path, limit=1500):
        self.db = sqlite3.connect(path)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('CREATE TABLE IF NOT EXISTS outbox (seq INTEGER PRIMARY KEY AUTOINCREMENT, payload TEXT NOT NULL)')
        self.limit = limit
        self.dropped = 0
        self._attempts = {}
        self._history_cursor = 0

    def put(self, payload):
        cur = self.db.execute('INSERT INTO outbox(payload) VALUES (?)', (json.dumps(payload, allow_nan=False),))
        seq = cur.lastrowid
        n = self.count()
        if n > self.limit:
            self.db.execute('DELETE FROM outbox WHERE seq IN (SELECT seq FROM outbox ORDER BY seq LIMIT ?)', (n-self.limit,))
            self.dropped += n-self.limit
        self.db.commit()
        return seq

    def batch(self, size=12):
        return [(seq, json.loads(payload)) for seq, payload in self.db.execute('SELECT seq,payload FROM outbox ORDER BY seq LIMIT ?', (size,))]

    def scheduled_batch(self, size=12, now=None):
        """Reserve fresh slots plus fair history retries; never discard to advance.

        Missing ACKs cannot monopolize all send slots. Retry timing is transient:
        restarting may duplicate delivery, but all unacknowledged rows remain on
        disk. Acknowledgement, or the explicit capacity/drop policy, removes rows.
        """
        if size <= 0:
            return []
        now = time.monotonic() if now is None else now
        ids = [row[0] for row in self.db.execute('SELECT seq FROM outbox ORDER BY seq')]
        present = set(ids)
        self._attempts = {seq: value for seq, value in self._attempts.items() if seq in present}
        due = [seq for seq in ids if self._attempts.get(seq, (0, float('-inf')))[1] <= now]
        if not due:
            return []
        recent = due[-min(4, size):][::-1]
        remaining = [seq for seq in due if seq not in recent]
        history = [seq for seq in remaining if seq > self._history_cursor]
        history += [seq for seq in remaining if seq <= self._history_cursor]
        history = history[:size-len(recent)]
        if history:
            self._history_cursor = history[-1]
        selected = recent + history
        result = []
        for seq in selected:
            row = self.db.execute('SELECT payload FROM outbox WHERE seq=?', (seq,)).fetchone()
            if row is None:
                continue
            attempt = min(self._attempts.get(seq, (0, 0.))[0]+1, 5)
            self._attempts[seq] = (attempt, now+min(15., 2.**(attempt-1)))
            result.append((seq, json.loads(row[0])))
        return result

    def ack(self, seq):
        self.db.execute('DELETE FROM outbox WHERE seq=?', (seq,)); self.db.commit()
        self._attempts.pop(seq, None)

    def count(self): return self.db.execute('SELECT COUNT(*) FROM outbox').fetchone()[0]

class Occupancy:
    """Incremental scan map, not SLAM: it inherits localization drift."""
    def __init__(self, resolution=.25, width=800, height=760, origin=(-100., -95.)):
        self.resolution, self.width, self.height, self.origin = resolution, width, height, origin
        self.logodds = np.zeros((height, width), np.float32)
        self.seen = np.zeros((height, width), bool)
        self.scans = 0

    def cell(self, x, y):
        return int(math.floor((x-self.origin[0])/self.resolution)), int(math.floor((y-self.origin[1])/self.resolution))

    def integrate(self, scan):
        x, y, yaw = scan['pose']
        x += LIDAR_OFFSET[0]*math.cos(yaw)
        y += LIDAR_OFFSET[0]*math.sin(yaw)
        points = []
        for i, r in enumerate(scan['ranges']):
            # None represents a valid max-range/no-return ray; invalid rays are omitted.
            if r is not None and r < scan['min']: continue
            hit = r is not None and r < scan['max']-.05
            distance = min(r if r is not None else scan['max'], scan['max'])
            a = yaw + scan['angle_min'] + i*scan['angle_increment']
            ex, ey = x+distance*math.cos(a), y+distance*math.sin(a)
            n = max(2, int(distance/self.resolution*1.5))
            xs = np.floor((np.linspace(x, ex, n)-self.origin[0])/self.resolution).astype(int)
            ys = np.floor((np.linspace(y, ey, n)-self.origin[1])/self.resolution).astype(int)
            valid = (xs>=0)&(xs<self.width)&(ys>=0)&(ys<self.height)
            fx, fy = xs[:-1][valid[:-1]], ys[:-1][valid[:-1]]
            # Several samples can land in the endpoint cell. Do not mark a hit
            # cell free before applying its occupied evidence.
            if valid[-1]:
                keep = (fx != xs[-1]) | (fy != ys[-1])
                fx, fy = fx[keep], fy[keep]
            self.logodds[fy, fx] = np.maximum(-4, self.logodds[fy, fx]-.35)
            self.seen[fy, fx] = True
            if valid[-1]:
                self.seen[ys[-1], xs[-1]] = True
                self.logodds[ys[-1], xs[-1]] = np.clip(self.logodds[ys[-1], xs[-1]] + (.9 if hit else -.35), -4, 4)
            if hit: points.append([ex, ey, .6])
        self.scans += 1
        return points

    def data(self):
        values = np.rint(100/(1+np.exp(-self.logodds))).astype(np.int8)
        values[~self.seen] = -1
        return values
