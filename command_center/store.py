"""Web projection of durably received telemetry; never opens edge or truth logs."""
import base64
import json
import math
from pathlib import Path
import sqlite3
import threading
import time


def read_json(path):
    try: return json.loads(Path(path).read_text())
    except (OSError, ValueError): return {}


class Store:
    def __init__(self, run):
        self.run = Path(run); self.lock = threading.RLock()
        self.db = sqlite3.connect(self.run/'dashboard.sqlite', check_same_thread=False)
        self.db.executescript('''PRAGMA journal_mode=WAL;
            CREATE TABLE IF NOT EXISTS snapshots (seq INTEGER PRIMARY KEY, stamp REAL, received REAL, data TEXT);
            CREATE TABLE IF NOT EXISTS cameras (seq INTEGER PRIMARY KEY, stamp REAL, jpeg BLOB);
            CREATE TABLE IF NOT EXISTS scans (seq INTEGER PRIMARY KEY, stamp REAL, points TEXT);
            CREATE TABLE IF NOT EXISTS events (id INTEGER PRIMARY KEY, seq INTEGER, stamp REAL, received REAL, kind TEXT, message TEXT);
            CREATE TABLE IF NOT EXISTS commands (id TEXT PRIMARY KEY, created REAL, expires REAL, status TEXT, request TEXT, reason TEXT);
            CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT);
        ''')
        version = self.db.execute("SELECT value FROM meta WHERE key='projection_version'").fetchone()
        if version != ('2',):
            # Rebuild earlier projections from their complete durable source. Keep
            # operator commands and center-observed link events across migration.
            for table in ('snapshots', 'cameras', 'scans'): self.db.execute('DELETE FROM '+table)
            self.db.execute("DELETE FROM events WHERE kind NOT IN ('command','link')")
            self.db.execute("INSERT OR REPLACE INTO meta VALUES ('cursor','0')")
            self.db.execute("INSERT OR REPLACE INTO meta VALUES ('projection_version','2')")
            self.db.commit()
        row = self.db.execute("SELECT value FROM meta WHERE key='cursor'").fetchone()
        self.cursor = int(row[0]) if row else 0
        row = self.db.execute('SELECT data FROM snapshots ORDER BY seq DESC LIMIT 1').fetchone()
        self.latest = json.loads(row[0]) if row else None
        self.connection = None

    def event(self, seq, stamp, received, kind, message):
        self.db.execute('INSERT INTO events(seq,stamp,received,kind,message) VALUES (?,?,?,?,?)',
                        (seq, stamp, received, kind, message))

    def rebuild_transition(self, seq):
        row = self.db.execute('SELECT data FROM snapshots WHERE seq=?', (seq,)).fetchone()
        if not row: return
        item = json.loads(row[0])
        previous = self.db.execute('SELECT data FROM snapshots WHERE seq<? ORDER BY seq DESC LIMIT 1', (seq,)).fetchone()
        old = json.loads(previous[0]) if previous else {}
        self.db.execute("DELETE FROM events WHERE seq=? AND kind IN ('critical','warning','recovered','safety','mission')", (seq,))
        old_issues = {i['code']: i for i in (old.get('health') or {}).get('issues', [])}
        issues = {i['code']: i for i in item['health']['issues']}
        def event(kind, message): self.event(seq, item['stamp'], item['received'], kind, message)
        for code in sorted(issues.keys()-old_issues.keys()): event(issues[code]['severity'], issues[code]['message'])
        for code in sorted(old_issues.keys()-issues.keys()): event('recovered', code.replace('_', ' ')+' recovered')
        for key, field in [('safety', 'held'), ('mission', 'status')]:
            current, previous = item.get(key) or {}, old.get(key) or {}
            if current.get(field) != previous.get(field):
                message = ('Motion held: '+current.get('reason', '') if current.get('held') else 'Motion hold released') if key == 'safety' else 'Mission '+str(current.get('status'))
                event(key, message)

    def ingest(self):
        path = self.run/'twin-history.sqlite'
        if not path.exists(): return
        with self.lock:
            source = sqlite3.connect(path.as_uri()+'?mode=ro', uri=True, timeout=1.)
            try:
                rows = source.execute('SELECT rowid, seq, received, payload FROM packets WHERE rowid>? ORDER BY rowid LIMIT 64',
                                      (self.cursor,)).fetchall()
            except sqlite3.OperationalError:
                rows = []  # The center may still be creating its schema.
            finally: source.close()
            changed = set()
            for rowid, seq, received, raw in rows:
                packet = json.loads(raw); self.cursor = rowid
                if packet.get('run_id') != self.run.name: continue
                item = {k: packet.get(k) for k in ('stamp', 'state', 'health', 'safety', 'transport', 'mission', 'planned_path', 'drive_status')}
                item.update(seq=seq, received=received, robot_id=packet['robot_id'])
                item['health'] = item['health'] or dict(status='UNKNOWN', sensors={}, issues=[])
                cam = packet.get('camera')
                if cam:
                    self.db.execute('INSERT OR IGNORE INTO cameras VALUES (?,?,?)', (seq, cam['stamp'], base64.b64decode(cam['data'])))
                scan = packet.get('scan')
                if scan:
                    x, y, yaw = scan['pose']; x += .48*math.cos(yaw); y += .48*math.sin(yaw)
                    points = [[round(x+r*math.cos(yaw+scan['angle_min']+i*scan['angle_increment']), 2),
                                        round(y+r*math.sin(yaw+scan['angle_min']+i*scan['angle_increment']), 2)]
                                       for i, r in enumerate(scan['ranges']) if r is not None and scan['min'] < r < scan['max']-.05]
                    self.db.execute('INSERT OR IGNORE INTO scans VALUES (?,?,?)', (seq, scan['stamp'], json.dumps(points)))
                for result in packet.get('command_results', []):
                    row = self.db.execute('SELECT status FROM commands WHERE id=?', (result['request_id'],)).fetchone()
                    if row and row[0] != result['status'] and result['status'] != 'DISPATCHED':
                        self.db.execute('UPDATE commands SET status=?,reason=? WHERE id=?', (result['status'], result['reason'], result['request_id']))
                        self.event(seq, item['stamp'], received, 'command', result['status']+': '+result['reason'])
                self.db.execute('INSERT OR IGNORE INTO snapshots VALUES (?,?,?,?)', (seq, item['stamp'], received, json.dumps(item, allow_nan=False)))
                changed.add(seq)
                successor = self.db.execute('SELECT MIN(seq) FROM snapshots WHERE seq>?', (seq,)).fetchone()[0]
                if successor is not None: changed.add(successor)
                if not self.latest or item['stamp'] >= self.latest['stamp']: self.latest = item
            for seq in sorted(changed): self.rebuild_transition(seq)
            self.db.execute("INSERT OR REPLACE INTO meta VALUES ('cursor',?)", (str(self.cursor),))
            self.db.execute("UPDATE commands SET status='UNCONFIRMED',reason='No robot acknowledgement before expiry; inspect mission status before issuing another command' WHERE status='PENDING' AND expires<?", (time.time(),))
            self.db.commit()

    def connected(self):
        status = read_json(self.run/'twin-status.json')
        live = (bool(self.latest) and time.time()-status.get('updated_wall', 0) < 2.5
                and status.get('transport_age_wall_s') is not None and status['transport_age_wall_s'] < 2.
                and status.get('sim_time', 0)-(self.latest or {}).get('stamp', -10.) < 2.
                and not status.get('simulation_paused_or_stalled', True))
        return live, status

    def connection_event(self):
        with self.lock:
            live, status = self.connected()
            if self.latest and live != self.connection:
                self.event(self.latest['seq'], self.latest['stamp'], time.time(), 'link',
                           'Telemetry live' if live else 'Telemetry stale; showing last received observations')
                self.db.commit(); self.connection = live

    def snapshot(self, seq=None):
        with self.lock:
            live, status = self.connected()
            item = self.latest
            if seq is not None:
                row = self.db.execute('SELECT data FROM snapshots WHERE seq<=? ORDER BY seq DESC LIMIT 1', (seq,)).fetchone()
                item = json.loads(row[0]) if row else None
            end = item['seq'] if item else 0
            if item:
                item = dict(item)
                camera = self.db.execute('SELECT stamp FROM cameras WHERE seq<=? ORDER BY seq DESC LIMIT 1', (end,)).fetchone()
                scan = self.db.execute('SELECT stamp,points FROM scans WHERE seq<=? ORDER BY seq DESC LIMIT 1', (end,)).fetchone()
                item['camera_stamp'] = camera[0] if camera else None
                item['scan_stamp'] = scan[0] if scan else None
                item['returns'] = json.loads(scan[1]) if scan else []
            bounds = self.db.execute('SELECT MIN(seq),MAX(seq),COUNT(*) FROM snapshots').fetchone()
            trail = [json.loads(r[0])[:2] for r in self.db.execute("SELECT json_extract(data,'$.state.pose') FROM snapshots WHERE seq<=? ORDER BY seq DESC LIMIT 900", (end,))][::-1]
            events = [dict(id=r[0], seq=r[1], stamp=r[2], received=r[3], kind=r[4], message=r[5])
                      for r in self.db.execute('SELECT * FROM events WHERE seq<=? ORDER BY seq DESC,id DESC LIMIT 60', (end,))]
            commands = [dict(id=r[0], created=r[1], expires=r[2], status=r[3], request=json.loads(r[4]), reason=r[5])
                        for r in self.db.execute('SELECT * FROM commands ORDER BY created DESC LIMIT 8')] if seq is None else []
            return dict(run_id=self.run.name, live=live and seq is None, replay=seq is not None, robot=item, trail=trail,
                        events=events, commands=commands, bounds=dict(first=bounds[0], last=bounds[1], count=bounds[2]),
                        center=status if seq is None else {}, simulation_time=status.get('sim_time', 0) if seq is None else (item or {}).get('stamp', 0))

    def camera(self, seq):
        with self.lock:
            row = self.db.execute('SELECT jpeg FROM cameras WHERE seq<=? ORDER BY seq DESC LIMIT 1', (seq,)).fetchone()
            return bytes(row[0]) if row else None

    def add_command(self, command):
        with self.lock:
            existing = self.db.execute('SELECT request FROM commands WHERE id=?', (command['request_id'],)).fetchone()
            if existing:
                previous = json.loads(existing[0])
                if any(previous.get(k) != command.get(k) for k in ('action', 'route', 'robot_id', 'run_id')):
                    raise ValueError('Request ID already belongs to a different command')
                return False
            self.db.execute('INSERT INTO commands VALUES (?,?,?,?,?,?)',
                            (command['request_id'], time.time(), command['expires_wall'], 'PENDING', json.dumps(command), 'Awaiting robot acknowledgement'))
            self.db.commit(); return True
