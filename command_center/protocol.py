"""Bounded command validation and persistent idempotency at the robot edge."""
import json
import math
import sqlite3
import uuid


def validate_command(command, run_id, now):
    if str(uuid.UUID(command['request_id'])) != command['request_id']: raise ValueError('Invalid request ID')
    if command.get('run_id') != run_id or command.get('robot_id') != 'husky_01': raise ValueError('Wrong run or robot')
    if command.get('action') not in ('start', 'cancel', 'hold', 'resume'): raise ValueError('Unknown action')
    if command['action'] == 'start' and command.get('route') not in ('quick', 'inspection'): raise ValueError('Unknown route')
    expiry = command['expires_wall']
    if not isinstance(expiry, (int, float)) or not math.isfinite(expiry) or expiry <= now or expiry > now+15.:
        raise ValueError('Command expired or expiry is outside the 15-second window')


class CommandLedger:
    def __init__(self, path):
        self.db = sqlite3.connect(path)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('CREATE TABLE IF NOT EXISTS commands (id TEXT PRIMARY KEY, command TEXT, result TEXT)')

    def reserve(self, command):
        result = dict(request_id=command['request_id'], status='DISPATCHED', reason='Awaiting onboard acceptance')
        cur = self.db.execute('INSERT OR IGNORE INTO commands VALUES (?,?,?)',
                              (command['request_id'], json.dumps(command), json.dumps(result)))
        self.db.commit()
        return bool(cur.rowcount)

    def finish(self, result):
        self.db.execute('UPDATE commands SET result=? WHERE id=?', (json.dumps(result), result['request_id']))
        self.db.commit()

    def results(self):
        return [json.loads(row[0]) for row in self.db.execute('SELECT result FROM commands ORDER BY rowid DESC LIMIT 24')]
