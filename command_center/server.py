"""Loopback-only HTTP UI; operator commands cross the telemetry link, lab faults are explicit services."""
import argparse
import fcntl
from concurrent.futures import Future
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import queue
import secrets
import signal
import threading
import time
from urllib.parse import urlparse, parse_qs
import uuid
from .store import Store, read_json
from .protocol import validate_command

ROOT = Path(__file__).resolve().parents[1]
STATIC = Path(__file__).parent/'static'


def make_bridge():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    class Bridge(Node):
        def __init__(self):
            super().__init__('web_command_gateway')
            self.queue = queue.Queue(maxsize=32)
            self.pub = self.create_publisher(String, '/unit/husky/command', 10)
            self.fault_clients = {name: self.create_client(SetBool, topic) for name, topic in {
                'camera_freeze': '/simulation/camera_freeze', 'imu_dropout': '/simulation/imu_dropout',
                'lidar_dropout': '/simulation/lidar_dropout', 'position_dropout': '/simulation/localization_enabled',
                'position_jump': '/simulation/position_jump',
                'link_down': '/unit/husky/set_link'}.items()}
            self.create_timer(.05, self.tick)

        def submit(self, kind, value):
            future = Future()
            self.queue.put_nowait((kind, value, future))
            return future

        def tick(self):
            while not self.queue.empty():
                kind, value, answer = self.queue.get_nowait()
                if kind == 'command':
                    self.pub.publish(String(data=json.dumps(value))); answer.set_result(dict(queued=True))
                else:
                    client = self.fault_clients[value['fault']]
                    if not client.service_is_ready():
                        answer.set_result(dict(success=False, message='Simulation service unavailable')); continue
                    active = value['active']
                    if value['fault'] in ('position_dropout', 'link_down'): active = not active
                    pending = client.call_async(SetBool.Request(data=active))
                    def done(f, answer=answer):
                        try:
                            response = f.result(); answer.set_result(dict(success=response.success, message=response.message))
                        except Exception as exc: answer.set_exception(exc)
                    pending.add_done_callback(done)
    rclpy.init(); return Bridge()


def handler_type(store, bridge, token, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            if args and str(args[1]) not in ('200', '202', '204'): super().log_message(fmt, *args)

        def reply(self, code, data, mime='application/json'):
            if not isinstance(data, bytes): data = json.dumps(data, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', mime); self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store'); self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            try: self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError): pass

        def valid_host(self):
            return self.headers.get('Host') in (f'127.0.0.1:{port}', f'localhost:{port}')

        def do_GET(self):
            if not self.valid_host(): self.reply(403, dict(error='Loopback host required')); return
            parsed = urlparse(self.path); query = parse_qs(parsed.query)
            try:
                if parsed.path == '/api/session':
                    self.reply(200, dict(token=token, writable=bridge is not None)); return
                if parsed.path == '/api/state':
                    seq = int(query['seq'][0]) if 'seq' in query else None
                    self.reply(200, store.snapshot(seq)); return
                if parsed.path == '/api/camera':
                    seq = int(query.get('seq', ['0'])[0]); camera = store.camera(seq)
                    self.reply(200 if camera else 404, camera or b'', 'image/jpeg'); return
                if parsed.path == '/api/scene':
                    source = store.run/'config/scene.json'
                    self.reply(200, read_json(source if source.exists() else ROOT/'config/scene.json')); return
                files = {'/': ('index.html', 'text/html; charset=utf-8'), '/app.js': ('app.js', 'text/javascript'),
                         '/style.css': ('style.css', 'text/css'), '/favicon.ico': (None, 'image/x-icon')}
                if parsed.path in files:
                    name, mime = files[parsed.path]
                    self.reply(200 if name else 204, (STATIC/name).read_bytes() if name else b'', mime); return
                self.reply(404, dict(error='Not found'))
            except (ValueError, TypeError): self.reply(400, dict(error='Invalid request'))

        def do_POST(self):
            origin = self.headers.get('Origin')
            if (not self.valid_host() or self.headers.get('X-FleetScope-Token') != token
                    or origin not in (None, f'http://127.0.0.1:{port}', f'http://localhost:{port}')):
                self.reply(403, dict(error='Same-origin command token required')); return
            if bridge is None: self.reply(409, dict(error='Recorded run is read-only')); return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 4096: raise ValueError('Request body must be 1–4096 bytes')
                value = json.loads(self.rfile.read(size))
                if not isinstance(value, dict): raise ValueError('Request body must be a JSON object')
                if self.path == '/api/commands':
                    live, _ = store.connected()
                    if not live: self.reply(409, dict(error='Telemetry is stale. Command not sent.')); return
                    command = dict(request_id=value.get('request_id', str(uuid.uuid4())), action=value['action'],
                                   route=value.get('route') if value['action']=='start' else None,
                                   robot_id='husky_01', run_id=store.run.name, expires_wall=time.time()+8.)
                    validate_command(command, store.run.name, time.time())
                    if store.add_command(command): bridge.submit('command', command).result(timeout=2.)
                    self.reply(202, dict(request_id=command['request_id'], message='Track robot acknowledgement in the command log')); return
                if self.path == '/api/simulation':
                    if value.get('fault') not in bridge.fault_clients or type(value.get('active')) is not bool:
                        raise ValueError('Unknown simulation fault or non-boolean active flag')
                    result = bridge.submit('fault', value).result(timeout=5.)
                    self.reply(200 if result['success'] else 503, result); return
                self.reply(404, dict(error='Unknown action'))
            except (ValueError, KeyError, TypeError) as exc: self.reply(400, dict(error=str(exc)))
            except (TimeoutError, queue.Full): self.reply(503, dict(error='Gateway acknowledgement timed out; inspect state before retrying'))
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=Path(os.environ.get('FLEETSCOPE_RUN', Path(os.environ.get('FLEETSCOPE_RUNS', ROOT/'runs'))/'autonomy-latest')).expanduser())
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--read-only', action='store_true', help='Browse a recorded run without ROS or commands')
    args = parser.parse_args()
    run = args.run.expanduser().resolve()
    if not run.is_dir(): raise SystemExit('Run directory does not exist')
    lock = (run/'dashboard.lock').open('w')
    try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError: raise SystemExit('A dashboard already owns this run. Open its existing URL or stop it first.')
    store = Store(run); bridge = None if args.read_only else make_bridge()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_type(store, bridge, secrets.token_urlsafe(32), args.port))
    server.daemon_threads = True
    worker = threading.Thread(target=server.serve_forever, daemon=True); worker.start()
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopping.set()); signal.signal(signal.SIGINT, lambda *_: stopping.set())
    print(f'FleetScope command center: http://127.0.0.1:{args.port}', flush=True)
    try:
        while not stopping.is_set():
            if bridge:
                import rclpy
                rclpy.spin_once(bridge, timeout_sec=.05)
            else: stopping.wait(.1)
            store.ingest(); store.connection_event()
    finally:
        server.shutdown(); server.server_close(); worker.join(timeout=2.)
        if bridge:
            bridge.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
        store.db.close()


if __name__ == '__main__': main()
