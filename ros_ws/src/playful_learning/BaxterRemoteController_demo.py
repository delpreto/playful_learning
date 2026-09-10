"""Serve the browser demo on the NEW computer. Python 3.8+, no packages."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from BaxterRemoteController_client import BaxterRemoteController

ASSETS = Path(__file__).resolve().parent / 'demo'
CAMERAS = ('left_hand_camera', 'right_hand_camera')


class Handler(BaseHTTPRequestHandler):
    def respond(self, status, content_type, data):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy',
                         "default-src 'self'; img-src 'self' blob:; "
                         "style-src 'self' 'unsafe-inline'; frame-ancestors 'none'")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        if self.path.startswith('/camera/'):
            camera_name = self.path[len('/camera/'):-len('.png')]
            if not self.path.endswith('.png') or camera_name not in CAMERAS:
                return self.respond(404, 'application/json', b'{"message":"Unknown camera"}')
            port = self.server.server_port
            hosts = ('127.0.0.1:%s' % port, 'localhost:%s' % port)
            if (self.headers.get('Host') not in hosts or
                    self.headers.get('Sec-Fetch-Site', 'same-origin') not in ('same-origin', 'none') or
                    self.headers.get('Origin') not in [None] + ['http://' + h for h in hosts]):
                return self.respond(403, 'application/json', b'{"message":"Use the local demo page"}')
            try:
                data = self.server.client.get_camera_frame(camera_name)
            except Exception as exc:
                error = json.dumps({'message': str(exc)}).encode('utf-8')
                return self.respond(503, 'application/json', error)
            return self.respond(200, 'image/png', data)
        files = {'/': ('index.html', 'text/html; charset=utf-8'),
                 '/index.html': ('index.html', 'text/html; charset=utf-8'),
                 '/style.css': ('style.css', 'text/css; charset=utf-8'),
                 '/app.js': ('app.js', 'text/javascript; charset=utf-8')}
        if self.path not in files:
            return self.respond(404, 'text/plain', b'Not found')
        filename, content_type = files[self.path]
        self.respond(200, content_type, (ASSETS / filename).read_bytes())

    def do_POST(self):
        request_id = None
        try:
            # Accept only this local page, not cross-site forms or DNS rebinding.
            port = self.server.server_port
            hosts = ('127.0.0.1:%s' % port, 'localhost:%s' % port)
            if (self.headers.get('Host') not in hosts or
                    self.headers.get('Origin') not in ['http://' + h for h in hosts] or
                    self.headers.get('Content-Type', '').split(';')[0] != 'application/json'):
                return self.respond(403, 'text/plain', b'Use the local demo page')
            if self.path != '/rpc':
                return self.respond(404, 'text/plain', b'Not found')
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 3 * 1024 * 1024:
                raise ValueError('Invalid request length')
            self.connection.settimeout(5)
            request = json.loads(self.rfile.read(length).decode('utf-8'))
            request_id = request['id']
            result = self.server.client.call(request['method'], **request.get('params', {}))
            reply = {'jsonrpc': '2.0', 'id': request_id, 'result': result}
        except Exception as exc:
            reply = {'jsonrpc': '2.0', 'id': request_id,
                     'error': {'code': -32000, 'message': str(exc)}}
        self.respond(200, 'application/json', json.dumps(reply, allow_nan=False).encode('utf-8'))

    def log_message(self, *args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server', required=True, help='Baxter desktop URL, e.g. http://192.168.1.50:8765')
    parser.add_argument('--port', type=int, default=8000)
    args = parser.parse_args()
    # Browser polling renews the lease; closing the page must let it expire.
    with BaxterRemoteController(args.server, heartbeat=False) as client:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
        server.client = client
        print('Open http://127.0.0.1:%s on this computer.' % args.port)
        try:
            server.serve_forever(poll_interval=0.2)
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()


if __name__ == '__main__':
    main()
