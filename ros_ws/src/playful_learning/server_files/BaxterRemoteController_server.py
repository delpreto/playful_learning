#!/usr/bin/env python
"""JSON/HTTP bridge for Baxter. Standard libraries only; Python 2.7 or 3.

Run with --simulate to try the API without ROS. See ../README.md.
"""
from __future__ import print_function

import argparse
import base64
import binascii
import collections
import hashlib
import json
import math
import threading
import time
import uuid

from BaxterCameras import CAMERA_NAMES

try:
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
    from SocketServer import ThreadingMixIn
    STRINGS = (basestring,)
    NUMBERS = (int, long, float)
    INTEGERS = (int, long)
except ImportError:
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn
    STRINGS = (str,)
    NUMBERS = (int, float)
    INTEGERS = (int,)

LIMBS = ('left', 'right')
JOINTS = ('s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2')
# Required and optional named arguments. No arbitrary controller attribute access.
METHODS = {
    'get_state': ((), ()), 'get_end_effector_poses': ((), ()),
    'get_head_state': ((), ()), 'get_head_pan_rad': ((), ()),
    'get_head_tilt_rad': ((), ()), 'set_head_tilt_rad': (('angle_rad',), ()),
    'set_head_pan_rad': (('angle_rad',), ('speed_percent', 'timeout_s', 'tolerance_rad')),
    'nod_head': ((), ('times', 'internod_delay_s')),
    'set_halo_led': (('red_percent', 'green_percent'), ()),
    'set_sonar_leds': (('led_states',), ()),
    'show_screen_color': (('color_rgb',), ()),
    'show_screen_image_rgb': (('width', 'height', 'rgb_base64'), ()),
    'get_joint_angles_rad': ((), ()), 'get_joint_velocities_rad_s': ((), ()),
    'get_joint_efforts_Nm': ((), ()),
    'is_movement_in_progress': ((), ('limb_name',)),
    'get_gripper_position_open_percent': (('limb_name',), ()),
    'get_gripper_force_percent': (('limb_name',), ()),
    'is_gripper_grasping': (('limb_name',), ()),
    'is_gripper_moving': (('limb_name',), ()),
    'trajectory_succeeded': (('limb_name',), ()),
    'get_operation': (('operation_id',), ()),
    'heartbeat': ((), ()), 'release_control': ((), ()),
    'abort_movement': ((), ('limb_name',)), 'stop_gripper': (('limb_name',), ()),
    'move_to_neutral': ((), ('limb_name', 'timeout_s', 'tolerance_rad')),
    'move_to_resting': ((), ('limb_name', 'timeout_s', 'tolerance_rad')),
    'move_to_joint_angles_rad': (('joint_angles_rad_byLimb',), ('timeout_s', 'tolerance_rad')),
    'move_to_gripper_pose': (('gripper_position_m_byLimb', 'gripper_orientation_quaternion_wijk_byLimb'),
                             ('seed_joint_angles_rad_byLimb', 'timeout_s', 'tolerance_rad')),
    'get_joint_angles_rad_for_gripper_pose': (('limb_name', 'gripper_position_m', 'gripper_orientation_quaternion_wijk'),
                                            ('seed_joint_angles_rad',)),
    'build_trajectory_from_joint_angles': (('limb_name', 'times_from_start_s', 'joint_angles_rad'),
                                         ('goal_time_tolerance_s',)),
    'build_trajectory_from_gripper_poses': (('limb_name', 'times_from_start_s', 'gripper_positions_m',
                                          'gripper_orientations_quaternion_wijk'),
                                         ('goal_time_tolerance_s', 'initial_seed_joint_angles_rad')),
    # 'run_trajectory': ((), ('limb_names',)),
    'move_gripper': (('limb_name', 'gripper_open_percent'), ('force_threshold_percent',)),
    'open_gripper': (('limb_name',), ()), 'close_gripper': (('limb_name',), ()),
    'calibrate_gripper': (('limb_name',), ()),
    'jog_joint': (('limb_name', 'joint_name', 'delta_rad'), ()),
    'jog_endpoint': (('limb_name', 'axis', 'delta'), ()),
    'jog_gripper': (('limb_name', 'delta_percent'), ())}
OPERATIONS = set(name for name in METHODS if name.startswith(('move_', 'build_', 'jog_')))
OPERATIONS.update(('run_trajectory', 'open_gripper', 'close_gripper', 'calibrate_gripper',
                   'get_joint_angles_rad_for_gripper_pose'))
HEAD_MOTIONS = set(('set_head_pan_rad', 'nod_head'))
HEAD_OPERATIONS = HEAD_MOTIONS | set(('set_halo_led', 'set_sonar_leds',
                                     'show_screen_color', 'show_screen_image_rgb'))
OPERATIONS.update(HEAD_OPERATIONS)


def finite_number(value):
    return (isinstance(value, NUMBERS) and not isinstance(value, bool)
            and not math.isnan(value) and not math.isinf(value))


def check_angles(limb, angles, full=False):
    if isinstance(angles, list):
        values = angles if len(angles) == 7 else []
    elif isinstance(angles, dict):
        names = set(limb + '_' + j for j in JOINTS)
        if not set(angles).issubset(names) or (full and set(angles) != names):
            raise ValueError('Use fully qualified joint names, e.g. left_s0')
        values = list(angles.values())
    else:
        values = []
    if not values or not all(finite_number(v) and abs(v) <= 4 for v in values):
        raise ValueError('Joint angles must be finite radians; lists must contain seven angles')


def validate(method, p):
    required, optional = METHODS[method]
    if not isinstance(p, dict) or any(key not in p for key in required):
        raise ValueError('Missing named arguments: ' + ', '.join(required))
    if set(p) - set(required + optional + ('should_print',)):
        raise ValueError('Unexpected named argument')
    p.pop('should_print', None)
    if method == 'set_head_tilt_rad':
        raise NotImplementedError('Baxter supports nodding, not a commanded tilt angle; use nod_head()')
    for name, low, high in [('angle_rad', -1.39, 1.39), ('speed_percent', 1, 100),
                            ('red_percent', 0, 100), ('green_percent', 0, 100),
                            ('internod_delay_s', 0, 5)]:
        if name in p and (not finite_number(p[name]) or not low <= p[name] <= high):
            raise ValueError('%s must be between %s and %s' % (name, low, high))
    if 'times' in p and (not isinstance(p['times'], INTEGERS) or isinstance(p['times'], bool)
                         or not 1 <= p['times'] <= 10):
        raise ValueError('times must be an integer between one and ten')
    if method == 'set_sonar_leds':
        leds = p['led_states']
        if isinstance(leds, list):
            states = len(leds) == 12 and all(isinstance(v, INTEGERS) and v in (0, 1) for v in leds)
            if len(leds) > 12 or (not states and any(not isinstance(v, INTEGERS) or
                    isinstance(v, bool) or not 0 <= v <= 11 for v in leds)):
                raise ValueError('Use up to twelve LED indices 0-11, or twelve 0/1 states')
        elif leds not in ('auto', 'on', 'off'):
            raise ValueError('Use auto, on, off, LED indices, or twelve 0/1 states')
    if method == 'show_screen_color':
        color = p['color_rgb']
        if (not isinstance(color, list) or len(color) != 3 or
                any(not isinstance(v, INTEGERS) or isinstance(v, bool) or not 0 <= v <= 255 for v in color)):
            raise ValueError('color_rgb must be three integer values between 0 and 255')
    if method == 'show_screen_image_rgb':
        for key, limit in [('width', 1024), ('height', 600)]:
            if not isinstance(p[key], INTEGERS) or isinstance(p[key], bool) or not 1 <= p[key] <= limit:
                raise ValueError('Image dimensions must be integers, at most 1024 by 600')
        data = p['rgb_base64']
        expected_bytes = p['width'] * p['height'] * 3
        if not isinstance(data, STRINGS) or len(data) != 4 * ((expected_bytes + 2) // 3):
            raise ValueError('Image data must be base64-encoded RGB pixels matching its dimensions')
        try:
            decoded = base64.b64decode(data.encode('ascii'))
        except (ValueError, TypeError, UnicodeError, binascii.Error):
            raise ValueError('Invalid base64 image data')
        if len(decoded) != expected_bytes or base64.b64encode(decoded).decode('ascii') != data:
            raise ValueError('Invalid base64 image data')
    limb = p.get('limb_name')
    if limb is not None and limb not in LIMBS:
        raise ValueError('limb_name must be left or right')
    if 'limb_name' in required and limb is None:
        raise ValueError('A limb_name is required')
    if p.get('limb_names') is not None:
        names = p['limb_names']
        if not isinstance(names, list) or not names or any(n not in LIMBS for n in names) or len(names) != len(set(names)):
            raise ValueError('limb_names must be a nonempty list of left/right')
    for name, low, high in [('timeout_s', 0.1, 120), ('tolerance_rad', 0.0001, 0.1),
                            ('goal_time_tolerance_s', 0, 10), ('gripper_open_percent', 0, 100),
                            ('force_threshold_percent', 0, 75)]:
        if name in p and (not finite_number(p[name]) or not low <= p[name] <= high):
            raise ValueError('%s must be between %s and %s' % (name, low, high))
    for name in ('joint_angles_rad_byLimb', 'seed_joint_angles_rad_byLimb',
                 'gripper_position_m_byLimb', 'gripper_orientation_quaternion_wijk_byLimb'):
        if p.get(name) is not None:
            if not isinstance(p[name], dict) or not p[name] or any(n not in LIMBS for n in p[name]):
                raise ValueError(name + ' must map left/right to values')
    if 'joint_angles_rad_byLimb' in p:
        for name, angles in p['joint_angles_rad_byLimb'].items():
            check_angles(name, angles)
    for name in ('seed_joint_angles_rad', 'initial_seed_joint_angles_rad'):
        if p.get(name) is not None:
            check_angles(limb, p[name], full=True)
    for name, angles in (p.get('seed_joint_angles_rad_byLimb') or {}).items():
        if angles is not None:
            check_angles(name, angles, full=True)
    vectors = []
    for key, size in [('gripper_position_m', 3), ('gripper_orientation_quaternion_wijk', 4)]:
        if key in p:
            vectors.append((p[key], size))
    for key, size in [('gripper_position_m_byLimb', 3), ('gripper_orientation_quaternion_wijk_byLimb', 4)]:
        for value in (p.get(key) or {}).values():
            vectors.append((value, size))
    if method == 'move_to_gripper_pose':
        if set(p['gripper_position_m_byLimb']) != set(p['gripper_orientation_quaternion_wijk_byLimb']):
            raise ValueError('Position and orientation limb names must match')
    if method.startswith('build_trajectory_'):
        times = p['times_from_start_s']
        if (not isinstance(times, list) or not 1 <= len(times) <= 1000 or
                not all(finite_number(t) and 0 < t <= 3600 for t in times) or
                any(a >= b for a, b in zip(times, times[1:]))):
            raise ValueError('Use 1-1000 strictly increasing positive times, at most 3600 seconds')
        if 'joint_angles_rad' in p:
            if not isinstance(p['joint_angles_rad'], list) or len(p['joint_angles_rad']) != len(times):
                raise ValueError('Waypoint and time counts must match')
            for angles in p['joint_angles_rad']:
                check_angles(limb, angles, full=True)
        else:
            for key, size in [('gripper_positions_m', 3), ('gripper_orientations_quaternion_wijk', 4)]:
                if not isinstance(p[key], list) or len(p[key]) != len(times):
                    raise ValueError('Pose and time counts must match')
                vectors.extend((v, size) for v in p[key])
    for vector, size in vectors:
        if not isinstance(vector, list) or len(vector) != size or not all(finite_number(v) for v in vector):
            raise ValueError('Pose vectors must contain %s finite numbers' % size)
        if size == 4 and abs(sum(v*v for v in vector) - 1) > 0.02:
            raise ValueError('Use a unit quaternion in [w, x, y, z] order')
    if method == 'jog_joint':
        if p['joint_name'] not in JOINTS and p['joint_name'] not in [limb + '_' + j for j in JOINTS]:
            raise ValueError('Unknown joint')
        if not finite_number(p['delta_rad']) or abs(p['delta_rad']) > math.radians(5):
            raise ValueError('Joint jog is limited to five degrees per click')
    if method == 'jog_endpoint':
        if p['axis'] not in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
            raise ValueError('Unknown pose axis')
        limit = 0.02 if p['axis'] in ('x', 'y', 'z') else math.radians(5)
        if not finite_number(p['delta']) or abs(p['delta']) > limit:
            raise ValueError('Pose jog is limited to 2 cm or 5 degrees per click')
    if method == 'jog_gripper' and (not finite_number(p['delta_percent']) or abs(p['delta_percent']) > 10):
        raise ValueError('Gripper jog is limited to ten percent per click')


class RemoteAPI(object):
    def __init__(self, backend, lease_timeout=5):
        self.backend = backend
        self.lease_timeout = lease_timeout
        self.lock = threading.RLock()
        self.owner = None
        self.last_seen = 0
        self.active = None
        self.operations = collections.OrderedDict()
        self.requests = collections.OrderedDict()
        self.closed = threading.Event()
        self.watchdog = threading.Thread(target=self.watch)
        self.watchdog.daemon = True
        self.watchdog.start()

    def watch(self):
        while not self.closed.wait(0.2):
            with self.lock:
                if self.active:
                    state = self.backend.state()
                    if state.get('feedback_stale') or (self.active['method'] in HEAD_MOTIONS and
                            state.get('head', {}).get('feedback_stale', True)):
                        self.active['cancel'].set()
                        self.backend.stop()
                if self.owner and time.time() - self.last_seen > self.lease_timeout:
                    if self.active:
                        self.active['cancel'].set()
                    self.backend.stop()
                    self.owner = None

    def work(self, record, method, params):
        result, error = None, None
        try:
            result = self.backend.execute(method, params, record['cancel'])
        except Exception as exc:
            error = str(exc)
        with self.lock:
            record['status'] = ('cancelled' if record['cancel'].is_set() else
                                'failed' if error else 'succeeded')
            record['result'], record['error'] = result, error
            self.active = None

    def call(self, method, params, client_id, request_id):
        if method not in METHODS:
            raise KeyError('Method is not exposed: ' + method)
        validate(method, params)
        with self.lock:
            if self.closed.is_set():
                raise RuntimeError('Server is shutting down')
            if self.owner == client_id:
                self.last_seen = time.time()
            if method == 'heartbeat':
                return {'has_control': self.owner == client_id}
            if method == 'release_control':
                if self.owner == client_id:
                    if self.active:
                        self.active['cancel'].set()
                    self.backend.stop()
                    self.owner = None
                return True
            if method == 'get_operation':
                record = self.operations.get(params['operation_id'])
                if record is None:
                    raise ValueError('Unknown/expired operation; it may belong to an earlier server run')
                return dict((key, record[key]) for key in ('operation_id', 'status', 'result', 'error'))
            if method in ('abort_movement', 'stop_gripper'):
                limb = params.get('limb_name')
                affected = self.active and (limb is None or limb in self.active['limbs'])
                gripper_only = method == 'stop_gripper'
                if affected and (not gripper_only or self.active['method'] in
                                 ('move_gripper', 'open_gripper', 'close_gripper', 'jog_gripper', 'calibrate_gripper')):
                    self.active['cancel'].set()
                    self.backend.stop(None if limb is None else self.active['limbs'], gripper_only=gripper_only)
                else:
                    self.backend.stop(None if limb is None else [limb], gripper_only=gripper_only)
                return {'stop_requested': True}
            if method in OPERATIONS:
                comparison = dict(params)
                if 'rgb_base64' in comparison:
                    # Keep deduplication small; do not retain 128 full screen images.
                    comparison['rgb_base64'] = hashlib.sha256(comparison['rgb_base64'].encode('ascii')).hexdigest()
                previous = self.requests.get((client_id, request_id))
                if previous:
                    if previous[:2] != (method, comparison):
                        raise ValueError('Request ID was reused with different arguments')
                    return {'operation_id': previous[2]}
                if self.owner and self.owner != client_id:
                    raise RuntimeError('Another client owns control; close it or wait for its lease to expire')
                if self.active:
                    raise RuntimeError('Busy: wait for the active operation or abort it')
                state = self.backend.state()
                head = state.get('head', {})
                if head.get('fault') or head.get('panning') or head.get('nodding'):
                    raise RuntimeError(head.get('fault') or 'Baxter head is still moving')
                if method in HEAD_MOTIONS and head.get('feedback_stale', True):
                    raise RuntimeError('Head feedback is stale or unavailable')
                if state.get('fault') or state.get('feedback_stale') or any(state['movement_in_progress'].values()):
                    raise RuntimeError(state.get('fault') or ('Robot feedback is stale' if state.get('feedback_stale') else 'Baxter is still moving'))
                if self.owner != client_id:
                    self.backend.clear_trajectories()
                self.owner, self.last_seen = client_id, time.time()
                limbs = params.get('limb_names') or list(
                    (params.get('joint_angles_rad_byLimb') or params.get('gripper_position_m_byLimb') or {}).keys())
                if not limbs:
                    limbs = [params['limb_name']] if params.get('limb_name') else list(LIMBS)
                if method in HEAD_OPERATIONS:
                    limbs = []
                operation_id = uuid.uuid4().hex
                record = {'operation_id': operation_id, 'status': 'running', 'result': None,
                          'error': None, 'cancel': threading.Event(), 'limbs': limbs, 'method': method}
                self.active = record
                self.operations[operation_id] = record
                self.requests[(client_id, request_id)] = (method, comparison, operation_id)
                while len(self.operations) > 128:
                    self.operations.popitem(last=False)
                while len(self.requests) > 128:
                    self.requests.popitem(last=False)
                worker = threading.Thread(target=self.work, args=(record, method, params))
                worker.daemon = True
                worker.start()
                return {'operation_id': operation_id}
            busy = bool(self.active)
            active_limbs = list(self.active['limbs']) if self.active else []
            if method == 'trajectory_succeeded':
                return self.backend.trajectory_result(params['limb_name'])
        state = self.backend.state()
        state['commands_busy'] = busy
        if method == 'get_state':
            return state
        if method == 'get_head_state':
            return state['head']
        if method == 'get_head_pan_rad':
            return state['head']['pan_rad']
        if method == 'get_head_tilt_rad':
            return None
        fields = {'get_joint_angles_rad': 'joint_angles_rad',
                  'get_joint_velocities_rad_s': 'joint_velocities_rad_s',
                  'get_joint_efforts_Nm': 'joint_efforts_Nm',
                  'get_end_effector_poses': 'end_effector_poses'}
        if method in fields:
            return state[fields[method]]
        if method == 'is_movement_in_progress':
            limb = params.get('limb_name')
            if limb:
                return limb in active_limbs or state['movement_in_progress'][limb]
            head = state.get('head', {})
            return bool(busy or any(state['movement_in_progress'].values()) or
                        head.get('panning') or head.get('nodding'))
        gripper_fields = {'get_gripper_position_open_percent': 'position_percent',
                          'get_gripper_force_percent': 'force_percent',
                          'is_gripper_grasping': 'grasping', 'is_gripper_moving': 'moving'}
        return state['grippers'][params['limb_name']][gripper_fields[method]]

    def close(self):
        if self.closed.is_set():
            return
        self.closed.set()
        with self.lock:
            if self.active:
                self.active['cancel'].set()
            self.backend.stop()
        if threading.current_thread() is not self.watchdog:
            self.watchdog.join(1)
        if hasattr(self.backend, 'close'):
            self.backend.close()


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        BaseHTTPRequestHandler.setup(self)
        self.connection.settimeout(5)

    def do_GET(self):
        # Images use binary HTTP responses, avoiding base64 overhead in JSON.
        # Never hold the command lock while waiting for or encoding a frame.
        status, content_type = 200, 'image/png'
        try:
            paths = dict(('/camera/' + name + '.png', name) for name in CAMERA_NAMES)
            if self.path not in paths:
                status = 404
                raise ValueError('Unknown camera path')
            if self.server.api.closed.is_set():
                raise RuntimeError('Server is shutting down')
            data = self.server.api.backend.get_camera_frame(paths[self.path])
        except Exception as exc:
            status = status if status != 200 else 503
            content_type = 'application/json'
            data = json.dumps({'error': str(exc)}).encode('utf-8')
        try:
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(data)
        except IOError:
            pass  # The client may have stopped waiting for this image.

    def do_POST(self):
        request_id = None
        try:
            if self.path != '/rpc':
                raise ValueError('Use POST /rpc')
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 3 * 1024 * 1024:
                raise ValueError('Request must be between 1 byte and 3 MiB')
            request = json.loads(self.rfile.read(size).decode('utf-8'))
            if not isinstance(request, dict) or request.get('jsonrpc') != '2.0':
                raise ValueError('Expected a JSON-RPC 2.0 request object')
            request_id = request.get('id')
            method = request.get('method')
            client = self.headers.get('X-Client-ID', '')
            if not isinstance(request_id, STRINGS) or not 1 <= len(request_id) <= 100:
                raise ValueError('A string request id is required')
            if not isinstance(method, STRINGS) or not client or len(client) > 100:
                raise ValueError('A method and X-Client-ID header are required')
            result = self.server.api.call(method, request.get('params', {}), client, request_id)
            response = {'jsonrpc': '2.0', 'id': request_id, 'result': result}
        except Exception as exc:
            code = -32601 if isinstance(exc, KeyError) else -32602 if isinstance(exc, (ValueError, TypeError)) else -32000
            response = {'jsonrpc': '2.0', 'id': request_id,
                        'error': {'code': code, 'message': str(exc)}}
        try:
            data = json.dumps(response, allow_nan=False, separators=(',', ':')).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)
        except (IOError, ValueError):
            pass  # A lost response must never rerun a motion.

    def log_message(self, *args):
        pass


class Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1', help='Desktop router-facing IP; default localhost')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--simulate', action='store_true', help='No ROS or robot; illustrative motion only')
    parser.add_argument('--lease-timeout', type=float, default=5)
    args = parser.parse_args()
    if not 2 <= args.lease_timeout <= 120:
        parser.error('--lease-timeout must be between two and 120 seconds')
    # Bind before constructing the controller, which enables/calibrates the robot.
    server = Server((args.host, args.port), Handler)
    try:
        if args.simulate:
            from remote_simulation import SimulationBackend
            backend = SimulationBackend()
        else:
            from remote_robot import BaxterBackend
            backend = BaxterBackend()
        server.api = RemoteAPI(backend, args.lease_timeout)
        if not args.simulate:
            import rospy
            rospy.on_shutdown(server.api.close)
        print('Baxter %s server: http://%s:%s/rpc' %
              ('SIMULATION' if args.simulate else 'ROBOT', args.host, args.port))
        # rospy owns SIGINT on the real server; its shutdown hook closes the API.
        server.timeout = 0.2
        while not server.api.closed.is_set():
            server.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if hasattr(server, 'api'):
                server.api.close()
        finally:
            server.server_close()


if __name__ == '__main__':
    main()
