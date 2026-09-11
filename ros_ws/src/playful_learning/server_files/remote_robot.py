"""Small adapter around the existing controller. Python 2.7 compatible.

Motion runs in the remote server's worker. Reads and cancellation stay available.
Private SDK access is confined here; the original controller files are unchanged.
"""
from __future__ import print_function

import copy
import base64
import math
import threading
import time

LIMBS = ('left', 'right')
JOINTS = ('s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2')
ACTIVE = (0, 1, 6, 7)  # pending, active, preempting, recalling


class BaxterBackend(object):
    simulation = False

    def __init__(self):
        # Construct on the main thread: this initializes rospy and enables Baxter.
        from BaxterController import BaxterController
        self.controller = BaxterController()
        from BaxterCameras import BaxterCameras
        self.cameras = BaxterCameras()
        from BaxterHeadController import BaxterHeadController
        self.head = BaxterHeadController()
        import rospy
        from sensor_msgs.msg import JointState
        from baxter_core_msgs.msg import EndpointState, EndEffectorState
        self.feedback = {}
        self.subscribers = [rospy.Subscriber('/robot/joint_states', JointState,
                            lambda msg: self.feedback.update(joints=time.time()), queue_size=1)]
        for limb in LIMBS:
            self.subscribers.append(rospy.Subscriber('/robot/limb/' + limb + '/endpoint_state',
                EndpointState, lambda msg, name=limb: self.feedback.update({name: time.time()}), queue_size=1))
            self.subscribers.append(rospy.Subscriber('/robot/end_effector/' + limb + '_gripper/state',
                EndEffectorState, lambda msg, name=limb: self.feedback.update({name + '_gripper': time.time()}), queue_size=1))
        self.dispatch_lock = threading.RLock()
        self.trajectories = {}
        self.trajectory_results = dict.fromkeys(LIMBS)
        self.fault = None

    def get_camera_frame(self, camera_name):
        return self.cameras.get_camera_frame(camera_name)

    def state(self):
        c = self.controller
        poses, grippers, moving = {}, {}, {}
        for limb in LIMBS:
            pose = c._limbs[limb].endpoint_pose()
            p, q = pose['position'], pose['orientation']
            poses[limb] = {'position_m': [p.x, p.y, p.z],
                           'orientation_wijk': [q.w, q.x, q.y, q.z]}
            grippers[limb] = {
                'position_percent': c.get_gripper_position_open_percent(limb),
                'force_percent': c.get_gripper_force_percent(limb),
                'moving': c.is_gripper_moving(limb),
                'grasping': c.is_gripper_grasping(limb)}
            worker = c._move_to_joint_angles_rad_thread[limb]
            client = c._trajectories[limb]._client
            # Older actionlib logs an error if get_state() has no goal to inspect.
            moving[limb] = bool((worker and worker.is_alive()) or
                               (client.gh is not None and client.get_state() in ACTIVE) or
                               grippers[limb]['moving'])
        age = max(time.time() - self.feedback.get(key, 0)
                  for key in ('joints', 'left', 'right', 'left_gripper', 'right_gripper'))
        return {'joint_angles_rad': c.get_joint_angles_rad(),
                'joint_velocities_rad_s': c.get_joint_velocities_rad_s(),
                'joint_efforts_Nm': c.get_joint_efforts_Nm(),
                'end_effector_poses': poses, 'grippers': grippers,
                'movement_in_progress': moving, 'timestamp': time.time(),
                'simulation': False, 'fault': self.fault, 'feedback_age_s': age,
                'feedback_stale': age > 2, 'head': self.head.get_state()}

    def clear_trajectories(self):
        self.trajectories.clear()
        self.trajectory_results = dict.fromkeys(LIMBS)

    def trajectory_result(self, limb_name):
        return self.trajectory_results[limb_name]

    def stop(self, limb_names=None, gripper_only=False):
        # No joins here: stop must not wait behind an executing command.
        with self.dispatch_lock:
            if limb_names is None and not gripper_only:
                self.head.stop()
            for limb in (LIMBS if limb_names is None else limb_names):
                if not gripper_only:
                    self.controller._should_abort_move_to_joint_angles_rad[limb] = True
                    self.controller._trajectories[limb].stop()
                self.controller._grippers[limb].stop(block=False)

    def close(self):
        self.head.close()

    def move_joints(self, targets, params, cancel):
        c = self.controller
        timeout = params.get('timeout_s', 30)
        tolerance = params.get('tolerance_rad', 0.008726646)
        targets = copy.deepcopy(targets)
        with self.dispatch_lock:
            if cancel.is_set():
                raise RuntimeError('Cancelled before movement')
            started = time.time()
            c.move_to_joint_angles_rad(targets, timeout_s=timeout,
                                       tolerance_rad=tolerance)
        deadline = started + timeout + 1
        stopping_at = None
        while any(c._move_to_joint_angles_rad_thread[limb].is_alive() for limb in targets):
            if cancel.is_set() or time.time() > deadline:
                stopping_at = stopping_at or time.time()
                self.stop(list(targets))
            if stopping_at and time.time() - stopping_at > 3:
                self.fault = 'Movement worker did not stop; inspect Baxter and restart the server.'
                raise RuntimeError(self.fault)
            time.sleep(0.02)
        # Allow feedback to settle after the SDK worker exits. Do not resend motion.
        motion_elapsed = time.time() - started
        settle_deadline = time.time() + 0.5
        while True:
            if cancel.is_set():
                for limb in targets:
                    c._limbs[limb].set_joint_positions(c._limbs[limb].joint_angles())
                raise RuntimeError('Cancelled')
            actual = c.get_joint_angles_rad()
            errors = []
            for limb, angles in targets.items():
                if isinstance(angles, list):
                    angles = dict(zip([limb + '_' + j for j in JOINTS], angles))
                errors.extend((abs(actual[limb][joint] - value), joint)
                              for joint, value in angles.items())
            error, joint = max(errors)
            if cancel.is_set():
                continue
            if stopping_at is None and error <= tolerance:
                return targets
            if stopping_at is not None or time.time() >= settle_deadline:
                elapsed = time.time() - started
                reason = 'Motion timed out' if stopping_at is not None or motion_elapsed >= timeout else 'Target not reached'
                raise RuntimeError('%s after %.1f s (timeout %.1f s): %s error %.4f rad (%.2f deg), '
                                   'tolerance %.4f rad (%.2f deg)' %
                                   (reason, elapsed, timeout, joint, error, math.degrees(error),
                                    tolerance, math.degrees(tolerance)))
            time.sleep(0.02)

    def execute(self, method, params, cancel):
        """Execute one approved operation, including its completion check."""
        c = self.controller
        p = copy.deepcopy(params)
        if self.fault:
            raise RuntimeError(self.fault)
        limb = p.get('limb_name')

        if method in ('set_head_pan_rad', 'nod_head'):
            try:
                if method == 'set_head_pan_rad':
                    return self.head.set_head_pan_rad(cancel=cancel, **p)
                return self.head.nod_head(cancel=cancel, **p)
            except Exception:
                head = self.head.get_state()
                if head.get('fault') or head['feedback_stale'] or head['panning'] or head['nodding']:
                    self.fault = head.get('fault') or 'Head stop was not confirmed; inspect Baxter and restart server.'
                raise
        if method in ('set_halo_led', 'set_sonar_leds', 'show_screen_color', 'show_screen_image_rgb'):
            with self.dispatch_lock:
                if cancel.is_set():
                    raise RuntimeError('Cancelled before publishing head output')
                if method == 'set_halo_led':
                    self.head.set_halo_led(**p)
                elif method == 'set_sonar_leds':
                    self.head.set_sonar_leds(**p)
                elif method == 'show_screen_color':
                    self.head.show_screen_color(**p)
                else:
                    self.head.show_screen_image_rgb(p['width'], p['height'], base64.b64decode(p['rgb_base64']))
            return {'published': True}

        if method == 'get_joint_angles_rad_for_gripper_pose':
            return c.get_joint_angles_rad_for_gripper_pose(**p)

        if method.startswith('build_trajectory_'):
            times = p['times_from_start_s']
            if method == 'build_trajectory_from_gripper_poses':
                angles = []
                seed = p.get('initial_seed_joint_angles_rad')
                for position, orientation in zip(p['gripper_positions_m'],
                                                 p['gripper_orientations_quaternion_wijk']):
                    if cancel.is_set():
                        raise RuntimeError('Cancelled')
                    seed = c.get_joint_angles_rad_for_gripper_pose(limb, position, orientation,
                                                                  seed_joint_angles_rad=seed)
                    if seed is None:
                        return False
                    angles.append(seed)
            else:
                angles = p['joint_angles_rad']
            if cancel.is_set():
                raise RuntimeError('Cancelled')
            # Store a complete build only; failed builds cannot partially replace it.
            self.trajectories[limb] = (times, angles, p.get('goal_time_tolerance_s', 0.1))
            self.trajectory_results[limb] = None
            return True

        if method == 'run_trajectory':
            limbs = p.get('limb_names') or list(LIMBS)
            if any(name not in self.trajectories for name in limbs):
                raise ValueError('Build a trajectory for every requested limb first')
            # Rebuild from saved data on each run to avoid appended/reused goal points.
            duration = 0
            with self.dispatch_lock:
                if cancel.is_set():
                    raise RuntimeError('Cancelled')
                for name in limbs:
                    times, angles, tolerance = self.trajectories[name]
                    c.build_trajectory_from_joint_angles(name, times, angles, tolerance)
                    self.trajectory_results[name] = None
                    duration = max(duration, max(times))
                for name in limbs:
                    c._trajectories[name].run(wait_for_completion=False)
            deadline = time.time() + duration + max(5, duration * 0.5)
            stopping_at = None
            while any(c._trajectories[name]._client.get_state() in ACTIVE for name in limbs):
                if cancel.is_set() or time.time() > deadline:
                    stopping_at = stopping_at or time.time()
                    self.stop(limbs)
                if stopping_at and time.time() - stopping_at > 3:
                    self.fault = 'Trajectory cancellation was not acknowledged; inspect Baxter and restart server.'
                    raise RuntimeError(self.fault)
                time.sleep(0.02)
            for name in limbs:
                trajectory = c._trajectories[name]
                result = trajectory.result()
                self.trajectory_results[name] = bool(trajectory._client.get_state() == 3 and
                                                       result is not None and result.error_code == 0)
            if not all(self.trajectory_results[name] for name in limbs):
                raise RuntimeError('Trajectory cancelled or failed; inspect the local ROS log')
            return True

        if method in ('move_gripper', 'open_gripper', 'close_gripper', 'jog_gripper'):
            target = p.get('gripper_open_percent', 100 if method == 'open_gripper' else 0)
            if method == 'jog_gripper':
                target = c.get_gripper_position_open_percent(limb) + p['delta_percent']
            target = max(0, min(100, target))
            gripper = c._grippers[limb]
            with self.dispatch_lock:
                if cancel.is_set():
                    raise RuntimeError('Cancelled')
                before = gripper._state
                c.move_gripper(limb, target, p.get('force_threshold_percent', 15))
                sequence = gripper._cmd_sequence
            # Confirm this command, not a cached idle/grasp from a previous command.
            started = time.time()
            stopping_at = None
            while True:
                if (cancel.is_set() or time.time() - started > 5) and stopping_at is None:
                    stopping_at = time.time()
                    before = gripper._state
                    self.stop([limb], gripper_only=True)
                    sequence = gripper._cmd_sequence
                feedback = gripper._state
                command = 'stop' if stopping_at else 'go'
                acknowledged = (feedback is not before and
                    feedback.command_sender == gripper._cmd_sender % command and
                    feedback.command_sequence in (0, gripper._cmd_sequence if stopping_at else sequence))
                if acknowledged and not c.is_gripper_moving(limb):
                    if stopping_at:
                        raise RuntimeError('Cancelled' if cancel.is_set() else 'Gripper timed out')
                    if abs(c.get_gripper_position_open_percent(limb) - target) < 2 or c.is_gripper_grasping(limb):
                        return c.get_gripper_position_open_percent(limb)
                if stopping_at and time.time() - stopping_at > 3:
                    self.fault = 'Gripper stop was not acknowledged; inspect Baxter and restart server.'
                    raise RuntimeError(self.fault)
                time.sleep(0.05)

        if method == 'move_to_joint_angles_rad':
            targets = p['joint_angles_rad_byLimb']
        elif method in ('move_to_neutral', 'move_to_resting'):
            poses = c.get_neutral_joint_angles_rad() if method == 'move_to_neutral' else c.get_resting_joint_angles_rad()
            targets = dict((name, poses[name]) for name in (LIMBS if limb is None else [limb]))
        elif method == 'jog_joint':
            name = limb + '_' + p['joint_name'].replace(limb + '_', '')
            targets = {limb: {name: c.get_joint_angles_rad()[limb][name] + p['delta_rad']}}
        elif method in ('move_to_gripper_pose', 'jog_endpoint'):
            if method == 'jog_endpoint':
                pose = self.state()['end_effector_poses'][limb]
                position, q = pose['position_m'], pose['orientation_wijk']
                axis, delta = p['axis'], p['delta']
                if axis in ('x', 'y', 'z'):
                    position[('x', 'y', 'z').index(axis)] += delta
                else:
                    rotation = [math.cos(delta / 2), 0, 0, 0]
                    rotation[('roll', 'pitch', 'yaw').index(axis) + 1] = math.sin(delta / 2)
                    w, x, y, z = rotation
                    a, b, d, e = q
                    q = [w*a-x*b-y*d-z*e, w*b+x*a+y*e-z*d,
                         w*d-x*e+y*a+z*b, w*e+x*d-y*b+z*a]
                positions, orientations = {limb: position}, {limb: q}
            else:
                positions = p['gripper_position_m_byLimb']
                orientations = p['gripper_orientation_quaternion_wijk_byLimb']
            targets = {}
            seeds = p.get('seed_joint_angles_rad_byLimb') or c.get_joint_angles_rad()
            for name in positions:
                if cancel.is_set():
                    raise RuntimeError('Cancelled')
                targets[name] = c.get_joint_angles_rad_for_gripper_pose(
                    name, positions[name], orientations[name], seed_joint_angles_rad=seeds.get(name))
                if targets[name] is None:
                    raise ValueError('No inverse-kinematics solution for the %s arm' % name)
        else:
            raise ValueError('Unsupported operation: ' + method)
        return self.move_joints(targets, p, cancel)
