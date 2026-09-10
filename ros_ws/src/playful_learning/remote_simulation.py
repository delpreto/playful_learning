"""Hardware-free demo backend. Its poses and IK are illustrative, not Baxter physics."""
from __future__ import division

import copy
import collections
import math
import threading
import time

from BaxterCameras import CAMERA_NAMES, image_to_png

SimulatedImage = collections.namedtuple('SimulatedImage', 'width height step encoding data')


class SimulationBackend(object):
    simulation = True

    def __init__(self):
        self._lock = threading.RLock()
        self._stopped = threading.Event()
        self._active_limbs = []
        self._active_gripper_only = False
        self._trajectories = {}
        self._results = {}
        self._neutral = {}
        self._resting = {}
        self._state = {"simulation": True, "joint_angles_rad": {},
                       "joint_velocities_rad_s": {}, "joint_efforts_Nm": {},
                       "end_effector_poses": {}, "grippers": {},
                       "movement_in_progress": {"left": False, "right": False}}
        self._state['head'] = {'pan_rad': 0.0, 'tilt_rad': None, 'tilt_supported': False,
                               'panning': False, 'nodding': False, 'feedback_age_s': 0.0,
                               'feedback_stale': False, 'fault': None,
                               'halo_red_percent': None, 'halo_green_percent': None,
                               'sonar_leds': 'auto', 'screen': None}
        for limb in ("left", "right"):
            names = [limb + "_" + name for name in ("s0", "s1", "e0", "e1", "w0", "w1", "w2")]
            self._neutral[limb] = dict(zip(names, [0, -.55, 0, .75, 0, 1.26, 0]))
            values = [-.8 if limb == "left" else .8, -.25, 0, 1.5, 0, -1.3,
                      0 if limb == "left" else -1.5]
            self._resting[limb] = dict(zip(names, values))
            self._state["joint_angles_rad"][limb] = dict(zip(names, values))
            self._state["joint_velocities_rad_s"][limb] = dict.fromkeys(names, 0.0)
            self._state["joint_efforts_Nm"][limb] = dict.fromkeys(names, 0.0)
            self._state["end_effector_poses"][limb] = {
                "position_m": [.6, .35 if limb == "left" else -.35, .2],
                "orientation_wijk": [1.0, 0.0, 0.0, 0.0]}
            self._state["grippers"][limb] = {"position_percent": 100.0,
                "force_percent": 0.0, "moving": False, "grasping": False}

    def get_camera_frame(self, camera_name):
        if camera_name not in CAMERA_NAMES:
            raise ValueError('Unknown camera: ' + camera_name)
        # A distinct color for each camera and a moving stripe to show refreshes.
        color = ((25, 125, 95), (145, 80, 125))[CAMERA_NAMES.index(camera_name)]
        stripe = int(time.time() * 40) % 320
        row = bytes(bytearray(color)) * 320
        row = row[:stripe * 3] + b'\xff\xff\xff' * min(8, 320 - stripe) + row[(stripe + 8) * 3:]
        grid = b'\xb0\xc0\xc0' * 320
        data = b''.join(grid if y % 40 == 0 else row for y in range(200))
        return image_to_png(SimulatedImage(320, 200, 960, 'rgb8', data))

    def state(self):
        with self._lock:
            snapshot = copy.deepcopy(self._state)
        snapshot["timestamp"] = time.time()
        return snapshot

    def stop(self, limb_names=None, gripper_only=False):
        with self._lock:
            if limb_names is None and not gripper_only and (
                    self._state['head']['panning'] or self._state['head']['nodding']):
                self._stopped.set()
            affected = set(limb_names or ("left", "right")) & set(self._active_limbs)
            if affected and (not gripper_only or self._active_gripper_only):
                self._stopped.set()

    def clear_trajectories(self):
        with self._lock:
            self._trajectories.clear()
            self._results.clear()

    def trajectory_result(self, limb_name):
        with self._lock:
            return self._results.get(limb_name)

    def _pose_angles(self, limb, position):
        # Deterministic sample values only; no physical inverse kinematics.
        angles = self.state()["joint_angles_rad"][limb]
        angles[limb + "_s0"] = position[1]
        angles[limb + "_s1"] = .6 - position[0]
        angles[limb + "_e1"] = 1.0 + position[2]
        return angles

    def _animate(self, angles, poses, grippers, cancel_event, duration=.6):
        start = self.state()
        limbs = set(angles) | set(poses) | set(grippers)
        with self._lock:
            self._active_limbs = list(limbs)
            self._active_gripper_only = not angles and not poses
            for limb in limbs:
                self._state["movement_in_progress"][limb] = True
                self._state["grippers"][limb]["moving"] = limb in grippers
        began = time.time()
        try:
            while True:
                if cancel_event.is_set() or self._stopped.is_set():
                    raise RuntimeError("Cancelled")
                fraction = min(1.0, (time.time() - began) / duration)
                with self._lock:
                    for limb, targets in angles.items():
                        for joint, target in targets.items():
                            initial = start["joint_angles_rad"][limb][joint]
                            value = initial + (target - initial) * fraction
                            self._state["joint_angles_rad"][limb][joint] = value
                            self._state["joint_velocities_rad_s"][limb][joint] = (target - initial) / duration
                            self._state["joint_efforts_Nm"][limb][joint] = 2 * math.sin(value)
                    for limb, target in poses.items():
                        pose = self._state["end_effector_poses"][limb]
                        for field, values in target.items():
                            initial = start["end_effector_poses"][limb][field]
                            pose[field] = [a + (b - a) * fraction for a, b in zip(initial, values)]
                        norm = math.sqrt(sum(v * v for v in pose["orientation_wijk"]))
                        pose["orientation_wijk"] = [v / norm for v in pose["orientation_wijk"]] if norm else [1, 0, 0, 0]
                    for limb, target in grippers.items():
                        initial = start["grippers"][limb]["position_percent"]
                        self._state["grippers"][limb]["position_percent"] = initial + (target - initial) * fraction
                if fraction == 1.0:
                    return True
                cancel_event.wait(.03)
        finally:
            with self._lock:
                for limb in limbs:
                    self._state["movement_in_progress"][limb] = False
                    self._state["grippers"][limb]["moving"] = False
                    self._state["joint_velocities_rad_s"][limb] = dict.fromkeys(start["joint_angles_rad"][limb], 0.0)
                self._active_limbs = []

    def execute(self, method, params, cancel_event):
        self._stopped.clear()
        if cancel_event.is_set():
            raise RuntimeError("Cancelled")
        params = copy.deepcopy(params)
        snapshot = self.state()
        limb = params.get("limb_name")
        limbs = [limb] if limb else ["left", "right"]
        names = [limb + "_" + name for name in ("s0", "s1", "e0", "e1", "w0", "w1", "w2")] if limb else []
        angles, poses, grippers = {}, {}, {}

        if method in ('set_head_pan_rad', 'nod_head'):
            field = 'panning' if method == 'set_head_pan_rad' else 'nodding'
            began = time.time()
            count = params.get('times', 1)
            duration = .6 if field == 'panning' else count * .6 + (count - 1) * params.get('internod_delay_s', 0)
            initial = snapshot['head']['pan_rad']
            with self._lock:
                self._state['head'][field] = True
            try:
                while True:
                    if cancel_event.is_set() or self._stopped.is_set():
                        raise RuntimeError('Cancelled')
                    fraction = min(1.0, (time.time() - began) / duration)
                    with self._lock:
                        if field == 'panning':
                            self._state['head']['pan_rad'] = initial + (params['angle_rad'] - initial) * fraction
                    if fraction == 1:
                        return self._state['head']['pan_rad'] if field == 'panning' else True
                    time.sleep(.02)
            finally:
                with self._lock:
                    self._state['head'][field] = False
        if method in ('set_halo_led', 'set_sonar_leds', 'show_screen_color', 'show_screen_image_rgb'):
            with self._lock:
                if cancel_event.is_set():
                    raise RuntimeError('Cancelled')
                head = self._state['head']
                if method == 'set_halo_led':
                    head['halo_red_percent'], head['halo_green_percent'] = params['red_percent'], params['green_percent']
                elif method == 'set_sonar_leds':
                    head['sonar_leds'] = params['led_states']
                elif method == 'show_screen_color':
                    head['screen'] = {'type': 'color', 'color_rgb': params['color_rgb']}
                else:
                    head['screen'] = {'type': 'image', 'width': params['width'], 'height': params['height']}
            return {'published': True}

        if method == "get_joint_angles_rad_for_gripper_pose":
            return self._pose_angles(limb, params["gripper_position_m"])
        if method.startswith("build_trajectory_from_"):
            times = params["times_from_start_s"]
            points = params.get("joint_angles_rad")
            if points is None:
                points = [self._pose_angles(limb, pos) for pos in params["gripper_positions_m"]]
            points = [dict(zip(names, point)) if isinstance(point, (list, tuple)) else point for point in points]
            with self._lock:
                self._trajectories[limb] = {"times": times, "angles": points,
                    "positions": params.get("gripper_positions_m"),
                    "orientations": params.get("gripper_orientations_quaternion_wijk")}
                self._results[limb] = None
            return True
        if method == "run_trajectory":
            limbs = params.get("limb_names") or ["left", "right"]
            with self._lock:
                trajectories = {name: copy.deepcopy(self._trajectories[name]) for name in limbs}
                for name in limbs:
                    self._results[name] = None
            try:
                # Compress timing for a quick demo; each waypoint takes 0.4 s.
                for index in range(max(len(path["times"]) for path in trajectories.values())):
                    angles, poses = {}, {}
                    for name, path in trajectories.items():
                        if index < len(path["angles"]):
                            angles[name] = path["angles"][index]
                            if path["positions"] is not None:
                                poses[name] = {"position_m": path["positions"][index],
                                               "orientation_wijk": path["orientations"][index]}
                    self._animate(angles, poses, {}, cancel_event, duration=.4)
                with self._lock:
                    for name in limbs:
                        self._results[name] = True
                return True
            except Exception:
                with self._lock:
                    for name in limbs:
                        self._results[name] = False
                raise

        if method in ("move_to_neutral", "move_to_resting"):
            source = self._neutral if method == "move_to_neutral" else self._resting
            angles = {name: source[name] for name in limbs}
        elif method == "move_to_joint_angles_rad":
            angles = params["joint_angles_rad_byLimb"]
        elif method in ("move_to_trajectory_start", "move_to_trajectory_index"):
            angles = {limb: self._trajectories[limb]["angles"][params.get("step_index", 0)]}
        elif method == "jog_joint":
            joint = params["joint_name"]
            if not joint.startswith(limb + "_"):
                joint = limb + "_" + joint
            angles = {limb: {joint: snapshot["joint_angles_rad"][limb][joint] + params["delta_rad"]}}
        elif method == "move_to_gripper_pose":
            for name, position in params["gripper_position_m_byLimb"].items():
                poses[name] = {"position_m": position,
                    "orientation_wijk": params["gripper_orientation_quaternion_wijk_byLimb"][name]}
                angles[name] = self._pose_angles(name, position)
        elif method == "jog_endpoint":
            pose = snapshot["end_effector_poses"][limb]
            axis, delta = params["axis"], params["delta"]
            if axis in ("x", "y", "z"):
                pose["position_m"][("x", "y", "z").index(axis)] += delta
            else:
                q = [math.cos(delta / 2), 0, 0, 0]
                q[("roll", "pitch", "yaw").index(axis) + 1] = math.sin(delta / 2)
                a, b, c, d = q
                w, x, y, z = pose["orientation_wijk"]
                pose["orientation_wijk"] = [a*w-b*x-c*y-d*z, a*x+b*w+c*z-d*y,
                                              a*y-b*z+c*w+d*x, a*z+b*y-c*x+d*w]
            poses[limb] = pose
            angles[limb] = self._pose_angles(limb, pose["position_m"])
        elif method in ("move_gripper", "jog_gripper", "open_gripper", "close_gripper"):
            target = params.get("gripper_open_percent", 100 if method == "open_gripper" else 0)
            if method == "jog_gripper":
                target = snapshot["grippers"][limb]["position_percent"] + params["delta_percent"]
            grippers[limb] = max(0, min(100, target))
            with self._lock:
                # This demo assumes an empty gripper; there is no contact force.
                self._state["grippers"][limb]["force_percent"] = 0.0
                self._state["grippers"][limb]["grasping"] = False
        else:
            raise ValueError("Unsupported simulation method: " + method)

        for name, target in angles.items():
            if isinstance(target, (list, tuple)):
                joint_names = [name + "_" + joint for joint in ("s0", "s1", "e0", "e1", "w0", "w1", "w2")]
                angles[name] = dict(zip(joint_names, target))
        return self._animate(angles, poses, grippers, cancel_event)
