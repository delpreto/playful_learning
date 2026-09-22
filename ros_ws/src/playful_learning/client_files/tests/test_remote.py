"""Hardware-free operation/lease regression tests: python -B -m unittest test_remote."""
from __future__ import print_function

import threading
import time
import unittest
from unittest.mock import Mock, patch

from BaxterRemoteController_server import RemoteAPI, validate
from BaxterRemoteController_client import BaxterRemoteController
from remote_robot import BaxterBackend


class BlockingBackend(object):
    """An operation stays active until explicitly released, even after cancellation."""

    def __init__(self):
        self.started = threading.Event()
        self.finish = threading.Event()
        self.stopped = threading.Event()
        self.cancel = None
        self.calls = 0

    def state(self):
        return {"movement_in_progress": {"left": False, "right": False}}

    def execute(self, method, params, cancel):
        self.calls += 1
        self.cancel = cancel
        self.started.set()
        self.finish.wait(3)
        if cancel.is_set():
            raise RuntimeError("Cancelled")
        return True

    def stop(self, limb_names=None, gripper_only=False):
        self.stopped.set()

    def clear_trajectories(self):
        pass

    def trajectory_result(self, limb_name):
        return None


class RemoteOperationTests(unittest.TestCase):
    def setUp(self):
        self.backend = BlockingBackend()
        self.api = RemoteAPI(self.backend, lease_timeout=30)

    def tearDown(self):
        self.backend.finish.set()
        self.api.close()

    def start_move(self):
        result = self.api.call("move_to_neutral", {"limb_name": "left"}, "client-a", "move-1")
        self.assertTrue(self.backend.started.wait(1))
        return result

    def wait_result(self, accepted):
        deadline = time.time() + 2
        while time.time() < deadline:
            result = self.api.call("get_operation", accepted, "client-a", "read")
            if result["status"] != "running":
                return result
            time.sleep(.01)
        self.fail("Worker did not finish")

    def test_duplicate_request_never_starts_second_motion(self):
        original = self.start_move()
        duplicate = self.api.call("move_to_neutral", {"limb_name": "left"}, "client-a", "move-1")
        self.assertEqual(original, duplicate)
        self.assertEqual(self.backend.calls, 1)
        with self.assertRaises(ValueError):
            self.api.call("move_to_neutral", {"limb_name": "right"}, "client-a", "move-1")

    def test_busy_reservation_does_not_depend_on_robot_feedback(self):
        self.start_move()
        with self.assertRaises(RuntimeError):
            self.api.call("move_to_neutral", {}, "client-a", "move-2")
        self.assertTrue(self.api.call("get_state", {}, "observer", "read")["commands_busy"])

    def test_release_keeps_operation_reserved_until_worker_exits(self):
        accepted = self.start_move()
        self.api.call("release_control", {}, "client-a", "release")
        self.assertTrue(self.backend.cancel.is_set())
        self.assertTrue(self.backend.stopped.is_set())
        with self.assertRaises(RuntimeError):
            self.api.call("move_to_neutral", {}, "client-b", "move-2")
        self.backend.finish.set()
        self.assertEqual(self.wait_result(accepted)["status"], "cancelled")

    def test_lease_expiry_cancels_without_waiting_for_worker(self):
        self.start_move()
        with self.api.lock:
            self.api.last_seen = time.time() - self.api.lease_timeout - 1
        self.assertTrue(self.backend.stopped.wait(1))
        self.assertTrue(self.backend.cancel.is_set())
        self.assertIsNone(self.api.owner)
        self.assertIsNotNone(self.api.active)

    def test_gripper_stop_does_not_cancel_arm_operation(self):
        self.start_move()
        self.api.call("stop_gripper", {"limb_name": "left"}, "client-a", "stop")
        self.assertFalse(self.backend.cancel.is_set())

    def test_calibration_is_deduplicated_and_cancelled_on_lease_expiry(self):
        params = {'limb_name': 'right'}
        accepted = self.api.call('calibrate_gripper', params, 'client-a', 'calibrate-1')
        self.assertTrue(self.backend.started.wait(1))
        self.assertEqual(self.api.call('calibrate_gripper', params, 'client-a', 'calibrate-1'), accepted)
        self.assertEqual(self.backend.calls, 1)
        with self.api.lock:
            self.api.last_seen = time.time() - self.api.lease_timeout - 1
        self.assertTrue(self.backend.stopped.wait(1))
        self.assertTrue(self.backend.cancel.is_set())
        self.assertIsNotNone(self.api.active)
        self.backend.finish.set()
        self.assertEqual(self.wait_result(accepted)['status'], 'cancelled')

    def test_stale_robot_feedback_cancels_even_with_connected_client(self):
        self.start_move()
        self.backend.state = lambda: {'movement_in_progress': {'left': False, 'right': False},
                                      'feedback_stale': True}
        self.assertTrue(self.backend.stopped.wait(1))
        self.assertTrue(self.backend.cancel.is_set())
        self.assertEqual(self.api.owner, 'client-a')

    def test_other_limb_is_not_reported_moving_by_active_operation(self):
        self.start_move()
        self.assertTrue(self.api.call('is_movement_in_progress', {'limb_name': 'left'}, 'client-a', 'read'))
        self.assertFalse(self.api.call('is_movement_in_progress', {'limb_name': 'right'}, 'client-a', 'read'))

    def test_completion_and_cancellation_are_distinct(self):
        accepted = self.start_move()
        self.backend.finish.set()
        result = self.wait_result(accepted)
        self.assertEqual(result["status"], "succeeded")
        self.assertTrue(result["result"])

    def test_closed_api_rejects_new_motion(self):
        self.api.close()
        with self.assertRaises(RuntimeError):
            self.api.call("move_to_neutral", {}, "client-a", "late-request")
        self.assertEqual(self.backend.calls, 0)

    def test_invalid_waypoints_rejected_before_worker_start(self):
        for times in ([0, 1], [2, 1], [1, float("nan")]):
            with self.assertRaises(ValueError):
                validate("build_trajectory_from_joint_angles", {
                    "limb_name": "left", "times_from_start_s": times,
                    "joint_angles_rad": [[0] * 7, [0] * 7]})
        with self.assertRaises(ValueError):
            validate("move_to_joint_angles_rad", {"joint_angles_rad_byLimb": {"left": {"right_s0": 0}}})


class FakeTrajectory(object):
    def __init__(self):
        self._client = self
        self.times = []
        self.points = []
        self.runs = []
        self.status = 3
        self.error_code = 0
        self.missing_result = False

    def run(self, wait_for_completion=False):
        # Match the original trajectory's append behavior so rebuilding matters.
        self.points.extend([0] + self.times)
        self.runs.append(list(self.points))

    def get_state(self):
        return self.status

    def result(self):
        return None if self.missing_result else self


class FinishedWorker(object):
    def is_alive(self):
        return False


class FakeClock(object):
    def __init__(self):
        self.now = 100.0

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeController(object):
    def __init__(self):
        self._trajectories = {"left": FakeTrajectory()}
        self._move_to_joint_angles_rad_thread = {"left": FinishedWorker()}
        self.moves = []
        self.ik_started = threading.Event()
        self.ik_finish = threading.Event()

    def build_trajectory_from_joint_angles(self, limb, times, angles, tolerance):
        self._trajectories[limb].times = list(times)
        self._trajectories[limb].points = []

    def move_to_joint_angles_rad(self, targets, **options):
        self.moves.append(targets)

    def get_joint_angles_rad(self):
        return {"left": {"left_s0": 0.0}}

    def get_joint_angles_rad_for_gripper_pose(self, *args, **options):
        self.ik_started.set()
        self.ik_finish.wait(2)
        return {"left_s0": 0.1}


class RobotAdapterTests(unittest.TestCase):
    def setUp(self):
        # Deliberately skip __init__: no ROS imports, subscribers, or robot access.
        self.backend = BaxterBackend.__new__(BaxterBackend)
        self.backend.controller = FakeController()
        self.backend.dispatch_lock = threading.RLock()
        self.backend.trajectories = {"left": ([1, 2], [[0] * 7, [.1] * 7], .1)}
        self.backend.trajectory_results = {"left": None}
        self.backend.fault = None
        self.cancel = threading.Event()

    def test_gripper_force_defaults_reach_sdk_from_browser_and_python_client(self):
        c = self.backend.controller = Mock()
        c.is_gripper_moving.return_value = False
        c.is_gripper_grasping.return_value = False
        # Use real SDK parameter/command semantics without ROS or physical motion.
        gripper = Mock(_cmd_sender='test_%s', _cmd_sequence=0)
        c._grippers = {'left': gripper, 'right': gripper}
        def command_position(target, block=False):
            gripper._cmd_sequence += 1
            gripper._state = Mock(command_sender='test_go',
                                  command_sequence=gripper._cmd_sequence)
            c.get_gripper_position_open_percent.return_value = target
        gripper.command_position.side_effect = command_position
        def dispatch(method, **params):
            validate(method, params)
            return self.backend.execute(method, params, self.cancel)
        client = BaxterRemoteController.__new__(BaxterRemoteController)
        client._start = dispatch
        cases = [('open_gripper', {}, 100, 75),
                 ('close_gripper', {}, 0, 75),
                 ('jog_gripper', {'delta_percent': 5}, 55, 75),
                 ('move_gripper', {'gripper_open_percent': 80}, 80, 75),
                 ('move_gripper', {'gripper_open_percent': 80,
                                   'force_threshold_percent': 20}, 80, 20)]
        for limb in ('left', 'right'):
            for method, params, target, force in cases:
                for use_client in (False, True):
                    with self.subTest(limb=limb, method=method, force=force, client=use_client):
                        gripper.reset_mock()
                        c.get_gripper_position_open_percent.return_value = 50
                        with patch('remote_robot.time', FakeClock()):
                            result = (getattr(client, method)(limb, **params) if use_client
                                      else dispatch(method, limb_name=limb, **params))
                        self.assertEqual(result, target)
                        gripper.set_parameters.assert_called_once_with(
                            {'moving_force': force, 'holding_force': 15})
                        gripper.command_position.assert_called_once_with(target, block=False)
        c.move_gripper.assert_not_called()  # Its legacy 30% cap must not apply.

    def test_gripper_moving_force_api_limits(self):
        params = {'limb_name': 'left', 'gripper_open_percent': 100}
        for force in (0, 30, 75):
            validate('move_gripper', dict(params, force_threshold_percent=force))
        for force in (-1, 75.1, 100, float('nan'), float('inf'), True):
            with self.subTest(force=force), self.assertRaises(ValueError):
                validate('move_gripper', dict(params, force_threshold_percent=force))

    def test_state_checks_trajectory_status_only_when_a_goal_exists(self):
        from unittest.mock import Mock

        c = self.backend.controller = Mock()
        c._limbs = {}
        c._trajectories = {}
        c._move_to_joint_angles_rad_thread = {}
        for limb in ('left', 'right'):
            c._limbs[limb] = Mock()
            c._limbs[limb].endpoint_pose.return_value = {
                'position': Mock(x=0.6, y=0.3, z=0.2),
                'orientation': Mock(w=1.0, x=0.0, y=0.0, z=0.0)}
            client = Mock(gh=None)
            client.get_state.side_effect = AssertionError('Queried a client without a goal')
            c._trajectories[limb] = Mock(_client=client)
            c._move_to_joint_angles_rad_thread[limb] = None
        c.get_gripper_position_open_percent.return_value = 100.0
        c.get_gripper_force_percent.return_value = 0.0
        c.is_gripper_moving.return_value = False
        c.is_gripper_grasping.return_value = False
        c.get_joint_angles_rad.return_value = {}
        c.get_joint_velocities_rad_s.return_value = {}
        c.get_joint_efforts_Nm.return_value = {}
        self.backend.feedback = dict.fromkeys(
            ('joints', 'left', 'right', 'left_gripper', 'right_gripper'), time.time())
        self.backend.head = Mock()
        self.backend.head.get_state.return_value = {}

        # A prepared trajectory is not yet a goal; repeated browser reads stay idle.
        for unused in range(3):
            self.assertEqual(self.backend.state()['movement_in_progress'],
                             {'left': False, 'right': False})
        for limb in ('left', 'right'):
            c._trajectories[limb]._client.get_state.assert_not_called()

        client = c._trajectories['left']._client
        client.gh = object()
        client.get_state.side_effect = None
        for status in range(10):
            client.get_state.return_value = status
            moving = self.backend.state()['movement_in_progress']
            self.assertEqual(moving, {'left': status in (0, 1, 6, 7), 'right': False})
        self.assertEqual(client.get_state.call_count, 10)
        c._trajectories['right']._client.get_state.assert_not_called()

        # Non-trajectory movement must still be detected when there is no goal.
        client.gh = None
        c._move_to_joint_angles_rad_thread['left'] = Mock()
        c._move_to_joint_angles_rad_thread['left'].is_alive.return_value = True
        c.is_gripper_moving.side_effect = lambda limb: limb == 'right'
        self.assertEqual(self.backend.state()['movement_in_progress'],
                         {'left': True, 'right': True})
        self.assertEqual(client.get_state.call_count, 10)

    def test_replay_rebuilds_goal_instead_of_accumulating_points(self):
        for unused in range(2):
            self.backend.execute("run_trajectory", {"limb_names": ["left"]}, self.cancel)
        self.assertEqual(self.backend.controller._trajectories["left"].runs, [[0, 1, 2], [0, 1, 2]])

    def test_action_status_and_result_both_must_indicate_success(self):
        trajectory = self.backend.controller._trajectories["left"]
        for status, error_code, missing in [(3, -5, False), (2, 0, False), (3, 0, True)]:
            trajectory.status, trajectory.error_code, trajectory.missing_result = status, error_code, missing
            with self.assertRaises(RuntimeError):
                self.backend.execute("run_trajectory", {"limb_names": ["left"]}, self.cancel)
            self.assertFalse(self.backend.trajectory_result("left"))

    def test_finished_joint_worker_without_target_is_failure(self):
        with patch('remote_robot.time', FakeClock()), self.assertRaisesRegex(RuntimeError, 'Target not reached.*left_s0'):
            self.backend.execute("move_to_joint_angles_rad", {
                "joint_angles_rad_byLimb": {"left": {"left_s0": .5}}}, self.cancel)

    def test_joint_feedback_can_settle_without_resending_motion(self):
        c = self.backend.controller
        c.move_to_joint_angles_rad = Mock()
        c.get_joint_angles_rad = Mock(side_effect=[
            {'left': {'left_s0': .49}}, {'left': {'left_s0': .495}}])
        with patch('remote_robot.time', FakeClock()):
            result = self.backend.move_joints({'left': {'left_s0': .5}}, {}, self.cancel)
        self.assertEqual(result, {'left': {'left_s0': .5}})
        c.move_to_joint_angles_rad.assert_called_once_with(
            result, timeout_s=30, tolerance_rad=0.008726646)
        self.assertEqual(c.get_joint_angles_rad.call_count, 2)

    def test_custom_motion_limits_and_failure_detail_are_preserved(self):
        c = self.backend.controller
        c.move_to_joint_angles_rad = Mock()
        with patch('remote_robot.time', FakeClock()), self.assertRaisesRegex(
                RuntimeError, r'Target not reached.*timeout 90.0 s.*left_s0 error 0.5000 rad.*tolerance 0.0010 rad'):
            self.backend.move_joints({'left': {'left_s0': .5}},
                                     {'timeout_s': 90, 'tolerance_rad': .001}, self.cancel)
        c.move_to_joint_angles_rad.assert_called_once_with(
            {'left': {'left_s0': .5}}, timeout_s=90, tolerance_rad=.001)
        self.assertIsNone(self.backend.fault)
        # A normal target miss does not prevent a subsequent command.
        self.backend.move_joints({'left': {'left_s0': 0}}, {}, self.cancel)

    def test_cancel_during_feedback_settling_is_not_success(self):
        c = self.backend.controller
        c._limbs = {'left': Mock()}
        clock = FakeClock()
        def read_angles():
            if clock.now > 100:
                self.cancel.set()
                return {'left': {'left_s0': .5}}
            return {'left': {'left_s0': .49}}
        c.get_joint_angles_rad = read_angles
        with patch('remote_robot.time', clock), self.assertRaisesRegex(RuntimeError, 'Cancelled'):
            self.backend.move_joints({'left': {'left_s0': .5}}, {}, self.cancel)
        c._limbs['left'].set_joint_positions.assert_called_once()

    def test_sdk_timeout_reports_elapsed_time_and_target_error(self):
        clock = FakeClock()
        self.backend.controller.move_to_joint_angles_rad = Mock(side_effect=lambda *a, **k: clock.sleep(2))
        with patch('remote_robot.time', clock), self.assertRaisesRegex(RuntimeError, 'Motion timed out.*timeout 2.0 s.*left_s0'):
            self.backend.move_joints({'left': {'left_s0': .5}}, {'timeout_s': 2}, self.cancel)
        self.assertIsNone(self.backend.fault)

    def test_watchdog_stop_cannot_turn_into_success_at_the_target(self):
        c = self.backend.controller
        worker = c._move_to_joint_angles_rad_thread['left'] = Mock()
        worker.is_alive.side_effect = lambda: not self.backend.stop.called
        self.backend.stop = Mock()
        with patch('remote_robot.time', FakeClock()), self.assertRaisesRegex(RuntimeError, 'Motion timed out'):
            self.backend.move_joints({'left': {'left_s0': 0}}, {'timeout_s': .1}, self.cancel)
        self.backend.stop.assert_called_once_with(['left'])
        self.assertIsNone(self.backend.fault)

    def test_worker_that_ignores_stop_still_sets_a_fault(self):
        self.backend.controller._move_to_joint_angles_rad_thread['left'] = Mock()
        self.backend.controller._move_to_joint_angles_rad_thread['left'].is_alive.return_value = True
        self.backend.stop = Mock()
        with patch('remote_robot.time', FakeClock()), self.assertRaisesRegex(RuntimeError, 'Movement worker did not stop'):
            self.backend.move_joints({'left': {'left_s0': .5}}, {'timeout_s': .1}, self.cancel)
        self.assertIsNotNone(self.backend.fault)

    def test_cancelled_ik_never_starts_delayed_motion(self):
        errors = []
        def solve_and_move():
            try:
                self.backend.execute("move_to_gripper_pose", {
                    "gripper_position_m_byLimb": {"left": [.6, .3, .2]},
                    "gripper_orientation_quaternion_wijk_byLimb": {"left": [1, 0, 0, 0]}}, self.cancel)
            except RuntimeError as exc:
                errors.append(str(exc))
        worker = threading.Thread(target=solve_and_move)
        worker.daemon = True
        worker.start()
        self.assertTrue(self.backend.controller.ik_started.wait(1))
        self.cancel.set()
        self.backend.controller.ik_finish.set()
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertTrue(errors)
        self.assertEqual(self.backend.controller.moves, [])


if __name__ == "__main__":
    unittest.main()
