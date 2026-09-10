"""Hardware-free operation/lease regression tests: python -B -m unittest test_remote."""
from __future__ import print_function

import threading
import time
import unittest

from BaxterRemoteController_server import RemoteAPI, validate
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
        with self.assertRaises(RuntimeError):
            self.backend.execute("move_to_joint_angles_rad", {
                "joint_angles_rad_byLimb": {"left": {"left_s0": .5}}}, self.cancel)

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
