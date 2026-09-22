"""Calibration feedback, operation ownership, and HTTP checks without hardware."""
import threading
import time
import unittest
from unittest.mock import Mock

from BaxterRemoteController_client import BaxterRemoteController, RemoteError
from BaxterRemoteController_server import Handler, RemoteAPI, Server, validate
from remote_robot import BaxterBackend
from remote_simulation import SimulationBackend


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.backend = BaxterBackend.__new__(BaxterBackend)
        self.backend.fault = None
        self.backend.dispatch_lock = threading.RLock()
        self.backend.trajectories = {}
        self.backend.trajectory_results = {}
        self.backend.feedback = {}
        self.backend.head = Mock()
        self.backend.state = Mock(return_value={
            'movement_in_progress': {'left': False, 'right': False}})
        self.controller = self.backend.controller = Mock()
        self.controller._should_abort_move_to_joint_angles_rad = {}
        self.controller._trajectories = {'left': Mock(), 'right': Mock()}
        self.controller._grippers = {}
        for limb in ('left', 'right'):
            gripper = Mock()
            gripper._state = object()
            gripper.type.return_value = 'electric'
            gripper.calibrated.return_value = True
            gripper.ready.return_value = True
            gripper.error.return_value = False
            gripper.moving.return_value = False
            self.controller._grippers[limb] = gripper
        self.controller.calibrate_gripper.side_effect = self.calibrate
        self.cancel = threading.Event()

    def calibrate(self, limb):
        self.controller._grippers[limb]._state = object()
        self.backend.feedback[limb + '_gripper'] = time.time()
        # The user's controller method returns None, even on success.

    def test_controller_method_is_called_and_feedback_verified_for_each_limb(self):
        for limb in ('left', 'right'):
            self.assertIs(self.backend.execute('calibrate_gripper', {'limb_name': limb}, self.cancel), True)
            self.controller.calibrate_gripper.assert_called_with(limb)
            self.controller._grippers[limb].stop.assert_not_called()

    def test_failed_calibration_is_not_reported_as_success(self):
        gripper = self.controller._grippers['right']
        for field, value in [('calibrated', False), ('ready', False), ('error', True), ('moving', True)]:
            with self.subTest(field=field):
                getattr(gripper, field).return_value = value
                with self.assertRaisesRegex(RuntimeError, 'right gripper calibration failed'):
                    self.backend.execute('calibrate_gripper', {'limb_name': 'right'}, self.cancel)
                gripper.stop.assert_called_with(block=False)
                getattr(gripper, field).return_value = not value

    def test_old_feedback_cannot_confirm_calibration(self):
        self.controller.calibrate_gripper.side_effect = None
        self.backend.feedback['right_gripper'] = time.time()
        with self.assertRaisesRegex(RuntimeError, 'fresh_feedback=False'):
            self.backend.execute('calibrate_gripper', {'limb_name': 'right'}, self.cancel)
        def stale_calibration(limb):
            self.calibrate(limb)
            self.backend.feedback[limb + '_gripper'] = time.time() - 10
        self.controller.calibrate_gripper.side_effect = stale_calibration
        with self.assertRaisesRegex(RuntimeError, 'fresh_feedback=False'):
            self.backend.execute('calibrate_gripper', {'limb_name': 'right'}, self.cancel)

    def test_sdk_exception_requests_stop(self):
        self.controller.calibrate_gripper.side_effect = RuntimeError('SDK failure')
        with self.assertRaisesRegex(RuntimeError, 'SDK failure'):
            self.backend.execute('calibrate_gripper', {'limb_name': 'right'}, self.cancel)
        self.controller._grippers['right'].stop.assert_called_once_with(block=False)

    def test_invalid_or_precancelled_requests_do_not_calibrate(self):
        for params in ({}, {'limb_name': None}, {'limb_name': 'both'},
                       {'limb_name': 'right', 'timeout_s': 15}):
            with self.assertRaises(ValueError):
                validate('calibrate_gripper', params)
        self.controller._grippers['right'].type.return_value = 'suction'
        with self.assertRaises(ValueError):
            self.backend.execute('calibrate_gripper', {'limb_name': 'right'}, self.cancel)
        self.cancel.set()
        with self.assertRaisesRegex(RuntimeError, 'Cancelled'):
            self.backend.execute('calibrate_gripper', {'limb_name': 'right'}, self.cancel)
        self.controller.calibrate_gripper.assert_not_called()

    def test_http_reads_and_stop_work_while_sdk_calibration_blocks(self):
        started, release = threading.Event(), threading.Event()
        def blocking_calibration(limb):
            started.set()
            release.wait(5)
            self.calibrate(limb)
        self.controller.calibrate_gripper.side_effect = blocking_calibration
        server = Server(('127.0.0.1', 0), Handler)
        server.api = RemoteAPI(self.backend, lease_timeout=30)
        thread = threading.Thread(target=lambda: server.serve_forever(poll_interval=.02), daemon=True)
        thread.start()
        try:
            with BaxterRemoteController('http://127.0.0.1:%s' % server.server_port,
                                        heartbeat=False, request_timeout_s=1) as client:
                operation = client.calibrate_gripper('right')
                self.assertTrue(started.wait(1))
                self.assertTrue(client.get_state()['commands_busy'])
                with self.assertRaises(RemoteError):
                    client.calibrate_gripper('left')
                client.stop_gripper('left')
                self.assertFalse(server.api.active['cancel'].is_set())
                client.stop_gripper('right')
                self.assertTrue(server.api.active['cancel'].is_set())
                self.assertFalse(release.is_set())
                self.assertEqual(operation.status()['status'], 'running')
                self.controller._grippers['right'].stop.assert_called_once_with(block=False)
                release.set()
                with self.assertRaises(RemoteError):
                    operation.wait(timeout_s=2)
                self.assertEqual(operation.status()['status'], 'cancelled')
                # The adapter sends another stop after the blocking call returns.
                self.assertEqual(self.controller._grippers['right'].stop.call_count, 2)
                self.controller.calibrate_gripper.side_effect = self.calibrate
                self.assertIs(client.calibrate_gripper('right').wait(timeout_s=2), True)
        finally:
            release.set()
            server.api.close()
            server.shutdown()
            server.server_close()
            thread.join(1)

    def test_simulation_calibrates_only_the_requested_gripper(self):
        backend = SimulationBackend()
        backend._state['grippers']['left']['position_percent'] = 25
        backend._state['grippers']['right']['position_percent'] = 40
        self.assertIs(backend.execute('calibrate_gripper', {'limb_name': 'right'}, self.cancel), True)
        state = backend.state()
        self.assertEqual(state['grippers']['left']['position_percent'], 25)
        self.assertEqual(state['grippers']['right']['position_percent'], 100)
        self.assertFalse(any(state['movement_in_progress'].values()))


if __name__ == '__main__':
    unittest.main()
