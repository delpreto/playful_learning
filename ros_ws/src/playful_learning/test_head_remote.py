"""Head operation, validation, and upload regression checks without ROS."""
import base64
import unittest

from BaxterRemoteController_server import RemoteAPI, validate
from test_remote import BlockingBackend


class HeadBackend(BlockingBackend):
    def __init__(self):
        BlockingBackend.__init__(self)
        self.head = {'pan_rad': 0.0, 'tilt_rad': None, 'tilt_supported': False,
                     'panning': False, 'nodding': False, 'feedback_stale': False}

    def state(self):
        state = BlockingBackend.state(self)
        state['head'] = dict(self.head)
        return state


class HeadRemoteTests(unittest.TestCase):
    def setUp(self):
        self.backend = HeadBackend()
        self.api = RemoteAPI(self.backend, lease_timeout=30)

    def tearDown(self):
        self.backend.finish.set()
        self.api.close()

    def test_head_operation_reserves_control_without_reporting_arm_motion(self):
        self.api.call('set_head_pan_rad', {'angle_rad': .2}, 'owner', 'pan')
        self.assertTrue(self.backend.started.wait(1))
        self.assertFalse(self.api.call('is_movement_in_progress', {'limb_name': 'left'}, 'observer', 'read'))
        with self.assertRaises(RuntimeError):
            self.api.call('show_screen_color', {'color_rgb': [0, 0, 0]}, 'observer', 'screen')
        self.api.call('abort_movement', {'limb_name': 'left'}, 'observer', 'stop-arm')
        self.assertFalse(self.backend.cancel.is_set())
        self.api.call('abort_movement', {}, 'observer', 'stop-all')
        self.assertTrue(self.backend.cancel.is_set())

    def test_head_feedback_loss_cancels_with_connected_owner(self):
        self.api.call('nod_head', {}, 'owner', 'nod')
        self.assertTrue(self.backend.started.wait(1))
        self.backend.head['feedback_stale'] = True
        self.assertTrue(self.backend.stopped.wait(1))
        self.assertTrue(self.backend.cancel.is_set())

    def test_unavailable_or_moving_head_rejects_pan(self):
        for field in ('feedback_stale', 'panning', 'nodding'):
            self.backend.head[field] = True
            with self.assertRaises(RuntimeError):
                self.api.call('set_head_pan_rad', {'angle_rad': 0}, 'owner', field)
            self.backend.head[field] = False
        self.assertEqual(self.backend.calls, 0)

    def test_global_idle_includes_head_motion_from_robot_feedback(self):
        self.backend.head['nodding'] = True
        self.assertTrue(self.api.call('is_movement_in_progress', {}, 'observer', 'read'))
        self.assertFalse(self.api.call('is_movement_in_progress', {'limb_name': 'left'}, 'observer', 'arm-read'))

    def test_tilt_is_explicitly_unsupported_without_acquiring_control(self):
        self.assertIsNone(self.api.call('get_head_tilt_rad', {}, 'reader', 'tilt'))
        with self.assertRaises(NotImplementedError):
            self.api.call('set_head_tilt_rad', {'angle_rad': 0}, 'reader', 'tilt-set')
        self.assertIsNone(self.api.owner)

    def test_screen_request_deduplication_does_not_retain_image_payload(self):
        data = base64.b64encode(b'\x00\x80\xff' * 1024 * 600).decode('ascii')
        params = {'width': 1024, 'height': 600, 'rgb_base64': data}
        accepted = self.api.call('show_screen_image_rgb', params, 'owner', 'image')
        self.assertEqual(self.api.call('show_screen_image_rgb', dict(params), 'owner', 'image'), accepted)
        self.assertEqual(len(self.api.requests[('owner', 'image')][1]['rgb_base64']), 64)
        changed = dict(params, rgb_base64=base64.b64encode(b'\xff\x80\x00' * 1024 * 600).decode('ascii'))
        with self.assertRaises(ValueError):
            self.api.call('show_screen_image_rgb', changed, 'owner', 'image')
        self.assertEqual(self.backend.calls, 1)

    def test_invalid_commands_are_rejected_before_dispatch(self):
        cases = [('set_head_pan_rad', {'angle_rad': float('nan')}),
                 ('set_head_pan_rad', {'angle_rad': 1.5}),
                 ('set_head_pan_rad', {'angle_rad': 0, 'speed_percent': 0}),
                 ('nod_head', {'times': True}), ('nod_head', {'times': 11}),
                 ('set_halo_led', {'red_percent': -1, 'green_percent': 0}),
                 ('set_sonar_leds', {'led_states': [12]}),
                 ('set_sonar_leds', {'led_states': [True]}),
                 ('show_screen_color', {'color_rgb': [256, 0, 0]}),
                 ('show_screen_image_rgb', {'width': 1025, 'height': 1, 'rgb_base64': ''}),
                 ('show_screen_image_rgb', {'width': 1, 'height': 1, 'rgb_base64': '!!!!'}),
                 ('show_screen_image_rgb', {'width': 1, 'height': 1, 'rgb_base64': 'AA=='})]
        for method, params in cases:
            with self.assertRaises(ValueError):
                validate(method, params)
        self.assertEqual(self.backend.calls, 0)


if __name__ == '__main__':
    unittest.main()
