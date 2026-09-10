"""Head controller checks with fake ROS; never contacts Baxter."""
import sys
import threading
import types
import unittest

import BaxterHeadController as head_module


class Message(object):
    def __init__(self, data=None):
        self.data = data


class PanMessage(object):
    REQUEST_PAN_ENABLE = 1
    def __init__(self, target, speed_ratio, enable_pan_request):
        self.target, self.speed_ratio = target, speed_ratio
        self.enable_pan_request = enable_pan_request


class State(object):
    def __init__(self, pan=0, panning=False, nodding=False):
        self.pan, self.isTurning, self.isNodding = pan, panning, nodding


class Clock(object):
    def __init__(self):
        self.now, self.tick = 100.0, None
    def time(self):
        return self.now
    def sleep(self, delay):
        self.now += delay
        if self.tick:
            self.tick()


class Publisher(object):
    def __init__(self, topic, message_type, **options):
        self.topic, self.options, self.messages, self.hook = topic, options, [], None
    def publish(self, message):
        self.messages.append(message)
        if self.hook:
            self.hook(message)


class HeadControllerTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.publishers = Clock(), {}
        self.original_time = head_module.time
        head_module.time = self.clock
        names = ('rospy', 'baxter_core_msgs', 'baxter_core_msgs.msg', 'sensor_msgs',
                 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg')
        self.original_modules = dict((name, sys.modules.get(name)) for name in names)
        modules = dict((name, types.ModuleType(name)) for name in names)
        ros = modules['rospy']
        ros.core = types.ModuleType('core')
        ros.core.is_initialized = lambda: True
        ros.is_shutdown = lambda: False
        ros.init_node = lambda name: self.fail('Existing ROS node must be reused')
        ros.Publisher = self.make_publisher
        class Subscription(object):
            closed = False
            def unregister(self):
                self.closed = True
        self.subscription = Subscription()
        ros.Subscriber = lambda *args, **options: self.subscription
        modules['baxter_core_msgs.msg'].HeadPanCommand = PanMessage
        modules['baxter_core_msgs.msg'].HeadState = State
        modules['sensor_msgs.msg'].Image = Message
        for name in ('Bool', 'Float32', 'UInt16'):
            setattr(modules['std_msgs.msg'], name, Message)
        sys.modules.update(modules)
        self.head = head_module.BaxterHeadController()

    def tearDown(self):
        self.head.close()
        head_module.time = self.original_time
        for name, value in self.original_modules.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value

    def make_publisher(self, topic, message_type, **options):
        publisher = Publisher(topic, message_type, **options)
        self.publishers[topic] = publisher
        return publisher

    def feedback(self, pan=0, panning=False, nodding=False):
        self.head._on_head_state(State(pan, panning, nodding))

    def test_startup_is_nonblocking_and_motion_requires_feedback(self):
        state = self.head.get_state()
        self.assertIsNone(state['pan_rad'])
        self.assertTrue(state['feedback_stale'])
        self.assertIsNone(self.head.get_head_tilt_rad())
        with self.assertRaises(NotImplementedError):
            self.head.set_head_tilt_rad(.1)
        with self.assertRaises(RuntimeError):
            self.head.set_head_pan_rad(.1)
        self.assertEqual(self.head._pan_pub.messages, [])
        self.assertFalse(self.head._pan_pub.options.get('latch', False))
        self.assertFalse(self.head._nod_pub.options.get('latch', False))
        self.assertTrue(self.head._screen_pub.options['latch'])

    def test_pan_waits_for_target_and_reports_unreached_target(self):
        self.feedback()
        self.clock.tick = lambda: self.feedback(pan=.2)
        self.assertEqual(self.head.set_head_pan_rad(.2), .2)
        self.assertEqual(self.head._pan_pub.messages[0].speed_ratio, .25)
        self.clock.tick = lambda: self.feedback(pan=.2)
        with self.assertRaises(RuntimeError) as error:
            self.head.set_head_pan_rad(.6, timeout_s=.1)
        self.assertIn('did not reach', str(error.exception))
        self.assertIsNone(self.head.get_state()['fault'])

    def test_pan_stop_prevents_later_target_publications(self):
        self.feedback()
        stopped = [False]
        def tick():
            self.feedback()
            if not stopped[0]:
                stopped[0] = True
                self.head.stop()
        self.clock.tick = tick
        with self.assertRaises(RuntimeError) as error:
            self.head.set_head_pan_rad(.5)
        self.assertIn('Cancelled', str(error.exception))
        targets = [message.target for message in self.head._pan_pub.messages]
        self.assertEqual(targets[0], .5)
        self.assertTrue(all(value == 0 for value in targets[1:]))

    def test_nod_requires_an_observed_start_and_finish(self):
        self.feedback()
        states = [True, False]
        self.clock.tick = lambda: self.feedback(nodding=states.pop(0) if states else False)
        self.assertTrue(self.head.nod_head())
        self.assertFalse(self.head._nod_pub.messages[-1].data)

    def test_fast_nod_transitions_are_not_lost_between_polls(self):
        self.feedback()
        def gesture(message):
            if message.data:
                self.feedback(nodding=True)
                self.feedback(nodding=False)
        self.head._nod_pub.hook = gesture
        self.assertTrue(self.head.nod_head())
        with self.assertRaises(RuntimeError) as error:
            self.head.nod_head(times=2, internod_delay_s=3)
        self.assertIn('Fresh head feedback', str(error.exception))
        self.assertEqual(sum(message.data for message in self.head._nod_pub.messages), 2)

    def test_nod_that_never_starts_is_an_explicit_failure(self):
        self.feedback()
        self.clock.tick = lambda: self.feedback()
        with self.assertRaises(RuntimeError) as error:
            self.head.nod_head()
        self.assertIn('did not start', str(error.exception))
        self.assertIn('not confirmed', self.head.get_state()['fault'])

    def test_nod_that_never_finishes_is_an_explicit_failure(self):
        self.feedback()
        self.clock.tick = lambda: self.feedback(nodding=True)
        with self.assertRaises(RuntimeError) as error:
            self.head.nod_head()
        self.assertIn('did not finish', str(error.exception))
        self.assertTrue(self.head.get_state()['nodding'])

    def test_cancelled_nod_waits_for_gesture_to_finish(self):
        self.feedback()
        cancel, ticks = threading.Event(), [0]
        def tick():
            ticks[0] += 1
            self.feedback(nodding=ticks[0] < 4)
            cancel.set()
        self.clock.tick = tick
        with self.assertRaises(RuntimeError) as error:
            self.head.nod_head(cancel=cancel)
        self.assertIn('Cancelled', str(error.exception))
        self.assertEqual(ticks[0], 4)
        self.assertIsNone(self.head.get_state()['fault'])

    def test_precancelled_nod_publishes_no_motion(self):
        self.feedback()
        cancel = threading.Event()
        cancel.set()
        with self.assertRaises(RuntimeError):
            self.head.nod_head(cancel=cancel)
        self.assertEqual(self.head._nod_pub.messages, [])

    def test_rgb_screen_dimensions_channels_and_data(self):
        data = b'\xff\x00\x00\x00\x80\xff'
        self.assertEqual(self.head.show_screen_image_rgb(2, 1, data), {'published': True})
        message = self.head._screen_pub.messages[-1]
        self.assertEqual((message.width, message.height, message.step, message.encoding), (2, 1, 6, 'rgb8'))
        self.assertEqual(message.data, data)
        self.head.show_screen_color([1, 2, 3])
        message = self.head._screen_pub.messages[-1]
        self.assertEqual((message.width, message.height), (1024, 600))
        self.assertEqual(message.data, b'\x01\x02\x03' * (1024 * 600))
        with self.assertRaises(ValueError):
            self.head.show_screen_image_rgb(2, 2, data)

    def test_led_masks_repeat_and_close_restores_automatic_control(self):
        self.head.set_sonar_leds([0, 5, 11])
        self.assertEqual(self.head._sonar_pub.messages[-1].data, 0x8821)
        self.head.set_sonar_leds([0, 1] * 6)
        self.assertEqual(self.head._sonar_pub.messages[-1].data, 0x8aaa)
        self.head.set_halo_led(12, 34)
        threading.Event().wait(.04)
        self.assertGreater(len(self.head._red_pub.messages), 1)
        self.head.close()
        self.assertFalse(self.head._led_thread.is_alive())
        self.assertTrue(self.subscription.closed)
        self.assertEqual(self.head._sonar_pub.messages[-1].data, 0)
        count = len(self.head._sonar_pub.messages)
        threading.Event().wait(.02)
        self.assertEqual(len(self.head._sonar_pub.messages), count)


if __name__ == '__main__':
    unittest.main()
