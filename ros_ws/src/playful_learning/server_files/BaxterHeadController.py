"""Head pan/nod, LEDs, and RGB display; Python 2.7 and installed ROS only.

Adapted from learning_trajectories/BaxterHeadController.py.

Pan/nod wait for measured feedback. LED/display results acknowledge publication,
not hardware state. Baxter exposes a nod gesture, not controllable head tilt.
"""
from __future__ import division

import math
import threading
import time

try:
    NUMBERS = (int, long, float)
    INTEGERS = (int, long)
except NameError:
    NUMBERS, INTEGERS = (int, float), (int,)


class BaxterHeadController(object):
    def __init__(self):
        import rospy
        from baxter_core_msgs.msg import HeadPanCommand, HeadState
        from sensor_msgs.msg import Image
        from std_msgs.msg import Bool, Float32, UInt16
        self._ros, self._Pan, self._Image = rospy, HeadPanCommand, Image
        self._Bool, self._Float32, self._UInt16 = Bool, Float32, UInt16
        if not rospy.core.is_initialized():
            rospy.init_node('baxter_head_controller')
        self._lock = threading.RLock()
        self._closed, self._stopped = threading.Event(), threading.Event()
        self._state = None
        self._received = 0
        self._sequence = self._nod_starts = self._nod_finishes = 0
        self._motion = None
        self._fault = None
        self._sonar_mask, self._halo = None, None
        self._pan_pub = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=1)
        self._nod_pub = rospy.Publisher('/robot/head/command_head_nod', Bool, queue_size=1)
        self._screen_pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=1, latch=True)
        prefix = '/robot/sonar/head_sonar/lights/'
        self._red_pub = rospy.Publisher(prefix + 'set_red_level', Float32, queue_size=1)
        self._green_pub = rospy.Publisher(prefix + 'set_green_level', Float32, queue_size=1)
        self._sonar_pub = rospy.Publisher(prefix + 'set_lights', UInt16, queue_size=1)
        self._subscriber = rospy.Subscriber('/robot/head/head_state', HeadState, self._on_head_state, queue_size=1)
        self._led_thread = threading.Thread(target=self._publish_leds)
        self._led_thread.daemon = True
        self._led_thread.start()

    def _on_head_state(self, message):
        with self._lock:
            previous = self._state.isNodding if self._state is not None else False
            self._nod_starts += int(message.isNodding and not previous)
            self._nod_finishes += int(previous and not message.isNodding)
            self._state, self._received = message, time.time()
            self._sequence += 1

    def get_state(self):
        with self._lock:
            state = self._state
            age = time.time() - self._received if state is not None else None
            return {'pan_rad': state.pan if state is not None else None,
                    'tilt_rad': None, 'tilt_supported': False,
                    'panning': bool(state and state.isTurning),
                    'nodding': bool(state and state.isNodding),
                    'feedback_age_s': age, 'feedback_stale': age is None or age > 2,
                    'fault': self._fault}

    def get_head_pan_rad(self):
        return self.get_state()['pan_rad']

    def get_head_tilt_rad(self):
        return None

    def set_head_tilt_rad(self, angle_rad):
        raise NotImplementedError('Baxter supports nod gestures, not a commanded tilt angle')

    def _begin_motion(self, name, cancel):
        with self._lock:
            if self._closed.is_set() or self._ros.is_shutdown():
                raise RuntimeError('Head controller is closed')
            if cancel.is_set():
                raise RuntimeError('Cancelled before head movement')
            state = self.get_state()
            if state['feedback_stale'] or self._fault:
                raise RuntimeError(self._fault or 'Fresh head feedback is required')
            if self._motion or state['panning'] or state['nodding']:
                raise RuntimeError('Head is already moving')
            self._motion = name
            self._stopped.clear()

    def set_head_pan_rad(self, angle_rad, speed_percent=25, timeout_s=10,
                         tolerance_rad=.05, cancel=None):
        for value, low, high in ((angle_rad, -1.39, 1.39), (speed_percent, 1, 100),
                                  (timeout_s, .1, 120), (tolerance_rad, .0001, .1)):
            if (not isinstance(value, NUMBERS) or isinstance(value, bool) or
                    math.isnan(value) or math.isinf(value) or not low <= value <= high):
                raise ValueError('Invalid head pan angle, speed, timeout, or tolerance')
        cancel = cancel if cancel is not None else threading.Event()
        self._begin_motion('pan', cancel)
        started, stopping_at, error = time.time(), None, None
        baseline = self._sequence
        try:
            while True:
                with self._lock:
                    state = self.get_state()
                    cancelled = cancel.is_set() or self._stopped.is_set() or self._closed.is_set()
                    if state['feedback_stale'] or self._ros.is_shutdown():
                        self.stop()
                        self._fault = 'Head feedback lost; stop was not confirmed'
                        raise RuntimeError(self._fault)
                    if stopping_at is None and (cancelled or time.time() - started > timeout_s):
                        error = 'Cancelled' if cancelled else 'Head did not reach its pan target'
                        self.stop()
                        stopping_at, baseline = time.time(), self._sequence
                    if stopping_at is not None:
                        if self._sequence > baseline and not state['panning']:
                            raise RuntimeError(error)
                        if time.time() - stopping_at > 3:
                            self._fault = 'Head pan stop was not confirmed'
                            raise RuntimeError(self._fault)
                    else:
                        if self._sequence > baseline and not state['panning'] and abs(state['pan_rad'] - angle_rad) <= tolerance_rad:
                            return state['pan_rad']
                        self._pan_pub.publish(self._Pan(angle_rad, speed_percent / 100, self._Pan.REQUEST_PAN_ENABLE))
                time.sleep(.02)
        finally:
            with self._lock:
                self._motion = None

    def nod_head(self, times=1, internod_delay_s=0, cancel=None):
        if not isinstance(times, INTEGERS) or isinstance(times, bool) or not 1 <= times <= 10:
            raise ValueError('times must be an integer from 1 to 10')
        if (not isinstance(internod_delay_s, NUMBERS) or isinstance(internod_delay_s, bool) or
                math.isnan(internod_delay_s) or math.isinf(internod_delay_s) or not 0 <= internod_delay_s <= 5):
            raise ValueError('internod_delay_s must be between zero and five')
        cancel = cancel if cancel is not None else threading.Event()
        self._begin_motion('nod', cancel)
        try:
            for index in range(times):
                with self._lock:
                    if cancel.is_set() or self._stopped.is_set() or self._closed.is_set():
                        raise RuntimeError('Cancelled before head nod')
                    if self.get_state()['feedback_stale'] or self._ros.is_shutdown():
                        raise RuntimeError('Fresh head feedback is required before each nod')
                    baseline_start, baseline_finish = self._nod_starts, self._nod_finishes
                    self._nod_pub.publish(self._Bool(True))
                started, began_at, stopping_at, error = time.time(), None, None, None
                while True:
                    with self._lock:
                        state = self.get_state()
                        if state['feedback_stale'] or self._ros.is_shutdown():
                            self.stop()
                            self._fault = 'Head feedback lost; nod stop was not confirmed'
                            raise RuntimeError(self._fault)
                        if self._nod_starts > baseline_start and began_at is None:
                            began_at = time.time()
                        cancelled = cancel.is_set() or self._stopped.is_set() or self._closed.is_set()
                        expired = time.time() - (began_at or started) > 5
                        if stopping_at is None and (cancelled or expired):
                            error = 'Cancelled' if cancelled else 'Head nod did not %s' % ('finish' if began_at else 'start')
                            self.stop()
                            stopping_at = time.time()
                        finished = (began_at is not None and self._nod_finishes > baseline_finish and not state['nodding'])
                        if finished:
                            if error:
                                raise RuntimeError(error)
                            self._nod_pub.publish(self._Bool(False))
                            break
                        if stopping_at is not None:
                            if time.time() - stopping_at > 3:
                                self._fault = 'Head nod stop was not confirmed (%s)' % error
                                raise RuntimeError(self._fault)
                        elif began_at is None:
                            self._nod_pub.publish(self._Bool(True))
                        else:
                            # False releases the request; the gesture may still need to finish.
                            self._nod_pub.publish(self._Bool(False))
                    time.sleep(.02)
                if index + 1 < times:
                    until = time.time() + internod_delay_s
                    while time.time() < until:
                        if cancel.is_set() or self._stopped.is_set() or self._closed.is_set():
                            raise RuntimeError('Cancelled between head nods')
                        time.sleep(min(.02, max(0, until - time.time())))
            return True
        finally:
            with self._lock:
                self._motion = None

    def stop(self):
        """Signal cancellation and publish hold/release; this is not a stop acknowledgement."""
        with self._lock:
            self._stopped.set()
            self._nod_pub.publish(self._Bool(False))
            state = self.get_state()
            if not state['feedback_stale']:
                self._pan_pub.publish(self._Pan(state['pan_rad'], .25, self._Pan.REQUEST_PAN_ENABLE))

    def set_halo_led(self, red_percent, green_percent):
        if any(not isinstance(v, NUMBERS) or isinstance(v, bool) or not 0 <= v <= 100
               for v in (red_percent, green_percent)):
            raise ValueError('Halo levels must be percentages from zero to 100')
        with self._lock:
            if self._closed.is_set():
                raise RuntimeError('Head controller is closed')
            self._halo = (red_percent, green_percent)
            self._red_pub.publish(self._Float32(red_percent))
            self._green_pub.publish(self._Float32(green_percent))
        return {'published': True}

    def set_sonar_leds(self, led_states):
        if led_states in ('auto', 'on', 'off'):
            mask = {'auto': 0, 'on': 0x8fff, 'off': 0x8000}[led_states]
        elif isinstance(led_states, (list, tuple)):
            mask = 0x8000
            if len(led_states) == 12 and all(isinstance(v, INTEGERS) and v in (0, 1) for v in led_states):
                indices = [i for i, value in enumerate(led_states) if value]
            else:
                if any(not isinstance(v, INTEGERS) or isinstance(v, bool) or not 0 <= v <= 11 for v in led_states):
                    raise ValueError('Sonar LED indices must be integers from zero to 11')
                indices = led_states
            for index in indices:
                mask |= 1 << index
        else:
            raise ValueError('Use auto/on/off, twelve LED states, or indices from zero to 11')
        with self._lock:
            if self._closed.is_set():
                raise RuntimeError('Head controller is closed')
            self._sonar_mask = mask if mask else None
            self._sonar_pub.publish(self._UInt16(mask))
        return {'published': True}

    def _publish_leds(self):
        # Aim above 100 Hz to leave room for publication/scheduling overhead.
        while not self._closed.wait(.005):
            with self._lock:
                if self._closed.is_set() or self._ros.is_shutdown():
                    return
                if self._sonar_mask is not None:
                    self._sonar_pub.publish(self._UInt16(self._sonar_mask))
                if self._halo is not None:
                    self._red_pub.publish(self._Float32(self._halo[0]))
                    self._green_pub.publish(self._Float32(self._halo[1]))

    def show_screen_color(self, color_rgb):
        if (not isinstance(color_rgb, (list, tuple)) or len(color_rgb) != 3 or
                any(not isinstance(v, INTEGERS) or isinstance(v, bool) or not 0 <= v <= 255 for v in color_rgb)):
            raise ValueError('color_rgb must contain three integers from zero to 255')
        return self.show_screen_image_rgb(1024, 600, bytes(bytearray(color_rgb)) * (1024 * 600))

    def show_screen_image_rgb(self, width, height, rgb_data):
        if (not isinstance(width, INTEGERS) or not isinstance(height, INTEGERS) or
                isinstance(width, bool) or isinstance(height, bool) or
                not 1 <= width <= 1024 or not 1 <= height <= 600 or
                not isinstance(rgb_data, (bytes, bytearray)) or len(rgb_data) != width * height * 3):
            raise ValueError('Use RGB bytes matching dimensions up to 1024 by 600')
        message = self._Image()
        message.width, message.height, message.encoding = width, height, 'rgb8'
        message.is_bigendian, message.step, message.data = 0, width * 3, bytes(rgb_data)
        with self._lock:
            if self._closed.is_set():
                raise RuntimeError('Head controller is closed')
            self._screen_pub.publish(message)
        return {'published': True}

    def close(self):
        with self._lock:
            if self._closed.is_set():
                return
            self.stop()
            self._closed.set()
            self._sonar_mask, self._halo = None, None
            self._sonar_pub.publish(self._UInt16(0))
        self._led_thread.join(.5)
        self._subscriber.unregister()
