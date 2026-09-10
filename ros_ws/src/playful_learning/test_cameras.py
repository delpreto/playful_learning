"""Hardware-free PNG and on-demand camera tests: python -B -m unittest test_cameras."""
from __future__ import print_function

import json
import struct
import sys
import threading
import types
import unittest
import zlib

try:
    from httplib import HTTPConnection
except ImportError:
    from http.client import HTTPConnection

from BaxterCameras import BaxterCameras, image_to_png


class Image(object):
    def __init__(self, encoding="rgb8", width=2, height=1, step=6,
                 data=b"\x01\x02\x03\x04\x05\x06"):
        self.encoding, self.width, self.height = encoding, width, height
        self.step, self.data, self.is_bigendian = step, data, 0


def read_png(data):
    """Decode the tiny 8-bit test images independently of the encoder."""
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise AssertionError("Not a PNG")
    offset, compressed, header = 8, b"", None
    while offset < len(data):
        size = struct.unpack(">I", data[offset:offset + 4])[0]
        kind = data[offset + 4:offset + 8]
        payload = data[offset + 8:offset + 8 + size]
        crc = struct.unpack(">I", data[offset + 8 + size:offset + 12 + size])[0]
        if zlib.crc32(kind + payload) & 0xffffffff != crc:
            raise AssertionError("Invalid PNG CRC")
        if kind == b"IHDR":
            header = struct.unpack(">IIBBBBB", payload)
        elif kind == b"IDAT":
            compressed += payload
        elif kind == b"IEND":
            break
        offset += size + 12
    width, height, bits, color, compression, filtering, interlace = header
    if (bits, compression, filtering, interlace) != (8, 0, 0, 0):
        raise AssertionError("Expected an ordinary 8-bit noninterlaced PNG")
    channels = {0: 1, 2: 3, 6: 4}[color]
    raw = bytearray(zlib.decompress(compressed))
    stride, pixels = width * channels, bytearray()
    previous = bytearray(stride)
    if len(raw) != height * (stride + 1):
        raise AssertionError("PNG scanline length does not match dimensions")
    for row_index in range(height):
        begin = row_index * (stride + 1)
        filter_type, row = raw[begin], raw[begin + 1:begin + 1 + stride]
        for index in range(stride):
            left = row[index - channels] if index >= channels else 0
            up = previous[index]
            corner = previous[index - channels] if index >= channels else 0
            prediction = left + up - corner
            paeth = min((left, up, corner), key=lambda value: abs(prediction - value))
            correction = (0, left, up, (left + up) // 2, paeth)[filter_type]
            row[index] = (row[index] + correction) & 255
        pixels.extend(row)
        previous = row
    return width, height, channels, pixels


class PngTests(unittest.TestCase):
    def test_rgb_and_bgr_preserve_colors(self):
        for encoding, values in [("rgb8", [1, 2, 3, 4, 5, 6]),
                                 ("bgr8", [3, 2, 1, 6, 5, 4])]:
            result = read_png(image_to_png(Image(encoding=encoding, data=bytearray(values))))
            self.assertEqual(result, (2, 1, 3, bytearray([1, 2, 3, 4, 5, 6])))

    def test_alpha_and_monochrome_channels(self):
        for encoding, values in [("rgba8", [1, 2, 3, 99]), ("bgra8", [3, 2, 1, 99])]:
            result = read_png(image_to_png(Image(encoding, 1, 1, 4, bytearray(values))))
            self.assertEqual(result, (1, 1, 4, bytearray([1, 2, 3, 99])))
        result = read_png(image_to_png(Image("mono8", 2, 1, 2, b"\x00\xff")))
        self.assertEqual(result, (2, 1, 1, bytearray([0, 255])))

    def test_padded_rows_do_not_shift_pixels(self):
        data = bytearray([3, 2, 1, 6, 5, 4, 250, 251,
                          9, 8, 7, 12, 11, 10, 252, 253])
        result = read_png(image_to_png(Image("bgr8", 2, 2, 8, data)))
        self.assertEqual(result, (2, 2, 3, bytearray(range(1, 13))))

    def test_invalid_image_layout_and_encoding_are_rejected(self):
        for message in [Image(encoding="mono16"), Image(width=0), Image(height=-1),
                        Image(step=5), Image(height=2, data=b"\x00" * 11)]:
            with self.assertRaises(ValueError):
                image_to_png(message)


class CameraGetterTests(unittest.TestCase):
    def setUp(self):
        self.original_modules = dict((name, sys.modules.get(name))
            for name in ("rospy", "sensor_msgs", "sensor_msgs.msg"))
        self.calls, self.error = [], None
        rospy = types.ModuleType("rospy")
        rospy.ROSException = type("ROSException", (Exception,), {})
        rospy.ROSInterruptException = type("ROSInterruptException", (rospy.ROSException,), {})
        rospy.wait_for_message = self.wait_for_message
        sensor_msgs = types.ModuleType("sensor_msgs")
        messages = types.ModuleType("sensor_msgs.msg")
        messages.Image = Image
        sensor_msgs.msg = messages
        sys.modules.update({"rospy": rospy, "sensor_msgs": sensor_msgs, "sensor_msgs.msg": messages})
        self.rospy = rospy
        self.cameras = BaxterCameras()

    def tearDown(self):
        for name, original in self.original_modules.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original

    def wait_for_message(self, topic, message_type, timeout=None):
        self.calls.append((topic, message_type, timeout))
        if self.error:
            raise self.error
        return Image(data=bytearray([len(self.calls), 2, 3, 4, 5, 6]))

    def test_each_getter_reads_one_new_frame_from_the_expected_topic(self):
        left = self.cameras.get_wrist_camera_frame("left")
        right = self.cameras.get_wrist_camera_frame("right")
        repeated = self.cameras.get_camera_frame("left_hand_camera")
        self.assertNotEqual(left, repeated)
        for png in (left, right, repeated):
            self.assertEqual(read_png(png)[:3], (2, 1, 3))
        self.assertEqual(self.calls, [
            ("/cameras/left_hand_camera/image", Image, 1.0),
            ("/cameras/right_hand_camera/image", Image, 1.0),
            ("/cameras/left_hand_camera/image", Image, 1.0)])

    def test_invalid_names_do_not_make_ros_calls(self):
        for name in ("left", "unknown", "../head_camera", "head_camera"):
            with self.assertRaises(ValueError):
                self.cameras.get_camera_frame(name)
        with self.assertRaises(ValueError):
            self.cameras.get_wrist_camera_frame("head")
        self.assertEqual(self.calls, [])

    def test_camera_timeout_is_an_explicit_error(self):
        self.error = self.rospy.ROSException("timeout exceeded")
        with self.assertRaises(RuntimeError) as caught:
            self.cameras.get_wrist_camera_frame("left")
        self.assertIn("left_hand_camera", str(caught.exception))
        self.assertEqual(len(self.calls), 1)


class CameraHttpTests(unittest.TestCase):
    def test_waiting_for_a_camera_does_not_block_state_or_stop(self):
        from BaxterRemoteController_server import Handler, RemoteAPI, Server
        from test_remote import BlockingBackend
        class Backend(BlockingBackend):
            def get_camera_frame(self, name):
                camera_waiting.set()
                release_camera.wait(5)
                return image_to_png(Image())
        camera_waiting, release_camera, camera_done = (threading.Event() for unused in range(3))
        server = Server(("127.0.0.1", 0), Handler)
        server.api, server.token = RemoteAPI(Backend()), "test-token"
        worker = threading.Thread(target=lambda: server.serve_forever(poll_interval=.02))
        worker.daemon = True
        worker.start()
        def get_image():
            connection = HTTPConnection("127.0.0.1", server.server_port, timeout=3)
            try:
                connection.request("GET", "/camera/left_hand_camera.png", headers={"Authorization": "Bearer test-token"})
                connection.getresponse().read()
            finally:
                connection.close()
                camera_done.set()
        camera_worker = threading.Thread(target=get_image)
        camera_worker.daemon = True
        camera_worker.start()
        try:
            self.assertTrue(camera_waiting.wait(1))
            for method in ("get_state", "abort_movement"):
                connection = HTTPConnection("127.0.0.1", server.server_port, timeout=1)
                try:
                    body = json.dumps({"jsonrpc": "2.0", "id": method, "method": method, "params": {}})
                    connection.request("POST", "/rpc", body, headers={
                        "Authorization": "Bearer test-token", "X-Client-ID": "test-client"})
                    response = connection.getresponse()
                    result = json.loads(response.read().decode("utf-8"))
                    self.assertEqual(response.status, 200)
                    self.assertIn("result", result)
                finally:
                    connection.close()
            self.assertFalse(camera_done.is_set())
            self.assertTrue(server.api.backend.stopped.is_set())
        finally:
            release_camera.set()
            camera_worker.join(1)
            server.api.close()
            server.shutdown()
            server.server_close()
            worker.join(1)

    def test_binary_frames_and_explicit_http_errors(self):
        from BaxterRemoteController_server import Handler, Server
        class Backend(object):
            error = None
            calls = 0
            def get_camera_frame(self, name):
                self.calls += 1
                if self.error:
                    raise self.error
                return image_to_png(Image())
        class API(object):
            closed = threading.Event()
            backend = Backend()
            owner, last_seen = "controller", 123
        server = Server(("127.0.0.1", 0), Handler)
        server.api, server.token = API(), "test-token"
        worker = threading.Thread(target=lambda: server.serve_forever(poll_interval=.02))
        worker.daemon = True
        worker.start()
        try:
            cases = [("/camera/left_hand_camera.png", "bad-token", 401, "application/json"),
                     ("/camera/unknown.png", "test-token", 404, "application/json"),
                     ("/camera/head_camera.png", "test-token", 404, "application/json"),
                     ("/camera/left_hand_camera.png", "test-token", 200, "image/png"),
                     ("/camera/left_hand_camera.png", "test-token", 503, "application/json")]
            for path, token, expected_status, content_type in cases:
                if expected_status == 503:
                    server.api.backend.error = RuntimeError("Camera unavailable")
                connection = HTTPConnection("127.0.0.1", server.server_port, timeout=2)
                try:
                    connection.request("GET", path, headers={"Authorization": "Bearer " + token})
                    response = connection.getresponse()
                    data = response.read()
                    self.assertEqual(response.status, expected_status)
                    self.assertEqual(response.getheader("Content-Type"), content_type)
                    self.assertEqual(response.getheader("Cache-Control"), "no-store")
                    if expected_status == 200:
                        self.assertEqual(read_png(data)[:3], (2, 1, 3))
                finally:
                    connection.close()
            self.assertEqual(server.api.backend.calls, 2)
            self.assertEqual((server.api.owner, server.api.last_seen), ("controller", 123))
        finally:
            server.shutdown()
            server.server_close()
            worker.join(1)


if __name__ == "__main__":
    unittest.main()
