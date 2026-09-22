"""Capability discovery tests that do not access physical webcams."""

import errno
import struct
import sys
import unittest
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import webcam_modes as modes


class WebcamModeTests(unittest.TestCase):
    def test_windows_range_parser_and_resolution_priority(self):
        output = '''
  vcodec=mjpeg min s=1920x1080 fps=15 max s=3840x2160 fps=30
  vcodec=mjpeg min s=1920x1080 fps=30 max s=1920x1080 fps=120
  pixel_format=yuyv422 min s=3840x2160 fps=5 max s=3840x2160 fps=10
  vcodec=mjpeg min s=1920x1080 fps=15 max s=3840x2160 fps=30
  pixel_format=nv12 min s=1280x720 fps=15 max s=1280x720 fps=30000/1001
'''
        parsed = modes.parse_dshow_modes(output)
        with patch.object(modes.sys, "platform", "win32"), \
                patch.object(modes, "_windows_modes", return_value=parsed):
            ranked = modes.discover_modes(SimpleNamespace())
        self.assertEqual(ranked[0], modes.CameraMode(3840, 2160, Fraction(30), "MJPG"))
        self.assertEqual(ranked[1], modes.CameraMode(3840, 2160, Fraction(10), "YUY2"))
        self.assertEqual(ranked[2].fps, 120)
        self.assertEqual(ranked[3].fps, Fraction(30000, 1001))
        self.assertEqual(len(ranked), 4)

    def test_windows_identical_names_use_distinct_device_paths(self):
        output = [(32, "dshow", "vcodec=mjpeg min s=640x480 fps=30 max s=640x480 fps=30")]
        capture = Mock()
        capture.__enter__ = Mock(return_value=output)
        capture.__exit__ = Mock(return_value=False)
        with patch.object(modes.av, "formats_available", {"dshow"}), \
                patch.object(modes.av.logging, "Capture", return_value=capture), \
                patch.object(modes.av, "open", side_effect=ValueError("list complete")) as av_open:
            for path in (r"\\?\usb#camera-one", r"\\?\usb#camera-two"):
                modes._windows_modes(SimpleNamespace(name="Identical Camera", path=path))
        self.assertEqual(av_open.call_args_list[0].args[0], r"video=@device_pnp_\\?\usb#camera-one")
        self.assertEqual(av_open.call_args_list[1].args[0], r"video=@device_pnp_\\?\usb#camera-two")

    def test_linux_discrete_and_stepwise_modes(self):
        mjpg = int.from_bytes(b"MJPG", "little")
        yuyv = int.from_bytes(b"YUYV", "little")

        def ioctl(fd, request, data, mutate):
            self.assertEqual(fd, 42)
            self.assertTrue(mutate)
            number = request & 255
            values = list(struct.unpack("=" + "I" * (len(data) // 4), data))
            index = values[0]
            if number == 2 and index < 2:
                self.assertEqual(request, 0xC0405602)
                self.assertEqual(values[1], 1)
                values[11] = (mjpg, yuyv)[index]
            elif number == 74 and index == 0:
                self.assertEqual(request, 0xC02C564A)
                if values[1] == mjpg:
                    values[2:5] = [1, 3840, 2160]
                else:
                    # The upper bounds are not on a stride; snap down to 1280x720.
                    values[2:9] = [3, 640, 1281, 16, 480, 723, 8]
            elif number == 75 and values[1] == mjpg and index < 2:
                self.assertEqual(request, 0xC034564B)
                self.assertEqual(values[2:4], [3840, 2160])
                values[4:7] = [1, 1001, (15000, 30000)[index]]
            elif number == 75 and values[1] == yuyv and index == 0:
                self.assertEqual(values[2:4], [1280, 720])
                values[4:11] = [3, 1, 120, 1, 15, 1, 120]
            else:
                raise OSError(errno.EINVAL, "enumeration complete")
            struct.pack_into("=" + "I" * len(values), data, 0, *values)

        fcntl = SimpleNamespace(ioctl=ioctl)
        with patch.dict(sys.modules, {"fcntl": fcntl}), \
                patch.object(modes.os, "O_NONBLOCK", 2048, create=True), \
                patch.object(modes.os, "open", return_value=42) as open_device, \
                patch.object(modes.os, "close") as close_device, \
                patch.object(modes.sys, "platform", "linux"):
            ranked = modes.discover_modes(SimpleNamespace(index=3, path="/dev/video3"))
        self.assertEqual(ranked[0], modes.CameraMode(3840, 2160, Fraction(30000, 1001), "MJPG"))
        self.assertEqual(ranked[1].fps, Fraction(15000, 1001))
        self.assertEqual(ranked[2], modes.CameraMode(1280, 720, Fraction(120), "YUYV"))
        self.assertEqual(open_device.call_args.args[0], "/dev/video3")
        close_device.assert_called_once_with(42)

    def test_linux_device_closes_on_driver_error(self):
        fcntl = SimpleNamespace(ioctl=Mock(side_effect=OSError(errno.EIO, "disconnected")))
        with patch.dict(sys.modules, {"fcntl": fcntl}), \
                patch.object(modes.os, "O_NONBLOCK", 2048, create=True), \
                patch.object(modes.os, "open", return_value=42), \
                patch.object(modes.os, "close") as close_device:
            with self.assertRaises(OSError):
                modes._linux_modes(SimpleNamespace(index=0, path="/dev/video0"))
        close_device.assert_called_once_with(42)

    def test_rejects_silent_resolution_or_fps_fallback(self):
        requested = modes.CameraMode(3840, 2160, Fraction(30), "MJPG")
        for width, height, fps in [(1920, 1080, 30), (3840, 2160, 15), (3840, 2160, 0)]:
            with self.subTest(width=width, height=height, fps=fps):
                actual = {modes.cv2.CAP_PROP_FRAME_WIDTH: width,
                          modes.cv2.CAP_PROP_FRAME_HEIGHT: height,
                          modes.cv2.CAP_PROP_FPS: fps}
                camera = Mock()
                camera.get.side_effect = actual.__getitem__
                with self.assertRaisesRegex(RuntimeError, "camera negotiated"):
                    modes.configure_camera(camera, requested)

    def test_allows_driver_rounding_of_fractional_fps(self):
        requested = modes.CameraMode(1920, 1080, Fraction(30000, 1001), "MJPG")
        actual = {modes.cv2.CAP_PROP_FRAME_WIDTH: 1920,
                  modes.cv2.CAP_PROP_FRAME_HEIGHT: 1080,
                  modes.cv2.CAP_PROP_FPS: 29.97003}
        camera = Mock()
        camera.get.side_effect = actual.__getitem__
        configured = modes.configure_camera(camera, requested)
        self.assertAlmostEqual(float(configured.fps), float(requested.fps), places=5)


if __name__ == "__main__":
    unittest.main()
