"""Native camera option and resource tests without accessing physical devices."""

import sys
import unittest
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import av
import pyav_webcam_capture as capture
from webcam_modes import CameraMode


class NativeCaptureTests(unittest.TestCase):
    mode = CameraMode(64, 48, Fraction(30000, 1001), "MJPG")
    info = SimpleNamespace(index=7, path=r"\\?\usb#camera:unique", name="Identical camera")

    def container(self, codec="mjpeg", width=64, height=48, rate=Fraction(30000, 1001),
                  pixel_format="yuvj422p", frames=None):
        context = SimpleNamespace(name=codec, width=width, height=height,
                                  format=SimpleNamespace(name=pixel_format))
        stream = SimpleNamespace(codec_context=context, average_rate=rate)
        result = Mock(streams=SimpleNamespace(video=[stream]))
        if frames is None:
            frames = [av.VideoFrame(width, height, "yuv420p")]
        result.decode.return_value = iter(frames)
        return result

    def open_mock(self, container, platform="win32", info=None, mode=None):
        with patch.object(capture.sys, "platform", platform), \
                patch.object(capture.av, "formats_available", {"dshow", "v4l2"}), \
                patch.object(capture.av, "open", return_value=container) as opener:
            camera = capture.open_camera(info or self.info, mode or self.mode)
        return camera, opener

    def test_windows_unique_path_native_frame_and_idempotent_close(self):
        frame = av.VideoFrame(64, 48, "yuvj422p")
        container = self.container(frames=[frame])
        camera, opener = self.open_mock(container)
        self.assertEqual(opener.call_args.args[0], r"video=@device_pnp_\\?\usb#camera_unique")
        self.assertEqual(opener.call_args.kwargs["format"], "dshow")
        options = opener.call_args.kwargs["options"]
        self.assertEqual(options["video_size"], "64x48")
        self.assertEqual(options["framerate"], "30000/1001")
        self.assertEqual(options["use_video_device_timestamps"], "false")
        self.assertGreater(int(options["rtbufsize"]), 0)
        self.assertNotIn("vcodec", options)  # PyAV silently ignores this CLI option.
        self.assertIs(camera.read(), frame)
        self.assertEqual(frame.format.name, "yuvj422p")
        camera.close()
        camera.close()
        container.close.assert_called_once()
        with self.assertRaisesRegex(RuntimeError, "closed"):
            camera.read()

    def test_linux_requests_compressed_format_and_unique_path(self):
        info = SimpleNamespace(index=7, path="/dev/v4l/by-id/usb-camera-index0")
        camera, opener = self.open_mock(self.container(), "linux", info)
        self.assertEqual(opener.call_args.args[0], info.path)
        self.assertEqual(opener.call_args.kwargs["format"], "v4l2")
        self.assertEqual(opener.call_args.kwargs["options"]["input_format"], "mjpeg")
        self.assertEqual(opener.call_args.kwargs["options"]["timestamps"], "abs")
        self.assertEqual(opener.call_args.kwargs["timeout"], (3.0, 3.0))
        camera.close()

    def test_raw_format_option_mapping_on_both_platforms(self):
        mode = CameraMode(64, 48, Fraction(30), "YUY2")
        for platform, option in [("win32", "pixel_format"), ("linux", "input_format")]:
            with self.subTest(platform=platform):
                container = self.container(codec="rawvideo", rate=Fraction(30),
                                           pixel_format="yuyv422")
                camera, opener = self.open_mock(container, platform, mode=mode)
                self.assertEqual(opener.call_args.kwargs["options"][option], "yuyv422")
                camera.close()

    def test_init_failures_release_the_camera(self):
        containers = [self.container(width=32), self.container(rate=Fraction(15)),
                      self.container(codec="h264")]
        no_video = self.container()
        no_video.streams.video = []
        containers.append(no_video)
        for container in containers:
            with self.subTest(container=container):
                with self.assertRaises(RuntimeError):
                    self.open_mock(container)
                container.close.assert_called_once()

    def test_unknown_reported_rate_does_not_reject_otherwise_valid_mode(self):
        camera, _ = self.open_mock(self.container(rate=None))
        camera.close()

    def test_stream_end_and_changed_dimensions_are_errors(self):
        camera, _ = self.open_mock(self.container(frames=[]))
        with self.assertRaisesRegex(RuntimeError, "ended"):
            camera.read()
        camera.close()
        camera, _ = self.open_mock(self.container(frames=[av.VideoFrame(32, 24)]))
        with self.assertRaisesRegex(RuntimeError, "dimensions"):
            camera.read()
        camera.close()

    def test_missing_windows_path_never_falls_back_to_friendly_name(self):
        with patch.object(capture.sys, "platform", "win32"), \
                patch.object(capture.av, "open") as opener:
            with self.assertRaisesRegex(RuntimeError, "unique"):
                capture.open_camera(SimpleNamespace(index=0, path="", name="Duplicate"), self.mode)
        opener.assert_not_called()

    def test_linux_source_epoch_is_saved_before_frame_pts_is_replaced(self):
        frame = av.VideoFrame(64, 48)
        frame.time_base = Fraction(1, 1_000_000)
        frame.pts = 1_700_000_000_123456
        camera, _ = self.open_mock(self.container(frames=[frame]), "linux")
        self.assertIs(camera.read(), frame)
        frame.pts = 0
        self.assertEqual(camera.frame_epoch_ns, 1_700_000_000_123456_000)
        camera.close()

    def test_windows_queued_frames_preserve_capture_intervals(self):
        frames = []
        for pts in (100, 140, 180):
            frame = av.VideoFrame(64, 48)
            frame.time_base = Fraction(1, 1000)
            frame.pts = pts
            frames.append(frame)
        # First read estimates the graph-clock offset. Delivery of the next
        # image is delayed 100 ms; its capture time advances only 40 ms.
        with patch.object(capture.time, "time_ns", return_value=1_700_000_000_000_000_000), \
                patch.object(capture.time, "perf_counter_ns",
                             side_effect=[0, 100_000_000, 200_000_000, 210_000_000]):
            camera, _ = self.open_mock(self.container(frames=frames))
            timestamps = []
            for _ in frames:
                camera.read()
                timestamps.append(camera.frame_epoch_ns)
        self.assertEqual(timestamps, [1_700_000_000_100_000_000,
                                      1_700_000_000_140_000_000,
                                      1_700_000_000_180_000_000])
        camera.close()

    def test_missing_source_timestamp_clears_previous_frame_time(self):
        first, second = av.VideoFrame(64, 48), av.VideoFrame(64, 48)
        first.time_base, first.pts = Fraction(1, 1000), 1_700_000_000_000
        camera, _ = self.open_mock(self.container(frames=[first, second]), "linux")
        camera.read()
        self.assertIsNotNone(camera.frame_epoch_ns)
        camera.read()
        self.assertIsNone(camera.frame_epoch_ns)
        camera.close()

    def test_windows_freezes_best_warmup_clock_estimate(self):
        frames = []
        for pts in (100, 140, 180):
            frame = av.VideoFrame(64, 48)
            frame.time_base, frame.pts = Fraction(1, 1000), pts
            frames.append(frame)
        with patch.object(capture.time, "time_ns", return_value=1_700_000_000_000_000_000), \
                patch.object(capture.time, "perf_counter_ns",
                             side_effect=[0, 130_000_000, 145_000_000, 180_000_000]):
            camera, _ = self.open_mock(self.container(frames=frames))
            camera.read()  # Initial observed offset is 30 ms.
            camera.read()  # Draining the queue improves offset estimate to 5 ms.
            camera.freeze_clock()
            camera.read()  # Subsequent readings cannot change the fixed timeline.
        self.assertEqual(camera.frame_epoch_ns, 1_700_000_000_185_000_000)
        camera.close()

    def test_windows_rejects_reset_clock_after_recording_start(self):
        first, second = av.VideoFrame(64, 48), av.VideoFrame(64, 48)
        first.time_base, first.pts = Fraction(1, 1000), 200
        second.time_base, second.pts = Fraction(1, 1000), 100
        camera, _ = self.open_mock(self.container(frames=[first, second]))
        camera.read()
        camera.freeze_clock()
        with self.assertRaisesRegex(RuntimeError, "timestamp moved backwards"):
            camera.read()
        camera.close()


if __name__ == "__main__":
    unittest.main()
