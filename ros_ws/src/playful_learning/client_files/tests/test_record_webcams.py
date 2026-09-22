"""Hardware-free recorder tests; requires opencv-python, av and cv2-enumerate-cameras.

Run: python -m unittest discover -s tests -p test_record_webcams.py
"""

import csv
import io
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from contextlib import ExitStack, redirect_stdout, redirect_stderr
from datetime import datetime
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import record_webcams as recorder
from webcam_modes import CameraMode


def device(index, name="USB Camera", vid=1234):
    return SimpleNamespace(index=index, name=name, vid=vid, backend=recorder.cv2.CAP_DSHOW)


class FakeCapture:
    def __init__(self, fail_after=None, block_after=None, mode=None, late_frame=False):
        self.fail_after = fail_after
        self.block_after = block_after
        self.mode = mode or CameraMode(32, 24, Fraction(100), "MJPG")
        self.late_frame = late_frame
        self.grabs = 0
        self.blocked = threading.Event()
        self.unblock = threading.Event()
        self.released = threading.Event()

    def isOpened(self):
        return True

    def set(self, *_):
        return True

    def grab(self):
        # A camera supplies frames at its own rate, independently of GUI redraws.
        time.sleep(1 / float(self.mode.fps))
        self.grabs += 1
        if self.block_after is not None and self.grabs > self.block_after:
            self.blocked.set()
            self.unblock.wait(5)
            return self.late_frame
        return self.fail_after is None or self.grabs <= self.fail_after

    def retrieve(self):
        frame = recorder.np.full((self.mode.height, self.mode.width, 3),
                                 self.grabs * 10 % 256, dtype=recorder.np.uint8)
        return True, frame

    def release(self):
        self.released.set()


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.resources = ExitStack()
        self.addCleanup(self.resources.close)
        folder = self.resources.enter_context(tempfile.TemporaryDirectory())
        self.output = Path(folder)
        self.resources.enter_context(patch.object(recorder, "OUTPUT_DIR", self.output))
        self.mode = CameraMode(32, 24, Fraction(100), "MJPG")
        self.resources.enter_context(patch.object(recorder, "PREVIEW_SIZE", (80, 60)))
        self.resources.enter_context(patch.object(recorder, "CAMERA_TIMEOUT", 0.08))
        self.resources.enter_context(patch.object(recorder, "STARTUP_TIMEOUT", 1.0))
        self.resources.enter_context(redirect_stdout(io.StringIO()))

    def mock_captures(self, captures):
        self.resources.enter_context(patch.object(
            recorder.cv2, "VideoCapture", side_effect=lambda index, _: captures[index]))
        self.resources.enter_context(patch.object(
            recorder, "discover_modes", side_effect=lambda info: [captures[info.index].mode]))
        self.resources.enter_context(patch.object(
            recorder, "configure_camera", side_effect=lambda capture, mode: mode))

    def record_with_deadline(self, indexes, stop=None):
        stop = stop or recorder.Session()
        expired = threading.Event()

        def abort():
            expired.set()
            stop.set()

        watchdog = threading.Timer(3.0, abort)
        watchdog.daemon = True
        watchdog.start()
        try:
            recorder.record([device(index) for index in indexes], stop)
        finally:
            watchdog.cancel()
        self.assertFalse(expired.is_set(), "Recorder exceeded its test deadline")

    def mock_gui(self, show=None):
        for name in ("namedWindow", "resizeWindow", "destroyAllWindows"):
            self.resources.enter_context(patch.object(recorder.cv2, name))
        self.resources.enter_context(patch.object(recorder.cv2, "imshow", side_effect=show))
        self.resources.enter_context(patch.object(recorder.cv2, "waitKey", return_value=-1))
        self.resources.enter_context(patch.object(recorder.cv2, "getWindowProperty", return_value=1))

    def inspect_video(self, path, size=(32, 24)):
        with recorder.av.open(str(path)) as video:
            stream = video.streams.video[0]
            self.assertEqual(stream.codec_context.name, "h264")
            frames = list(video.decode(video=0))
        with path.with_suffix(".csv").open(newline="", encoding="utf-8") as csv_file:
            reader = csv.DictReader(csv_file)
            self.assertEqual(tuple(reader.fieldnames), recorder.CSV_COLUMNS)
            rows = list(reader)
        self.assertEqual(len(frames), len(rows))
        for index, (frame, row) in enumerate(zip(frames, rows)):
            self.assertEqual(int(row["frame_index"]), index)
            self.assertAlmostEqual(float(frame.pts * frame.time_base),
                                   float(row["seconds_since_start"]), places=6)
            self.assertEqual((frame.width, frame.height), size)
            human = datetime.fromisoformat(row["timestamp_human"])
            self.assertIsNotNone(human.utcoffset())
            self.assertAlmostEqual(human.timestamp(),
                                   float(row["timestamp_epoch_seconds"]), places=5)
        return frames, rows

    def test_selection_infers_usb_and_accepts_explicit_indexes(self):
        devices = [device(0, "Integrated Camera"), device(3, "USB Camera"),
                   device(8, "USB Camera"), device(11, "USB Camera"),
                   device(9, "Virtual Camera", None)]
        with patch.object(recorder, "enumerate_cameras", return_value=devices) as enumerate_mock:
            for platform, backend in (("win32", recorder.cv2.CAP_DSHOW),
                                      ("linux", recorder.cv2.CAP_V4L2)):
                with self.subTest(platform=platform), \
                        patch.object(recorder.sys, "platform", platform), \
                        patch("builtins.input", return_value=""):
                    self.assertEqual([item.index for item in recorder.choose_cameras()], [3, 8, 11])
                    enumerate_mock.assert_called_with(backend)
            with patch("builtins.input", side_effect=["3,3", "99", "oops", "9, 0 8"]):
                self.assertEqual([item.index for item in recorder.choose_cameras()], [9, 0, 8])

    def test_no_inferred_devices_requires_explicit_selection(self):
        with patch.object(recorder, "enumerate_cameras", return_value=[device(0, "Built-in Camera")]):
            with patch("builtins.input", side_effect=["", "0"]):
                self.assertEqual(recorder.choose_cameras()[0].index, 0)
        with patch.object(recorder, "enumerate_cameras", return_value=[]):
            with self.assertRaisesRegex(SystemExit, "No cameras"):
                recorder.choose_cameras()

    def test_real_h264_irregular_shared_timeline_and_csv_roundtrip(self):
        prefix = "2026-09-17_12-34-56"
        modes = [self.mode, CameraMode(64, 36, Fraction(30000, 1001), "MJPG")]
        recordings = [recorder.Recording(device(index, "USB: Camera / 2K"), prefix, mode)
                      for index, mode in zip((3, 8), modes)]
        points = [[123, 40_000, 180_000, 510_000, 1_030_000],
                  [567, 33_934, 67_300, 120_123, 366_667]]
        epoch = 1_789_656_789_123_456_789
        try:
            for recording, mode, timestamps in zip(recordings, modes, points):
                self.assertEqual(recording.stream.codec_context.framerate, mode.fps)
                for index, pts in enumerate(timestamps):
                    frame = recorder.np.full((mode.height, mode.width, 3), index * 40,
                                             dtype=recorder.np.uint8)
                    recording.write(frame, pts, epoch + pts * 1000)
        finally:
            for recording in recordings:
                recording.close()
        for recording, mode, timestamps in zip(recordings, modes, points):
            self.assertRegex(recording.path.name,
                             r"^2026-09-17_12-34-56_USB-Camera-2K-[38]\.mp4$")
            frames, rows = self.inspect_video(recording.path, (mode.width, mode.height))
            self.assertEqual([frame.pts * frame.time_base for frame in frames],
                             [Fraction(pts, 1_000_000) for pts in timestamps])
            self.assertEqual([float(row["seconds_since_start"]) for row in rows],
                             [pts / 1_000_000 for pts in timestamps])
            self.assertEqual(len(frames), len(timestamps))
            recording.close()  # Closing a retired writer again is safe.

    def test_three_irregular_timelines_keep_timing_and_common_duration(self):
        # Large time gaps exercise long-recording clock precision without a long test.
        end_pts = 1_800_123_456
        timelines = [[0, 33_334, 68_701, 10_000_001, 900_009_876, 1_799_900_012],
                     [0, 45_670, 91_012, 9_999_999, 899_888_777, 1_799_987_654],
                     [0, 66_678, 134_901, 10_000_999, 900_111_333, 1_799_950_555]]
        epoch = 1_789_656_789_123_456_789
        for index, points in enumerate(timelines):
            recording = recorder.Recording(device(index), "2026-09-17_12-34-56", self.mode)
            try:
                for pts in points:
                    frame = recorder.np.zeros((24, 32, 3), dtype=recorder.np.uint8)
                    recording.write(frame, pts, epoch + pts * 1000)
            finally:
                recording.close(end_pts)
            frames, _ = self.inspect_video(recording.path)
            self.assertEqual([frame.pts * frame.time_base for frame in frames],
                             [Fraction(pts, 1_000_000) for pts in points])
            with recorder.av.open(str(recording.path)) as video:
                self.assertEqual(video.duration, end_pts)
                stream = video.streams.video[0]
                self.assertEqual(stream.duration * stream.time_base,
                                 Fraction(end_pts, 1_000_000))

    def test_existing_video_or_csv_is_preserved_without_creating_other_file(self):
        for suffix, other_suffix in ((".mp4", ".csv"), (".csv", ".mp4")):
            with self.subTest(existing=suffix):
                prefix = "2026-09-17_12-34-{}".format("55" if suffix == ".mp4" else "56")
                existing = self.output / (prefix + "_USB-Camera-3" + suffix)
                existing.write_bytes(b"existing recording")
                with self.assertRaises(FileExistsError):
                    recorder.Recording(device(3), prefix, self.mode)
                self.assertEqual(existing.read_bytes(), b"existing recording")
                self.assertFalse(existing.with_suffix(other_suffix).exists())

    def test_unavailable_modes_release_and_retry_or_exit_cleanly(self):
        modes = [CameraMode(96, 54, Fraction(100), "MJPG"),
                 CameraMode(64, 48, Fraction(50), "MJPG")]
        self.mock_gui()
        for fallback_works in (True, False):
            with self.subTest(fallback_works=fallback_works):
                folder = self.output / str(fallback_works)
                captures = [FakeCapture(fail_after=3, mode=mode) for mode in modes]
                attempted = []

                def configure(capture, mode):
                    attempted.append(mode)
                    if mode == modes[0] or not fallback_works:
                        raise RuntimeError("USB resources unavailable")
                    self.assertTrue(captures[0].released.is_set())
                    return mode

                with patch.object(recorder, "OUTPUT_DIR", folder), \
                        patch.object(recorder, "discover_modes", return_value=modes), \
                        patch.object(recorder.cv2, "VideoCapture", side_effect=captures) as opened, \
                        patch.object(recorder, "configure_camera", side_effect=configure), \
                        redirect_stdout(io.StringIO()) as messages:
                    self.record_with_deadline([3])
                self.assertEqual(attempted, modes)
                self.assertEqual(opened.call_count, 2)
                self.assertTrue(all(capture.released.is_set() for capture in captures))
                self.assertEqual(captures[0].grabs, 0)
                self.assertIn("96x54 @ 100 FPS unavailable", messages.getvalue())
                videos = list(folder.glob("*.mp4"))
                if fallback_works:
                    self.assertEqual(len(videos), 1)
                    self.assertEqual(len(self.inspect_video(videos[0], (64, 48))[0]), 3)
                    self.assertIn("64 x 48 at 50 FPS", messages.getvalue())
                else:
                    self.assertEqual(videos, [])
                    self.assertEqual(list(folder.glob("*.csv")), [])
                    self.assertIn("could not open any advertised camera mode", messages.getvalue())

    def test_unplug_finalizes_one_video_and_other_camera_continues(self):
        captures = {3: FakeCapture(fail_after=3), 8: FakeCapture(fail_after=12)}
        self.mock_captures(captures)
        self.mock_gui()
        self.record_with_deadline([3, 8])
        videos = {int(path.stem.rsplit("-", 1)[1]): path for path in self.output.glob("*.mp4")}
        self.assertEqual(set(videos), {3, 8})
        failed_frames, _ = self.inspect_video(videos[3])
        healthy_frames, _ = self.inspect_video(videos[8])
        self.assertEqual(len(failed_frames), 3)
        self.assertEqual(len(healthy_frames), 12)
        self.assertGreater(healthy_frames[-1].time, failed_frames[-1].time)
        self.assertTrue(all(capture.released.is_set() for capture in captures.values()))

    def test_startup_retries_lower_modes_to_open_all_three_cameras(self):
        high = CameraMode(64, 48, Fraction(100), "MJPG")
        low = CameraMode(32, 24, Fraction(100), "MJPG")
        allocated = {}
        captures = []
        configured = []
        self.mock_gui()

        class LimitedUsbCapture(FakeCapture):
            def __init__(self, index):
                super().__init__(fail_after=3)
                self.index = index
                # Third camera's default open needs both earlier streams at low resolution.
                self.opened = (index != 11 or
                               all(index in allocated and allocated[index].mode == low
                                   for index in (3, 8)))

            def isOpened(self):
                return self.opened

            def release(self):
                if allocated.get(self.index) is self:
                    del allocated[self.index]
                super().release()

        def open_capture(index, _):
            capture = LimitedUsbCapture(index)
            captures.append(capture)
            return capture

        def configure(capture, mode):
            capture.mode = mode
            allocated[capture.index] = capture
            configured.append((capture.index, mode))
            return mode

        with patch.object(recorder, "discover_modes", return_value=[high, low]), \
                patch.object(recorder.cv2, "VideoCapture", side_effect=open_capture), \
                patch.object(recorder, "configure_camera", side_effect=configure), \
                redirect_stdout(io.StringIO()) as messages:
            self.record_with_deadline([3, 8, 11])
        self.assertIn("Retrying setup with camera 8 limited to 32x24", messages.getvalue())
        self.assertIn("Retrying setup with camera 3 limited to 32x24", messages.getvalue())
        self.assertEqual({index: mode for index, mode in configured}, {3: low, 8: low, 11: high})
        videos = {int(path.stem.rsplit("-", 1)[1]): path for path in self.output.glob("*.mp4")}
        self.assertEqual(set(videos), {3, 8, 11})
        self.assertEqual(len(list(self.output.glob("*.csv"))), 3)
        for index, path in videos.items():
            size = (64, 48) if index == 11 else (32, 24)
            self.assertEqual(len(self.inspect_video(path, size)[0]), 3)
        self.assertEqual(allocated, {})
        self.assertTrue(all(capture.released.is_set() for capture in captures))
        # Prepared streams from discarded attempts never capture or leave colliding files.
        captured = [capture for capture in captures if capture.grabs]
        self.assertEqual(len(captured), 3)
        self.assertTrue(all(capture.grabs == 4 for capture in captured))

    def test_native_resolutions_and_independent_rates_share_one_clock(self):
        modes = {3: CameraMode(96, 54, Fraction(100), "MJPG"),
                 8: CameraMode(32, 48, Fraction(25), "MJPG"),
                 11: CameraMode(64, 36, Fraction(50), "MJPG")}
        captures = {3: FakeCapture(fail_after=24, mode=modes[3]),
                    8: FakeCapture(fail_after=6, mode=modes[8]),
                    11: FakeCapture(fail_after=12, mode=modes[11])}
        displayed = []
        self.mock_captures(captures)
        self.mock_gui(lambda _, image: displayed.append(image.shape))
        self.record_with_deadline([3, 8, 11])
        decoded = {}
        origins = []
        for path in self.output.glob("*.mp4"):
            index = int(path.stem.rsplit("-", 1)[1])
            mode = modes[index]
            frames, rows = self.inspect_video(path, (mode.width, mode.height))
            decoded[index] = frames
            # First images begin at zero; their actual capture time remains in CSV.
            origins.extend(float(row["timestamp_epoch_seconds"]) -
                           float(row["seconds_since_start"]) for row in rows[1:])
        self.assertEqual({index: len(frames) for index, frames in decoded.items()},
                         {3: 24, 8: 6, 11: 12})
        # Different frame rates must not force the faster camera into shared rounds.
        self.assertLess(decoded[3][-1].time, 0.7)
        self.assertLess(max(origins) - min(origins), 0.005)
        self.assertEqual(decoded[8][0].time, 0)
        self.assertEqual(decoded[3][0].time, 0)
        self.assertTrue(displayed)
        self.assertTrue(all(shape == (60, 240, 3) for shape in displayed))

    def test_one_blocked_encoder_does_not_delay_other_camera_workers(self):
        captures = {index: FakeCapture(fail_after=12) for index in (3, 8, 11)}
        self.mock_captures(captures)
        self.mock_gui()
        original_write = recorder.Recording.write
        slow_encoder_entered = threading.Event()
        other_cameras_progressed = threading.Event()
        counts = {8: 0, 11: 0}
        writes_from = {}

        def encode_independently(recording, *args):
            index = int(recording.path.stem.rsplit("-", 1)[1])
            writes_from.setdefault(index, set()).add(threading.get_ident())
            if index == 3 and recording.count == 0:
                slow_encoder_entered.set()
                if not other_cameras_progressed.wait(1.0):
                    raise RuntimeError("other cameras stalled behind this encoder")
            original_write(recording, *args)
            if index in counts:
                counts[index] += 1
                if min(counts.values()) >= 5:
                    other_cameras_progressed.set()

        with patch.object(recorder.Recording, "write", encode_independently), \
                patch.object(recorder, "CAMERA_TIMEOUT", 1.5):
            self.record_with_deadline([3, 8, 11])
        self.assertTrue(slow_encoder_entered.is_set())
        self.assertTrue(other_cameras_progressed.is_set())
        self.assertEqual(set(writes_from), {3, 8, 11})
        worker_ids = set().union(*writes_from.values())
        self.assertEqual(len(worker_ids), 3)
        self.assertNotIn(threading.get_ident(), worker_ids)
        for path in self.output.glob("*.mp4"):
            self.assertEqual(len(self.inspect_video(path)[0]), 12)

    def test_all_cameras_are_ready_before_first_capture(self):
        captures = {index: FakeCapture(fail_after=2) for index in (3, 8, 11)}
        self.mock_captures(captures)
        self.mock_gui()
        configured = set()
        ready_when_grabbed = []

        def configure(capture, mode):
            if len(configured) == 2:
                time.sleep(0.1)  # Slow final setup must not start the other cameras early.
            configured.add(id(capture))
            return mode

        for capture in captures.values():
            original_grab = capture.grab

            def grab_when_ready(grab=original_grab):
                ready_when_grabbed.append(len(configured))
                return grab()

            capture.grab = grab_when_ready
        with patch.object(recorder, "configure_camera", side_effect=configure):
            self.record_with_deadline([3, 8, 11])
        self.assertTrue(ready_when_grabbed)
        self.assertEqual(set(ready_when_grabbed), {3})

    def test_normal_stop_uses_one_endpoint_for_three_rates(self):
        modes = {3: CameraMode(32, 24, Fraction(100), "MJPG"),
                 8: CameraMode(32, 24, Fraction(25), "MJPG"),
                 11: CameraMode(32, 24, Fraction(50), "MJPG")}
        captures = {index: FakeCapture(mode=mode) for index, mode in modes.items()}
        self.mock_captures(captures)
        self.mock_gui()
        stop = recorder.Session()
        original_write = recorder.Recording.write
        counts = {}

        def finish_after_enough_frames(recording, *args):
            original_write(recording, *args)
            counts[recording.path.name] = recording.count
            if len(counts) == 3 and min(counts.values()) >= 5:
                stop.set()

        with patch.object(recorder.Recording, "write", finish_after_enough_frames):
            self.record_with_deadline([3, 8, 11], stop)
        videos = list(self.output.glob("*.mp4"))
        self.assertEqual(len(videos), 3)
        end_pts = (stop.stop_ns - stop.start_ns) // 1000
        for path in videos:
            frames, _ = self.inspect_video(path)
            self.assertEqual(frames[0].time, 0)
            self.assertLess(frames[-1].time, end_pts / 1_000_000)
            with recorder.av.open(str(path)) as video:
                self.assertEqual(video.duration, end_pts)

    def test_capture_timestamps_follow_blocking_retrieve(self):
        delivered = []

        class BlockingRetrieveCapture(FakeCapture):
            def grab(self):
                # DirectShow can return here immediately and wait for an image in retrieve.
                self.grabs += 1
                return self.grabs <= 3

            def retrieve(self):
                time.sleep(0.025)
                result = super().retrieve()
                delivered.append(time.time_ns())
                return result

        capture = BlockingRetrieveCapture()
        self.mock_captures({3: capture})
        self.mock_gui()
        self.record_with_deadline([3])
        path, = self.output.glob("*.mp4")
        frames, rows = self.inspect_video(path)
        self.assertEqual(len(frames), 3)
        for delivery_ns, row in zip(delivered, rows):
            seconds, nanos = row["timestamp_epoch_seconds"].split(".")
            timestamp_ns = int(seconds) * 1_000_000_000 + int(nanos)
            self.assertGreaterEqual(timestamp_ns, delivery_ns)
        self.assertGreaterEqual(frames[1].time, 0.045)

    def test_preview_letterboxes_without_changing_capture_pixels(self):
        original = recorder.np.full((20, 80, 3), 123, dtype=recorder.np.uint8)
        preview = recorder.preview_frame(original)
        self.assertEqual(preview.shape, (60, 80, 3))
        self.assertTrue(recorder.np.all(preview[:20] == 0))
        self.assertTrue(recorder.np.all(preview[20:40] == 123))
        self.assertTrue(recorder.np.all(preview[40:] == 0))
        self.assertEqual(original.shape, (20, 80, 3))
        self.assertTrue(recorder.np.all(original == 123))

    def test_blocked_driver_times_out_while_other_camera_continues(self):
        captures = {3: FakeCapture(block_after=2), 8: FakeCapture(fail_after=14)}
        self.mock_captures(captures)
        self.mock_gui()
        try:
            self.record_with_deadline([3, 8])
        finally:
            captures[3].unblock.set()
            self.assertTrue(captures[3].released.wait(1.0))
        lengths = {}
        for path in self.output.glob("*.mp4"):
            lengths[int(path.stem.rsplit("-", 1)[1])] = len(self.inspect_video(path)[0])
        self.assertEqual(lengths, {3: 2, 8: 14})
        self.assertTrue(captures[3].blocked.is_set())

    def test_terminal_quit_finalizes_video_while_driver_remains_blocked(self):
        capture = FakeCapture(block_after=2, late_frame=True)
        stop = recorder.Session()
        self.mock_captures({3: capture})
        self.mock_gui()
        quit_bytes = iter(bytes([value]) for value in b"  QuIt  \n")

        def quit_when_blocked(*_):
            if not capture.blocked.wait(2):
                stop.set()
            return next(quit_bytes, b"")

        with patch.object(recorder.sys, "stdin", SimpleNamespace(fileno=lambda: 0)), \
                patch.object(recorder.os, "read", side_effect=quit_when_blocked), \
                patch.object(recorder, "CAMERA_TIMEOUT", 10.0):
            terminal = threading.Thread(target=recorder.wait_for_quit, args=(stop,), daemon=True)
            terminal.start()
            started = time.monotonic()
            try:
                self.record_with_deadline([3], stop)
                self.assertTrue(capture.blocked.is_set())
                self.assertFalse(capture.released.is_set())
                self.assertLess(time.monotonic() - started, 3.0)
                path, = self.output.glob("*.mp4")
                self.assertEqual(len(self.inspect_video(path)[0]), 2)
                completed_csv = path.with_suffix(".csv").read_bytes()
            finally:
                capture.unblock.set()
                terminal.join(1.0)
                self.assertTrue(capture.released.wait(1.0))
            # A driver returning a good frame after shutdown must not touch closed files.
            self.assertEqual(path.with_suffix(".csv").read_bytes(), completed_csv)
            self.assertEqual(len(self.inspect_video(path)[0]), 2)

    def test_one_cleanup_error_does_not_prevent_other_video_finalizing(self):
        captures = {3: FakeCapture(), 8: FakeCapture()}
        stop = recorder.Session()
        self.mock_captures(captures)
        self.mock_gui()
        original_close = recorder.Recording.close
        original_write = recorder.Recording.write
        attempted = []
        written = {}

        def write_and_stop(recording, *args):
            original_write(recording, *args)
            written[recording.path.name] = recording.count
            if len(written) == 2 and min(written.values()) >= 3:
                stop.set()

        def close_then_fail(recording, end_pts=None):
            attempted.append(recording.path.name)
            original_close(recording, end_pts)
            if recording.path.stem.endswith("-3"):
                raise OSError("simulated close failure")

        with patch.object(recorder.Recording, "close", close_then_fail), \
                patch.object(recorder.Recording, "write", write_and_stop), \
                redirect_stderr(io.StringIO()) as errors:
            self.record_with_deadline([3, 8], stop)
        self.assertEqual(len(set(attempted)), 2)
        self.assertIn("simulated close failure", errors.getvalue())
        videos = list(self.output.glob("*.mp4"))
        self.assertEqual(len(videos), 2)
        for path in videos:
            self.assertGreaterEqual(len(self.inspect_video(path)[0]), 3)

    def test_completed_mp4_fragments_survive_abrupt_process_exit(self):
        # A fresh process intentionally skips all Python/encoder cleanup.
        source = """
import os
import sys
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, sys.argv[1])
import record_webcams as recorder
from webcam_modes import CameraMode
recorder.OUTPUT_DIR = Path(sys.argv[2])
info = SimpleNamespace(index=3, name='USB Camera')
mode = CameraMode(32, 24, Fraction(30000, 1001), 'MJPG')
recording = recorder.Recording(info, '2026-09-17_12-34-56', mode)
for index in range(75):
    frame = recorder.np.full((24, 32, 3), index % 255, dtype=recorder.np.uint8)
    pts = 123 + index * 1_001_000 // 30
    recording.write(frame, pts, 1_789_656_789_000_000_000 + pts * 1000)
os._exit(0)
"""
        subprocess.run([sys.executable, "-B", "-c", source,
                        str(Path(recorder.__file__).parent), str(self.output)],
                       check=True, capture_output=True, text=True, timeout=15)
        path, = self.output.glob("*.mp4")
        with recorder.av.open(str(path)) as video:
            frames = list(video.decode(video=0))
        with path.with_suffix(".csv").open(newline="", encoding="utf-8") as csv_file:
            rows = list(csv.DictReader(csv_file))
        self.assertEqual(len(rows), 75)
        self.assertGreater(len(frames), 0)
        self.assertLessEqual(len(frames), len(rows))
        # An unfinished final fragment may be lost while CSV rows are already flushed.
        for frame, row in zip(frames, rows):
            self.assertAlmostEqual(float(frame.pts * frame.time_base),
                                   float(row["seconds_since_start"]), places=6)

    def test_startup_failure_exits_cleanly_with_terminal_input_still_open(self):
        # Buffered input() in a daemon can lock stdin during interpreter shutdown.
        source = """
import sys
import time
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, sys.argv[1])
import record_webcams as recorder
from webcam_modes import CameraMode
recorder.OUTPUT_DIR = Path(sys.argv[2])
info = SimpleNamespace(index=3, name='USB Camera', backend=recorder.cv2.CAP_DSHOW)
recorder.choose_cameras = lambda: [info]
recorder.discover_modes = lambda info: [CameraMode(32, 24, Fraction(30), 'MJPG')]
def failed_camera(*args):
    time.sleep(0.1)  # Let the terminal daemon block while stdin remains open.
    return SimpleNamespace(isOpened=lambda: False, release=lambda: None)
recorder.cv2.VideoCapture = failed_camera
recorder.cv2.destroyAllWindows = lambda: None
recorder.main()
print('main returned', flush=True)
"""
        process = subprocess.Popen([sys.executable, "-B", "-c", source,
                                    str(Path(recorder.__file__).parent), str(self.output)],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True)
        try:
            # Do not communicate() yet: it would close stdin and hide the shutdown bug.
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            self.fail("Recorder failed to exit while its terminal input remained open")
        finally:
            stdout, stderr = process.communicate(timeout=5)
        self.assertEqual(process.returncode, 0, stderr)
        self.assertIn("main returned", stdout)
        self.assertNotIn("Fatal Python error", stderr)
        self.assertEqual(list(self.output.glob("*.mp4")), [])


if __name__ == "__main__":
    unittest.main()
