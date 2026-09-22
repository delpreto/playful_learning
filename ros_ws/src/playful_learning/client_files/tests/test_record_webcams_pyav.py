"""Hardware-free checks for native PyAV capture, MP4 timing, and worker shutdown.

Run: python -m unittest discover -s tests -p test_record_webcams_pyav.py
"""

import csv
import io
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from contextlib import ExitStack, redirect_stdout
from datetime import datetime
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import record_webcams_pyav as recorder
from webcam_modes import CameraMode


def device(index, name="USB Camera", vid=1234):
    return SimpleNamespace(index=index, name=name, vid=vid,
                           backend=recorder.cv2.CAP_DSHOW, path="camera-{}".format(index))


def native_frame(mode, value=100, frame_type=None):
    frame = (frame_type or recorder.av.VideoFrame)(mode.width, mode.height, "yuv420p")
    for index, plane in enumerate(frame.planes):
        plane.update(bytes([value if index == 0 else 128]) * plane.buffer_size)
    return frame


class NativeOnlyFrame(recorder.av.VideoFrame):
    def to_ndarray(self, *args, **kwargs):
        raise AssertionError("Recording must not copy a full frame into NumPy")


class FakeCapture:
    def __init__(self, session, mode, fail_after=None, block_after=None):
        self.session, self.mode = session, mode
        self.fail_after, self.block_after = fail_after, block_after
        self.count = 0
        self.preroll_count = 0
        self.blocked = threading.Event()
        self.unblock = threading.Event()
        self.closed = threading.Event()

    def read(self):
        time.sleep(1 / float(self.mode.fps))
        if self.session.started.is_set():
            self.count += 1
            if self.block_after is not None and self.count > self.block_after:
                self.blocked.set()
                self.unblock.wait(5.0)
            if self.fail_after is not None and self.count > self.fail_after:
                raise OSError("camera disconnected")
        else:
            self.preroll_count += 1
        return native_frame(self.mode)

    def close(self):
        self.closed.set()


class NativeRecorderTests(unittest.TestCase):
    def setUp(self):
        self.resources = ExitStack()
        self.addCleanup(self.resources.close)
        self.output = Path(self.resources.enter_context(tempfile.TemporaryDirectory()))
        self.resources.enter_context(patch.object(recorder, "OUTPUT_DIR", self.output))
        self.resources.enter_context(patch.object(recorder, "PREVIEW_SIZE", (80, 60)))
        self.resources.enter_context(patch.object(recorder, "CAMERA_TIMEOUT", 0.15))
        self.resources.enter_context(patch.object(recorder, "STARTUP_TIMEOUT", 1.5))
        self.resources.enter_context(redirect_stdout(io.StringIO()))
        self.mode = CameraMode(32, 24, Fraction(100), "MJPG")

    def mock_gui(self, show=None):
        for name in ("namedWindow", "resizeWindow", "destroyAllWindows"):
            self.resources.enter_context(patch.object(recorder.cv2, name))
        self.resources.enter_context(patch.object(recorder.cv2, "imshow", side_effect=show))
        self.resources.enter_context(patch.object(recorder.cv2, "waitKey", return_value=-1))
        self.resources.enter_context(patch.object(recorder.cv2, "getWindowProperty", return_value=1))

    def mock_captures(self, captures, opened=None):
        def open_capture(info, mode, *args, **kwargs):
            if opened is not None:
                opened.add(info.index)
            return captures[info.index]

        self.resources.enter_context(patch.object(recorder, "open_camera", side_effect=open_capture))
        self.resources.enter_context(patch.object(
            recorder, "discover_modes", side_effect=lambda info: [captures[info.index].mode]))

    def record_with_deadline(self, indexes, session):
        expired = threading.Event()

        def abort():
            expired.set()
            session.set()

        watchdog = threading.Timer(4.0, abort)
        watchdog.daemon = True
        watchdog.start()
        try:
            result = recorder.record([device(index) for index in indexes], session)
        finally:
            watchdog.cancel()
        self.assertFalse(expired.is_set(), "Recorder exceeded its test deadline")
        return result

    def inspect_video(self, path, size=(32, 24)):
        with recorder.av.open(str(path)) as video:
            self.assertEqual(video.streams.video[0].codec_context.name, "h264")
            duration = video.duration
            frames = list(video.decode(video=0))
        with path.with_suffix(".csv").open(newline="", encoding="utf-8") as csv_file:
            reader = csv.DictReader(csv_file)
            self.assertEqual(tuple(reader.fieldnames), recorder.CSV_COLUMNS)
            rows = list(reader)
        self.assertEqual(len(frames), len(rows))
        for index, (frame, row) in enumerate(zip(frames, rows)):
            self.assertEqual(int(row["frame_index"]), index)
            self.assertEqual((frame.width, frame.height), size)
            self.assertAlmostEqual(float(frame.pts * frame.time_base),
                                   float(row["seconds_since_start"]), places=6)
            human = datetime.fromisoformat(row["timestamp_human"])
            self.assertIsNotNone(human.utcoffset())
            self.assertAlmostEqual(human.timestamp(),
                                   float(row["timestamp_epoch_seconds"]), places=5)
        return frames, rows, duration

    def test_infers_all_three_usb_devices_on_windows_and_linux(self):
        devices = [device(0, "Integrated Camera"), device(3), device(8), device(11),
                   device(99, "Virtual Camera", None)]
        with patch.object(recorder, "enumerate_cameras", return_value=devices) as enumerated:
            for platform, backend in (("win32", recorder.cv2.CAP_DSHOW),
                                      ("linux", recorder.cv2.CAP_V4L2)):
                with self.subTest(platform=platform), \
                        patch.object(recorder.sys, "platform", platform), \
                        patch("builtins.input", return_value=""):
                    self.assertEqual([info.index for info in recorder.choose_cameras()], [3, 8, 11])
                    enumerated.assert_called_with(backend)
            with patch("builtins.input", side_effect=["3,3", "123", "99, 0 8"]):
                self.assertEqual([info.index for info in recorder.choose_cameras()], [99, 0, 8])

    def test_native_frames_preserve_three_irregular_timelines_and_common_duration(self):
        end_pts = 1_800_123_456
        timelines = [[0, 33_334, 68_701, 10_000_001, 900_009_876, 1_799_900_012],
                     [0, 45_670, 91_012, 9_999_999, 899_888_777, 1_799_987_654],
                     [0, 66_678, 134_901, 10_000_999, 900_111_333, 1_799_950_555]]
        modes = [self.mode, CameraMode(64, 36, Fraction(30000, 1001), "MJPG"),
                 CameraMode(32, 48, Fraction(15), "MJPG")]
        epoch = 1_789_656_789_123_456_789
        for index, (mode, points) in enumerate(zip(modes, timelines)):
            recording = recorder.Recording(device(index, "USB: Camera / 2K"),
                                           "2026-09-17_12-34-56", mode)
            try:
                for pts in points:
                    recording.write(native_frame(mode, frame_type=NativeOnlyFrame),
                                    pts, epoch + pts * 1000)
            finally:
                recording.close(end_pts)
            self.assertRegex(recording.path.name,
                             r"^2026-09-17_12-34-56_USB-Camera-2K-[012]\.mp4$")
            frames, _, duration = self.inspect_video(recording.path, (mode.width, mode.height))
            self.assertEqual([frame.pts * frame.time_base for frame in frames],
                             [Fraction(pts, 1_000_000) for pts in points])
            self.assertEqual(duration, end_pts)
            recording.close(end_pts)  # Repeated cleanup must be harmless.

    def test_existing_video_or_csv_is_preserved(self):
        for suffix, other_suffix in ((".mp4", ".csv"), (".csv", ".mp4")):
            with self.subTest(existing=suffix):
                prefix = "2026-09-17_12-34-{}".format("55" if suffix == ".mp4" else "56")
                existing = self.output / (prefix + "_USB-Camera-3" + suffix)
                existing.write_bytes(b"existing recording")
                with self.assertRaises(FileExistsError):
                    recorder.Recording(device(3), prefix, self.mode)
                self.assertEqual(existing.read_bytes(), b"existing recording")
                self.assertFalse(existing.with_suffix(other_suffix).exists())

    def test_mjpeg_intra_frames_allow_inter_frame_h264_compression(self):
        # MJPEG decoders label every input an I-frame; that hint must not force
        # every encoded output into a much larger H.264 keyframe.
        recording = recorder.Recording(device(3), "2026-09-17_12-34-56", self.mode)
        try:
            for index in range(12):
                frame = native_frame(self.mode)
                frame.pict_type = 1
                recording.write(frame, index * 10_000, 1_789_656_789_000_000_000 + index * 10_000_000)
        finally:
            recording.close(120_000)
        frames, _, _ = self.inspect_video(recording.path)
        self.assertTrue(frames[0].key_frame)
        self.assertTrue(any(not frame.key_frame for frame in frames[1:]))

    def test_mjpeg_keeps_full_color_range_while_raw_capture_keeps_limited_range(self):
        cases = [("MJPG", "yuvj422p", 2, (0, 255)),
                 ("YUYV", "yuv422p", 1, (16, 235))]
        for index, (fourcc, pixel_format, color_range, levels) in enumerate(cases):
            with self.subTest(format=fourcc):
                mode = CameraMode(32, 24, Fraction(30), fourcc)
                recording = recorder.Recording(device(index), "2026-09-17_12-34-56", mode)
                try:
                    for frame_index, level in enumerate(levels):
                        frame = recorder.av.VideoFrame(32, 24, pixel_format)
                        frame.color_range = color_range
                        for plane_index, plane in enumerate(frame.planes):
                            value = level if plane_index == 0 else 128
                            plane.update(bytes([value]) * plane.buffer_size)
                        recording.write(frame, frame_index * 33_333,
                                        1_789_656_789_000_000_000 + frame_index * 33_333_000)
                finally:
                    recording.close(66_666)
                with recorder.av.open(str(recording.path)) as video:
                    # H.264 may omit the limited-range flag because it is the
                    # default. Full range must be explicitly signaled.
                    decoded_ranges = (2,) if color_range == 2 else (0, 1)
                    self.assertIn(video.streams.video[0].codec_context.color_range, decoded_ranges)
                    frames = list(video.decode(video=0))
                self.assertEqual(len(frames), 2)
                for frame, endpoint in zip(frames, (0, 255)):
                    self.assertIn(frame.color_range, decoded_ranges)
                    self.assertIn(frame.format.name, ("yuv420p", "yuvj420p"))
                    if color_range == 1:
                        self.assertEqual(frame.format.name, "yuv420p")
                    rgb = frame.to_ndarray(format="rgb24").astype(recorder.np.int16)
                    self.assertLessEqual(int(recorder.np.max(recorder.np.abs(rgb - endpoint))), 3)

    def test_preview_converts_only_a_thumbnail_and_preserves_native_frame(self):
        mode = CameraMode(320, 80, Fraction(30), "MJPG")
        frame = native_frame(mode, frame_type=NativeOnlyFrame)
        preview = recorder.preview_frame(frame)
        self.assertEqual(preview.shape, (60, 80, 3))
        self.assertTrue(recorder.np.all(preview[:20] == 0))
        self.assertTrue(recorder.np.all(preview[20:40] > 0))
        self.assertTrue(recorder.np.all(preview[40:] == 0))
        self.assertEqual((frame.width, frame.height, frame.format.name), (320, 80, "yuv420p"))

    def test_setup_drains_frames_but_saves_nothing_before_every_camera_is_ready(self):
        session = recorder.Session()
        captures = {index: FakeCapture(session, self.mode, fail_after=3)
                    for index in (3, 8, 11)}
        self.mock_gui()
        opened, recorded = set(), []
        original_write = recorder.Recording.write

        def open_camera(info, mode, *args, **kwargs):
            if len(opened) == 2:
                time.sleep(0.12)
            opened.add(info.index)
            return captures[info.index]

        def write_after_ready(recording, *args):
            recorded.append((session.started.is_set(), frozenset(opened)))
            original_write(recording, *args)

        with patch.object(recorder, "open_camera", side_effect=open_camera), \
                patch.object(recorder, "discover_modes", return_value=[self.mode]), \
                patch.object(recorder.Recording, "write", write_after_ready):
            self.record_with_deadline([3, 8, 11], session)
        self.assertGreaterEqual(sum(capture.preroll_count for capture in captures.values()), 4)
        self.assertTrue(recorded)
        self.assertEqual(set(recorded), {(True, frozenset((3, 8, 11)))})

    def test_queued_prestart_frames_are_dropped_and_source_timing_survives_fast_reads(self):
        session = recorder.Session()
        offsets_us = [-20_000, -5_000, 4_123, 34_567, 69_012, 119_345]

        class BufferedCapture(FakeCapture):
            def __init__(self, mode):
                super().__init__(session, mode)
                self.frame_epoch_ns = None
                self.offsets = iter(offsets_us)
                self.burst_started = False
                self.clock_freezes = 0

            def read(self):
                if not session.started.is_set():
                    time.sleep(0.01)
                    if not session.started.is_set():
                        self.frame_epoch_ns = time.time_ns()
                        return native_frame(self.mode)
                if not self.burst_started:
                    # All source timestamps are in the past when their queued
                    # images arrive; subsequent reads deliver them immediately.
                    time.sleep(0.15)
                    self.burst_started = True
                try:
                    offset_us = next(self.offsets)
                except StopIteration:
                    raise OSError("end of buffered stream") from None
                self.frame_epoch_ns = session.start_epoch_ns + offset_us * 1000
                return native_frame(self.mode)

            def freeze_clock(self):
                self.clock_freezes += 1

        capture = BufferedCapture(self.mode)
        self.mock_captures({3: capture})
        self.mock_gui()
        with patch.object(recorder, "CAMERA_TIMEOUT", 0.5):
            self.record_with_deadline([3], session)
        path, = self.output.glob("*.mp4")
        frames, rows, _ = self.inspect_video(path)
        self.assertEqual([frame.pts * frame.time_base for frame in frames],
                         [Fraction(pts, 1_000_000) for pts in (0, 34_567, 69_012, 119_345)])
        saved_epoch_ns = []
        for row in rows:
            seconds, nanos = row["timestamp_epoch_seconds"].split(".")
            saved_epoch_ns.append(int(seconds) * 1_000_000_000 + int(nanos))
        self.assertEqual(saved_epoch_ns,
                         [session.start_epoch_ns + offset * 1000 for offset in offsets_us[2:]])
        self.assertEqual(capture.clock_freezes, 1)
        self.assertTrue(capture.closed.is_set())

    def test_one_slow_encoder_leaves_other_workers_independent_after_shared_start(self):
        session = recorder.Session()
        captures = {index: FakeCapture(session, self.mode, fail_after=12)
                    for index in (3, 8, 11)}
        opened = set()
        self.mock_captures(captures, opened)
        self.mock_gui()
        original_write = recorder.Recording.write
        other_workers_progressed = threading.Event()
        slow_encoder_entered = threading.Event()
        counts, threads, starts = {}, {}, []

        def encode_independently(recording, *args):
            index = int(recording.path.stem.rsplit("-", 1)[1])
            self.assertEqual(opened, {3, 8, 11})
            self.assertTrue(session.started.is_set())
            threads.setdefault(index, set()).add(threading.get_ident())
            if recording.count == 0:
                starts.append(args[1])
            if index == 3 and recording.count == 0:
                slow_encoder_entered.set()
                if not other_workers_progressed.wait(1.0):
                    raise RuntimeError("other cameras stalled behind the slow encoder")
            original_write(recording, *args)
            counts[index] = recording.count
            if counts.get(8, 0) >= 5 and counts.get(11, 0) >= 5:
                other_workers_progressed.set()

        with patch.object(recorder.Recording, "write", encode_independently), \
                patch.object(recorder, "CAMERA_TIMEOUT", 1.5):
            self.record_with_deadline([3, 8, 11], session)
        self.assertTrue(slow_encoder_entered.is_set())
        self.assertTrue(other_workers_progressed.is_set())
        self.assertEqual(starts, [0, 0, 0])
        worker_ids = set().union(*threads.values())
        self.assertEqual(len(worker_ids), 3)
        self.assertNotIn(threading.get_ident(), worker_ids)
        self.assertTrue(all(capture.closed.is_set() for capture in captures.values()))
        self.assertEqual(len(list(self.output.glob("*.mp4"))), 3)
        for path in self.output.glob("*.mp4"):
            self.assertGreaterEqual(len(self.inspect_video(path)[0]), 10)

    def test_unplug_finalizes_affected_video_and_other_camera_continues(self):
        session = recorder.Session()
        captures = {3: FakeCapture(session, self.mode, fail_after=3),
                    8: FakeCapture(session, self.mode, fail_after=14)}
        self.mock_captures(captures)
        self.mock_gui()
        self.record_with_deadline([3, 8], session)
        videos = {int(path.stem.rsplit("-", 1)[1]): path for path in self.output.glob("*.mp4")}
        self.assertEqual(set(videos), {3, 8})
        failed_frames, _, failed_duration = self.inspect_video(videos[3])
        healthy_frames, _, healthy_duration = self.inspect_video(videos[8])
        self.assertGreater(len(failed_frames), 0)
        self.assertGreater(len(healthy_frames), len(failed_frames) + 5)
        self.assertGreater(healthy_duration, failed_duration)
        self.assertTrue(all(capture.closed.is_set() for capture in captures.values()))

    def test_normal_stop_gives_three_native_sizes_and_rates_the_same_endpoint(self):
        session = recorder.Session()
        modes = {3: self.mode, 8: CameraMode(64, 36, Fraction(25), "MJPG"),
                 11: CameraMode(32, 48, Fraction(50), "MJPG")}
        captures = {index: FakeCapture(session, mode) for index, mode in modes.items()}
        self.mock_captures(captures)
        self.mock_gui()
        original_write = recorder.Recording.write
        counts = {}

        def stop_when_ready(recording, *args):
            original_write(recording, *args)
            counts[recording.path.name] = recording.count
            if len(counts) == 3 and min(counts.values()) >= 5:
                session.set()

        with patch.object(recorder.Recording, "write", stop_when_ready):
            self.record_with_deadline([3, 8, 11], session)
        end_pts = (session.stop_ns - session.start_ns) // 1000
        videos = list(self.output.glob("*.mp4"))
        self.assertEqual(len(videos), 3)
        for path in videos:
            mode = modes[int(path.stem.rsplit("-", 1)[1])]
            frames, _, duration = self.inspect_video(path, (mode.width, mode.height))
            self.assertEqual(frames[0].time, 0)
            self.assertLess(frames[-1].time, end_pts / 1_000_000)
            self.assertEqual(duration, end_pts)

    def test_terminal_quit_finalizes_video_even_when_native_read_is_blocked(self):
        session = recorder.Session()
        capture = FakeCapture(session, self.mode, block_after=3)
        self.mock_captures({3: capture})
        self.mock_gui()
        quit_bytes = iter(bytes([value]) for value in b"  QuIt  \n")

        def quit_when_blocked(*args):
            if not capture.blocked.wait(2.0):
                session.set()
            return next(quit_bytes, b"")

        with patch.object(recorder.sys, "stdin", SimpleNamespace(fileno=lambda: 0)), \
                patch.object(recorder.os, "read", side_effect=quit_when_blocked), \
                patch.object(recorder, "CAMERA_TIMEOUT", 10.0):
            terminal = threading.Thread(target=recorder.wait_for_quit, args=(session,), daemon=True)
            terminal.start()
            try:
                self.record_with_deadline([3], session)
                self.assertTrue(capture.blocked.is_set())
                self.assertFalse(capture.closed.is_set())
                path, = self.output.glob("*.mp4")
                frames_before = len(self.inspect_video(path)[0])
                self.assertGreater(frames_before, 0)
                csv_before = path.with_suffix(".csv").read_bytes()
            finally:
                capture.unblock.set()
                terminal.join(1.0)
                self.assertTrue(capture.closed.wait(1.0))
            self.assertEqual(path.with_suffix(".csv").read_bytes(), csv_before)
            self.assertEqual(len(self.inspect_video(path)[0]), frames_before)

    def test_completed_mp4_fragments_survive_abrupt_process_exit(self):
        source = """
import os
import sys
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, sys.argv[1])
import record_webcams_pyav as recorder
from webcam_modes import CameraMode
recorder.OUTPUT_DIR = Path(sys.argv[2])
mode = CameraMode(32, 24, Fraction(30), 'MJPG')
recording = recorder.Recording(SimpleNamespace(index=3, name='USB Camera'),
                               '2026-09-17_12-34-56', mode)
for index in range(75):
    frame = recorder.av.VideoFrame(32, 24, 'yuv420p')
    for plane in frame.planes:
        plane.update(bytes([128]) * plane.buffer_size)
    pts = index * 1000000 // 30
    recording.write(frame, pts, 1789656789000000000 + pts * 1000)
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
        for frame, row in zip(frames, rows):
            self.assertAlmostEqual(float(frame.pts * frame.time_base),
                                   float(row["seconds_since_start"]), places=6)


if __name__ == "__main__":
    unittest.main()
