"""Record webcams with direct PyAV capture on Windows/Linux. Type quit + Enter.

Install: python -m pip install --upgrade pip
         python -m pip install opencv-python cv2-enumerate-cameras av
Run:     python record_webcams_pyav.py

Capture and encoding keep native PyAV frames; only small preview images become
NumPy/OpenCV images. The separate record_webcams.py uses OpenCV capture.

Videos use H.264, fragmented MP4, and a shared variable-frame-rate timeline.
Each camera tries advertised modes in descending resolution, then FPS, and
uses the highest one it can open. The chosen mode is printed in the terminal.
Cameras record independently against one shared start clock.
Each worker captures, encodes, and saves its own video and CSV. Frame timing
is variable: jitter does not speed up or slow down playback. The first image
is shown from time zero; the last image is held until the shared stop time.
Healthy videos therefore have equal durations; a failed camera ends early.
CSV rows describe saved frames: zero-based index, video presentation seconds,
individual host capture epoch seconds, and local ISO timestamp with UTC offset.
Frame timestamps use the camera stream's timing, mapped to wall-clock time.
Windows timestamps are estimates calibrated during warmup, not sensor exposure.
Hardware synchronization is not available on ordinary USB webcams.
"""

import csv
import os
import re
import sys
import threading
import time
from contextlib import ExitStack
from datetime import datetime
from fractions import Fraction
from pathlib import Path

import av
import cv2
import numpy as np
from cv2_enumerate_cameras import enumerate_cameras

from stream_webcams import INTERNAL_NAMES
from webcam_modes import discover_modes
from pyav_webcam_capture import open_camera


OUTPUT_DIR = Path(__file__).resolve().parent / "recordings_pyav"
PREVIEW_SIZE = (640, 480)  # Display only; videos retain each camera's native size.
PREVIEW_FPS = 15  # Display only; recording runs at the camera's available rate.
CRF = 23  # H.264 quality: lower means better quality and larger files.
CAMERA_TIMEOUT = 3.0
STARTUP_TIMEOUT = 60.0  # Allows discovery and trying modes on multiple cameras.
TIME_BASE = Fraction(1, 1_000_000)
WINDOW = "PyAV webcam recording - type quit in the terminal"
CSV_COLUMNS = ("frame_index", "seconds_since_start", "timestamp_epoch_seconds",
               "timestamp_human")
CAMERA_SETUP_LOCK = threading.Lock()  # Avoid concurrent device setup and log discovery.
_PREVIEW_LOCAL = threading.local()


def choose_cameras():
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
    devices = {info.index: info for info in enumerate_cameras(backend)}
    inferred = []
    for index, info in devices.items():
        external = (info.vid is not None and
                    not any(word in info.name.lower() for word in INTERNAL_NAMES))
        if external:
            inferred.append(index)
        print("{}: {}{}".format(index, info.name, " [likely external USB]" if external else ""))
    if not devices:
        raise SystemExit("No cameras found.")
    while True:
        answer = input("Enter for {}, or camera indexes (e.g. 1,2): ".format(inferred)).strip()
        try:
            indexes = [int(item) for item in re.split(r"[,\s]+", answer)] if answer else inferred
            if not indexes or len(set(indexes)) != len(indexes):
                raise ValueError
            return [devices[index] for index in indexes]
        except (ValueError, KeyError):
            print("Choose one or more distinct indexes from the list above.")


class Session:
    """One release signal, monotonic start, and stop time for every camera."""

    def __init__(self):
        self.started = threading.Event()
        self.stopped = threading.Event()
        self.lock = threading.Lock()
        self.start_ns = self.stop_ns = None
        self.start_epoch_ns = None

    def start(self):
        with self.lock:
            if not self.stopped.is_set():
                self.start_ns = time.perf_counter_ns()
                self.start_epoch_ns = time.time_ns()
                self.started.set()

    def set(self):
        with self.lock:
            if not self.stopped.is_set():
                self.stop_ns = time.perf_counter_ns()
                self.stopped.set()
                self.started.set()  # Also wake workers if quit happens during setup.

    def is_set(self):
        return self.stopped.is_set()

    def wait(self, timeout=None):
        return self.stopped.wait(timeout)


class Camera:
    """One worker owns capture, encoding, CSV, and its latest preview image."""

    def __init__(self, info, session, prefix, encoder_threads, mode_ceiling=None):
        self.info, self.session = info, session
        self.prefix, self.encoder_threads = prefix, encoder_threads
        self.mode_ceiling = mode_ceiling
        self.modes = []
        self.ready = threading.Event()
        self.done = threading.Event()
        self.cancel = threading.Event()
        self.writer_lock = threading.RLock()
        self.active = True
        self.writer = None
        self.mode = None
        self.last_pts = -1
        self.error = None
        self.failed_ns = None
        self.stage = "setup"
        self.last_progress_ns = time.perf_counter_ns()
        self.preview = np.zeros((PREVIEW_SIZE[1], PREVIEW_SIZE[0], 3), dtype=np.uint8)
        self.thread = threading.Thread(target=self.capture, daemon=True)
        self.thread.start()

    def stopping(self):
        return self.session.is_set() or self.cancel.is_set()

    def capture(self):
        camera = None
        try:
            with CAMERA_SETUP_LOCK:
                if self.stopping():
                    return
                self.modes = discover_modes(self.info)
                for mode in self.modes:
                    if self.mode_ceiling and mode.width * mode.height > self.mode_ceiling:
                        continue
                    if self.stopping():
                        return
                    try:
                        camera = open_camera(self.info, mode,
                                             timeout=max(CAMERA_TIMEOUT, 3 / float(mode.fps)))
                        self.mode = mode
                        break
                    except (RuntimeError, OSError, ValueError, av.error.FFmpegError) as exc:
                        print("{} ({}): {}x{} @ {:g} FPS unavailable: {}".format(
                            self.info.name, self.info.index, mode.width, mode.height,
                            float(mode.fps), exc))
                        if camera is not None:
                            camera.close()
                            camera = None
                if self.mode is None:
                    raise RuntimeError("could not open any advertised camera mode")
                print("{} ({}): {} x {} at {:g} FPS".format(
                    self.info.name, self.info.index, self.mode.width, self.mode.height,
                    float(self.mode.fps)))
            with self.writer_lock:
                if self.stopping():
                    return
                self.writer = Recording(self.info, self.prefix, self.mode, self.encoder_threads)
            # Drain probing buffers and calibrate the input clock before joining
            # the shared start. Earlier cameras keep draining while others open.
            warmup_end = time.monotonic() + 0.25
            while not self.stopping() and time.monotonic() < warmup_end:
                camera.read()
            self.ready.set()
            next_preview_ns = 0
            clock_frozen = False
            while not self.stopping():
                self.stage = "capture"
                self.last_progress_ns = time.perf_counter_ns()
                frame = camera.read()
                monotonic_ns, epoch_ns = time.perf_counter_ns(), time.time_ns()
                if (frame.width, frame.height) != (self.mode.width, self.mode.height):
                    raise RuntimeError("camera returned a different resolution than requested")
                # Drain startup frames while other cameras open, so recording
                # starts with current images instead of each device's old queue.
                if not self.session.started.is_set() or self.session.start_ns is None:
                    continue
                if not clock_frozen:
                    if hasattr(camera, "freeze_clock"):
                        camera.freeze_clock()
                    clock_frozen = True
                capture_epoch_ns = getattr(camera, "frame_epoch_ns", None)
                if capture_epoch_ns is not None:
                    epoch_ns = capture_epoch_ns
                    monotonic_ns = self.session.start_ns + epoch_ns - self.session.start_epoch_ns
                if monotonic_ns < self.session.start_ns:
                    continue
                with self.writer_lock:
                    if self.cancel.is_set() or self.writer.closed:
                        break
                    if self.session.is_set() and monotonic_ns >= self.session.stop_ns:
                        break
                    self.stage = "encode"
                    pts = (0 if self.writer.count == 0 else max(
                        self.last_pts + 1, (monotonic_ns - self.session.start_ns) // 1000))
                    self.writer.write(frame, pts, epoch_ns)
                    self.last_pts = pts
                    self.last_progress_ns = time.perf_counter_ns()
                if monotonic_ns >= next_preview_ns:
                    self.preview = preview_frame(frame)  # Atomic reference replacement for GUI.
                    next_preview_ns = monotonic_ns + 1_000_000_000 // PREVIEW_FPS
        except Exception as exc:
            self.fail(exc)
        finally:
            self.ready.set()
            try:
                self.finish()
            finally:
                self.active = False
                self.done.set()
                # A broken driver's close() may hang; files are already finalized.
                if camera is not None:
                    camera.close()

    def fail(self, error):
        self.cancel.set()
        self.active = False
        if self.error is None:
            self.error = str(error)
            self.failed_ns = time.perf_counter_ns()
            print("{} ({}): {}. Recording stopped for this camera.".format(
                self.info.name, self.info.index, error))

    def finish(self, timeout=0.1):
        """Watchdog may close a writer during a stuck read, but never during encoding."""
        if not self.writer_lock.acquire(timeout=timeout):
            return False
        try:
            if self.writer is not None and not self.writer.closed:
                endings = [stamp for stamp in (self.failed_ns, self.session.stop_ns)
                           if stamp is not None]
                end_ns = min(endings) if endings else time.perf_counter_ns()
                end_pts = max(0, (end_ns - (self.session.start_ns or end_ns)) // 1000)
                self.writer.close(end_pts)
            return True
        except Exception as exc:
            print("Error closing camera {}: {}".format(self.info.index, exc), file=sys.stderr)
            return True
        finally:
            self.writer_lock.release()


class Recording:
    """Per-camera files; the Camera writer lock protects write/close operations."""

    def __init__(self, info, prefix, mode, encoder_threads=2):
        name = re.sub(r"[^A-Za-z0-9_-]+", "-", info.name).strip("-_")[:80] or "camera"
        self.path = OUTPUT_DIR / "{}_{}-{}.mp4".format(prefix, name, info.index)
        self.count = 0
        self.closed = False
        self.pending_packet = None
        self.last_pts = None
        self.nominal_duration = max(1, round(1_000_000 / mode.fps))
        self.resources = ExitStack()
        self.created_paths = []
        try:
            if self.path.exists() or self.path.with_suffix(".csv").exists():
                raise FileExistsError("Recording already exists: {}".format(self.path))
            video_file = self.resources.enter_context(self.path.open("xb", buffering=0))
            self.created_paths.append(self.path)
            self.csv_file = self.resources.enter_context(
                self.path.with_suffix(".csv").open("x", newline="", encoding="utf-8", buffering=1))
            self.created_paths.append(self.path.with_suffix(".csv"))
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(CSV_COLUMNS)
            self.container = self.resources.enter_context(av.open(
                video_file, "w", format="mp4", options={
                    "movflags": "frag_keyframe+delay_moov+default_base_moof",
                    "movie_timescale": "1000000",
                    "frag_duration": "1000000",
                    "flush_packets": "1",
                }))
            self.stream = self.container.add_stream("libx264", rate=mode.fps)
            self.stream.width, self.stream.height = mode.width, mode.height
            # MJPEG arrives in full-range YUV. Preserve that range to avoid a
            # costly full-frame range conversion; normal even sizes stay 4:2:0.
            full_range = mode.fourcc in ("MJPG", "JPEG")
            chroma = "420p" if mode.width % 2 == mode.height % 2 == 0 else "444p"
            self.stream.pix_fmt = ("yuvj" if full_range else "yuv") + chroma
            self.stream.codec_context.color_range = 2 if full_range else 1
            self.stream.time_base = self.stream.codec_context.time_base = TIME_BASE
            self.stream.codec_context.max_b_frames = 0
            self.stream.codec_context.thread_count = encoder_threads
            self.stream.codec_context.gop_size = max(1, round(mode.fps))
            self.stream.options = {"crf": str(CRF), "preset": "ultrafast", "tune": "zerolatency"}
            self.container.start_encoding()
        except BaseException:
            self.resources.close()
            for path in self.created_paths:
                path.unlink()
            raise
        print("Saving {}".format(self.path))

    def write(self, frame, pts, epoch_ns):
        # Keep decoded YUV in native buffers instead of copying a full-size BGR
        # array back into PyAV. JPEG inputs mark every frame as I: clear that hint
        # so H.264 can choose inter-frames and maintain a small file size.
        frame.pts, frame.time_base = pts, TIME_BASE
        frame.pict_type = 0
        for packet in self.stream.encode(frame):
            self.accept_packet(packet)
        epoch = epoch_ns / 1_000_000_000
        human = datetime.fromtimestamp(epoch).astimezone().isoformat(timespec="microseconds")
        self.csv.writerow((self.count, "{:.6f}".format(pts / 1_000_000),
                           "{}.{:09d}".format(*divmod(epoch_ns, 1_000_000_000)), human))
        self.count += 1
        self.last_pts = pts

    def accept_packet(self, packet):
        # Keep one compressed packet until its exact display duration is known.
        if self.pending_packet is not None:
            self.pending_packet.duration = max(1, packet.pts - self.pending_packet.pts)
            self.container.mux(self.pending_packet)
        self.pending_packet = packet

    def close(self, end_pts=None):
        if self.closed:
            return
        self.closed = True
        try:
            for packet in self.stream.encode():
                self.accept_packet(packet)
            if self.pending_packet is not None:
                if end_pts is None:
                    end_pts = self.last_pts + self.nominal_duration
                self.pending_packet.duration = max(1, end_pts - self.pending_packet.pts)
                self.container.mux(self.pending_packet)
                self.pending_packet = None
        finally:
            self.resources.close()
        if self.count == 0:
            for path in self.created_paths:
                path.unlink()
            return
        rate = " ({:.1f} FPS)".format(self.count * 1_000_000 / end_pts) if end_pts else ""
        print("Saved {} frames{}: {}".format(self.count, rate, self.path.name))


def wait_ready(cameras, session, timeout):
    """Wait for camera/encoder setup without blocking terminal quit."""
    pending = list(cameras)
    deadline = time.monotonic() + timeout
    while pending and not session.is_set() and time.monotonic() < deadline:
        pending = [camera for camera in pending if not camera.ready.is_set()]
        session.wait(0.005)
    if not session.is_set():
        for camera in pending:
            camera.fail(TimeoutError("camera setup timed out"))
            camera.finish()


def wait_for_quit(stop):
    # Avoid input() here: its buffered-stdin lock can crash Python at shutdown
    # if recording ends while this daemon is still waiting for terminal input.
    line = bytearray()
    try:
        while not stop.is_set():
            character = os.read(sys.stdin.fileno(), 1)
            if not character:
                stop.set()
            elif character in (b"\r", b"\n"):
                if line.strip().lower() == b"quit":
                    stop.set()
                line.clear()
            else:
                line.extend(character)
    except (OSError, ValueError):
        stop.set()


def record(devices, session):
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    prefix = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    cameras = []
    encoder_threads = max(1, min(4, (os.cpu_count() or 1) // max(1, len(devices))))
    ceilings = {}
    try:
        while not session.is_set():
            for info in devices:
                cameras.append(Camera(info, session, prefix, encoder_threads,
                                      ceilings.get(info.index)))
            wait_ready(cameras, session, STARTUP_TIMEOUT)
            if (session.is_set() or not cameras or not cameras[0].active or
                    all(camera.active for camera in cameras)):
                break
            # If later devices cannot open, free USB capacity held by earlier ones.
            # Each retry lowers a finite advertised resolution, before recording.
            candidates = []
            for position, camera in enumerate(cameras):
                if camera.active and camera.mode:
                    area = camera.mode.width * camera.mode.height
                    lower = [mode for mode in camera.modes if mode.width * mode.height < area]
                    if lower:
                        candidates.append((area, position, camera, lower[0]))
            if not candidates:
                break
            _, _, selected, lower = max(candidates, key=lambda item: item[:2])
            for camera in cameras:
                camera.cancel.set()
            deadline = time.monotonic() + 2
            for camera in cameras:
                camera.thread.join(max(0, deadline - time.monotonic()))
            if any(camera.thread.is_alive() for camera in cameras):
                break  # Never reopen devices still owned by an unresponsive driver.
            ceilings[selected.info.index] = lower.width * lower.height
            print("Retrying setup with camera {} limited to {}x{} to make room "
                  "for all {} cameras.".format(selected.info.index, lower.width,
                                               lower.height, len(devices)))
            cameras = []
        if session.is_set() or not any(camera.active for camera in cameras):
            return cameras
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, min(PREVIEW_SIZE[0] * len(cameras), 1600), PREVIEW_SIZE[1])
        session.start()
        while not session.is_set() and any(camera.active for camera in cameras):
            now_ns = time.perf_counter_ns()
            previews = []
            for camera in cameras:
                if camera.active and camera.stage == "capture":
                    timeout = max(CAMERA_TIMEOUT, 3 / float(camera.mode.fps))
                    if (now_ns - camera.last_progress_ns) / 1_000_000_000 > timeout:
                        camera.fail(TimeoutError("camera capture timed out"))
                        camera.finish()
                frame = camera.preview.copy()
                elapsed = max(0.001, (now_ns - session.start_ns) / 1_000_000_000)
                count = camera.writer.count if camera.writer else 0
                status = " [{:.1f} FPS]".format(count / elapsed) if camera.active else " [STOPPED]"
                label = "{}: {}{}".format(camera.info.index, camera.info.name, status)
                cv2.putText(frame, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0) if camera.active else (0, 0, 255), 2)
                previews.append(frame)
            cv2.imshow(WINDOW, cv2.hconcat(previews))
            cv2.waitKey(1)
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                session.set()
            session.wait(1 / 30)  # GUI only; capture and encoding have no GUI rate limit.
    finally:
        session.set()
        deadline = time.monotonic() + 2.0
        for camera in cameras:
            camera.thread.join(max(0, deadline - time.monotonic()))
        for camera in cameras:
            # A hung camera read holds no writer lock: finalize safely from here.
            if not camera.finish():
                print("Camera {} is still encoding/writing; completed MP4 fragments "
                      "are preserved.".format(camera.info.index), file=sys.stderr)
        cv2.destroyAllWindows()
    return cameras


def preview_frame(frame):
    """Downscale in PyAV, then convert only the thumbnail to a BGR NumPy array."""
    width, height = frame.width, frame.height
    scale = min(PREVIEW_SIZE[0] / width, PREVIEW_SIZE[1] / height)
    width, height = max(1, round(width * scale)), max(1, round(height * scale))
    if not hasattr(_PREVIEW_LOCAL, "reformatter"):
        _PREVIEW_LOCAL.reformatter = av.video.reformatter.VideoReformatter()
    resized = _PREVIEW_LOCAL.reformatter.reformat(
        frame, width=width, height=height, format="bgr24").to_ndarray()
    tile = np.zeros((PREVIEW_SIZE[1], PREVIEW_SIZE[0], 3), dtype=np.uint8)
    x, y = (PREVIEW_SIZE[0] - width) // 2, (PREVIEW_SIZE[1] - height) // 2
    tile[y:y + height, x:x + width] = resized
    return tile


def main():
    devices = choose_cameras()
    stop = Session()
    print("Recording to {}. Type quit + Enter to stop.".format(OUTPUT_DIR))
    threading.Thread(target=wait_for_quit, args=(stop,), daemon=True).start()
    record(devices, stop)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print("Stopped.")
