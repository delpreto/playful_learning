"""Open native PyAV webcam frames with DirectShow or Video4Linux2.

Open, read, and close each camera in the same worker thread. PyAV's timeout is
best effort: some device drivers block without checking FFmpeg's interrupt hook.
"""

import math
import sys
import time

import av


_COMPRESSED_FORMATS = {
    "MJPG": "mjpeg", "JPEG": "mjpeg", "H264": "h264", "AVC1": "h264",
    "HEVC": "hevc", "H265": "hevc", "VP80": "vp8", "VP90": "vp9",
}
_RAW_FORMATS = {
    "YUY2": "yuyv422", "YUYV": "yuyv422", "UYVY": "uyvy422",
    "I420": "yuv420p", "YU12": "yuv420p", "NV12": "nv12",
    "NV21": "nv21", "Y800": "gray", "GREY": "gray", "Y16 ": "gray16le",
    "RGB3": "rgb24", "BGR3": "bgr24",
}


def _input_options(info, mode):
    options = {"video_size": "{}x{}".format(mode.width, mode.height),
               "framerate": str(mode.fps)}
    codec = _COMPRESSED_FORMATS.get(mode.fourcc)
    pixel_format = _RAW_FORMATS.get(mode.fourcc)
    if mode.fourcc and not (codec or pixel_format):
        raise RuntimeError("Unsupported native camera format: " + mode.fourcc)

    if sys.platform == "win32":
        if not info.path:
            raise RuntimeError("Camera has no unique DirectShow device path.")
        device = info.path
        if not device.startswith("@device_"):
            device = "@device_pnp_" + device
        source = "video=" + device.replace(":", "_")
        backend = "dshow"
        # Timestamp arrival at DirectShow's callback, before its packet queue.
        # These are graph-clock ticks, not Unix epoch timestamps.
        options["use_video_device_timestamps"] = "false"
        # Keep a finite device queue, large enough for at least two raw frames.
        # Compressed capture needs much less space than raw RGB capture.
        bytes_per_pixel = 1 if codec else 4
        options["rtbufsize"] = str(max(1024 * 1024,
                                      mode.width * mode.height * bytes_per_pixel * 2))
        if pixel_format:
            options["pixel_format"] = pixel_format
        # PyAV 12 does not expose AVFormatContext.video_codec_id before opening.
        # FFmpeg's CLI-only `vcodec` option is silently ignored here. DirectShow
        # chooses a format matching size/rate; verify its codec below instead.
    elif sys.platform.startswith("linux"):
        source = info.path or "/dev/video{}".format(info.index)
        backend = "v4l2"
        options["timestamps"] = "abs"  # FFmpeg converts kernel monotonic times to epoch.
        if codec or pixel_format:
            options["input_format"] = codec or pixel_format
    else:
        raise RuntimeError("Native webcam capture supports Windows and Linux.")
    if backend not in av.formats_available:
        raise RuntimeError("This PyAV build lacks the {} capture device.".format(backend))
    return source, backend, options, codec, pixel_format


class NativeCamera:
    """Native frames and their arrival/capture timestamps, before decoding queues.

    frame_epoch_ns is an integer epoch time, or None if the input supplies no
    timestamp. Linux uses the driver's timestamp converted by FFmpeg. Windows
    estimates the offset between DirectShow's graph clock and the host clock
    during warmup; it is not a sensor exposure timestamp. Call freeze_clock()
    after warmup to preserve source timing without ongoing clock recalibration.
    """

    def __init__(self, container, mode, codec, pixel_format, backend):
        self.container = container
        self.mode = mode
        self.closed = False
        self.backend = backend
        self.frame_epoch_ns = None
        self._source_offset_ns = None
        self._last_source_ns = None
        self._clock_frozen = False
        self._epoch_offset_ns = time.time_ns() - time.perf_counter_ns()
        if not container.streams.video:
            raise RuntimeError("Camera did not provide a video stream.")
        self.stream = container.streams.video[0]
        context = self.stream.codec_context
        if (context.width, context.height) != (mode.width, mode.height):
            raise RuntimeError("Requested {}x{}, but camera negotiated {}x{}".format(
                mode.width, mode.height, context.width, context.height))
        rate = self.stream.average_rate
        if rate is not None and (not math.isfinite(float(rate)) or rate <= 0 or
                                 not math.isclose(float(rate), float(mode.fps), rel_tol=0.01)):
            raise RuntimeError("Requested {:g} fps, but camera negotiated {:g} fps".format(
                float(mode.fps), float(rate)))
        if codec and context.name != codec:
            raise RuntimeError("Requested {}, but camera negotiated {}. This PyAV "
                               "build cannot force DirectShow's compressed input codec."
                               .format(codec, context.name))
        if pixel_format and (context.name != "rawvideo" or
                             context.format is None or context.format.name != pixel_format):
            raise RuntimeError("Camera did not negotiate the requested {} raw format."
                               .format(pixel_format))
        # Slice threading works within a frame; frame threading can add latency.
        context.thread_count = 2
        context.thread_type = "SLICE"
        self._frames = iter(container.decode(self.stream))

    def read(self):
        if self.closed:
            raise RuntimeError("Camera is closed.")
        try:
            frame = next(self._frames)
        except StopIteration:
            raise RuntimeError("Camera stream ended.") from None
        if (frame.width, frame.height) != (self.mode.width, self.mode.height):
            raise RuntimeError("Camera changed frame dimensions during capture.")
        # Save the input timestamp before the caller replaces frame.pts to encode.
        self.frame_epoch_ns = None
        if frame.pts is not None and frame.time_base is not None:
            source_ns = int(frame.pts * frame.time_base * 1_000_000_000)
            if self.backend == "v4l2":
                self.frame_epoch_ns = source_ns
            else:
                observed_offset = time.perf_counter_ns() - source_ns
                if self._last_source_ns is not None and source_ns < self._last_source_ns:
                    if self._clock_frozen:
                        raise RuntimeError("Camera timestamp moved backwards during recording.")
                    self._source_offset_ns = None
                if self._source_offset_ns is None:
                    self._source_offset_ns = observed_offset
                elif not self._clock_frozen:
                    self._source_offset_ns = min(self._source_offset_ns, observed_offset)
                self._last_source_ns = source_ns
                # DirectShow's clock has an arbitrary origin. The minimum
                # observed delivery delay estimates its offset to the host
                # clock. Warmup reads improve this estimate; later queuing must
                # not make an older captured image appear newly captured.
                self.frame_epoch_ns = source_ns + self._source_offset_ns + self._epoch_offset_ns
        return frame

    def freeze_clock(self):
        """Fix the Windows graph-to-host clock estimate after draining warmup frames."""
        self._clock_frozen = True

    def close(self):
        if not self.closed:
            self.closed = True
            self.container.close()


def open_camera(info, mode, timeout=3.0):
    """Open a verified advertised mode without converting frames to NumPy/BGR.

    On Windows, compressed mode selection is checked after opening because
    PyAV 12 cannot force DirectShow's input codec. Unsupported selections fail
    explicitly so callers can try the next advertised mode.
    """
    source, backend, options, codec, pixel_format = _input_options(info, mode)
    container = av.open(source, mode="r", format=backend, options=options,
                        timeout=(timeout, timeout))
    try:
        return NativeCamera(container, mode, codec, pixel_format, backend)
    except BaseException:
        container.close()
        raise
