"""Discover native webcam modes through DirectShow (Windows) or V4L2 (Linux)."""

import errno
import math
import os
import re
import struct
import sys
import threading
from dataclasses import dataclass
from fractions import Fraction

import av
import cv2


@dataclass(frozen=True)
class CameraMode:
    width: int
    height: int
    fps: Fraction
    fourcc: str = None


_LOG_LOCK = threading.Lock()
_DSHOW_FOURCC = {
    "mjpeg": "MJPG", "h264": "H264", "hevc": "HEVC",
    "yuyv422": "YUY2", "uyvy422": "UYVY", "yuv420p": "I420",
    "nv12": "NV12", "gray": "Y800", "gray16le": "Y16 ",
}


def parse_dshow_modes(output):
    """FFmpeg reports a minimum/maximum size and FPS for each capture format."""
    modes = []
    pattern = (r"(?:vcodec|pixel_format)=(\w+).*?"
               r"max s=(\d+)x(\d+) fps=([\d.eE+/-]+)")
    for match in re.finditer(pattern, output):
        pixel_format, width, height, fps = match.groups()
        try:
            mode = CameraMode(int(width), int(height), Fraction(fps),
                              _DSHOW_FOURCC.get(pixel_format))
        except (ValueError, ZeroDivisionError):
            continue
        if mode.width > 0 and mode.height > 0 and mode.fps > 0:
            modes.append(mode)
    return modes


def _windows_modes(info):
    if "dshow" not in av.formats_available:
        raise RuntimeError("This PyAV build lacks DirectShow; install the Windows PyAV wheel.")
    if not info.path:
        raise RuntimeError("Camera has no unique device path for mode discovery.")
    # Friendly names are not unique when identical USB webcams are connected.
    device = info.path
    if not device.startswith("@device_"):
        device = "@device_pnp_" + device
    device = device.replace(":", "_")  # FFmpeg's DirectShow alternative-name convention.
    with _LOG_LOCK:
        previous_level = av.logging.get_level()
        av.logging.set_level(av.logging.INFO)
        try:
            with av.logging.Capture(local=True) as messages:
                try:
                    with av.open("video=" + device, format="dshow",
                                 options={"list_options": "true"}):
                        pass
                except (av.error.FFmpegError, OSError, ValueError):
                    # list_options intentionally exits without opening a stream.
                    pass
        finally:
            av.logging.set_level(previous_level)
    output = "".join(message for _, _, message in messages)
    modes = parse_dshow_modes(output)
    if not modes:
        detail = output.strip().splitlines()
        raise RuntimeError("Could not query camera modes: " +
                           (detail[-1] if detail else "DirectShow returned no formats"))
    return modes


def _linux_modes(info):
    import fcntl  # Unavailable on Windows.

    def query(fd, number, size, *values):
        data = bytearray(size)
        struct.pack_into("=" + "I" * len(values), data, 0, *values)
        # _IOWR('V', number, struct), from linux/videodev2.h.
        request = 0xC0000000 | (size << 16) | (ord("V") << 8) | number
        try:
            fcntl.ioctl(fd, request, data, True)
        except OSError as exc:
            if exc.errno == errno.EINVAL:
                return None  # End of this enumeration.
            raise
        return struct.unpack("=" + "I" * (size // 4), data)

    modes = []
    path = info.path or "/dev/video{}".format(info.index)
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    try:
        format_index = 0
        while True:
            # v4l2_fmtdesc: index, type=VIDEO_CAPTURE, flags, description[32], fourcc.
            pixel = query(fd, 2, 64, format_index, 1)
            if pixel is None:
                break
            format_index += 1
            pixel_format = pixel[11]
            fourcc = int(pixel_format).to_bytes(4, "little").decode("latin1")
            size_index = 0
            while True:
                size = query(fd, 74, 44, size_index, pixel_format)
                if size is None:
                    break
                size_index += 1
                if size[2] == 1:  # Discrete size.
                    width, height = size[3:5]
                elif size[2] in (2, 3):  # Continuous/stepwise: largest legal size.
                    min_w, max_w, step_w, min_h, max_h, step_h = size[3:9]
                    width = max_w - (max_w - min_w) % max(1, step_w)
                    height = max_h - (max_h - min_h) % max(1, step_h)
                else:
                    break
                interval_index = 0
                while True:
                    interval = query(fd, 75, 52, interval_index, pixel_format, width, height)
                    if interval is None:
                        break
                    interval_index += 1
                    # Discrete interval, or the minimum stepwise interval (maximum FPS).
                    numerator, denominator = interval[5:7]
                    if numerator and denominator and width and height:
                        modes.append(CameraMode(width, height,
                                                Fraction(denominator, numerator), fourcc))
                    if interval[4] != 1:
                        break
                if size[2] != 1:
                    break
    finally:
        os.close(fd)
    return modes


def discover_modes(info):
    """Largest pixel area first, then fastest FPS; prefer MJPEG for equal modes."""
    if sys.platform == "win32":
        modes = _windows_modes(info)
    elif sys.platform.startswith("linux"):
        modes = _linux_modes(info)
    else:
        raise RuntimeError("Automatic camera mode discovery supports Windows and Linux.")
    if not modes:
        raise RuntimeError("Camera driver did not enumerate resolutions and frame rates.")
    return sorted(set(modes), key=lambda m: (m.width * m.height, m.fps,
                                             m.fourcc == "MJPG", m.width), reverse=True)


def configure_camera(camera, mode):
    """Request an advertised mode and reject silent driver fallback to a lower mode."""
    if sys.platform == "win32":
        # DirectShow's FPS setter rebuilds the graph and can reset the pixel format.
        camera.set(cv2.CAP_PROP_FPS, float(mode.fps))
    if mode.fourcc:
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*mode.fourcc))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, mode.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, mode.height)
    if sys.platform != "win32":
        camera.set(cv2.CAP_PROP_FPS, float(mode.fps))
    elif mode.fourcc:
        # Width/height changes may also rebuild DirectShow's graph.
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*mode.fourcc))
    width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = camera.get(cv2.CAP_PROP_FPS)
    if ((width, height) != (mode.width, mode.height) or not math.isfinite(fps) or
            fps <= 0 or not math.isclose(fps, float(mode.fps), rel_tol=0.01)):
        raise RuntimeError("Requested {}x{} @ {:g} fps, but camera negotiated {}x{} @ {:g} fps"
                           .format(mode.width, mode.height, float(mode.fps), width, height, fps))
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Best effort; backend-dependent.
    return CameraMode(width, height, Fraction(fps).limit_denominator(100_000), mode.fourcc)
