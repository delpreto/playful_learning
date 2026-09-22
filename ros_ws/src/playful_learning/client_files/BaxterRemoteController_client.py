"""Small Python 3 client for BaxterRemoteController_server.py (standard library only)."""

import base64
import csv
from datetime import datetime, timezone
from functools import wraps
import hashlib
import inspect
import json
import os
from pathlib import Path
import struct
import threading
import time
import uuid
import warnings
import zlib
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


_LOG_LOCK = threading.RLock()
_LOG_CONTEXT = threading.local()


class _History:
    """Append-only CSV; image references are relative to this directory."""

    def __init__(self, directory):
        self.directory = Path(directory).resolve()

    def _asset(self, data, suffix):
        relative = "images/" + hashlib.sha256(data).hexdigest() + suffix
        target = self.directory / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            temporary = target.with_name(uuid.uuid4().hex + ".tmp")
            try:
                temporary.write_bytes(data)
                os.replace(str(temporary), str(target))
            finally:
                if temporary.exists():
                    temporary.unlink()
        return relative

    def value(self, value):
        if isinstance(value, Operation):
            return {"operation_id": value.operation_id}
        if isinstance(value, dict):
            value = dict(value)
            width, height = value.get("width"), value.get("height")
            key = "rgb_data" if "rgb_data" in value else "rgb_base64"
            if type(width) is int and type(height) is int and key in value:
                raw = value[key]
                if key == "rgb_base64":
                    try:
                        raw = base64.b64decode(raw, validate=True)
                    except (ValueError, TypeError):
                        pass
                if isinstance(raw, (bytes, bytearray)) and 0 < width <= 1024 and \
                        0 < height <= 600 and len(raw) == width * height * 3:
                    def chunk(kind, data):
                        return (struct.pack("!I", len(data)) + kind + data +
                                struct.pack("!I", zlib.crc32(kind + data) & 0xffffffff))
                    scanlines = b"".join(b"\0" + raw[y:y + width * 3]
                                         for y in range(0, len(raw), width * 3))
                    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack("!2I5B", width,
                           height, 8, 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(scanlines)) +
                           chunk(b"IEND", b""))
                    value[key] = self._asset(png, ".png")
            if isinstance(value.get("image_file"), (str, os.PathLike)):
                source = Path(value["image_file"])
                if source.is_file():
                    value["image_file"] = {"source": str(source),
                                          "path": self._asset(source.read_bytes(), source.suffix)}
            return {str(key): self.value(item) for key, item in value.items()}
        if isinstance(value, (bytes, bytearray)):
            data = bytes(value)
            suffix = ".png" if data.startswith(b"\x89PNG\r\n\x1a\n") else \
                     ".jpg" if data.startswith(b"\xff\xd8\xff") else ".bin"
            return self._asset(data, suffix)
        if isinstance(value, (list, tuple)):
            return [self.value(item) for item in value]
        if isinstance(value, Path):
            return str(value)
        if value is None or isinstance(value, (str, bool, int, float)):
            return value
        return "<%s.%s>" % (type(value).__module__, type(value).__qualname__)

    def append(self, row):
        self.directory.mkdir(parents=True, exist_ok=True)
        # A stuck logger must not delay heartbeats beyond the robot's lease.
        deadline = time.monotonic() + 0.1
        if not _LOG_LOCK.acquire(timeout=0.1):
            raise TimeoutError("Interface history lock is busy")
        try:
            with (self.directory / ".lock").open("a+b") as lock:
                if os.name == "nt":
                    import msvcrt
                    if lock.seek(0, 2) == 0:
                        lock.write(b"\0")
                        lock.flush()
                    lock.seek(0)
                else:
                    import fcntl
                while True:
                    try:
                        if os.name == "nt":
                            msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
                        else:
                            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                        break
                    except OSError:
                        if time.monotonic() >= deadline:
                            raise TimeoutError("Interface history lock is busy")
                        time.sleep(0.005)
                try:
                    with (self.directory / "robot_interface_log.csv").open(
                            "a", newline="", encoding="utf-8") as stream:
                        writer = csv.DictWriter(stream, fieldnames=list(row))
                        if stream.tell() == 0:
                            writer.writeheader()
                        writer.writerow(row)
                finally:
                    if os.name == "nt":
                        lock.seek(0)
                        msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)
                    else:
                        fcntl.flock(lock, fcntl.LOCK_UN)
        finally:
            _LOG_LOCK.release()


def _log_warning(error):
    # Logging must never turn an accepted robot command into a retryable error.
    try:
        warnings.warn("Baxter interface history could not be written: %s" % error, RuntimeWarning)
    except Exception:
        pass


def _logged(method):
    signature = inspect.signature(method)

    @wraps(method)
    def wrapper(self, *args, **kwargs):
        client = self.client if isinstance(self, Operation) else self
        started, tick = time.time(), time.monotonic()
        call_id, parent_id = uuid.uuid4().hex, getattr(_LOG_CONTEXT, "call_id", "")
        _LOG_CONTEXT.call_id = call_id
        result, error = None, None
        try:
            arguments = dict(signature.bind(self, *args, **kwargs).arguments)
            arguments.pop("self")
        except TypeError:
            arguments = {"args": args, "kwargs": kwargs}
        if isinstance(self, Operation):
            arguments["operation_id"] = self.operation_id
        history = getattr(client, "_history", None)
        if history is not None:
            try:
                arguments = history.value(arguments)
            except Exception as failure:
                _log_warning(failure)
        try:
            result = method(self, *args, **kwargs)
            return result
        except BaseException as failure:
            error = {"type": type(failure).__name__, "message": str(failure)}
            raise
        finally:
            finished, duration = time.time(), time.monotonic() - tick
            _LOG_CONTEXT.call_id = parent_id
            try:
                history = getattr(client, "_history", None)
                if history is not None:
                    history.append({
                        "epoch_timestamp": started,
                        "timestamp_utc": datetime.fromtimestamp(started, timezone.utc).isoformat(),
                        "completed_epoch_timestamp": finished, "duration_s": duration,
                        "client_id": getattr(client, "client_id", ""),
                        "call_id": call_id, "parent_call_id": parent_id,
                        "method": method.__qualname__,
                        "arguments": json.dumps(history.value(arguments)),
                        "responses": json.dumps(history.value(result)),
                        "error": json.dumps(error),
                    })
            except Exception as failure:
                _log_warning(failure)
    return wrapper


class RemoteError(RuntimeError):
    """The server rejected a request, an operation failed, or communication failed."""

    def __init__(self, message, code=None):
        super().__init__(message)
        self.code = code


class Operation:
    """An accepted command. Acceptance does not mean the robot reached its target."""

    def __init__(self, client, operation_id):
        self.client = client
        self.operation_id = operation_id

    def status(self):
        return self.client.call("get_operation", operation_id=self.operation_id)

    def wait(self, timeout_s=60):
        """Return the completed result, or raise. A timeout does not cancel motion."""
        deadline = time.monotonic() + timeout_s
        while True:
            state = self.status()
            if state["status"] == "succeeded":
                return state.get("result")
            if state["status"] in ("failed", "cancelled"):
                raise RemoteError(state.get("error") or "Operation " + state["status"])
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Operation %s is unfinished; motion was not cancelled" %
                                   self.operation_id)
            time.sleep(min(0.1, remaining))


class BaxterRemoteController:
    """A local-style API with explicit waits for motion and automatic heartbeats.

    Motion methods return Operation; call .wait() to wait for the result. IK 
    waits internally and returns the result. Use a context
    manager or close() to relinquish control and request cancellation on exit.
    Optional arguments in **options use the corresponding controller names.
    Public calls (including nested RPCs, heartbeats and Operation waits) append
    timestamped arguments/results to log_dir/robot_interface_log.csv. Images
    are deduplicated in log_dir/images; logging failures only emit warnings.
    """

    def __init__(self, server, request_timeout_s=3, heartbeat=True, *,
                 log_dir="robot_interface_log"):
        try:
            self._history = _History(log_dir)
        except Exception as error:
            self._history = None
            _log_warning(error)
        self.url = server.rstrip("/")
        if not self.url.endswith("/rpc"):
            self.url += "/rpc"
        if not self.url.startswith(("http://", "https://")):
            raise ValueError("Server must start with http:// or https://")
        self.client_id = uuid.uuid4().hex
        self.request_timeout_s = request_timeout_s
        self.heartbeat_error = None
        self._closed = threading.Event()
        self._heartbeat_thread = None
        if heartbeat:
            self._heartbeat_thread = threading.Thread(target=self._heartbeat, daemon=True)
            self._heartbeat_thread.start()

    def call(self, method, **params):
        """Make one JSON-RPC request. Commands are never automatically retried."""
        if self._closed.is_set():
            raise RemoteError("Client is closed")
        request_id = uuid.uuid4().hex
        body = json.dumps({"jsonrpc": "2.0", "id": request_id,
                           "method": method, "params": params}, allow_nan=False).encode("utf-8")
        request = Request(self.url, data=body, headers={
            "Content-Type": "application/json",
            "X-Client-ID": self.client_id,
        })
        try:
            with urlopen(request, timeout=self.request_timeout_s) as response:
                reply = json.loads(response.read().decode("utf-8"))
        except HTTPError as error:
            try:
                reply = json.loads(error.read().decode("utf-8"))
            except (ValueError, UnicodeError):
                raise RemoteError("HTTP %s: %s" % (error.code, error.reason)) from error
        except (URLError, OSError, ValueError) as error:
            raise RemoteError("Request to %s failed: %s; a motion command may have been accepted" %
                              (self.url, error)) from error
        if not isinstance(reply, dict):
            raise RemoteError("Invalid JSON-RPC response")
        if "error" in reply:
            error = reply["error"]
            raise RemoteError(error.get("message", "Remote error"), error.get("code"))
        if reply.get("id") != request_id or "result" not in reply:
            raise RemoteError("Invalid JSON-RPC response")
        return reply["result"]

    def _heartbeat(self):
        while not self._closed.wait(1):
            try:
                self.call("heartbeat")
                self.heartbeat_error = None
            except RemoteError as error:
                self.heartbeat_error = str(error)

    def close(self):
        if self._closed.is_set():
            return
        try:
            self.call("release_control")
        except RemoteError:
            # If disconnected, the server's lease deadline requests cancellation.
            pass
        finally:
            self._closed.set()
            if self._heartbeat_thread is not None:
                self._heartbeat_thread.join(timeout=self.request_timeout_s + 1)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def _start(self, method, **params):
        accepted = self.call(method, **params)
        return Operation(self, accepted["operation_id"])

    # State reads are synchronous and do not acquire control.
    def get_state(self):
        return self.call("get_state")

    def get_joint_angles_rad(self):
        return self.call("get_joint_angles_rad")

    def get_joint_velocities_rad_s(self):
        return self.call("get_joint_velocities_rad_s")

    def get_joint_efforts_Nm(self):
        return self.call("get_joint_efforts_Nm")

    def get_end_effector_poses(self):
        return self.call("get_end_effector_poses")

    def get_camera_frame(self, camera_name):
        """Return one PNG frame as bytes; an inactive camera raises RemoteError."""
        if camera_name not in ("left_hand_camera", "right_hand_camera"):
            raise ValueError("Only left_hand_camera and right_hand_camera are supported")
        if self._closed.is_set():
            raise RemoteError("Client is closed")
        request = Request(self.url[:-4] + "/camera/" + camera_name + ".png", headers={
            "X-Client-ID": self.client_id,
        })
        try:
            with urlopen(request, timeout=self.request_timeout_s) as response:
                frame = response.read()
        except HTTPError as error:
            message = "HTTP %s: %s" % (error.code, error.reason)
            try:
                reply = json.loads(error.read().decode("utf-8"))
                if isinstance(reply, dict) and isinstance(reply.get("error"), str):
                    message = reply["error"]
            except (ValueError, UnicodeError):
                pass
            raise RemoteError(message, error.code) from error
        except (URLError, OSError) as error:
            raise RemoteError("Camera request failed: %s" % error) from error
        if not frame.startswith(b"\x89PNG\r\n\x1a\n"):
            raise RemoteError("Camera response was not a PNG image")
        return frame

    def get_wrist_camera_frame(self, limb_name):
        if limb_name not in ("left", "right"):
            raise ValueError("Limb must be left or right")
        return self.get_camera_frame(limb_name + "_hand_camera")

    # Head feedback is synchronous; supported commands return an Operation.
    def get_head_state(self):
        return self.call("get_head_state")

    def get_head_pan_rad(self):
        return self.call("get_head_pan_rad")

    def get_head_tilt_rad(self):
        """Return None: Baxter's SDK does not provide a continuous tilt angle."""
        return self.call("get_head_tilt_rad")

    def set_head_tilt_rad(self, angle_rad):
        """Raise RemoteError: use nod_head() for the SDK's discrete nod gesture."""
        return self.call("set_head_tilt_rad", angle_rad=angle_rad)

    def set_head_pan_rad(self, angle_rad, speed_percent=25, timeout_s=10, tolerance_rad=0.05):
        return self._start("set_head_pan_rad", angle_rad=angle_rad,
                           speed_percent=speed_percent, timeout_s=timeout_s,
                           tolerance_rad=tolerance_rad)

    def nod_head(self, times=1, internod_delay_s=0):
        return self._start("nod_head", times=times, internod_delay_s=internod_delay_s)

    def set_halo_led(self, red_percent, green_percent):
        return self._start("set_halo_led", red_percent=red_percent, green_percent=green_percent)

    def set_sonar_leds(self, led_states):
        return self._start("set_sonar_leds", led_states=led_states)

    def show_screen_color(self, color_rgb):
        return self._start("show_screen_color", color_rgb=color_rgb)

    def show_screen_image_rgb(self, width, height, rgb_data):
        """Send packed RGB bytes, three bytes per pixel, at most 1024 by 600."""
        if (type(width) is not int or type(height) is not int or
                not 0 < width <= 1024 or not 0 < height <= 600):
            raise ValueError("Image dimensions must be integers within 1024 x 600")
        if not isinstance(rgb_data, (bytes, bytearray)) or len(rgb_data) != width * height * 3:
            raise ValueError("rgb_data must contain exactly width * height * 3 RGB bytes")
        return self._start("show_screen_image_rgb", width=width, height=height,
                           rgb_base64=base64.b64encode(rgb_data).decode("ascii"))

    def show_screen_image(self, image_file):
        """Fit a local image file onto Baxter's screen; requires Pillow on this client."""
        try:
            from PIL import Image, ImageOps
        except ImportError as error:
            raise ImportError("Install Pillow on the client: python -m pip install Pillow") from error
        with Image.open(image_file) as source:
            picture = ImageOps.exif_transpose(source).convert("RGBA")
            picture = ImageOps.contain(picture, (1024, 600), Image.Resampling.LANCZOS)
            screen = Image.new("RGB", (1024, 600), (0, 0, 0))
            screen.paste(picture, ((1024 - picture.width) // 2,
                                  (600 - picture.height) // 2), picture)
        return self.show_screen_image_rgb(1024, 600, screen.tobytes())

    def is_movement_in_progress(self, limb_name=None):
        return self.call("is_movement_in_progress", limb_name=limb_name)

    def wait_for_movement_completion(self, limb_name=None, timeout_s=60):
        """Wait for idle; return False on timeout. Idle does not establish success."""
        deadline = time.monotonic() + timeout_s
        while self.is_movement_in_progress(limb_name):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            time.sleep(min(0.1, remaining))
        return True

    # These return immediately with an Operation, whose .wait() gives the result.
    # Joint/pose moves default to timeout_s=30 and tolerance_rad=0.008726646.
    def move_to_joint_angles_rad(self, joint_angles_rad_byLimb, **options):
        return self._start("move_to_joint_angles_rad",
                           joint_angles_rad_byLimb=joint_angles_rad_byLimb, **options)

    def move_to_neutral(self, limb_name=None, **options):
        return self._start("move_to_neutral", limb_name=limb_name, **options)

    def move_to_resting(self, limb_name=None, **options):
        return self._start("move_to_resting", limb_name=limb_name, **options)

    def move_to_gripper_pose(self, gripper_position_m_byLimb,
                             gripper_orientation_quaternion_wijk_byLimb, **options):
        return self._start("move_to_gripper_pose",
                           gripper_position_m_byLimb=gripper_position_m_byLimb,
                           gripper_orientation_quaternion_wijk_byLimb=
                           gripper_orientation_quaternion_wijk_byLimb, **options)

    def abort_movement(self, limb_name=None):
        """Request cancellation; the acknowledgement is not a confirmed stop."""
        return self.call("abort_movement", limb_name=limb_name)

    # IK and building only prepare data, so their wrappers wait for the result.
    def get_joint_angles_rad_for_gripper_pose(self, limb_name, gripper_position_m,
                                              gripper_orientation_quaternion_wijk,
                                              seed_joint_angles_rad=None, **options):
        return self._start("get_joint_angles_rad_for_gripper_pose", limb_name=limb_name,
                           gripper_position_m=gripper_position_m,
                           gripper_orientation_quaternion_wijk=gripper_orientation_quaternion_wijk,
                           seed_joint_angles_rad=seed_joint_angles_rad, **options).wait()

    def calibrate_gripper(self, limb_name):
        """Calibrate one gripper; moves its fingers. Wait for verified success."""
        return self._start("calibrate_gripper", limb_name=limb_name)

    def move_gripper(self, limb_name, gripper_open_percent, force_threshold_percent=75):
        """Move with a 0-75% moving-force threshold; holding force stays at 15%."""
        return self._start("move_gripper", limb_name=limb_name,
                           gripper_open_percent=gripper_open_percent,
                           force_threshold_percent=force_threshold_percent)

    def open_gripper(self, limb_name):
        return self._start("open_gripper", limb_name=limb_name)

    def close_gripper(self, limb_name):
        return self._start("close_gripper", limb_name=limb_name)

    def stop_gripper(self, limb_name):
        return self.call("stop_gripper", limb_name=limb_name)

    def get_gripper_position_open_percent(self, limb_name):
        return self.call("get_gripper_position_open_percent", limb_name=limb_name)

    def get_gripper_force_percent(self, limb_name):
        return self.call("get_gripper_force_percent", limb_name=limb_name)

    def is_gripper_grasping(self, limb_name):
        return self.call("is_gripper_grasping", limb_name=limb_name)

    def is_gripper_moving(self, limb_name):
        return self.call("is_gripper_moving", limb_name=limb_name)

    def jog_joint(self, limb_name, joint_name, delta_rad):
        return self._start("jog_joint", limb_name=limb_name,
                           joint_name=joint_name, delta_rad=delta_rad)

    def jog_endpoint(self, limb_name, axis, delta):
        """Base-frame x/y/z in meters or roll/pitch/yaw about base axes in radians."""
        return self._start("jog_endpoint", limb_name=limb_name, axis=axis, delta=delta)

    def jog_gripper(self, limb_name, delta_percent):
        return self._start("jog_gripper", limb_name=limb_name, delta_percent=delta_percent)


# Centralized instrumentation keeps the public API and its execution unchanged.
for _class in (BaxterRemoteController, Operation):
    for _name, _method in list(vars(_class).items()):
        if callable(_method) and (not _name.startswith("_") or
                                 (_class is BaxterRemoteController and _name == "__init__")):
            setattr(_class, _name, _logged(_method))
