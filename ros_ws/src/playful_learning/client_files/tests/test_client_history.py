"""Hardware-free history tests; BAXTER_CLIENT_SOURCE can select the Eko copy."""

import base64
import csv
import importlib.util
import io
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
import unittest
from unittest.mock import patch
import zlib


SOURCE = Path(os.environ.get("BAXTER_CLIENT_SOURCE", str(
    Path(__file__).resolve().parents[1] / "BaxterRemoteController_client.py")))
spec = importlib.util.spec_from_file_location("history_client", SOURCE)
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)
PNG = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aX1sAAAAASUVORK5CYII=")


class HistoryTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.requests = []
        self.operation_status = "succeeded"
        opener = patch.object(client, "urlopen", self.respond)
        opener.start()
        self.addCleanup(opener.stop)
        self.robot = client.BaxterRemoteController("http://test", heartbeat=False,
                                                  log_dir=self.directory)

    def respond(self, request, timeout):
        if request.data is None:
            return io.BytesIO(PNG)
        body = json.loads(request.data)
        self.requests.append(body)
        result = True
        if body["method"] == "get_state":
            result = {"joint": 0.5, "feedback_stale": False}
        elif body["method"] == "get_operation":
            result = {"status": self.operation_status, "result": {"completed": True},
                      "error": "test failure"}
        elif body["method"] in ("jog_endpoint", "show_screen_image_rgb"):
            result = {"operation_id": "op-1"}
        reply = {"jsonrpc": "2.0", "id": body["id"], "result": result}
        if body["method"] == "fail":
            reply = {"jsonrpc": "2.0", "id": body["id"],
                     "error": {"message": "rejected", "code": -32000}}
        return io.BytesIO(json.dumps(reply).encode())

    def rows(self):
        with (self.directory / "robot_interface_log.csv").open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        for row in rows:
            for key in ("arguments", "responses", "error"):
                row[key] = json.loads(row[key])
        return rows

    def test_calls_arguments_results_nested_ids_and_timestamps(self):
        self.robot.get_state()
        operation = self.robot.jog_endpoint("left", "x", 0.002)
        self.assertEqual(operation.wait(), {"completed": True})
        self.robot.close()
        rows = self.rows()
        by_method = {row["method"]: row for row in rows}
        move = by_method["BaxterRemoteController.jog_endpoint"]
        self.assertEqual(move["arguments"], {"limb_name": "left", "axis": "x", "delta": 0.002})
        self.assertEqual(move["responses"], {"operation_id": "op-1"})
        rpc = next(row for row in rows if row["arguments"].get("method") == "jog_endpoint")
        self.assertEqual(rpc["parent_call_id"], move["call_id"])
        self.assertEqual(by_method["Operation.wait"]["arguments"]["operation_id"], "op-1")
        self.assertEqual(by_method["Operation.wait"]["responses"], {"completed": True})
        self.assertIn("BaxterRemoteController.__init__", by_method)
        self.assertIn("BaxterRemoteController.close", by_method)
        for row in rows:
            self.assertTrue(row["timestamp_utc"].endswith("+00:00"))
            self.assertLessEqual(float(row["epoch_timestamp"]), float(row["completed_epoch_timestamp"]))

    def test_errors_timeouts_and_no_retries(self):
        with self.assertRaisesRegex(client.RemoteError, "rejected"):
            self.robot.call("fail", value=3)
        with self.assertRaises(ValueError):
            self.robot.get_wrist_camera_frame("invalid")
        operation = client.Operation(self.robot, "op-2")
        self.operation_status = "running"
        with self.assertRaises(TimeoutError):
            operation.wait(timeout_s=0)
        self.operation_status = "failed"
        with self.assertRaisesRegex(client.RemoteError, "test failure"):
            operation.wait()
        errors = [row["error"]["type"] for row in self.rows() if row["error"]]
        self.assertIn("RemoteError", errors)
        self.assertIn("ValueError", errors)
        self.assertIn("TimeoutError", errors)
        self.assertEqual(sum(request["method"] == "fail" for request in self.requests), 1)

    def test_camera_frames_deduplicate_and_rgb_is_a_png(self):
        self.assertEqual(self.robot.get_wrist_camera_frame("left"), PNG)
        self.robot.get_camera_frame("left_hand_camera")
        self.robot.show_screen_image_rgb(1, 1, b"\xff\x00\x00")
        rows = self.rows()
        camera = [row for row in rows if row["method"].endswith("camera_frame")]
        self.assertEqual(len(set(row["responses"] for row in camera)), 1)
        self.assertEqual((self.directory / camera[0]["responses"]).read_bytes(), PNG)
        screen = next(row for row in rows if row["method"].endswith(".show_screen_image_rgb"))
        image = (self.directory / screen["arguments"]["rgb_data"]).read_bytes()
        self.assertEqual(struct.unpack("!2I", image[16:24]), (1, 1))
        size = struct.unpack("!I", image[33:37])[0]
        self.assertEqual(zlib.decompress(image[41:41 + size]), b"\x00\xff\x00\x00")
        self.assertEqual(len(list((self.directory / "images").iterdir())), 2)
        self.assertNotIn(base64.b64encode(b"\xff\x00\x00").decode(),
                         (self.directory / "robot_interface_log.csv").read_text())

    def test_source_image_is_copied_even_if_display_fails(self):
        source = self.directory / "input.png"
        source.write_bytes(PNG)
        with patch.dict(sys.modules, {"PIL": None}):
            with self.assertRaises(ImportError):
                self.robot.show_screen_image(source)
        row = self.rows()[-1]
        image = row["arguments"]["image_file"]
        self.assertEqual(image["source"], str(source))
        self.assertEqual((self.directory / image["path"]).read_bytes(), PNG)

    def test_logging_failure_preserves_command_result_and_exception(self):
        with patch.object(self.robot._history, "append", side_effect=OSError("disk full")):
            with self.assertWarnsRegex(RuntimeWarning, "disk full"):
                operation = self.robot.jog_endpoint("left", "x", 0.002)
            self.assertEqual(operation.operation_id, "op-1")
            with self.assertWarns(RuntimeWarning), self.assertRaises(client.RemoteError):
                self.robot.call("fail")
        self.assertEqual(len(self.requests), 2)

    def test_threads_produce_complete_csv_rows(self):
        with ThreadPoolExecutor(max_workers=5) as pool:
            list(pool.map(lambda _: self.robot.get_state(), range(25)))
        rows = self.rows()
        self.assertEqual(len(rows), 51)
        self.assertEqual(len({row["call_id"] for row in rows}), len(rows))

    def test_busy_os_lock_does_not_block_heartbeats(self):
        with (self.directory / ".lock").open("r+b") as lock:
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            try:
                started = time.monotonic()
                with self.assertWarnsRegex(RuntimeWarning, "lock is busy"):
                    self.assertTrue(self.robot.call("heartbeat"))
                self.assertLess(time.monotonic() - started, 0.5)
            finally:
                if os.name == "nt":
                    lock.seek(0)
                    msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)
                else:
                    fcntl.flock(lock, fcntl.LOCK_UN)
        # The contended row was skipped, and the next append still works.
        self.assertEqual(len(self.rows()), 1)
        self.assertTrue(self.robot.call("heartbeat"))
        self.assertEqual(len(self.rows()), 2)
        self.assertEqual(len(self.requests), 2)

    def test_processes_share_one_header_and_complete_rows(self):
        script = """
import importlib.util, sys
spec = importlib.util.spec_from_file_location('client', sys.argv[1])
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)
robot = client.BaxterRemoteController('http://test', heartbeat=False, log_dir=sys.argv[2])
for _ in range(15):
    try:
        robot.get_wrist_camera_frame('invalid')
    except ValueError:
        pass
"""
        processes = [subprocess.Popen([sys.executable, "-B", "-c", script, str(SOURCE),
                                       str(self.directory)]) for _ in range(4)]
        for process in processes:
            self.assertEqual(process.wait(timeout=30), 0)
        self.assertEqual(len(self.rows()), 65)


if __name__ == "__main__":
    unittest.main()
