"""Exercise real recorder callbacks with ROS-shaped NumPy data, without DDS."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

spec = importlib.util.spec_from_file_location(
    "visual_reference_under_test", Path(__file__).with_name("read_visual_reference.py"))
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class RecorderTest(unittest.TestCase):
    def run_recorder(self, fail=False):
        clock = [0.0]
        callbacks = []
        sport_type, low_type = type("SportModeState", (), {}), type("LowState", (), {})
        messages = {
            sport_type: SimpleNamespace(
                error_code=np.uint32(2), mode=np.uint8(3), gait_type=np.uint8(1),
                imu_state=SimpleNamespace(rpy=np.array([0.1, -0.2, 0.3], dtype=np.float32)),
                velocity=np.array([0.0, 0.0, 0.0], dtype=np.float32),
                yaw_speed=np.float32(0.0)),
            low_type: SimpleNamespace(
                bit_flag=np.uint8(4),
                motor_state=[SimpleNamespace(reserve=np.array([0, 0], dtype=np.uint32))
                             for _ in range(12)])}

        class Node:
            def create_subscription(self, kind, topic, callback, qos):
                callbacks.append((kind, callback))
                return object()

            def destroy_node(self):
                pass

        def spin_once(node, timeout_sec):
            if fail:
                raise RuntimeError("injected callback failure")
            clock[0] += timeout_sec
            for kind, callback in callbacks:
                callback(messages[kind])

        modules = {name: ModuleType(name) for name in (
            "rclpy", "rclpy.executors", "rclpy.qos", "unitree_go", "unitree_go.msg")}
        modules["rclpy"].init = lambda: None
        modules["rclpy"].shutdown = lambda: None
        modules["rclpy"].ok = lambda: True
        modules["rclpy"].create_node = lambda *a, **kw: Node()
        modules["rclpy"].spin_once = spin_once
        modules["rclpy.executors"].ExternalShutdownException = type(
            "ExternalShutdownException", (Exception,), {})
        modules["rclpy.qos"].qos_profile_sensor_data = object()
        modules["unitree_go.msg"].SportModeState = sport_type
        modules["unitree_go.msg"].LowState = low_type
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(sys.modules, modules), patch.object(
                    sys, "argv", ["recorder", "--seconds", "20", "--log-dir", directory]), \
                    patch.object(recorder.time, "monotonic", side_effect=lambda: clock[0]), \
                    contextlib.redirect_stdout(output):
                result = recorder.main()
            files = list(Path(directory).glob("*.jsonl"))
            self.assertEqual(len(files), 1)
            rows = [json.loads(line) for line in files[0].read_text().splitlines()]
        return result, rows, output.getvalue()

    def test_numpy_float32_callbacks_write_complete_capture(self):
        result, rows, output = self.run_recorder()
        self.assertEqual(result, 0)
        sport = [row for row in rows if row["event"] == "sport"]
        self.assertGreater(len(sport), 10)
        self.assertEqual(sport[0]["error_code"], 2)
        self.assertAlmostEqual(sport[0]["rpy"][0], 0.1, places=6)
        self.assertEqual(sport[0]["velocity"], [0.0, 0.0, 0.0])
        self.assertTrue(any(row["event"] == "remote_entry_cue" for row in rows))
        self.assertGreaterEqual(rows[-1]["elapsed_sec"], 20.0)
        self.assertEqual(rows[-1]["event"], "capture_end")
        self.assertFalse(rows[-1]["data_issue"])
        self.assertIn("read_visual_reference_v2", output)

    def test_callback_failure_is_recorded_as_incomplete(self):
        result, rows, output = self.run_recorder(fail=True)
        self.assertEqual(result, 1)
        self.assertTrue(any(row["event"] == "capture_error" for row in rows))
        self.assertTrue(rows[-1]["data_issue"])
        self.assertFalse(any(row["event"] == "remote_entry_cue" for row in rows))
        self.assertIn("采集出错并已取消", output)


if __name__ == "__main__":
    unittest.main()
