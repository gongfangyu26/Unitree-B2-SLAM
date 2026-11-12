#!/usr/bin/env python3
"""Passive ROS2 recorder: never sends robot commands or stops the robot."""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=int, default=40)
    parser.add_argument("--log-dir", type=Path, default=Path.home() / "b2_terrain_logs")
    args = parser.parse_args()
    if not 20 <= args.seconds <= 60:
        parser.error("--seconds must be 20..60")

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from unitree_go.msg import LowState, SportModeState

    args.log_dir.mkdir(parents=True, exist_ok=True)
    name = datetime.now().strftime("b2_remote_visual_reference_%Y%m%d_%H%M%S_%f.jsonl")
    path = args.log_dir / name
    rclpy.init()
    node = rclpy.create_node(
        "b2_read_visual_reference", enable_rosout=False, start_parameter_services=False)
    started = time.monotonic()
    latest = {}
    arrivals = {}
    previous = None
    instructed = False
    warned_stale = False
    ending_notice = False
    interrupted = False
    failed = False

    with path.open("x", encoding="utf-8") as log:
        def record(event, data):
            row = {"host_time": datetime.now().astimezone().isoformat(),
                   "elapsed_sec": round(time.monotonic() - started, 6),
                   "event": event, **data}
            log.write(json.dumps(row, ensure_ascii=False) + "\n")

        def display():
            nonlocal previous
            if "sport" not in latest or "low" not in latest:
                return
            s, low = latest["sport"], latest["low"]
            key = (s["error_code"], s["mode"], s["gait_type"],
                   low["bit_flag"], tuple(low["leg_motor_reserve0"]))
            if key != previous:
                flags = {i: hex(v) if v is not None else None
                         for i, v in enumerate(low["leg_motor_reserve0"]) if v != 0}
                print(f"STATE t={time.monotonic()-started:.2f}s "
                      f"error={s['error_code']} mode={s['mode']} gait={s['gait_type']} "
                      f"bit_flag={hex(low['bit_flag'])} motor_flags={flags}", flush=True)
                previous = key

        def sport_callback(msg):
            data = {"error_code": int(msg.error_code), "mode": int(msg.mode),
                    "gait_type": int(msg.gait_type),
                    "rpy": [float(v) for v in msg.imu_state.rpy],
                    "velocity": [float(v) for v in msg.velocity],
                    "yaw_speed": float(msg.yaw_speed)}
            latest["sport"] = data
            arrivals["sport"] = time.monotonic()
            record("sport", data)
            display()

        def low_callback(msg):
            data = {"bit_flag": int(msg.bit_flag),
                    "leg_motor_reserve0": [int(m.reserve[0]) if len(m.reserve) else None
                                           for m in msg.motor_state[:12]]}
            latest["low"] = data
            arrivals["low"] = time.monotonic()
            record("low", data)
            display()

        record("metadata", {"revision": "read_visual_reference_v2", "read_only": True,
                            "duration_sec": args.seconds,
                            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
                            "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION", "")})
        print(f"READ ONLY [read_visual_reference_v2] — 不发送控制或停车指令。记录：{path}",
              flush=True)
        print("先保持机器人停止；5秒后提示是否开始遥控器对照。", flush=True)
        # Only subscriptions; no SportClient, request publishers or service calls.
        sport_sub = node.create_subscription(
            SportModeState, "/lf/sportmodestate", sport_callback, qos_profile_sensor_data)
        low_sub = node.create_subscription(
            LowState, "/lf/lowstate", low_callback, qos_profile_sensor_data)
        try:
            while rclpy.ok() and time.monotonic() - started < args.seconds:
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                elapsed = now - started
                fresh = all(k in arrivals and now-arrivals[k] < 1.0
                            for k in ("sport", "low"))
                if elapsed >= 5 and not instructed:
                    if not fresh:
                        print("基线数据缺失/过期：取消对照，请勿切模式。", flush=True)
                        record("baseline_missing", {})
                        failed = True
                        break
                    instructed = True
                    record("remote_entry_cue", {})
                    print("现在可按已验证的方法用遥控器进入视觉模式，不推方向摇杆；"
                          "观察5–10秒后人工退出并确认停稳，不必等采集结束。", flush=True)
                if instructed and not fresh and not warned_stale:
                    warned_stale = True
                    failed = True
                    record("data_stale", {})
                    print("状态数据中断：请立即用可靠人工手段退出并确认停稳。", flush=True)
                elif fresh and warned_stale:
                    warned_stale = False
                    record("data_resumed", {})
                if instructed and elapsed >= args.seconds-10 and not ending_notice:
                    ending_notice = True
                    print("采集剩余约10秒，请确认已人工退出视觉模式并停稳。", flush=True)
                log.flush()
        except (KeyboardInterrupt, ExternalShutdownException):
            interrupted = True
            record("capture_interrupted", {})
        except Exception as exc:
            failed = True
            record("capture_error", {"type": type(exc).__name__, "message": str(exc)})
            print(f"采集出错并已取消：{type(exc).__name__}: {exc}；"
                  "请勿继续对照切模式，若已进入模式请人工退出并确认停稳。", flush=True)
        finally:
            record("capture_end", {"interrupted": interrupted, "data_issue": failed})
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            print("采集结束；本程序不会停车，请现场确认机器人已停稳。", flush=True)
            print(f"记录：{path}", flush=True)
    return 1 if failed or interrupted else 0


if __name__ == "__main__":
    raise SystemExit(main())
