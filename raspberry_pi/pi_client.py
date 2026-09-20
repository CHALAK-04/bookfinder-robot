#!/usr/bin/env python3
"""
FINAL Raspberry Pi client for Book-Finder Robot.

The Pi keeps the camera open, talks to the laptop over TCP, and forwards motion
commands to the ESP32 over Bluetooth. For X movement, the ESP32 sketch always
uses the line follower while driving.

Run on Pi:
    python final_pi_client.py <laptop_ip>

Mock test:
    MOCK_CAMERA=1 MOCK_ESP32=1 python final_pi_client.py 127.0.0.1
"""
import json
import os
import socket
import sys
import time
from typing import Dict, List, Tuple

from protocol import MessageStream
from camera_capture_final import CameraCaptureFinal
from esp32_robot_final import ESP32Robot

PORT = 5000
MAX_Z_STEPS = 60000
Z_FILE = "z_positions.json"
TAG_RES_DEFAULT = (1280, 720)
BOOK_RES_DEFAULT = (1920, 1080)


def connect_to_laptop(ip: str) -> MessageStream:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to laptop {ip}:{PORT} ...")
    sock.connect((ip, PORT))
    print("Connected to laptop.")
    return MessageStream(sock)


def save_z_positions(z_positions: Dict[str, int]):
    with open(Z_FILE, "w", encoding="utf-8") as f:
        json.dump(z_positions, f, indent=2)
    print(f"Saved {Z_FILE}: {z_positions}")


def load_z_positions() -> Dict[str, int]:
    with open(Z_FILE, "r", encoding="utf-8") as f:
        return {str(k): int(v) for k, v in json.load(f).items()}


class RobotPiApp:
    def __init__(self, laptop_ip: str):
        self.stream = connect_to_laptop(laptop_ip)
        self.camera = CameraCaptureFinal(default_resolution=TAG_RES_DEFAULT, jpeg_quality=85)
        self.esp32 = ESP32Robot()
        self.current_z = 0
        self.current_x_abs = 0.0       # 0 = starting/right side. Positive = toward other/left side.
        self.row_length_cm = None      # learned when first row reaches the end tag.
        self.scan_seq = 0
        self.calib_seq = 0

    def close(self):
        try:
            self.esp32.stop()
        except Exception:
            pass
        self.camera.close()
        self.esp32.close()
        self.stream.close()

    def send_calib_photo(self, resolution=TAG_RES_DEFAULT):
        jpeg = self.camera.capture_jpeg(resolution=resolution)
        self.stream.send_message({"type": "CALIB_PHOTO", "seq": self.calib_seq, "z_steps": self.current_z}, binary_payload=jpeg)
        print(f"CALIB_PHOTO seq={self.calib_seq} z={self.current_z} size={len(jpeg)/1024:.1f}KB")
        self.calib_seq += 1

    def run(self):
        while True:
            msg, payload = self.stream.recv_message()
            if msg is None:
                print("Laptop disconnected.")
                return
            cmd = msg.get("cmd")
            print(f"PC CMD: {msg}")

            if cmd == "START_CALIB":
                self.calib_seq = 0
                self.current_z = 0
                res = tuple(msg.get("resolution", TAG_RES_DEFAULT))
                self.send_calib_photo(resolution=res)

            elif cmd == "CONTINUE":
                step_size = int(msg.get("step_size", 100))
                if self.current_z + step_size > MAX_Z_STEPS:
                    self.stream.send_message({"type": "CALIB_MAX_Z", "z_steps": self.current_z})
                    continue
                if not self.esp32.elevate(step_size):
                    self.stream.send_message({"type": "CALIB_ERROR", "reason": "esp32_elevate_failed"})
                    continue
                self.current_z += step_size
                self.send_calib_photo(resolution=tuple(msg.get("resolution", TAG_RES_DEFAULT)))

            elif cmd == "CALIB_DONE":
                save_z_positions({str(k): int(v) for k, v in msg.get("z_positions", {}).items()})
                # Do not go down. We are already near the top, so Phase 2 starts top-down.
                self.stream.send_message({"type": "STATUS", "state": "calib_done_ready_to_scan", "current_z": self.current_z})

            elif cmd == "CALIB_ERROR":
                print(f"Calibration error from PC: {msg}")
                return

            elif cmd == "START_SCAN":
                self.handle_start_scan(msg)

            elif cmd == "GO_TO_RESULT":
                self.handle_go_to_result(msg)

            elif cmd == "RETURN_HOME":
                self.goto_z(0)
                self.stream.send_message({"type": "AT_HOME", "current_z": self.current_z})

            elif cmd == "EXIT":
                print("Exiting by PC request.")
                return

            elif cmd == "STOP":
                self.esp32.stop()
                self.stream.send_message({"type": "STATUS", "state": "stopped"})

            else:
                self.stream.send_message({"type": "ERROR", "reason": f"unknown_cmd_{cmd}"})

    def goto_z(self, target_z: int) -> bool:
        target_z = max(0, min(MAX_Z_STEPS, int(target_z)))
        delta = target_z - self.current_z
        print(f"GOTO_Z target={target_z}, current={self.current_z}, delta={delta}")
        ok = self.esp32.elevate(delta)
        if ok:
            self.current_z = target_z
        return ok

    def send_scan_photo(self, row_tag: str, row_index: int, direction: str, resolution=BOOK_RES_DEFAULT, outward_dir="FWD"):
        time.sleep(self.settle_s)
        jpeg = self.camera.capture_jpeg(resolution=resolution)
        msg = {
            "type": "SCAN_PHOTO",
            "seq": self.scan_seq,
            "row_tag": str(row_tag),
            "row_index_top_down": int(row_index),
            "x_cm": round(float(self.current_x_abs), 2),
            "direction": direction,
            "outward_dir": outward_dir,
            "moving_outward": bool(direction == outward_dir),
            "z_steps": int(self.current_z),
        }
        self.stream.send_message(msg, binary_payload=jpeg)
        print(f"SCAN_PHOTO seq={self.scan_seq} row={row_tag} x_abs={self.current_x_abs:.1f} z={self.current_z} size={len(jpeg)/1024:.1f}KB")
        self.scan_seq += 1

    def wait_scan_decision(self) -> Tuple[str, float]:
        msg, _ = self.stream.recv_message()
        if msg is None:
            raise ConnectionError("Laptop disconnected during scan")
        cmd = msg.get("cmd")
        if cmd == "CONTINUE_SCAN":
            return "continue", float(msg.get("step_cm", self.scan_step_cm))
        if cmd == "ROW_END":
            return "row_end", 0.0
        if cmd == "STOP":
            self.esp32.stop()
            return "stop", 0.0
        return "continue", self.scan_step_cm

    def handle_start_scan(self, msg):
        z_positions = msg.get("z_positions") or load_z_positions()
        z_positions = {str(k): int(v) for k, v in z_positions.items()}
        rows_top_down = sorted(z_positions.items(), key=lambda kv: kv[1], reverse=True)
        self.scan_step_cm = float(msg.get("scan_step_cm", 12.0))
        self.fine_step_cm = float(msg.get("fine_step_cm", 3.0))
        self.settle_s = float(msg.get("settle_s", 2.0))
        book_res = tuple(msg.get("resolution", BOOK_RES_DEFAULT))
        direction = "BACK" if str(msg.get("start_dir", "FWD")).upper().startswith("B") else "FWD"
        outward_dir = direction  # physical motor direction that moves from right/start side toward left/end tag
        self.max_scan_cm = float(msg.get("max_scan_cm", 300.0))
        self.scan_seq = 0
        self.row_length_cm = None
        print(f"Starting top-down serpentine scan: rows={rows_top_down}, outward_dir={outward_dir}")

        # Move to first/top row. Calibration ended near the top, but go to exact saved Z.
        for row_index, (row_tag, row_z) in enumerate(rows_top_down):
            if not self.goto_z(row_z):
                self.stream.send_message({"type": "SCAN_ERROR", "reason": "goto_z_failed", "row_tag": row_tag})
                return

            # At row start, take a photo before moving. This protects books at the edge.
            self.send_scan_photo(row_tag, row_index, direction, resolution=book_res, outward_dir=outward_dir)

            while True:
                decision, step_cm = self.wait_scan_decision()
                if decision == "stop":
                    self.stream.send_message({"type": "SCAN_ERROR", "reason": "stopped_by_pc"})
                    return
                if decision == "row_end":
                    # First completed row teaches us total shelf length.
                    if self.row_length_cm is None and direction == outward_dir:
                        self.row_length_cm = max(0.0, self.current_x_abs)
                        print(f"Learned row_length_cm={self.row_length_cm:.1f}")
                    self.stream.send_message({"type": "SCAN_ROW_DONE", "row_tag": row_tag, "x_cm": round(self.current_x_abs, 2)})
                    break

                resp = self.esp32.drive_cm_info(direction, step_cm)
                if resp is None:
                    self.stream.send_message({"type": "SCAN_ERROR", "reason": "drive_failed", "row_tag": row_tag})
                    return
                actual_cm = float(resp.get("cm_done", step_cm))
                if actual_cm <= 0:
                    actual_cm = step_cm

                if direction == outward_dir:
                    self.current_x_abs += actual_cm
                else:
                    self.current_x_abs -= actual_cm
                    if self.current_x_abs < 0.75:
                        self.current_x_abs = 0.0

                # Safety clamp in case the end tag is missed and the PC safety also fails.
                if self.current_x_abs > self.max_scan_cm + self.scan_step_cm:
                    self.stream.send_message({"type": "SCAN_ERROR", "reason": "max_x_safety_exceeded", "row_tag": row_tag, "x_cm": round(self.current_x_abs, 2)})
                    return

                self.send_scan_photo(row_tag, row_index, direction, resolution=book_res, outward_dir=outward_dir)

            # Flip direction for serpentine; next row begins where this row ended.
            direction = "BACK" if direction == "FWD" else "FWD"

        self.stream.send_message({"type": "SCAN_DONE", "current_x_cm": round(self.current_x_abs, 2), "current_z": self.current_z})

    def handle_go_to_result(self, msg):
        try:
            z_positions = load_z_positions()
            row_tag = str(msg["row_tag"])
            target_x = float(msg["x_cm"])
            if row_tag not in z_positions:
                self.stream.send_message({"type": "NAV_ERROR", "reason": "unknown_row_tag", "row_tag": row_tag})
                return

            # Move Z first so the pointer/camera is on the correct shelf level.
            if not self.goto_z(int(z_positions[row_tag])):
                self.stream.send_message({"type": "NAV_ERROR", "reason": "goto_z_failed"})
                return

            delta_x = target_x - self.current_x_abs
            if abs(delta_x) > 0.5:
                direction = "FWD" if delta_x > 0 else "BACK"
                if self.esp32.drive_cm_info(direction, abs(delta_x)) is None:
                    self.stream.send_message({"type": "NAV_ERROR", "reason": "drive_x_failed"})
                    return
                self.current_x_abs = target_x

            self.stream.send_message({"type": "ARRIVED_RESULT", "row_tag": row_tag, "x_cm": round(self.current_x_abs, 2), "z_steps": self.current_z})
        except Exception as exc:
            self.stream.send_message({"type": "NAV_ERROR", "reason": str(exc)})


def main():
    if len(sys.argv) < 2:
        print("Usage: python final_pi_client.py <laptop_ip>")
        sys.exit(1)
    app = RobotPiApp(sys.argv[1])
    try:
        app.run()
    finally:
        app.close()


if __name__ == "__main__":
    main()
