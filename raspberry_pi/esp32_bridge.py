#!/usr/bin/env python3
"""Pi-side Bluetooth controller for the final ESP32 sketch."""
import json
import os
import time

SERIAL_PORT = "/dev/rfcomm0"
BAUDRATE = 115200
TIMEOUT_S = 35.0
Z_CHUNK_STEPS = 2000


class ESP32Robot:
    def __init__(self, port=SERIAL_PORT, baudrate=BAUDRATE):
        self.mock = os.environ.get("MOCK_ESP32") == "1"
        self._next_id = 1
        if self.mock:
            print("[MOCK ESP32] movements succeed instantly")
            return
        import serial
        print(f"Opening ESP32 Bluetooth serial {port} @ {baudrate}...")
        self.ser = serial.Serial(port, baudrate, timeout=1.0)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        if self.ping():
            print("ESP32 link OK")
        else:
            print("WARNING: ESP32 did not respond to PING")

    def _send(self, d):
        line = (json.dumps(d) + "\n").encode("utf-8")
        self.ser.write(line)
        self.ser.flush()

    def _recv_until(self, deadline):
        """Read until a valid JSON line arrives. Ignore human debug lines from ESP32."""
        last_status = "timeout"
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            if not text.startswith("{"):
                print(f"ESP32 log: {text}")
                last_status = "debug_line"
                continue
            try:
                return json.loads(text), "ok"
            except Exception:
                last_status = f"bad_json:{text!r}"
                continue
        return None, last_status

    def _transact(self, d, expect="DONE", retries=3):
        if self.mock:
            time.sleep(0.1)
            return {"type": expect or "DONE"}
        cmd_id = self._next_id
        self._next_id += 1
        d = dict(d)
        d["id"] = cmd_id
        for attempt in range(1, retries + 1):
            self.ser.reset_input_buffer()
            self._send(d)
            resp, status = self._recv_until(time.time() + TIMEOUT_S)
            if resp is not None:
                if resp.get("type") == "ERROR":
                    print(f"ESP32 ERROR: {resp}")
                    return None
                if resp.get("id") not in (None, cmd_id):
                    status = f"stale id {resp.get('id')}"
                elif expect is None or resp.get("type") == expect:
                    return resp
                else:
                    status = f"wrong response {resp}"
            if attempt < retries:
                print(f"retry {attempt}/{retries}: {d.get('cmd')} failed ({status})")
                time.sleep(0.25)
        print(f"FAILED: {d.get('cmd')} after {retries} retries")
        return None

    def ping(self):
        return self._transact({"cmd": "PING"}, expect="PONG", retries=3) is not None

    def elevate(self, steps: int) -> bool:
        steps = int(steps)
        if steps == 0:
            return True
        sign = 1 if steps > 0 else -1
        remaining = abs(steps)
        while remaining > 0:
            chunk = min(Z_CHUNK_STEPS, remaining) * sign
            if self._transact({"cmd": "ELEVATE", "steps": chunk}, expect="DONE", retries=4) is None:
                return False
            remaining -= abs(chunk)
        return True

    def drive_cm_info(self, direction: str, cm: float):
        direction = "BACK" if str(direction).upper().startswith("B") else "FWD"
        return self._transact({"cmd": "DRIVE_CM", "dir": direction, "cm": float(cm)}, expect="DONE", retries=3)

    def drive_cm(self, direction: str, cm: float) -> bool:
        return self.drive_cm_info(direction, cm) is not None

    def stop(self) -> bool:
        if self.mock:
            return True
        self._send({"cmd": "STOP", "id": self._next_id})
        self._next_id += 1
        return True

    def close(self):
        if not self.mock and hasattr(self, "ser"):
            self.ser.close()
