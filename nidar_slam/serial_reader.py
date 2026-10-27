"""Robust newline-delimited JSON reader for ESP32 ultrasonic readings."""
import json
import threading
import time
import serial


class UltrasonicSerialReader:
    def __init__(self, port, baud_rate=115200, timeout_s=0.1, stale_s=0.5):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout_s = timeout_s
        self.stale_s = stale_s
        self._lock = threading.Lock()
        self._latest = {}
        self._running = False
        self._thread = None
        self._serial = None

    def start(self):
        self._serial = serial.Serial(self.port, self.baud_rate, timeout=self.timeout_s)
        time.sleep(1.5)  # ESP32 boards often reset when serial opens.
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self._running:
            try:
                line = self._serial.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                payload = json.loads(line)
                valid = payload.get("valid", {})
                now = time.monotonic()
                sample = {"received_monotonic": now, "seq": payload.get("seq")}
                for name in ("front", "left", "right", "rear"):
                    raw = payload.get(f"{name}_mm")
                    is_valid = bool(valid.get(name, False))
                    if isinstance(raw, (int, float)) and is_valid and 40 <= raw <= 4000:
                        sample[name] = float(raw) / 1000.0
                with self._lock:
                    self._latest = sample
            except (ValueError, TypeError, json.JSONDecodeError):
                continue
            except (serial.SerialException, OSError):
                time.sleep(0.1)

    def latest(self):
        with self._lock:
            sample = dict(self._latest)
        if not sample or time.monotonic() - sample.get("received_monotonic", 0) > self.stale_s:
            return {}
        return sample

    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
