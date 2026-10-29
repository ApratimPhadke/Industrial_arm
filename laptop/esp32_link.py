"""Serial link to the ESP32 arm_bridge firmware (RS02 chain).

* opens the port without toggling DTR/RTS where the OS allows it (no ESP32 reset)
* parses telemetry lines   T <id> <en> <mode> <pos> <vel> <trq> <temp> <fault> <cmd> <age>
                           S <state> <reason> <n> <ctrl> <vmax> <tec> <rec> <txdrop> <uptime>
* sends 'H' heartbeats so the ESP32 holds the RS02s if this program dies
* forwards '!FAULT' lines to a callback so the laptop can stop the RS04 too
"""
from __future__ import annotations

import queue
import threading
import time
from typing import Callable, Dict, Optional

import serial

ESP32_BAUD = 115200


class Esp32Link:
    def __init__(self, port: str, baud: int = ESP32_BAUD,
                 on_fault: Optional[Callable[[str], None]] = None,
                 printer: Callable[[str], None] = print):
        self.port = port
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.05
        self.ser.write_timeout = 0.3
        self.ser.dtr = False          # avoid auto-reset on open (works on Windows, mostly on Linux/mac)
        self.ser.rts = False
        self.ser.open()
        self.wlock = threading.Lock()
        self.running = True
        self.on_fault = on_fault or (lambda r: None)
        self.printer = printer
        self.verbose = True           # print OK/ERR/info lines
        self.telemetry: Dict[int, dict] = {}
        self.system: dict = {"state": "UNKNOWN", "reason": "", "n": 0}
        self.found: Dict[int, str] = {}
        self.last_rx = 0.0
        self.fw = ""
        self._listeners: list = []
        threading.Thread(target=self._read_loop, name="esp32-rx", daemon=True).start()
        threading.Thread(target=self._heartbeat_loop, name="esp32-hb", daemon=True).start()

    # ---------------- io ----------------
    def send(self, line: str) -> None:
        data = (line.strip() + "\n").encode()
        with self.wlock:
            self.ser.write(data)

    def _heartbeat_loop(self) -> None:
        while self.running:
            try:
                self.send("H")
            except Exception:
                pass
            time.sleep(0.1)

    def _read_loop(self) -> None:
        while self.running:
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError) as exc:
                if self.running:
                    self.printer(f"[ESP32] serial error: {exc}")
                    self.on_fault("ESP32_SERIAL_LOST")
                    time.sleep(0.5)
                continue
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            self.last_rx = time.monotonic()
            self._handle(line)

    def _handle(self, line: str) -> None:
        parts = line.split()
        tag = parts[0]
        if tag == "T" and len(parts) >= 11:
            try:
                cid = int(parts[1])
                self.telemetry[cid] = {
                    "enabled": parts[2] == "1", "cfg": parts[2] == "2", "mode": int(parts[3]),
                    "pos": float(parts[4]), "vel": float(parts[5]), "trq": float(parts[6]),
                    "temp": float(parts[7]), "fault": int(parts[8], 16), "cmd": float(parts[9]),
                    "fb_age_ms": int(parts[10]), "t": time.monotonic(),
                }
            except ValueError:
                pass
            return
        if tag == "S" and len(parts) >= 6:
            self.system = {"state": parts[1], "reason": parts[2], "n": int(parts[3]),
                           "ctrl": parts[4], "vmax": float(parts[5]), "t": time.monotonic()}
            return
        if tag == "M" and len(parts) >= 4 and parts[2].isdigit():      # LIST reply line
            self.found.setdefault(int(parts[2]), parts[4] if len(parts) > 4 else "")
        if tag == "FOUND" and len(parts) >= 3:
            self.found[int(parts[1])] = parts[2] + (" DUPLICATE" if "DUPLICATE" in line else "")
        if tag in ("READY",) or (tag == "OK" and len(parts) > 1 and parts[1] == "PONG"):
            self.fw = parts[2] if tag == "OK" else parts[1]
        if tag == "!FAULT":
            self.system["state"] = parts[1] if len(parts) > 1 else "FAULT"
            self.on_fault(" ".join(parts[1:3]))
        for q in list(self._listeners):
            q.put(line)
        if self.verbose or tag.startswith("!") or tag == "ERR":
            self.printer(f"  ESP32 >> {line}")

    # ---------------- helpers ----------------
    def request(self, line: str, expect: str = "OK", timeout: float = 1.0) -> Optional[str]:
        """Send a command and wait for a reply line starting with `expect` (or ERR)."""
        q: "queue.Queue[str]" = queue.Queue()
        self._listeners.append(q)
        try:
            self.send(line)
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                try:
                    reply = q.get(timeout=max(0.0, end - time.monotonic()))
                except queue.Empty:
                    break
                if reply.startswith(expect) or reply.startswith("ERR"):
                    return reply
            return None
        finally:
            self._listeners.remove(q)

    def wait_ready(self, timeout: float = 8.0) -> bool:
        """Wait until the firmware answers PING (it may be rebooting + scanning after port open)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            reply = self.request("PING", "OK PONG", 0.6)
            if reply and reply.startswith("OK PONG"):
                return True
        return False

    def online(self) -> bool:
        return time.monotonic() - self.last_rx < 1.0

    def close(self) -> None:
        self.running = False
        time.sleep(0.15)
        try:
            self.ser.close()
        except Exception:
            pass
