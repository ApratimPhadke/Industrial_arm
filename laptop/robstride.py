"""RobStride private CAN protocol + driver for the RobStride USB-CAN adapter (AT framing).

Used for the RS04 shoulder, but the Motor class works for any RobStride model on
that adapter (RS00..RS06) - only the scaling table differs.

Threads
  * reader  : parses AT frames from the adapter and updates motor state
  * control : every `period` s sends each enabled motor its setpoint (keep-alive),
              runs the goto controller and the safety checks
"""
from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional

import serial

HOST_ID = 0xFD
USBCAN_BAUD = 921600

# communication types
T_GET_ID, T_MIT, T_FEEDBACK, T_ENABLE, T_STOP = 0, 1, 2, 3, 4
T_SET_ZERO, T_SET_ID, T_PARAM_READ, T_PARAM_WRITE, T_SAVE, T_ACTIVE_REPORT = 6, 7, 17, 18, 22, 24

# parameter indices
P_RUN_MODE, P_SPD_REF, P_LIMIT_TORQUE = 0x7005, 0x700A, 0x700B
P_LIMIT_CUR, P_VBUS, P_ACC_RAD, P_CAN_TIMEOUT = 0x7018, 0x701C, 0x7022, 0x7028
PARAM_NAMES = {
    0x7005: "run_mode", 0x7006: "iq_ref", 0x700A: "spd_ref", 0x700B: "limit_torque",
    0x7016: "loc_ref", 0x7017: "limit_spd", 0x7018: "limit_cur", 0x7019: "mechPos",
    0x701A: "iqf", 0x701B: "mechVel", 0x701C: "VBUS", 0x7022: "acc_rad", 0x7028: "canTimeout",
}
U8_PARAMS = {0x7005}
U32_PARAMS = {0x7028}

FAULT_BITS = ["undervoltage", "overcurrent", "overtemp", "encoder", "stall/overload", "uncalibrated"]


@dataclass(frozen=True)
class Model:
    p: float   # +/- position range (rad)
    v: float   # +/- velocity range (rad/s)
    t: float   # +/- torque range (Nm)
    kp: float
    kd: float


MODELS = {
    "RS02": Model(12.566, 44.0, 17.0, 500.0, 5.0),
    "RS04": Model(12.566, 15.0, 120.0, 5000.0, 100.0),
}


def f2u(x: float, lo: float, hi: float) -> int:
    x = min(max(x, lo), hi)
    return int((x - lo) * 65535.0 / (hi - lo))


def u2f(x: int, lo: float, hi: float) -> float:
    return lo + (hi - lo) * x / 65535.0


def fault_text(bits: int) -> str:
    names = [n for i, n in enumerate(FAULT_BITS) if bits & (1 << i)]
    return ",".join(names) if names else "none"


@dataclass
class MotorConfig:
    model: str = "RS04"
    ctrl: str = "speed"                 # "speed" (run_mode 2) or "mit" (type-1 frames)
    current_limit_a: float = 6.0
    torque_limit_nm: float = 40.0
    accel_rads2: float = 3.0
    mit_kd: float = 4.0
    can_timeout_ms: int = 500           # motor disables itself if the laptop goes silent
    vel_limit: float = 2.0              # clamp on every command (rad/s)
    max_temp_c: float = 60.0
    max_torque_nm: float = 50.0         # measured |torque| above this -> fault
    fb_timeout_s: float = 0.30
    soft_limits: Optional[tuple] = None  # (lo, hi) rad or None
    softlim_overrun: float = 0.15
    goto_kp: float = 2.0
    goto_tol: float = 0.01
    goto_timeout_s: float = 25.0


@dataclass
class MotorState:
    pos: float = 0.0
    vel: float = 0.0
    trq: float = 0.0
    temp: float = 0.0
    fault: int = 0
    mode: int = 0
    last_fb: float = 0.0
    uid: int = 0


class UsbCanBus:
    """RobStride USB-CAN adapter in AT mode."""

    def __init__(self, port: str, period: float = 0.02):
        self.ser = serial.Serial(port, USBCAN_BAUD, timeout=0.01, write_timeout=0.2)
        self.port = port
        self.period = period
        self.wlock = threading.Lock()
        self.motors: Dict[int, "Motor"] = {}
        self.running = True
        self._id_replies: Dict[int, set] = {}
        self._param_waiters: Dict[tuple, list] = {}
        self.rx_frames = 0
        self.rx_errors = 0
        self.ser.write(b"AT+AT\r\n")
        self.ser.flush()
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self._reader = threading.Thread(target=self._read_loop, name="usbcan-rx", daemon=True)
        self._reader.start()
        self._ctrl = threading.Thread(target=self._control_loop, name="usbcan-ctrl", daemon=True)
        self._ctrl.start()

    # ---------------- framing ----------------
    def send(self, can_id: int, data: bytes = bytes(8)) -> None:
        data = bytes(data).ljust(8, b"\x00")[:8]
        frame = b"AT" + struct.pack(">I", ((can_id & 0x1FFFFFFF) << 3) | 0x04) + b"\x08" + data + b"\r\n"
        with self.wlock:
            self.ser.write(frame)

    def _read_loop(self) -> None:
        buf = bytearray()
        while self.running:
            try:
                chunk = self.ser.read(256)
            except (serial.SerialException, OSError):
                if self.running:
                    self.rx_errors += 1
                    time.sleep(0.1)
                continue
            if not chunk:
                continue
            buf += chunk
            while True:
                i = buf.find(b"AT")
                if i < 0:
                    del buf[:-1]          # keep a trailing 'A' in case 'T' arrives next
                    break
                if i:
                    del buf[:i]
                if len(buf) < 7:
                    break
                n = buf[6]
                if n > 8:
                    del buf[:2]
                    continue
                if len(buf) < 9 + n:
                    break
                if buf[7 + n:9 + n] != b"\r\n":
                    del buf[:2]
                    continue
                ext = struct.unpack(">I", bytes(buf[2:6]))[0] >> 3
                data = bytes(buf[7:7 + n])
                del buf[:9 + n]
                self.rx_frames += 1
                self._dispatch(ext, data)

    def _dispatch(self, ext: int, data: bytes) -> None:
        ctype = (ext >> 24) & 0x1F
        src = (ext >> 8) & 0xFF
        if ctype == T_GET_ID and (ext & 0xFF) == 0xFE and len(data) == 8:
            self._id_replies.setdefault(src, set()).add(struct.unpack("<Q", data)[0])
            return
        m = self.motors.get(src)
        if m is None:
            return
        if ctype in (T_FEEDBACK, T_ACTIVE_REPORT) and len(data) == 8:
            m._on_feedback(ext, data)
        elif ctype == T_PARAM_READ and len(data) == 8:
            idx = data[0] | (data[1] << 8)
            for w in self._param_waiters.pop((src, idx), []):
                w.append(None if (ext >> 16) & 0xFF else data[4:8])
                w.append(True)

    # ---------------- helpers ----------------
    def probe(self, motor_id: int, window: float = 0.03) -> set:
        """Return the set of UIDs answering on motor_id (empty = nobody, >1 = duplicate IDs)."""
        self._id_replies.pop(motor_id, None)
        self.send((T_GET_ID << 24) | (HOST_ID << 8) | motor_id)
        time.sleep(window)
        return self._id_replies.pop(motor_id, set())

    def scan(self, ids=range(1, 128)) -> Dict[int, set]:
        found = {}
        for mid in ids:
            uids = self.probe(mid, 0.015)
            if uids:
                found[mid] = uids
        return found

    def read_param(self, motor_id: int, idx: int, timeout: float = 0.2):
        """Returns raw 4 bytes or None (unsupported / no answer)."""
        waiter: list = []
        self._param_waiters.setdefault((motor_id, idx), []).append(waiter)
        self.send((T_PARAM_READ << 24) | (HOST_ID << 8) | motor_id, struct.pack("<HH", idx, 0))
        t0 = time.time()
        while time.time() - t0 < timeout:
            if waiter:
                return waiter[0]
            time.sleep(0.005)
        return None

    def add_motor(self, motor: "Motor") -> None:
        self.motors[motor.id] = motor

    def _control_loop(self) -> None:
        nxt = time.monotonic()
        while self.running:
            nxt += self.period
            for m in list(self.motors.values()):
                try:
                    m._tick()
                except (serial.SerialException, OSError) as exc:
                    m._raise(f"USBCAN_WRITE_ERROR {exc}")
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()

    def close(self) -> None:
        self.running = False
        time.sleep(self.period * 2)
        try:
            self.ser.close()
        except Exception:
            pass


class Motor:
    """One RobStride motor on a UsbCanBus, velocity-controlled with a safety layer."""

    def __init__(self, bus: UsbCanBus, motor_id: int, cfg: MotorConfig,
                 on_fault: Optional[Callable[[str], None]] = None, on_event: Optional[Callable[[str], None]] = None):
        self.bus, self.id, self.cfg = bus, motor_id, cfg
        self.mdl = MODELS[cfg.model]
        self.st = MotorState()
        self.enabled = False
        self.enabled_at = 0.0
        self.req = 0.0            # requested velocity
        self.cmd = 0.0            # velocity actually sent
        self.latched: Optional[str] = None
        self.goto_target: Optional[float] = None
        self.goto_vmax = 1.0
        self.goto_hits = 0
        self.goto_start = 0.0
        self._lock = threading.RLock()
        self._last_poll = 0.0
        self.on_fault = on_fault or (lambda r: None)
        self.on_event = on_event or (lambda e: None)
        bus.add_motor(self)

    # ---------------- low level ----------------
    def _cid(self, ctype: int) -> int:
        return (ctype << 24) | (HOST_ID << 8) | self.id

    def _write(self, idx: int, value) -> None:
        if idx in U8_PARAMS:
            payload = struct.pack("<B", int(value))
        elif idx in U32_PARAMS:
            payload = struct.pack("<I", int(value))
        else:
            payload = struct.pack("<f", float(value))
        self.bus.send(self._cid(T_PARAM_WRITE), (struct.pack("<HH", idx, 0) + payload).ljust(8, b"\0"))

    def _send_velocity(self, v: float) -> None:
        if self.cfg.ctrl == "mit":
            m = self.mdl
            p, vv = f2u(0, -m.p, m.p), f2u(v, -m.v, m.v)
            kp, kd, tff = f2u(0, 0, m.kp), f2u(self.cfg.mit_kd, 0, m.kd), f2u(0, -m.t, m.t)
            self.bus.send((T_MIT << 24) | (tff << 8) | self.id, struct.pack(">HHHH", p, vv, kp, kd))
        else:
            self._write(P_SPD_REF, v)

    def _on_feedback(self, ext: int, data: bytes) -> None:
        p, v, t, temp = struct.unpack(">HHHH", data)
        m = self.mdl
        s = self.st
        s.pos, s.vel, s.trq = u2f(p, -m.p, m.p), u2f(v, -m.v, m.v), u2f(t, -m.t, m.t)
        s.temp = temp / 10.0
        s.fault = (ext >> 16) & 0x3F
        s.mode = (ext >> 22) & 0x3
        s.last_fb = time.monotonic()

    # ---------------- lifecycle ----------------
    def enable(self) -> None:
        """Configure and enable at 0 rad/s (blocking ~0.25 s)."""
        c = self.cfg
        with self._lock:
            self.enabled = False
            self.bus.send(self._cid(T_STOP), b"\x01")                 # stop + clear faults
            time.sleep(0.03)
            self._write(P_RUN_MODE, 2 if c.ctrl == "speed" else 0)
            time.sleep(0.03)
            self.bus.send(self._cid(T_ENABLE))
            time.sleep(0.03)
            self._write(P_LIMIT_CUR, c.current_limit_a)
            self._write(P_LIMIT_TORQUE, c.torque_limit_nm)
            if c.ctrl == "speed":
                self._write(P_ACC_RAD, c.accel_rads2)
            self._write(P_CAN_TIMEOUT, int(c.can_timeout_ms * 20))
            self.req = self.cmd = 0.0
            self.goto_target = None
            self._send_velocity(0.0)
            self.st.fault = 0
            self.st.last_fb = time.monotonic()
            self.enabled_at = time.monotonic()
            self.enabled = True
        self.on_event(f"{self.cfg.model} id {self.id} enabled ({c.ctrl} mode)")

    def disable(self) -> None:
        with self._lock:
            self.enabled = False
            self.req = self.cmd = 0.0
            self.goto_target = None
            self.bus.send(self._cid(T_STOP))

    def hold(self) -> None:
        with self._lock:
            self.req = 0.0
            self.goto_target = None

    def clear(self) -> None:
        self.latched = None
        if not self.enabled or self.st.fault or self.st.mode != 2:
            self.enable()
        else:
            self.hold()

    # ---------------- motion ----------------
    def _check_allowed(self) -> None:
        if self.latched:
            raise RuntimeError(f"latched fault: {self.latched} (use 'clear')")

    def set_velocity(self, v: float) -> float:
        self._check_allowed()
        if not math.isfinite(v):
            raise ValueError("velocity must be finite")
        v = max(-self.cfg.vel_limit, min(self.cfg.vel_limit, v))
        if not self.enabled:
            self.enable()
        with self._lock:
            self.goto_target = None
            self.req = v
        return v

    def goto(self, pos: float, vmax: float = 1.0) -> None:
        self._check_allowed()
        if abs(pos) > self.mdl.p - 0.05:
            raise ValueError(f"target outside +/-{self.mdl.p - 0.05:.2f} rad")
        lim = self.cfg.soft_limits
        if lim and not lim[0] <= pos <= lim[1]:
            raise ValueError(f"target outside soft limits [{lim[0]:.3f}, {lim[1]:.3f}]")
        if not self.enabled:
            self.enable()
        with self._lock:
            self.goto_vmax = max(0.05, min(vmax, self.cfg.vel_limit))
            self.goto_target = pos
            self.goto_hits = 0
            self.goto_start = time.monotonic()
            self.req = 0.0

    def set_zero(self) -> None:
        if self.enabled:
            raise RuntimeError("disable the motor first (support the joint!)")
        self.bus.send(self._cid(T_SET_ZERO), b"\x01")

    def save_to_flash(self) -> None:
        if self.enabled:
            raise RuntimeError("disable the motor first")
        self._write(P_CAN_TIMEOUT, 0)          # don't persist the watchdog
        time.sleep(0.02)
        self.bus.send(self._cid(T_SAVE), bytes([1, 2, 3, 4, 5, 6, 7, 8]))

    def read_param(self, idx: int):
        raw = self.bus.read_param(self.id, idx)
        if raw is None:
            return None
        if idx in U8_PARAMS:
            return raw[0]
        if idx in U32_PARAMS:
            return struct.unpack("<I", raw)[0]
        return struct.unpack("<f", raw)[0]

    # ---------------- control tick (called by bus thread) ----------------
    def _raise(self, reason: str) -> None:
        with self._lock:
            self.req = 0.0
            self.goto_target = None
            first = self.latched is None
            self.latched = self.latched or reason
        if first:
            self.on_fault(reason)

    def _compute(self, now: float) -> float:
        if self.latched:
            return 0.0
        v = self.req
        if self.goto_target is not None:
            err = self.goto_target - self.st.pos
            if abs(err) < self.cfg.goto_tol:
                v = 0.0
                self.goto_hits += 1
                if self.goto_hits >= 3:
                    self.on_event(f"{self.cfg.model} id {self.id} reached {self.st.pos:.3f} rad")
                    self.goto_target = None
            else:
                self.goto_hits = 0
                v = max(-self.goto_vmax, min(self.goto_vmax, self.cfg.goto_kp * err))
            if self.goto_target is not None and now - self.goto_start > self.cfg.goto_timeout_s:
                self.goto_target = None
                self.on_event(f"{self.cfg.model} id {self.id} goto TIMEOUT at {self.st.pos:.3f} rad")
                v = 0.0
        v = max(-self.cfg.vel_limit, min(self.cfg.vel_limit, v))
        lim = self.cfg.soft_limits
        if lim:
            a = 0.8 * self.cfg.accel_rads2
            pos = self.st.pos
            if v > 0:
                v = min(v, 0.0 if pos >= lim[1] else math.sqrt(2 * a * (lim[1] - pos)))
            elif v < 0:
                v = max(v, 0.0 if pos <= lim[0] else -math.sqrt(2 * a * (pos - lim[0])))
        return v

    def _safety(self, now: float) -> None:
        if now - self.enabled_at < 0.3:
            return
        s, c = self.st, self.cfg
        tag = f"{c.model}_ID{self.id}"
        if now - s.last_fb > c.fb_timeout_s:
            self._raise(f"NO_FEEDBACK_{tag}")
        elif s.fault:
            self._raise(f"MOTOR_FAULT_{tag}_0x{s.fault:02X}({fault_text(s.fault)})")
        elif s.temp > c.max_temp_c:
            self._raise(f"OVER_TEMP_{tag}_{s.temp:.1f}C")
        elif abs(s.trq) > c.max_torque_nm:
            self._raise(f"OVER_TORQUE_{tag}_{s.trq:.1f}Nm")
        elif c.soft_limits and not (c.soft_limits[0] - c.softlim_overrun <= s.pos <= c.soft_limits[1] + c.softlim_overrun):
            self._raise(f"SOFT_LIMIT_{tag}_{s.pos:.2f}rad")

    def _tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            if self.enabled:
                self.cmd = self._compute(now)
                self._send_velocity(self.cmd)    # keep-alive; the motor answers with feedback
                self._safety(now)
            elif now - self._last_poll > 0.1:
                self._last_poll = now
                self.bus.send(self._cid(T_STOP))  # disabled: poll so telemetry stays live

    def state(self) -> dict:
        s = self.st
        age = time.monotonic() - s.last_fb if s.last_fb else float("inf")
        return {"pos": s.pos, "vel": s.vel, "trq": s.trq, "temp": s.temp, "fault": s.fault,
                "mode": s.mode, "enabled": self.enabled, "cmd": self.cmd, "age": age,
                "goto": self.goto_target}
