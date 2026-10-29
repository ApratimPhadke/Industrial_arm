#!/usr/bin/env python3
"""Vector Robotics arm console.

RS04 shoulder  : laptop -> RobStride USB-CAN adapter -> RS04
3x RS02        : laptop -> USB serial -> ESP32 (arm_bridge firmware) -> CAN -> RS02 chain

Run:  python arm_console.py            (ports auto-detected, or set them in arm_config.json)
      python arm_console.py --list-ports
      python arm_console.py --esp-port COM5 --usbcan-port COM6
Type 'help' at the arm> prompt.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    sys.exit("pyserial is missing. Install it with:  pip install pyserial")

from esp32_link import Esp32Link
from robstride import MotorConfig, Motor, UsbCanBus, PARAM_NAMES, fault_text

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "arm_config.json"
STATE_FILE = HERE / "arm_state.json"      # poses + RS04 soft limits, written by the console

if os.name == "nt":
    os.system("")                         # enable ANSI escape codes in the Windows console


def out(msg: str) -> None:
    """Print without wrecking the input prompt too badly."""
    sys.stdout.write("\r" + msg + "\n")
    sys.stdout.flush()


# ============================================================================ config
def load_json(path: Path, default: dict) -> dict:
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        return default
    except json.JSONDecodeError as exc:
        sys.exit(f"{path} is not valid JSON: {exc}")


def save_json(path: Path, data: dict) -> None:
    tmp = path.with_suffix(".tmp")
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


# ============================================================================ ports
def describe_ports() -> None:
    ports = list(list_ports.comports())
    print("\nSerial ports:")
    if not ports:
        print("  none found - check USB cables / drivers (CP210x or CH340 for most ESP32 boards)")
    for p in ports:
        print(f"  {p.device:<22} {p.description}  [{p.hwid}]")
    if os.path.isdir("/dev/serial/by-id"):
        print("\nStable Linux names (/dev/serial/by-id):")
        for name in sorted(os.listdir("/dev/serial/by-id")):
            full = os.path.join("/dev/serial/by-id", name)
            print(f"  {full} -> {os.path.realpath(full)}")
    print()


def _port_text(p) -> str:
    return " ".join(str(x or "") for x in (p.device, p.description, p.manufacturer, p.product, p.hwid)).lower()


def _is_arm_bridge(device: str, timeout: float = 4.5) -> bool:
    """Open a port and see whether the arm_bridge firmware answers PING."""
    try:
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = device, 115200, 0.1
        s.dtr = s.rts = False
        s.open()
    except (serial.SerialException, OSError):
        return False
    try:
        end, nxt = time.monotonic() + timeout, 0.0
        while time.monotonic() < end:
            if time.monotonic() > nxt:
                s.write(b"PING\n")
                nxt = time.monotonic() + 0.5
            line = s.readline().decode(errors="ignore")
            if "arm_bridge" in line:
                return True
        return False
    finally:
        s.close()


def resolve_ports(esp: Optional[str], can: Optional[str], need_esp: bool, need_can: bool):
    ports = list(list_ports.comports())
    same = lambda a, b: a and b and os.path.realpath(a) == os.path.realpath(b)  # noqa: E731
    if need_esp and not esp:
        cands = [p for p in ports if not same(p.device, can)]
        hinted = [p for p in cands if any(k in _port_text(p) for k in ("cp210", "silicon labs", "ch910", "esp32", "espressif"))]
        order = hinted + [p for p in cands if p not in hinted]
        print("[ports] looking for the ESP32 (arm_bridge firmware)...")
        for p in order:
            if _is_arm_bridge(p.device):
                esp = p.device
                break
    if need_can and not can:
        rest = [p for p in ports if not same(p.device, esp)]
        hinted = [p for p in rest if any(k in _port_text(p) for k in ("robstride", "usb-can", "usb can", "usb2can"))]
        if len(hinted) == 1:
            can = hinted[0].device
        elif len(rest) == 1:
            can = rest[0].device
    if (need_esp and not esp) or (need_can and not can):
        describe_ports()
        raise SystemExit(
            "Could not identify the ports automatically. Pass them explicitly, e.g.\n"
            "  python arm_console.py --esp-port COM5 --usbcan-port COM6\n"
            "or write them into arm_config.json (\"esp32\": {\"port\": ...}, \"usbcan\": {\"port\": ...}).\n"
            "Use --no-rs04 / --no-esp to run with only one of the two buses.")
    if need_esp and need_can and same(esp, can):
        raise SystemExit("ESP32 and USB-CAN resolve to the same serial device - refusing.")
    return esp, can


# ============================================================================ joints
class Joint:
    """Common interface for a joint on either bus."""
    name: str
    bus: str
    can_id: int
    model: str

    def set_velocity(self, v: float) -> str: ...
    def goto(self, pos: float, vmax: float) -> str: ...
    def hold(self) -> None: ...
    def enable(self) -> None: ...
    def disable(self) -> None: ...
    def zero(self) -> str: ...
    def save(self) -> str: ...
    def state(self) -> Optional[dict]: ...
    def set_limits(self, lims) -> str: ...
    def get_limits(self): ...


class UsbCanJoint(Joint):
    def __init__(self, name: str, motor: Motor, arm: "Arm"):
        self.name, self.m, self.arm = name, motor, arm
        self.bus, self.can_id, self.model = "usbcan", motor.id, motor.cfg.model

    def set_velocity(self, v):
        applied = self.m.set_velocity(v)
        return f"{self.name} -> {applied:+.3f} rad/s"

    def goto(self, pos, vmax):
        self.m.goto(pos, vmax)
        return f"{self.name} moving to {pos:+.3f} rad at <= {vmax:.2f} rad/s"

    def hold(self):
        self.m.hold()

    def enable(self):
        self.m.enable()

    def disable(self):
        self.m.disable()

    def zero(self):
        self.m.set_zero()
        return f"{self.name}: zero set (in RAM). Use 'save {self.name}' to keep it after power-off."

    def save(self):
        self.m.save_to_flash()
        return f"{self.name}: parameters saved to motor flash."

    def state(self):
        return self.m.state()

    def set_limits(self, lims):
        self.m.cfg.soft_limits = tuple(lims) if lims else None
        self.arm.state_data.setdefault("limits", {})[self.name] = list(lims) if lims else None
        save_json(STATE_FILE, self.arm.state_data)
        return f"{self.name}: soft limits {'OFF' if not lims else f'[{lims[0]:+.3f}, {lims[1]:+.3f}] rad'} (saved)"

    def get_limits(self):
        return self.m.cfg.soft_limits

    def diag(self) -> List[str]:
        lines = []
        for idx in (0x701C, 0x7005, 0x7018, 0x700B, 0x7022, 0x7028):
            val = self.m.read_param(idx)
            lines.append(f"  {self.name} 0x{idx:04X} {PARAM_NAMES.get(idx, ''):<12} = {val if val is not None else 'no answer'}")
        bus = self.m.bus
        lines.append(f"  USB-CAN rx_frames={bus.rx_frames} rx_errors={bus.rx_errors} latched={self.m.latched}")
        return lines


class EspJoint(Joint):
    def __init__(self, name: str, can_id: int, link: Esp32Link):
        self.name, self.can_id, self.link = name, can_id, link
        self.bus, self.model = "esp32", "RS02"

    def _req(self, cmd: str, expect: str) -> str:
        reply = self.link.request(cmd, expect, 1.0)
        if reply is None:
            raise RuntimeError(f"ESP32 did not answer '{cmd}'")
        if reply.startswith("ERR"):
            raise RuntimeError(reply[4:])
        return reply

    def set_velocity(self, v):
        r = self._req(f"I {self.can_id} {v:.4f}", "OK I")
        return f"{self.name} -> {float(r.split()[-1]):+.3f} rad/s"

    def goto(self, pos, vmax):
        self._req(f"G {self.can_id} {pos:.4f} {vmax:.3f}", "OK G")
        return f"{self.name} moving to {pos:+.3f} rad at <= {vmax:.2f} rad/s"

    def hold(self):
        self.link.send(f"I {self.can_id} 0")

    def enable(self):
        self._req(f"EN {self.can_id}", "OK EN")

    def disable(self):
        self._req(f"K {self.can_id}", "OK K")

    def zero(self):
        return self._req(f"Z {self.can_id}", "OK Z")[3:]

    def save(self):
        return self._req(f"SAVE {self.can_id}", "OK SAVE")[3:]

    def state(self):
        t = self.link.telemetry.get(self.can_id)
        if not t:
            return None
        d = dict(t)
        d["age"] = (time.monotonic() - t["t"]) + t["fb_age_ms"] / 1000.0
        return d

    def set_limits(self, lims):
        cmd = f"LIM {self.can_id} OFF" if not lims else f"LIM {self.can_id} {lims[0]:.4f} {lims[1]:.4f}"
        return self._req(cmd, "OK LIM")[3:]

    def get_limits(self):
        r = self._req(f"LIM {self.can_id}", "OK LIM").split()
        return (float(r[4]), float(r[5])) if len(r) >= 6 and r[3] == "ON" else None


# ============================================================================ arm
class Arm:
    def __init__(self, cfg: dict, esp_port: Optional[str], can_port: Optional[str], rs04_scan: bool = True):
        self.cfg = cfg
        self.state_data = load_json(STATE_FILE, {"poses": {}, "limits": {}})
        self.state_data.setdefault("poses", {})
        self.state_data.setdefault("limits", {})
        self.joints: Dict[str, Joint] = {}
        self.link: Optional[Esp32Link] = None
        self.can_bus: Optional[UsbCanBus] = None
        self.speed_limit = float(cfg.get("start_speed_limit", 2.0))
        self.logger: Optional[threading.Thread] = None
        self.logging = False
        self._fault_lock = threading.Lock()
        self.fault: Optional[str] = None

        if esp_port:
            out(f"[ESP32]   opening {esp_port}")
            self.link = Esp32Link(esp_port, cfg.get("esp32", {}).get("baud", 115200),
                                  on_fault=self._esp_fault, printer=out)
            if not self.link.wait_ready():
                raise SystemExit(f"ESP32 on {esp_port} does not answer PING - is arm_bridge flashed? Right port?")
            self.link.verbose = False
            self.link.request("LIST", "OK LIST", 1.0)
            self.link.verbose = True
            out(f"[ESP32]   firmware {self.link.fw}, RS02 found: {sorted(self.link.found) or 'none'}")
        if can_port:
            out(f"[USB-CAN] opening {can_port}")
            self.can_bus = UsbCanBus(can_port, period=cfg.get("usbcan", {}).get("period_s", 0.02))

        for j in cfg["joints"]:
            name, cid, bus = j["name"], int(j["can_id"]), j.get("bus", "esp32")
            if bus == "esp32":
                if not self.link:
                    continue
                if cid not in self.link.found:
                    out(f"[WARN]    joint '{name}' (RS02 id {cid}) was not found on the ESP32 bus")
                self.joints[name.lower()] = EspJoint(name, cid, self.link)
            elif bus == "usbcan":
                if not self.can_bus:
                    continue
                cid = self._find_usbcan_motor(name, cid, rs04_scan)
                if cid is None:
                    continue
                mc = MotorConfig(**{k: v for k, v in j.get("motor", {}).items() if k in MotorConfig.__dataclass_fields__})
                mc.model = j.get("model", "RS04")
                lim = self.state_data["limits"].get(name)
                mc.soft_limits = tuple(lim) if lim else (tuple(j["soft_limits"]) if j.get("soft_limits") else None)
                motor = Motor(self.can_bus, cid, mc, on_fault=self._usbcan_fault,
                              on_event=lambda e: out(f"  [RS04] {e}"))
                self.joints[name.lower()] = UsbCanJoint(name, motor, self)
            else:
                out(f"[WARN]    joint '{name}': unknown bus '{bus}' (use 'esp32' or 'usbcan')")
        self.apply_speed_limit(self.speed_limit)

    def _find_usbcan_motor(self, name: str, cid: int, scan: bool) -> Optional[int]:
        uids = self.can_bus.probe(cid)
        if len(uids) == 1:
            out(f"[USB-CAN] '{name}' answers on CAN ID {cid} (UID {next(iter(uids)):016X})")
            return cid
        if len(uids) > 1:
            out(f"[ERROR]   {len(uids)} motors share CAN ID {cid} on the USB-CAN bus - '{name}' disabled")
            return None
        if not scan:
            out(f"[ERROR]   no motor answers on CAN ID {cid} (USB-CAN) - '{name}' disabled")
            return None
        out(f"[USB-CAN] nothing on ID {cid}, scanning 1..127 ...")
        found = self.can_bus.scan()
        if len(found) == 1:
            new = next(iter(found))
            out(f"[WARN]    using the only motor found, ID {new}. Put \"can_id\": {new} for '{name}' in arm_config.json")
            return new
        out(f"[ERROR]   '{name}' not found (IDs on bus: {sorted(found) or 'none'}). Check 48 V power and CAN_H/CAN_L.")
        return None

    # ---------------- faults: whatever trips, the whole arm holds ----------------
    def _esp_fault(self, reason: str) -> None:
        with self._fault_lock:
            self.fault = self.fault or f"ESP32:{reason}"
        out(f"\a[FAULT] {reason} -> RS04 held too. Fix the cause, then 'clear'.")
        for j in self.joints.values():
            if isinstance(j, UsbCanJoint):
                j.m._raise(f"ESP32_{reason.replace(' ', '_')}")

    def _usbcan_fault(self, reason: str) -> None:
        with self._fault_lock:
            self.fault = self.fault or reason
        if reason.startswith("ESP32_") or reason == "HOST_ESTOP":
            return                       # already reported / ESP32 already latched
        out(f"\a[FAULT] {reason} -> holding every joint. Fix the cause, then 'clear'.")
        if self.link:
            try:
                self.link.send(f"F {reason[:40]}")
            except Exception:
                pass

    def estop(self) -> None:
        if self.link:
            self.link.send("E")
        for j in self.joints.values():
            if isinstance(j, UsbCanJoint):
                j.m._raise("HOST_ESTOP")

    def clear(self) -> None:
        self.fault = None
        if self.link:
            self.link.request("CLR", "OK CLR", 1.0)
        for j in self.joints.values():
            if isinstance(j, UsbCanJoint):
                j.m.clear()

    def stop_all(self) -> None:
        if self.link:
            self.link.send("S")
        for j in self.joints.values():
            if isinstance(j, UsbCanJoint):
                j.hold()

    def enable_all(self) -> None:
        for j in self.joints.values():
            if isinstance(j, UsbCanJoint):
                j.enable()
        if self.link:
            self.link.request("EN", "OK EN", 1.0)

    def disable_all(self) -> None:
        if self.link:
            self.link.send("K")
        for j in self.joints.values():
            if isinstance(j, UsbCanJoint):
                j.disable()

    def apply_speed_limit(self, v: float) -> None:
        self.speed_limit = v
        for j in self.joints.values():
            if isinstance(j, UsbCanJoint):
                j.m.cfg.vel_limit = min(v, j.m.mdl.v)
        if self.link:
            self.link.request(f"VMAX {v:.3f}", "OK VMAX", 1.0)

    # ---------------- lookup ----------------
    def joint(self, token: str) -> Joint:
        t = token.lower()
        if t in self.joints:
            return self.joints[t]
        if t in ("rs04", "shoulder"):
            for j in self.joints.values():
                if j.bus == "usbcan":
                    return j
        if t.isdigit():
            for j in self.joints.values():
                if j.can_id == int(t):
                    return j
            if self.link and int(t) in self.link.found:      # RS02 not named in the config
                return EspJoint(f"id{t}", int(t), self.link)
        raise KeyError(f"unknown joint '{token}' (names: {', '.join(j.name for j in self.joints.values())})")

    # ---------------- logging ----------------
    def start_log(self, path: str, hz: float = 20.0) -> None:
        if self.logging:
            raise RuntimeError("already logging")
        self.logging = True
        joints = list(self.joints.values())

        def run():
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                hdr = ["t"]
                for j in joints:
                    hdr += [f"{j.name}_{k}" for k in ("pos", "vel", "trq", "temp", "fault", "cmd")]
                w.writerow(hdr)
                t0 = time.monotonic()
                while self.logging:
                    row = [f"{time.monotonic() - t0:.3f}"]
                    for j in joints:
                        s = j.state() or {}
                        row += [s.get(k, "") for k in ("pos", "vel", "trq", "temp", "fault", "cmd")]
                    w.writerow(row)
                    time.sleep(1.0 / hz)
        self.logger = threading.Thread(target=run, daemon=True)
        self.logger.start()

    def stop_log(self) -> None:
        self.logging = False
        if self.logger:
            self.logger.join(1.0)

    def close(self) -> None:
        self.stop_log()
        if self.can_bus:
            self.can_bus.close()
        if self.link:
            self.link.close()


# ============================================================================ console
HELP = """
 MOVE ------------------------------------------------------------------------------
   <joint> <rad/s>             velocity, e.g.  shoulder 0.3   elbow -0.5   rs04 0.2
   id <CAN_ID> <rad/s>         same, by CAN ID (old syntax). Chains work:
                               id 1 0.4 id 2 -0.3 rs04 0.2      or use ';' between commands
   all <rad/s>                 every joint at the same speed
   goto <joint> <rad> [rad/s]  move to an angle and hold      e.g. goto elbow 1.57 0.5
   home [rad/s]                every joint to 0 rad
   pose save|go|list|del <name>  store / replay arm poses (arm_state.json)
   stop  (or s)                all joints 0 rad/s, motors stay enabled (holding)
 SAFETY ----------------------------------------------------------------------------
   estop (or e)                hold everything and LATCH; nothing moves until 'clear'
   clear                       clear a latched fault, re-enable at 0 rad/s
   kill [joint]                disable = freewheel (asks first - the arm will drop)
   enable [joint]              enable at 0 rad/s
   maxspeed [rad/s]            session speed limit (rejects faster commands)
   limits <joint> <lo> <hi>    soft position limits in rad;  limits <joint> off|show
 INFO / SETUP ----------------------------------------------------------------------
   status (or st)              table of all joints        watch [hz]   live table (Ctrl+C)
   log start [file.csv] | log stop     record telemetry to CSV
   test <joint> [rad/s] [s]    out-and-back motion check with PASS/FAIL
   zero <joint>                current angle = 0 (joint must be disabled first)
   save <joint>                write motor parameters to its flash (e.g. after zero)
   diag [joint]                bus status + parameter readback (VBUS, mode, limits)
   ctrl speed|mit              RS02 control mode on the ESP32 (needs 'kill' first)
   esp <text>                  send a raw command to the ESP32 (e.g. esp HELP)
   quiet / verbose             hide / show ESP32 OK lines
   help, quit
"""


class Console:
    def __init__(self, arm: Arm):
        self.arm = arm
        self.running = True

    # ---------------- helpers ----------------
    def _speed(self, tok: str) -> float:
        v = float(tok)
        if not math.isfinite(v):
            raise ValueError("speed must be a finite number")
        if abs(v) > self.arm.speed_limit + 1e-9:
            raise ValueError(f"{v} rad/s exceeds the session limit {self.arm.speed_limit} (see 'maxspeed')")
        return v

    @staticmethod
    def _confirm(question: str) -> bool:
        try:
            return input(f"{question} [y/N] ").strip().lower() in ("y", "yes")
        except (EOFError, KeyboardInterrupt):
            return False

    def _status_lines(self) -> List[str]:
        hdr = f"{'joint':<12}{'bus':<7}{'id':>4} {'state':<6}{'pos rad':>9}{'vel':>8}{'trq Nm':>8}{'temp':>7} {'fault':<14}{'cmd':>7}{'age ms':>8}"
        lines = [hdr, "-" * len(hdr)]
        for j in self.arm.joints.values():
            s = j.state()
            if not s:
                lines.append(f"{j.name:<12}{j.bus:<7}{j.can_id:>4} {'--':<6}  (no telemetry)")
                continue
            en = "ON" if s.get("enabled") else ("CFG" if s.get("cfg") else "off")
            age = s.get("age", float("inf"))
            age_txt = f"{age * 1000:8.0f}" if math.isfinite(age) and age < 99 else "     ---"
            lines.append(f"{j.name:<12}{j.bus:<7}{j.can_id:>4} {en:<6}{s['pos']:>+9.3f}{s['vel']:>+8.2f}"
                         f"{s['trq']:>+8.2f}{s['temp']:>7.1f} {fault_text(s['fault'])[:13]:<14}{s['cmd']:>+7.2f}{age_txt}")
        sysline = "NORMAL" if not self.arm.fault else f"LATCHED: {self.arm.fault}"
        if self.arm.link:
            es = self.arm.link.system
            sysline += f" | ESP32 {es.get('state')} {es.get('ctrl', '')} vmax={es.get('vmax', '?')}" + \
                       ("" if self.arm.link.online() else " (OFFLINE!)")
        lines.append(f"system: {sysline} | session limit {self.arm.speed_limit} rad/s" +
                     (" | logging" if self.arm.logging else ""))
        return lines

    # ---------------- commands ----------------
    def cmd_status(self, args):
        for line in self._status_lines():
            out(line)

    def cmd_watch(self, args):
        hz = float(args[0]) if args else 5.0
        link = self.arm.link
        if link:
            link.verbose = False
        out("live view - Ctrl+C to return")
        try:
            first = True
            while True:
                lines = self._status_lines()
                if not first:
                    sys.stdout.write(f"\x1b[{len(lines)}F")
                sys.stdout.write("\n".join(l + "\x1b[K" for l in lines) + "\n")
                sys.stdout.flush()
                first = False
                time.sleep(1.0 / hz)
        except KeyboardInterrupt:
            out("")
        finally:
            if link:
                link.verbose = True

    def cmd_move_chain(self, toks):
        """Handles '<joint> <v>' and 'id <n> <v>' sequences, e.g. 'id 1 0.4 id 2 -0.3 rs04 0.2'."""
        i = 0
        while i < len(toks):
            if toks[i] == "id":
                if i + 2 >= len(toks):
                    raise ValueError("usage: id <CAN_ID> <rad/s>")
                j, v = self.arm.joint(toks[i + 1]), self._speed(toks[i + 2])
                i += 3
            else:
                if i + 1 >= len(toks):
                    raise ValueError(f"missing speed after '{toks[i]}'")
                j, v = self.arm.joint(toks[i]), self._speed(toks[i + 1])
                i += 2
            out("  " + j.set_velocity(v))

    def cmd_all(self, args):
        v = self._speed(args[0])
        for j in self.arm.joints.values():
            out("  " + j.set_velocity(v))

    def cmd_goto(self, args):
        if len(args) < 2:
            raise ValueError("usage: goto <joint> <rad> [rad/s]")
        j = self.arm.joint(args[0])
        vmax = self._speed(args[2]) if len(args) > 2 else min(0.5, self.arm.speed_limit)
        out("  " + j.goto(float(args[1]), abs(vmax)))

    def cmd_home(self, args):
        vmax = abs(self._speed(args[0])) if args else min(0.5, self.arm.speed_limit)
        for j in self.arm.joints.values():
            out("  " + j.goto(0.0, vmax))

    def cmd_pose(self, args):
        poses = self.arm.state_data["poses"]
        sub = args[0] if args else "list"
        if sub == "list":
            if not poses:
                out("  no poses saved yet - 'pose save <name>'")
            for n, p in poses.items():
                out(f"  {n:<14} " + "  ".join(f"{k}={v:+.3f}" for k, v in p.items()))
            return
        if len(args) < 2:
            raise ValueError("usage: pose save|go|del <name> [rad/s]")
        name = args[1]
        if sub == "save":
            pose = {}
            for j in self.arm.joints.values():
                s = j.state()
                if not s:
                    raise RuntimeError(f"no telemetry for {j.name}")
                pose[j.name] = round(s["pos"], 4)
            poses[name] = pose
            save_json(STATE_FILE, self.arm.state_data)
            out(f"  pose '{name}' saved: {pose}")
        elif sub == "go":
            if name not in poses:
                raise KeyError(f"no pose '{name}'")
            vmax = abs(self._speed(args[2])) if len(args) > 2 else min(0.5, self.arm.speed_limit)
            for jn, pos in poses[name].items():
                out("  " + self.arm.joint(jn).goto(pos, vmax))
        elif sub in ("del", "delete"):
            poses.pop(name, None)
            save_json(STATE_FILE, self.arm.state_data)
            out(f"  pose '{name}' deleted")
        else:
            raise ValueError("usage: pose save|go|list|del <name>")

    def cmd_stop(self, args):
        self.arm.stop_all()
        out("  all joints -> 0 rad/s (holding)")

    def cmd_estop(self, args):
        self.arm.estop()
        out("  E-STOP latched: every joint holding. 'clear' to resume.")

    def cmd_clear(self, args):
        self.arm.clear()
        out("  cleared - all joints enabled at 0 rad/s")

    def cmd_kill(self, args):
        targets = [self.arm.joint(args[0])] if args else list(self.arm.joints.values())
        names = ", ".join(j.name for j in targets)
        if not self._confirm(f"  Disable {names}? Unpowered joints FALL under gravity. Is the arm supported?"):
            out("  cancelled")
            return
        for j in targets:
            j.disable()
        out(f"  disabled (freewheel): {names}")

    def cmd_enable(self, args):
        if args:
            self.arm.joint(args[0]).enable()
        else:
            self.arm.enable_all()
        out("  enabled at 0 rad/s")

    def cmd_maxspeed(self, args):
        if args:
            v = float(args[0])
            if not 0 < v <= 15:
                raise ValueError("limit must be in (0, 15] rad/s")
            self.arm.apply_speed_limit(v)
        out(f"  session speed limit: {self.arm.speed_limit} rad/s")

    def cmd_limits(self, args):
        if not args:
            raise ValueError("usage: limits <joint> <lo> <hi> | off | show")
        j = self.arm.joint(args[0])
        if len(args) == 1 or args[1] == "show":
            lim = j.get_limits()
            out(f"  {j.name}: " + (f"[{lim[0]:+.3f}, {lim[1]:+.3f}] rad" if lim else "no soft limits"))
        elif args[1] == "off":
            out("  " + j.set_limits(None))
        else:
            lo, hi = float(args[1]), float(args[2])
            if lo >= hi:
                raise ValueError("lo must be < hi")
            out("  " + j.set_limits((lo, hi)))

    def cmd_log(self, args):
        if args and args[0] == "stop":
            self.arm.stop_log()
            out("  logging stopped")
            return
        path = args[1] if len(args) > 1 else time.strftime("arm_log_%Y%m%d_%H%M%S.csv")
        self.arm.start_log(path)
        out(f"  logging to {path} at 20 Hz ('log stop' to finish)")

    def cmd_test(self, args):
        if not args:
            raise ValueError("usage: test <joint> [rad/s] [seconds]")
        j = self.arm.joint(args[0])
        v = abs(self._speed(args[1])) if len(args) > 1 else min(0.3, self.arm.speed_limit)
        secs = float(args[2]) if len(args) > 2 else 1.0
        s0 = j.state()
        if not s0:
            raise RuntimeError("no telemetry for this joint")
        p0 = s0["pos"]
        out(f"  {j.name}: +{v} rad/s for {secs}s, back -{v} rad/s for {secs}s (Ctrl+C aborts)")
        try:
            j.set_velocity(v)
            time.sleep(secs)
            j.set_velocity(0)
            time.sleep(0.6)
            p1 = j.state()["pos"]
            j.set_velocity(-v)
            time.sleep(secs)
            j.set_velocity(0)
            time.sleep(0.6)
            p2 = j.state()["pos"]
        except KeyboardInterrupt:
            j.hold()
            out("  aborted - joint holding")
            return
        travel, ret = p1 - p0, p2 - p0
        expect = v * secs
        ok = travel > 0.4 * expect and abs(ret) < max(0.05, 0.15 * abs(travel))
        out(f"  out-travel {travel:+.3f} rad (ideal ~{expect:.3f}), return error {ret:+.3f} rad -> {'PASS' if ok else 'FAIL'}")
        if travel < 0:
            out("  note: the joint moved the opposite way - check wiring/direction expectations")

    def cmd_zero(self, args):
        j = self.arm.joint(args[0])
        out("  " + j.zero())

    def cmd_save(self, args):
        j = self.arm.joint(args[0])
        out("  " + j.save())

    def cmd_diag(self, args):
        targets = [self.arm.joint(args[0])] if args else list(self.arm.joints.values())
        if self.arm.link and any(j.bus == "esp32" for j in targets):
            ids = {j.can_id for j in targets if j.bus == "esp32"}
            self.arm.link.send("DIAG" if len(ids) != 1 else f"DIAG {ids.pop()}")
            time.sleep(0.4)
        for j in targets:
            if isinstance(j, UsbCanJoint):
                for line in j.diag():
                    out(line)

    def cmd_ctrl(self, args):
        if not self.arm.link:
            raise RuntimeError("no ESP32 connected")
        r = self.arm.link.request(f"CTRL {args[0] if args else ''}".strip(), "OK CTRL", 1.0)
        out(f"  {r}")

    def cmd_esp(self, args):
        if not self.arm.link:
            raise RuntimeError("no ESP32 connected")
        self.arm.link.send(" ".join(args))
        time.sleep(0.3)

    def cmd_quiet(self, args):
        if self.arm.link:
            self.arm.link.verbose = False
        out("  ESP32 OK/info lines hidden (faults and errors still shown)")

    def cmd_verbose(self, args):
        if self.arm.link:
            self.arm.link.verbose = True
        out("  ESP32 lines shown")

    def cmd_help(self, args):
        out(HELP)
        out("  joints: " + ", ".join(f"{j.name} ({j.model} id {j.can_id})" for j in self.arm.joints.values()))

    def cmd_quit(self, args):
        if self._confirm("  Quit? All motors will be DISABLED and the arm will go limp. Is it supported?"):
            self.running = False

    ALIASES = {"s": "stop", "e": "estop", "st": "status", "?": "help", "h": "help", "exit": "quit",
               "q": "quit", "reset": "clear", "clr": "clear", "lim": "limits"}

    def execute(self, line: str) -> None:
        for part in line.split(";"):
            toks = part.lower().split()
            if not toks:
                continue
            name = self.ALIASES.get(toks[0], toks[0])
            fn = getattr(self, f"cmd_{name}", None)
            try:
                if fn:
                    fn(toks[1:])
                else:
                    self.cmd_move_chain(toks)
            except (ValueError, KeyError, RuntimeError, IndexError) as exc:
                msg = exc.args[0] if exc.args else exc
                out(f"  error: {msg}")
            except serial.SerialException as exc:
                out(f"  serial error: {exc}")

    def run(self) -> None:
        self.cmd_help([])
        while self.running:
            try:
                line = input("arm> ")
            except (EOFError, KeyboardInterrupt):
                out("")
                break
            try:
                self.execute(line)
            except KeyboardInterrupt:
                self.arm.stop_all()
                out("  interrupted - all joints holding")


# ============================================================================ main
def default_config() -> dict:
    return {
        "esp32": {"port": None, "baud": 115200},
        "usbcan": {"port": None, "period_s": 0.02},
        "start_speed_limit": 2.0,
        "joints": [
            {"name": "shoulder", "bus": "usbcan", "model": "RS04", "can_id": 127,
             "motor": {"ctrl": "speed", "current_limit_a": 6.0, "torque_limit_nm": 40.0,
                       "accel_rads2": 3.0, "can_timeout_ms": 500, "max_temp_c": 60.0,
                       "max_torque_nm": 50.0}},
            {"name": "elbow", "bus": "esp32", "model": "RS02", "can_id": 1},
            {"name": "wrist1", "bus": "esp32", "model": "RS02", "can_id": 2},
            {"name": "wrist2", "bus": "esp32", "model": "RS02", "can_id": 3},
        ],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Vector Robotics arm console (RS04 via USB-CAN + RS02 chain via ESP32)")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG), help="joint/port config (JSON)")
    ap.add_argument("--esp-port", help="ESP32 serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--usbcan-port", help="RobStride USB-CAN serial port, e.g. COM6 or /dev/ttyUSB1")
    ap.add_argument("--no-esp", action="store_true", help="run without the ESP32 / RS02 chain")
    ap.add_argument("--no-rs04", action="store_true", help="run without the USB-CAN / RS04")
    ap.add_argument("--no-enable", action="store_true", help="do not offer to enable motors at start")
    ap.add_argument("--yes", action="store_true", help="enable motors at start without asking")
    ap.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    args = ap.parse_args()

    if args.list_ports:
        describe_ports()
        return 0

    cfg_path = Path(args.config)
    if not cfg_path.exists():
        save_json(cfg_path, default_config())
        print(f"[config] created {cfg_path} - edit joint names / CAN IDs there")
    cfg = load_json(cfg_path, default_config())
    buses = {j.get("bus", "esp32") for j in cfg.get("joints", [])}
    need_esp = "esp32" in buses and not args.no_esp
    need_can = "usbcan" in buses and not args.no_rs04
    esp, can = resolve_ports(args.esp_port or cfg.get("esp32", {}).get("port"),
                             args.usbcan_port or cfg.get("usbcan", {}).get("port"), need_esp, need_can)

    arm = None
    try:
        arm = Arm(cfg, esp if need_esp else None, can if need_can else None)
        if not arm.joints:
            out("No joints available - check wiring, power and arm_config.json.")
            return 2
        console = Console(arm)
        console.cmd_status([])
        if not args.no_enable and (args.yes or console._confirm(
                "\nEnable all motors at 0 rad/s now (they will hold position)? Arm in a safe pose?")):
            arm.enable_all()
            time.sleep(0.4)
            console.cmd_status([])
        console.run()
    except serial.SerialException as exc:
        print(f"\nSerial error: {exc}")
        return 2
    finally:
        if arm:
            out("[exit] all joints -> 0 rad/s")
            try:
                arm.stop_all()
                time.sleep(0.1)
                # Without the laptop, the RS04 watchdog will disable it in ~0.5 s anyway,
                # so disable everything deliberately rather than leave the arm half-powered.
                out("[exit] disabling motors - the arm will go limp")
                arm.disable_all()
                time.sleep(0.1)
            finally:
                arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
