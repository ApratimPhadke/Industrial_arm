# Vector Robotics Arm — ESP32 + Laptop Control Package

One arm, four joints:

| Joint (default name) | Motor | Path from the laptop |
|---|---|---|
| `shoulder` | RS04 | laptop → RobStride USB-CAN adapter → RS04 (CAN ID 127) |
| `elbow` | RS02 | laptop → USB → ESP32 (`arm_bridge`) → CAN → RS02 (ID 1) |
| `wrist1` | RS02 | … same CAN chain … (ID 2) |
| `wrist2` | RS02 | … same CAN chain … (ID 3) |

Names and IDs are just defaults — change them in `laptop/arm_config.json`.
All angles are **rad** and speeds are **rad/s** at the motor output shaft.

---

## 0. What is in this folder

```
vector_arm/
├── README.md                     <- you are here
├── firmware/                     <- PlatformIO project (VS Code)
│   ├── platformio.ini            <- two environments: set_id and arm_bridge
│   └── src/
│       ├── set_id/main.cpp       <- STEP 1: give each RS02 its own CAN ID
│       └── arm_bridge/main.cpp   <- STEP 2: normal arm firmware (bridge + safety layer)
├── arduino/                      <- the same two programs as Arduino IDE sketches
│   ├── rs02_set_id/rs02_set_id.ino
│   └── arm_bridge/arm_bridge.ino
└── laptop/
    ├── arm_console.py            <- STEP 3: run this on the laptop
    ├── robstride.py              <- RobStride protocol + RS04 USB-CAN driver
    ├── esp32_link.py             <- serial link to the ESP32
    ├── arm_config.json           <- ports, joint names, CAN IDs, RS04 limits
    └── requirements.txt
```

Use **either** `firmware/` (PlatformIO) **or** `arduino/` (Arduino IDE). The code is identical.

---

## 1. Wiring

```
 LAPTOP
   ├── USB ── ESP32 dev board
   │            GPIO5 ──► TXD ┐
   │            GPIO4 ◄── RXD │  3.3 V CAN transceiver (SN65HVD230 or similar)
   │            3V3  ──── VCC │
   │            GND  ──── GND ┘
   │                          CAN_H ─┬── RS02 (ID 1) ── RS02 (ID 2) ── RS02 (ID 3) ─┬─ CAN_H
   │                          CAN_L ─┘   daisy chain, twisted pair, 1 Mbit/s       └─ CAN_L
   │                          [120 Ω across H/L here]               [120 Ω across H/L at the last motor]
   │
   └── USB ── RobStride USB-CAN adapter ── CAN_H/CAN_L ── RS04 (ID 127)   (120 Ω at both ends too)

 Motor power: 24–48 V to every motor. Supply GND, transceiver GND and ESP32 GND must be common.
```

Checklist:
- **Use a 3.3 V transceiver** (SN65HVD230, TJA1051T/3, MCP2562 with VIO=3.3 V). A 5 V-only TJA1050 can work but is out of spec on the ESP32 RX pin.
- **120 Ω termination at both ends** of each bus. Measure CAN_H↔CAN_L with power off: ~60 Ω is correct; 120 Ω means one terminator is missing; ~40 Ω means one too many.
- CAN_H to CAN_H, CAN_L to CAN_L — swapping them means no motor answers.
- The two buses (ESP32 chain and RS04 USB-CAN) are **separate**. Do not join them.

> The circuit-design files you mentioned did not come through with the code, so the pins above are the ones
> your firmware already used (TX = GPIO5, RX = GPIO4). If your board differs, change `CAN_TX_PIN` / `CAN_RX_PIN`
> at the top of both programs.

---

## 2. Install the tools (once)

**Firmware — PlatformIO (what you used for the STM32):**
1. VS Code → Extensions → install *PlatformIO IDE*.
2. *File → Open Folder…* → select `vector_arm/firmware`. PlatformIO downloads the ESP32 toolchain the first time (a few minutes).

**Firmware — Arduino IDE (alternative):**
1. *File → Preferences → Additional boards manager URLs*: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. *Tools → Board → Boards Manager* → install **esp32 by Espressif**.
3. Board: **ESP32 Dev Module**. Open `arduino/rs02_set_id/rs02_set_id.ino` or `arduino/arm_bridge/arm_bridge.ino`.

**USB drivers:** most ESP32 boards use a CP210x or CH340 USB chip. If no COM port appears when you plug the board in, install that driver.

**Laptop — Python 3.9 or newer:**
```bash
cd vector_arm/laptop
pip install -r requirements.txt        # installs pyserial
```
Linux only: add yourself to the serial group once, then log out/in: `sudo usermod -aG dialout $USER`

---

## 3. STEP 1 — Give each RS02 its own CAN ID

New RobStride motors all ship with **the same ID (usually 127)**. If two motors share an ID they both obey
every command — this is the most likely reason the old code behaved strangely with several motors connected.

**Flash the ID tool**
- PlatformIO: sidebar → `env:set_id` → **Upload**, then **Monitor**.
  (Command line: `pio run -e set_id -t upload && pio device monitor`)
- Arduino IDE: open `rs02_set_id.ino` → Upload → *Tools → Serial Monitor*, **115200 baud**, line ending **Newline**.

**Assign the IDs — one motor on the bus at a time**

1. Connect **only RS02 #1** (power + CAN). Press the ESP32 **EN/RST** button. You should see:
   ```
   Scanning IDs 1..127 ...
     ID 127  UID 3A0F00110B3C2E01
     1 ID(s) in use.
   ```
2. Type `assign 1` and press Enter:
   ```
   Changing ID 127 -> 1 (UID 3A0F00110B3C2E01) ...
   OK: motor now answers on ID 1. Power-cycle it and run 'scan' to confirm it stuck.
   ```
3. Switch the motor power off and on, type `scan` → it must now show `ID 1`.
4. **Put a label on the motor ("ID 1").** Disconnect it.
5. Repeat with RS02 #2 → `assign 2`, RS02 #3 → `assign 3`.
6. Connect all three in the daisy chain and type `scan` → you must see IDs 1, 2, 3 and **no** "MOTORS SHARE THIS ID" warning.

Other commands in the tool:

| Command | What it does |
|---|---|
| `scan` | list every motor (ID + unique hardware UID). Duplicate IDs are detected and flagged. |
| `assign <new>` | give the **only** connected motor the ID `<new>`; refuses if 0 or ≥2 motors are present |
| `set <old> <new>` | rename one motor while others are connected (refuses if `<old>` is duplicated or `<new>` is taken) |
| `wiggle <id>` | spins that motor ±1 rad/s for 0.7 s — find out which physical motor has which ID |
| `zero <id>` | store the current shaft angle as zero (saved to the motor's flash) |

Prefer the old "edit a number and re-flash" way? Set `#define AUTO_ASSIGN_ID 1` (2, 3…) at the top, flash, and the single connected motor gets that ID at boot — same as your STM32 `ASSIGN_NEW_ID`.

Want the RS04 to have a different ID too? Leave it at 127 — it sits on its own bus, so it cannot clash.

---

## 4. STEP 2 — Flash the arm firmware

- PlatformIO: `env:arm_bridge` → **Upload** (it is the default environment).
- Arduino IDE: open `arduino/arm_bridge/arm_bridge.ino` → Upload.

Before flashing, check the settings block at the top of `arm_bridge/main.cpp`:

| Setting | Default | Meaning |
|---|---|---|
| `EXPECTED_MOTORS` | 3 | warns if a different number of RS02 is found |
| `DEFAULT_CTRL_MODE` | `CTRL_SPEED` | `CTRL_SPEED` = RS02 internal speed loop (run_mode 2). `CTRL_MIT` = type-1 frames with kp=0 (what your last ESP32 sketch used). Can be switched at runtime with `CTRL`. |
| `CURRENT_LIMIT_A` | 3.0 | speed-mode current limit — raise once the arm is mechanically safe |
| `TORQUE_LIMIT_NM` | 10.0 | drive torque limit (RS02 peak is 17 Nm) |
| `ACCEL_RADS2` | 5.0 | speed-mode acceleration ramp |
| `MAX_TEMP_C`, `MAX_TORQUE_NM` | 60, 12 | safety trip points |
| `HOST_TIMEOUT_MS` | 500 | laptop heartbeat timeout (see Safety) |
| `MOTOR_CAN_TIMEOUT_MS` | 300 | the motor stops by itself if the ESP32 goes silent |
| `ESTOP_PIN` | -1 | optional e-stop input (see Safety) |
| `AUTO_ENABLE_ON_BOOT` | 0 | 1 = enable all RS02 at 0 rad/s right after power-up |

**Quick test without the laptop program** — open the serial monitor (115200, Newline):
```
READY arm_bridge-2.0 3           <- 3 RS02 found
PING                             -> OK PONG arm_bridge-2.0 3
TEL 0                            (silence the telemetry while typing)
LIST                             -> M 1 1 OFF ok ...  (index, CAN ID, state)
I 1 0.3                          -> motor 1 enables and turns at 0.3 rad/s
S                                -> stops, keeps holding
K                                -> disables all (freewheel)
TEL 100                          (telemetry back on)
```

---

## 5. STEP 3 — Run the arm from the laptop

```bash
cd vector_arm/laptop
python arm_console.py --list-ports                 # see what is connected
python arm_console.py                              # auto-detects both ports
python arm_console.py --esp-port COM5 --usbcan-port COM6        # Windows, explicit
python arm_console.py --esp-port /dev/ttyUSB0 --usbcan-port /dev/ttyACM0   # Linux
python arm_console.py --no-rs04                    # only the RS02 chain
python arm_console.py --no-esp                     # only the RS04
```

Tip: once you know the ports, write them into `arm_config.json` (`"esp32": {"port": "COM5"}`,
`"usbcan": {"port": "COM6"}`) and just run `python arm_console.py`. On Linux use the stable
`/dev/serial/by-id/...` names shown by `--list-ports` — `/dev/ttyUSB0/1` can swap between reboots.

At start the program checks every joint, prints a status table and asks
`Enable all motors at 0 rad/s now?` — answer `y` only when the arm is in a safe pose.

### A first session

```
arm> status                       table: position, speed, torque, temperature, faults of all 4 joints
arm> elbow 0.3                    elbow at +0.3 rad/s
arm> s                            everything stops and holds
arm> goto shoulder 0.5 0.4        shoulder to 0.5 rad at max 0.4 rad/s, then holds there
arm> id 1 0.4 id 2 -0.3 rs04 0.2  several joints in one line (your old syntax still works)
arm> limits elbow -1.2 1.4        soft limits: the joint slows down and stops at these angles
arm> pose save ready              remember the current pose
arm> home                         all joints to 0 rad
arm> pose go ready                back to the saved pose
arm> watch                        live updating table (Ctrl+C to leave)
arm> test wrist1                  short out-and-back move, prints PASS/FAIL
arm> e                            E-STOP: everything holds and is locked
arm> clear                        unlock after you fixed the cause
arm> quit                         asks, then disables all motors (support the arm first!)
```

### All console commands

| Command | Description |
|---|---|
| `<joint> <rad/s>` | velocity for one joint (`shoulder`, `rs04`, `elbow`, a CAN ID…) |
| `id <CAN_ID> <rad/s>` | same, by CAN ID; chains: `id 1 0.4 id 2 0.3 rs04 0.1` |
| `all <rad/s>` | every joint the same speed |
| `goto <joint> <rad> [rad/s]` | move to an angle and hold (default 0.5 rad/s) |
| `home [rad/s]` | every joint to 0 rad |
| `pose save/go/list/del <name>` | named poses, stored in `arm_state.json` |
| `stop` / `s` | all joints 0 rad/s, still enabled = holding |
| `estop` / `e` | hold + latch. Every motion command is refused until `clear` |
| `clear` | clear the latch, re-enable faulted joints at 0 rad/s |
| `kill [joint]` | disable (freewheel). Asks first — the arm will drop |
| `enable [joint]` | enable at 0 rad/s |
| `maxspeed [rad/s]` | session speed limit (starts at `start_speed_limit` = 2.0). Faster commands are **rejected**, not silently clamped |
| `limits <joint> <lo> <hi>` / `off` / `show` | soft position limits. RS02 limits live in the ESP32 flash, RS04 limits in `arm_state.json` |
| `status` / `st`, `watch [hz]` | joint table, live table |
| `log start [file.csv]`, `log stop` | record all joints at 20 Hz to CSV (open in Excel / plot later) |
| `test <joint> [rad/s] [s]` | out-and-back check with PASS/FAIL |
| `zero <joint>` | current angle becomes 0 (joint must be disabled — `kill <joint>` first, hold the link by hand) |
| `save <joint>` | write the motor's parameters to its flash (makes `zero` survive a power cycle) |
| `diag [joint]` | CAN error counters + reads VBUS, run mode, limits from the motor |
| `ctrl speed\|mit` | RS02 control mode (after `kill`) |
| `esp <text>` | send a raw command to the ESP32, e.g. `esp HELP` |
| `quiet` / `verbose` | hide / show the ESP32 `OK` lines |

### ESP32 serial protocol (for your own programs later)

Every command is one text line ending in `\n`. Several commands can share a line separated by `;`.

| Command | Reply |
|---|---|
| `I <id> <rad/s>` | `OK I <id> <applied>` — velocity by CAN ID (auto-enables) |
| `V <idx> <rad/s>` | velocity by scan index 1..N, `V 0 <v>` = all |
| `G <id> <rad> [vmax]` | `OK G …`, later `OK GOTO_DONE <id> <pos>` |
| `S` / `E` / `CLR` (`R`) / `F [reason]` | stop-hold / e-stop latch / clear / laptop-raised fault |
| `EN [id]` / `K [id]` | enable / disable |
| `Z <id>` / `SAVE <id>` | set zero / save to motor flash |
| `LIM <id> <lo> <hi>` / `LIM <id> OFF` / `LIM <id>` | soft limits (kept in ESP32 flash) |
| `RD <id> <hexidx>` / `WR <id> <hexidx> <float>` | parameter read (`OK RD …`) / write |
| `CTRL SPEED\|MIT`, `VMAX <v>`, `TEL <ms>`, `HB <ms>` | settings (`CTRL`, `VMAX` kept in flash) |
| `LIST`, `SCAN`, `DIAG [id]`, `PING`, `HELP`, `H` | info / heartbeat |

Asynchronous lines from the ESP32:
```
T <id> <en> <mode> <pos> <vel> <trq> <temp> <faultHex> <cmd> <fbAgeMs>     every 100 ms per motor
S <NORMAL|FAULT|ESTOP> <reason> <n> <ctrl> <vmax> <tec> <rec> <txdrop> <uptime>   every 500 ms
!FAULT ...   !EVT ...   !WARN ...
```

---

## 6. Safety model — what happens when something goes wrong

Faults **latch**: every joint is commanded to 0 rad/s (actively holding against gravity), and nothing moves until
you type `clear`. A fault on one bus is passed to the other, so the whole arm stops together.

| Event | Reaction |
|---|---|
| `estop` typed / ESP32 e-stop button | whole arm holds, latched |
| Motor reports a fault (over-current, over-temp, encoder, under-voltage…) | whole arm holds, latched, fault named in the message |
| Temperature > 60 °C or torque > limit | whole arm holds, latched |
| Joint goes past its soft limit by > 0.15 rad | whole arm holds, latched (normally the joint is slowed down and stopped *at* the limit) |
| No feedback from a motor for 250–300 ms (cable, power) | whole arm holds, latched |
| CAN bus-off on the RS02 chain | motors disabled by the bus failure, latched; `clear` re-enables after recovery |
| Laptop program crashes / USB to ESP32 unplugged | ESP32 notices the missing heartbeat after 0.5 s and holds the RS02s (not latched) |
| Laptop goes silent towards the RS04 | the RS04's own CAN watchdog disables it after 0.5 s |
| ESP32 crashes / loses power | each RS02's own CAN watchdog disables it after 0.3 s |
| `quit` | all joints stop, then are disabled after you confirm |

**Important:** "disabled" means no torque — a loaded joint **falls**. That applies to `kill`, `quit`, the motor
watchdogs, and a cut motor supply. A software stop is not a replacement for a hardware emergency stop:
put a **mushroom e-stop switch in the motor power line**. For a second, firmware-level button, wire a
normally-closed button between a free GPIO and GND, set `ESTOP_PIN` to that GPIO, and the ESP32 latches
an e-stop when the button is pressed **or the wire breaks**.

**Bring-up rules:** start with the arm free to move, low speeds (≤ 0.5 rad/s), low current limits; set
`limits` for each joint before trying `goto` or `pose go`; keep a hand near the power switch.

---

## 7. Speed mode vs MIT mode (RS02)

- **SPEED (default):** the motor runs its own PI speed loop with the acceleration ramp and current limit.
  `I 1 0` really holds the joint against gravity. Best for an arm.
- **MIT:** type-1 frames with kp = 0 and kd = 1, which your last ESP32 sketch used. The motor produces torque
  `kd × (target − actual speed)` — no integral term, so a loaded joint sags slowly even at "0 rad/s".

If a motor does not move in SPEED mode on your hardware, switch with `kill` → `ctrl mit` → `enable`
(the setting is saved in the ESP32). The RS04 equivalent is `"ctrl": "mit"` in `arm_config.json`.

---

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| `scan` finds nothing | motor power on? CAN_H/CAN_L swapped? termination (~60 Ω)? transceiver on GPIO5/GPIO4 with 3.3 V and GND? |
| `MOTORS SHARE THIS ID` | two motors still have the same ID — go back to STEP 1, one motor at a time |
| Laptop: "Could not identify the ports" | `python arm_console.py --list-ports`, then pass `--esp-port` / `--usbcan-port` |
| Laptop: "ESP32 … does not answer PING" | wrong port, `arm_bridge` not flashed (still `set_id`?), or Arduino Serial Monitor still open (only one program can use a port) |
| `'shoulder' not found` | RS04 power / USB-CAN wiring; the program scans all IDs and tells you which ID it found |
| `NO_FEEDBACK_ID…` fault | loose CAN connector on that motor, or its power dropped. Fix, then `clear` |
| `!WARN … error-passive` | wiring / termination problem — measure the 60 Ω |
| Joint moves the wrong way | expected — motor direction depends on mounting. Note it, and use the opposite sign |
| Joint sags at 0 rad/s | you are in MIT mode or the current limit is too low — use SPEED mode / raise `CURRENT_LIMIT_A` carefully |
| Commands typed in PuTTY run half-way | use the PlatformIO monitor or Arduino Serial Monitor (they send whole lines) |
| ESP32 resets when the laptop program opens the port | normal on some boards/OSes; the program waits for it to finish booting |

---

## 9. What changed compared with your previous code

**Bugs fixed**
- The last ESP32 sketch accepted *any* reply during the bus scan without checking which ID answered, and could not
  detect duplicate IDs — with several motors still on the same factory ID, one command moved all of them. Scans now check the
  replying ID and count distinct hardware UIDs.
- Old ESP32 versions used `Serial.parseInt()` / `delay()` in the command path, which can block the CAN loop
  for up to a second. Everything is now non-blocking; a stop is applied within one 20 ms tick.
- `single_arm_controller.py` re-triggered the global e-stop on *every* RS04 feedback frame while a fault was
  present (dozens of times per second). Faults now latch once and are cleared explicitly with `clear`.
- The older scripts' port auto-detection could pick the same physical port for both devices, or swap them. Now the ESP32 is
  identified by actually asking it (`PING`), and the same device is never opened twice.
- Velocity commands were only clamped to the motor's full ±44 rad/s range (`I 2 99` → 44 rad/s). Speeds are now
  validated and limited in three places: console session limit, ESP32 `VMAX`, and the motor's own limits.

**New functions**
- Laptop heartbeat + motor-side CAN watchdogs on both buses; cross-bus fault propagation; latched e-stop.
- Soft position limits that brake *before* the limit (stored in flash / `arm_state.json`).
- `goto`, `home`, named `pose` save/replay, `test` with PASS/FAIL, CSV `log`, live `watch` table.
- Parameter read/write and `diag` (VBUS, run mode, limits, CAN error counters).
- `zero` / `save` for the mechanical zero, `ctrl speed|mit` switch.
- One config file for ports, joint names and IDs instead of hard-coded IDs 2/8/9.
- ID tool: duplicate detection, `wiggle` to identify motors, safe one-motor `assign`.

---

## 10. Where the safety system goes next

The ESP32 already owns the safety state machine (`NORMAL / FAULT / ESTOP`) for the RS02 chain. Natural next steps:
1. Move the RS04 onto the ESP32 too (a second CAN transceiver or the same chain) so *all* safety checks run on the
   microcontroller and the laptop / Raspberry Pi only sends targets.
2. Wire a hardware e-stop input (`ESTOP_PIN`) and a relay that cuts motor power on a latched fault.
3. Add per-joint torque limits based on the arm's expected gravity load, and collision detection
   (measured torque vs expected torque).
4. Replace the laptop with the Raspberry Pi — `arm_console.py` runs unchanged on it.
