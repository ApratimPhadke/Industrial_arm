// =============================================================================
//  arm_bridge  —  ESP32 <-> RS02 CAN chain bridge + safety layer   (v2.0)
// =============================================================================
//  ESP32 TWAI (CAN 1 Mbit/s) drives the RS02 daisy chain; the laptop talks to
//  it over USB serial with short text commands (see HELP below / README).
//
//  Design rules
//   * Never blocks in loop(): CAN RX, serial RX, control tick, telemetry and
//     safety checks all run every pass, so a stop command is acted on in <20 ms.
//   * Every enabled motor gets a fresh setpoint every CTRL_PERIOD_MS, and the
//     motor's own CAN watchdog (0x7028) is armed, so if the ESP32 dies the
//     motors stop by themselves.
//   * Safety faults LATCH: all motors are held at 0 rad/s and motion commands
//     are refused until the laptop sends CLR.
//
//  Works in PlatformIO (env:arm_bridge) and Arduino IDE (copy to
//  arm_bridge/arm_bridge.ino). Board: any classic ESP32 (esp32dev).
// =============================================================================
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include "driver/twai.h"

// ============================== SETTINGS =====================================
#define FW_VERSION          "arm_bridge-2.0"
#define CAN_TX_PIN          GPIO_NUM_5
#define CAN_RX_PIN          GPIO_NUM_4
#define HOST_ID             0xFD
#define SERIAL_BAUD         115200
#define EXPECTED_MOTORS     3        // RS02 count on this chain (warning only)
#define MAX_MOTORS          8
#define STATUS_LED_PIN      2        // on-board LED, -1 = none
#define ESTOP_PIN           -1       // optional E-stop input, -1 = none (see README)
#define ESTOP_ACTIVE_LEVEL  HIGH     // NC button to GND + pull-up -> HIGH when pressed
#define AUTO_ENABLE_ON_BOOT 0        // 1 = enable all motors at 0 rad/s after boot scan

// --- control ---
#define CTRL_PERIOD_MS      20       // setpoint refresh, 50 Hz
#define DEFAULT_CTRL_MODE   CTRL_SPEED   // CTRL_SPEED (run_mode 2) or CTRL_MIT (type-1 frames)
#define DEFAULT_VEL_LIMIT   4.0f     // rad/s clamp on every command (runtime: VMAX)
#define ACCEL_RADS2         5.0f     // speed-mode ramp (0x7022)
#define CURRENT_LIMIT_A     3.0f     // speed-mode current limit (0x7018) - raise once tuned
#define TORQUE_LIMIT_NM     10.0f    // drive torque limit (0x700B), RS02 peak is 17 Nm
#define MIT_KD              1.0f     // damping gain used for velocity tracking in MIT mode
#define MOTOR_CAN_TIMEOUT_MS 300     // motor disables itself if no frame for this long (0 = off)

// --- safety ---
#define FB_TIMEOUT_MS       250      // no feedback from an enabled motor -> fault
#define MAX_TEMP_C          60.0f
#define MAX_TORQUE_NM       12.0f    // measured |torque| above this -> fault
#define SOFTLIM_OVERRUN     0.15f    // rad past a soft limit -> fault
#define HOST_TIMEOUT_MS     500      // laptop heartbeat lost -> hold (armed by first 'H')
#define FAULT_GRACE_MS      300      // checks ignored right after (re)enable

// --- goto (position move done with the velocity loop) ---
#define GOTO_KP             2.5f     // (rad/s) per rad of error
#define GOTO_TOL            0.01f    // rad
#define GOTO_TIMEOUT_MS     20000

#define DEFAULT_TEL_MS      100      // telemetry period, 0 = off (runtime: TEL)
// =============================================================================

// RS02 feedback / MIT scaling (from the RS02 manual)
static const float P_RANGE = 12.566f, V_RANGE = 44.0f, T_RANGE = 17.0f;
static const float KP_MAX = 500.0f, KD_MAX = 5.0f;

enum : uint8_t {
  T_GET_ID = 0, T_MIT = 1, T_FEEDBACK = 2, T_ENABLE = 3, T_STOP = 4, T_SET_ZERO = 6,
  T_PARAM_READ = 17, T_PARAM_WRITE = 18, T_SAVE = 22, T_ACTIVE_REPORT = 24
};
enum : uint16_t {
  P_RUN_MODE = 0x7005, P_IQ_REF = 0x7006, P_SPD_REF = 0x700A, P_LIMIT_TORQUE = 0x700B,
  P_LOC_REF = 0x7016, P_LIMIT_SPD = 0x7017, P_LIMIT_CUR = 0x7018, P_MECH_POS = 0x7019,
  P_IQF = 0x701A, P_MECH_VEL = 0x701B, P_VBUS = 0x701C, P_ACC_RAD = 0x7022,
  P_CAN_TIMEOUT = 0x7028
};
enum CtrlMode : uint8_t { CTRL_SPEED = 0, CTRL_MIT = 1 };
enum SysState : uint8_t { SYS_NORMAL = 0, SYS_FAULT, SYS_ESTOP };
static const char *const kSysName[] = {"NORMAL", "FAULT", "ESTOP"};

struct SoftLimit { float lo, hi; uint8_t on; };

struct Motor {
  uint8_t  id;
  uint64_t uid;
  bool     dup;            // several motors answer on this ID -> never enabled
  bool     enabled;        // enable sequence finished
  uint8_t  cfgStep;        // 0 = idle, >0 = enable sequence in progress
  uint32_t enabledAt;
  float    req;            // requested velocity (rad/s)
  float    cmd;            // velocity actually sent this tick
  bool     gotoOn;
  float    gotoPos, gotoVmax;
  uint8_t  gotoHits;
  uint32_t gotoStart;
  float    pos, vel, trq, temp;
  uint8_t  fault, mode;
  uint32_t lastFb;
  bool     fbSeen, modeWarned;
  SoftLimit lim;
};

static Motor    mot[MAX_MOTORS];
static uint8_t  nmot = 0;
static uint8_t  ctrlMode = DEFAULT_CTRL_MODE;
static float    velLimit = DEFAULT_VEL_LIMIT;
static uint16_t telMs = DEFAULT_TEL_MS;
static uint16_t hostTimeoutMs = HOST_TIMEOUT_MS;
static SysState sys = SYS_NORMAL;
static char     faultReason[48] = "NONE";
static uint32_t lastHostMs = 0;
static bool     hbArmed = false, hostHeld = false;
static uint32_t txDrop = 0, rxCount = 0, busOffCount = 0;
static bool     busRecovering = false;
static Preferences prefs;

// ------------------------------ small helpers --------------------------------
static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline uint16_t f2u(float x, float lo, float hi) {
  return (uint16_t)((clampf(x, lo, hi) - lo) * 65535.0f / (hi - lo));
}
static inline float u2f(uint16_t x, float lo, float hi) { return lo + (hi - lo) * (float)x / 65535.0f; }
static inline uint32_t mkid(uint8_t type, uint8_t motor) {
  return ((uint32_t)type << 24) | ((uint32_t)HOST_ID << 8) | motor;
}
static int findMotor(int id) {
  for (uint8_t i = 0; i < nmot; i++) if (mot[i].id == id) return i;
  return -1;
}
static bool anyEnabled() {
  for (uint8_t i = 0; i < nmot; i++) if (mot[i].enabled || mot[i].cfgStep) return true;
  return false;
}
static const char *paramName(uint16_t idx) {
  switch (idx) {
    case P_RUN_MODE: return "run_mode";     case P_IQ_REF: return "iq_ref";
    case P_SPD_REF: return "spd_ref";       case P_LIMIT_TORQUE: return "limit_torque";
    case P_LOC_REF: return "loc_ref";       case P_LIMIT_SPD: return "limit_spd";
    case P_LIMIT_CUR: return "limit_cur";   case P_MECH_POS: return "mechPos";
    case P_IQF: return "iqf";               case P_MECH_VEL: return "mechVel";
    case P_VBUS: return "VBUS";             case P_ACC_RAD: return "acc_rad";
    case P_CAN_TIMEOUT: return "canTimeout";
    default: return "param";
  }
}

// --------------------------------- CAN TX ------------------------------------
static bool canSend(uint32_t id29, const uint8_t d[8]) {
  if (busRecovering) return false;
  twai_message_t m = {};
  m.extd = 1;
  m.identifier = id29 & 0x1FFFFFFF;
  m.data_length_code = 8;
  memcpy(m.data, d, 8);
  if (twai_transmit(&m, 0) != ESP_OK) { txDrop++; return false; }   // never block the loop
  return true;
}
static void sendType(uint8_t type, uint8_t id, uint8_t b0) {
  uint8_t d[8] = {b0, 0, 0, 0, 0, 0, 0, 0};
  canSend(mkid(type, id), d);
}
static void writeParam(uint8_t id, uint16_t idx, const void *v, uint8_t len) {
  uint8_t d[8] = {(uint8_t)idx, (uint8_t)(idx >> 8), 0, 0, 0, 0, 0, 0};
  memcpy(&d[4], v, len);
  canSend(mkid(T_PARAM_WRITE, id), d);
}
static void writeF(uint8_t id, uint16_t idx, float v)     { writeParam(id, idx, &v, 4); }
static void writeU8(uint8_t id, uint16_t idx, uint8_t v)  { writeParam(id, idx, &v, 1); }
static void writeU32(uint8_t id, uint16_t idx, uint32_t v){ writeParam(id, idx, &v, 4); }
static void readParam(uint8_t id, uint16_t idx) {
  uint8_t d[8] = {(uint8_t)idx, (uint8_t)(idx >> 8), 0, 0, 0, 0, 0, 0};
  canSend(mkid(T_PARAM_READ, id), d);
}
// Type-1 operation-control frame used as a velocity command (kp = 0).
static void sendMitVelocity(uint8_t id, float v) {
  uint16_t p = f2u(0, -P_RANGE, P_RANGE), vv = f2u(v, -V_RANGE, V_RANGE);
  uint16_t kp = f2u(0, 0, KP_MAX), kd = f2u(MIT_KD, 0, KD_MAX), tff = f2u(0, -T_RANGE, T_RANGE);
  uint8_t d[8] = {(uint8_t)(p >> 8), (uint8_t)p, (uint8_t)(vv >> 8), (uint8_t)vv,
                  (uint8_t)(kp >> 8), (uint8_t)kp, (uint8_t)(kd >> 8), (uint8_t)kd};
  canSend(((uint32_t)T_MIT << 24) | ((uint32_t)tff << 8) | id, d);
}
static void sendVelocity(Motor &m, float v) {
  if (ctrlMode == CTRL_MIT) sendMitVelocity(m.id, v);
  else writeF(m.id, P_SPD_REF, v);
}

// ------------------------------- persistence ---------------------------------
static void loadSettings() {
  prefs.begin("arm", true);
  ctrlMode = prefs.getUChar("ctrl", DEFAULT_CTRL_MODE);
  velLimit = prefs.getFloat("vmax", DEFAULT_VEL_LIMIT);
  prefs.end();
  if (ctrlMode > CTRL_MIT) ctrlMode = DEFAULT_CTRL_MODE;
  if (!(velLimit > 0 && velLimit <= V_RANGE)) velLimit = DEFAULT_VEL_LIMIT;
}
static void loadLimit(Motor &m) {
  char key[8];
  snprintf(key, sizeof key, "lim%u", m.id);
  prefs.begin("arm", true);
  SoftLimit l = {-P_RANGE, P_RANGE, 0};
  if (prefs.getBytesLength(key) == sizeof(SoftLimit)) prefs.getBytes(key, &l, sizeof l);
  prefs.end();
  m.lim = l;
}
static void saveLimit(const Motor &m) {
  char key[8];
  snprintf(key, sizeof key, "lim%u", m.id);
  prefs.begin("arm", false);
  prefs.putBytes(key, &m.lim, sizeof m.lim);
  prefs.end();
}
static void saveSettings() {
  prefs.begin("arm", false);
  prefs.putUChar("ctrl", ctrlMode);
  prefs.putFloat("vmax", velLimit);
  prefs.end();
}

// ------------------------------ state changes --------------------------------
static void cancelMotion(Motor &m) { m.req = 0; m.gotoOn = false; }

static void holdAll() { for (uint8_t i = 0; i < nmot; i++) cancelMotion(mot[i]); }

static void latch(SysState s, const char *reason) {
  holdAll();
  if (sys != SYS_NORMAL && s == SYS_FAULT) return;          // keep first reason
  sys = s;
  strncpy(faultReason, reason, sizeof faultReason - 1);
  faultReason[sizeof faultReason - 1] = 0;
  Serial.printf("!FAULT %s %s -> all RS02 holding 0 rad/s. Send CLR to resume.\n", kSysName[s], faultReason);
}

static void startEnable(Motor &m) {
  if (m.dup) { Serial.printf("ERR ID %u is duplicated on the bus, refusing to enable\n", m.id); return; }
  m.enabled = false;
  m.cfgStep = 1;
  m.cmd = 0;
}
static void disableMotor(Motor &m) {
  cancelMotion(m);
  m.enabled = false;
  m.cfgStep = 0;
  sendType(T_STOP, m.id, 0);
}

// Non-blocking enable: one frame per control tick (~180 ms total).
static void cfgTick(Motor &m, uint32_t now) {
  const bool spd = (ctrlMode == CTRL_SPEED);
  switch (m.cfgStep) {
    case 1: sendType(T_STOP, m.id, 1); break;                          // stop + clear faults
    case 2: writeU8(m.id, P_RUN_MODE, spd ? 2 : 0); break;            // speed / operation mode
    case 3: sendType(T_ENABLE, m.id, 0); break;
    case 4: writeF(m.id, P_LIMIT_CUR, CURRENT_LIMIT_A); break;
    case 5: writeF(m.id, P_LIMIT_TORQUE, TORQUE_LIMIT_NM); break;
    case 6: if (spd) writeF(m.id, P_ACC_RAD, ACCEL_RADS2); break;
    case 7: writeU32(m.id, P_CAN_TIMEOUT, (uint32_t)MOTOR_CAN_TIMEOUT_MS * 20u); break;
    default:
      sendVelocity(m, 0);
      m.cfgStep = 0;
      m.enabled = true;
      m.enabledAt = now;
      m.lastFb = now;
      m.fault = 0;
      m.modeWarned = false;
      Serial.printf("!EVT ENABLED %u %s\n", m.id, spd ? "SPEED" : "MIT");
      return;
  }
  m.cfgStep++;
}

// --------------------------------- CAN RX ------------------------------------
static void handleFrame(const twai_message_t &r, uint32_t now) {
  if (!r.extd || r.data_length_code != 8) return;
  const uint32_t i = r.identifier;
  const uint8_t type = (i >> 24) & 0x1F;
  const int k = findMotor((i >> 8) & 0xFF);
  if (k < 0) return;
  Motor &m = mot[k];
  rxCount++;
  if (type == T_FEEDBACK || type == T_ACTIVE_REPORT) {
    m.mode  = (i >> 22) & 0x03;
    m.fault = (i >> 16) & 0x3F;
    m.pos   = u2f((r.data[0] << 8) | r.data[1], -P_RANGE, P_RANGE);
    m.vel   = u2f((r.data[2] << 8) | r.data[3], -V_RANGE, V_RANGE);
    m.trq   = u2f((r.data[4] << 8) | r.data[5], -T_RANGE, T_RANGE);
    m.temp  = ((r.data[6] << 8) | r.data[7]) / 10.0f;
    m.lastFb = now;
    m.fbSeen = true;
  } else if (type == T_PARAM_READ) {
    uint16_t idx = r.data[0] | (r.data[1] << 8);
    uint32_t raw;
    float f;
    memcpy(&raw, &r.data[4], 4);
    memcpy(&f, &r.data[4], 4);
    if ((i >> 16) & 0xFF) Serial.printf("ERR RD %u 0x%04X not supported\n", m.id, idx);
    else Serial.printf("OK RD %u 0x%04X %s %.4f raw=0x%08lX u8=%u\n", m.id, idx, paramName(idx), f,
                       (unsigned long)raw, (unsigned)(raw & 0xFF));
  }
}
static void pollCan(uint32_t now) {
  twai_message_t r;
  while (twai_receive(&r, 0) == ESP_OK) handleFrame(r, now);
}

static void pollBusAlerts(uint32_t now) {
  uint32_t a = 0;
  if (twai_read_alerts(&a, 0) != ESP_OK || !a) return;
  if (a & TWAI_ALERT_BUS_OFF) {
    busOffCount++;
    busRecovering = true;
    for (uint8_t i = 0; i < nmot; i++) { mot[i].enabled = false; mot[i].cfgStep = 0; }
    latch(SYS_FAULT, "CAN_BUS_OFF");
    twai_initiate_recovery();
  }
  if (a & TWAI_ALERT_BUS_RECOVERED) {
    twai_start();
    busRecovering = false;
    Serial.println("!EVT CAN_BUS_RECOVERED (motors disabled, send CLR to re-enable)");
  }
  if (a & TWAI_ALERT_ERR_PASS) Serial.println("!WARN CAN error-passive (wiring / termination?)");
  if (a & TWAI_ALERT_RX_QUEUE_FULL) Serial.println("!WARN CAN RX queue overflow");
  (void)now;
}

// ------------------------------- bus scan ------------------------------------
// Returns number of distinct UIDs answering on <id>.
static uint8_t probe(uint8_t id, uint64_t *uidOut) {
  twai_message_t r;
  while (twai_receive(&r, 0) == ESP_OK) {}
  uint8_t d[8] = {0};
  twai_message_t m = {};
  m.extd = 1; m.identifier = mkid(T_GET_ID, id); m.data_length_code = 8; memcpy(m.data, d, 8);
  twai_transmit(&m, pdMS_TO_TICKS(10));
  uint64_t uids[4];
  uint8_t n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 12) {
    if (twai_receive(&r, pdMS_TO_TICKS(2)) != ESP_OK) continue;
    uint32_t i = r.identifier;
    if (!r.extd || ((i >> 24) & 0x1F) != T_GET_ID || (i & 0xFF) != 0xFE || ((i >> 8) & 0xFF) != id) continue;
    uint64_t u;
    memcpy(&u, r.data, 8);
    bool seen = false;
    for (uint8_t k = 0; k < n; k++) seen |= (uids[k] == u);
    if (!seen && n < 4) uids[n++] = u;
  }
  if (!n) {   // nobody ACKs when the bus is empty: flush the stuck frame
    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK && st.msgs_to_tx) { twai_stop(); twai_start(); }
  }
  if (n && uidOut) *uidOut = uids[0];
  return n;
}

static void scanBus() {
  Serial.println("# scanning RS02 chain (IDs 1..127)...");
  nmot = 0;
  for (uint8_t id = 1; id <= 127 && nmot < MAX_MOTORS; id++) {
    uint64_t uid = 0;
    uint8_t n = probe(id, &uid);
    if (!n) continue;
    Motor &m = mot[nmot++];
    memset(&m, 0, sizeof m);
    m.id = id; m.uid = uid; m.dup = n > 1;
    loadLimit(m);
    Serial.printf("FOUND %u %08lX%08lX%s\n", id, (unsigned long)(uid >> 32),
                  (unsigned long)(uid & 0xFFFFFFFF), m.dup ? " DUPLICATE" : "");
    if (m.dup) Serial.printf("!WARN %u motors share ID %u - fix with rs02_set_id\n", n, id);
  }
  Serial.printf("SCAN_DONE %u expected=%u\n", nmot, EXPECTED_MOTORS);
  if (nmot != EXPECTED_MOTORS)
    Serial.printf("!WARN found %u RS02, expected %u\n", nmot, EXPECTED_MOTORS);
}

// ------------------------------ control tick ---------------------------------
static float computeCommand(Motor &m, uint32_t now) {
  if (sys != SYS_NORMAL) return 0;
  float v = m.req;
  if (m.gotoOn) {
    float err = m.gotoPos - m.pos;
    if (fabsf(err) < GOTO_TOL) {
      if (++m.gotoHits >= 3) {
        m.gotoOn = false;
        m.req = 0;
        Serial.printf("OK GOTO_DONE %u %.3f\n", m.id, m.pos);
      }
      v = 0;
    } else {
      m.gotoHits = 0;
      v = clampf(GOTO_KP * err, -m.gotoVmax, m.gotoVmax);
    }
    if (m.gotoOn && now - m.gotoStart > GOTO_TIMEOUT_MS) {
      cancelMotion(m);
      Serial.printf("!EVT GOTO_TIMEOUT %u pos=%.3f\n", m.id, m.pos);
      v = 0;
    }
  }
  v = clampf(v, -velLimit, velLimit);
  if (m.lim.on && m.fbSeen) {
    // Slow down early enough to stop AT the limit (v^2 = 2*a*d), never drive further out.
    const float a = 0.8f * ACCEL_RADS2;
    if (v > 0) v = fminf(v, m.pos >= m.lim.hi ? 0 : sqrtf(2 * a * (m.lim.hi - m.pos)));
    if (v < 0) v = fmaxf(v, m.pos <= m.lim.lo ? 0 : -sqrtf(2 * a * (m.pos - m.lim.lo)));
  }
  return v;
}

static void safetyCheck(Motor &m, uint32_t now) {
  if (!m.enabled || now - m.enabledAt < FAULT_GRACE_MS) return;
  char why[48];
  if (now - m.lastFb > FB_TIMEOUT_MS) {
    snprintf(why, sizeof why, "NO_FEEDBACK_ID%u", m.id);
    latch(SYS_FAULT, why);
  } else if (m.fault) {
    snprintf(why, sizeof why, "MOTOR_FAULT_ID%u_0x%02X", m.id, m.fault);
    latch(SYS_FAULT, why);
  } else if (m.temp > MAX_TEMP_C) {
    snprintf(why, sizeof why, "OVER_TEMP_ID%u_%.1fC", m.id, m.temp);
    latch(SYS_FAULT, why);
  } else if (fabsf(m.trq) > MAX_TORQUE_NM) {
    snprintf(why, sizeof why, "OVER_TORQUE_ID%u_%.1fNm", m.id, m.trq);
    latch(SYS_FAULT, why);
  } else if (m.lim.on && (m.pos > m.lim.hi + SOFTLIM_OVERRUN || m.pos < m.lim.lo - SOFTLIM_OVERRUN)) {
    snprintf(why, sizeof why, "SOFT_LIMIT_ID%u_%.2frad", m.id, m.pos);
    latch(SYS_FAULT, why);
  }
  if (m.mode != 2 && !m.modeWarned) {
    m.modeWarned = true;
    Serial.printf("!WARN ID %u reports mode %u (not running) - motor watchdog or internal fault?\n", m.id, m.mode);
  } else if (m.mode == 2) m.modeWarned = false;
}

static void controlTick(uint32_t now) {
  static uint8_t slow = 0;
  slow = (slow + 1) % 5;
  for (uint8_t i = 0; i < nmot; i++) {
    Motor &m = mot[i];
    if (m.cfgStep) { cfgTick(m, now); continue; }
    if (m.enabled) {
      m.cmd = computeCommand(m, now);
      sendVelocity(m, m.cmd);                 // also the keep-alive; motor replies with feedback
      safetyCheck(m, now);
    } else if (slow == 0 && !busRecovering) {
      sendType(T_STOP, m.id, 0);                 // disabled: 10 Hz poll keeps telemetry alive
    }
  }

  // Host heartbeat watchdog (armed once the laptop has sent 'H').
  if (hbArmed && hostTimeoutMs && now - lastHostMs > hostTimeoutMs && !hostHeld) {
    bool moving = false;
    for (uint8_t i = 0; i < nmot; i++) moving |= (mot[i].req != 0 || mot[i].gotoOn);
    if (moving) {
      holdAll();
      Serial.println("!EVT HOST_TIMEOUT -> holding all RS02 at 0 rad/s");
    }
    hostHeld = true;
  }

#if ESTOP_PIN >= 0
  if (digitalRead(ESTOP_PIN) == ESTOP_ACTIVE_LEVEL && sys != SYS_ESTOP) latch(SYS_ESTOP, "ESTOP_BUTTON");
#endif
}

// -------------------------------- telemetry ----------------------------------
static void telemetry(uint32_t now) {
  static uint32_t last = 0;
  static uint8_t n = 0;
  if (!telMs || now - last < telMs) return;
  last = now;
  // T <id> <en> <mode> <pos> <vel> <trq> <temp> <fault> <cmd> <fb_age_ms>
  for (uint8_t i = 0; i < nmot; i++) {
    const Motor &m = mot[i];
    Serial.printf("T %u %u %u %.4f %.3f %.3f %.1f %02X %.3f %lu\n", m.id,
                  m.enabled ? 1 : (m.cfgStep ? 2 : 0), m.mode, m.pos, m.vel, m.trq, m.temp,
                  m.fault, m.cmd, m.fbSeen ? (unsigned long)(now - m.lastFb) : 99999UL);
  }
  if (++n >= 5) {   // S <state> <reason> <nmot> <ctrl> <vmax> <tec> <rec> <txdrop> <uptime_ms>
    n = 0;
    twai_status_info_t st = {};
    twai_get_status_info(&st);
    Serial.printf("S %s %s %u %s %.2f %lu %lu %lu %lu\n", kSysName[sys], faultReason, nmot,
                  ctrlMode == CTRL_MIT ? "MIT" : "SPEED", velLimit, (unsigned long)st.tx_error_counter,
                  (unsigned long)st.rx_error_counter, (unsigned long)txDrop, (unsigned long)now);
  }
}

static void statusLed(uint32_t now) {
#if STATUS_LED_PIN >= 0
  bool on;
  if (sys != SYS_NORMAL) on = (now / 125) & 1;          // fast blink = fault
  else if (anyEnabled()) on = true;                      // solid = motors enabled
  else on = ((now / 1000) & 1) && (now % 1000) < 80;     // short blip = idle
  digitalWrite(STATUS_LED_PIN, on);
#else
  (void)now;
#endif
}

// ------------------------------ command parser -------------------------------
static void printHelp() {
  Serial.println("# ---------------- arm_bridge commands ----------------");
  Serial.println("#  I <id> <rad/s>      velocity for one RS02 by CAN ID (auto-enables)");
  Serial.println("#  V <idx> <rad/s>     velocity by scan index 1..N, 0 = all");
  Serial.println("#  G <id> <rad> [vmax] move to position, then hold");
  Serial.println("#  S                   all targets 0 (hold, stay enabled)");
  Serial.println("#  EN [id]             enable at 0 rad/s (all if no id)");
  Serial.println("#  K [id]              disable / freewheel (all if no id) - ARM WILL DROP");
  Serial.println("#  E                   e-stop: hold + latch      CLR (or R)  clear + re-enable");
  Serial.println("#  F [reason]          laptop-raised fault (hold + latch)");
  Serial.println("#  Z <id>              set current angle as zero (motor must be disabled)");
  Serial.println("#  SAVE <id>           write motor RAM params to its flash");
  Serial.println("#  LIM <id> <lo> <hi>  soft limits in rad (stored)   LIM <id> OFF");
  Serial.println("#  RD <id> <idx>       read param (hex idx, e.g. 701C = VBUS)");
  Serial.println("#  WR <id> <idx> <f>   write float param");
  Serial.println("#  CTRL SPEED|MIT      control mode (applies on next EN)");
  Serial.println("#  VMAX <rad/s>        velocity clamp     TEL <ms>  telemetry period");
  Serial.println("#  HB <ms>             host timeout (0=off)  H  heartbeat");
  Serial.println("#  LIST  SCAN  DIAG [id]  PING  HELP");
}

static Motor *motorArg(const char *tok) {
  if (!tok) { Serial.println("ERR missing motor id"); return nullptr; }
  int k = findMotor(atoi(tok));
  if (k < 0) { Serial.printf("ERR CAN ID %s not on the RS02 bus (try LIST)\n", tok); return nullptr; }
  return &mot[k];
}
static bool parseFloat(const char *tok, float &out) {
  if (!tok) return false;
  char *end;
  out = strtof(tok, &end);
  return end != tok && isfinite(out);
}
static bool motionAllowed() {
  if (sys == SYS_NORMAL) return true;
  Serial.printf("ERR LATCHED %s %s - send CLR first\n", kSysName[sys], faultReason);
  return false;
}
static void setVelocity(Motor &m, float v) {
  if (fabsf(v) > velLimit) Serial.printf("!WARN %.2f clamped to +/-%.2f rad/s\n", v, velLimit);
  m.gotoOn = false;
  m.req = clampf(v, -velLimit, velLimit);
  if (!m.enabled && !m.cfgStep) startEnable(m);
}

static void execLine(char *line) {
  for (char *p = line; *p; p++) *p = toupper((unsigned char)*p);
  char *save = nullptr;
  char *cmd = strtok_r(line, " \t,", &save);
  if (!cmd) return;
  char *a1 = strtok_r(nullptr, " \t,", &save);
  char *a2 = strtok_r(nullptr, " \t,", &save);
  char *a3 = strtok_r(nullptr, " \t,", &save);
  float f = 0, g = 0;

  if (!strcmp(cmd, "H")) return;                                   // heartbeat only
  if (!strcmp(cmd, "PING")) { Serial.printf("OK PONG %s %u\n", FW_VERSION, nmot); return; }
  if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) { printHelp(); return; }

  if (!strcmp(cmd, "I") || !strcmp(cmd, "V")) {
    if (!parseFloat(a2, f)) { Serial.println("ERR usage: I <id> <rad/s>  |  V <idx> <rad/s>"); return; }
    if (!motionAllowed()) return;
    if (cmd[0] == 'V') {
      int idx = a1 ? atoi(a1) : -1;
      if (idx == 0) { for (uint8_t i = 0; i < nmot; i++) setVelocity(mot[i], f); }
      else if (idx >= 1 && idx <= nmot) setVelocity(mot[idx - 1], f);
      else { Serial.println("ERR bad index"); return; }
      Serial.printf("OK V %d %.3f\n", idx, f);
    } else {
      Motor *m = motorArg(a1);
      if (!m) return;
      setVelocity(*m, f);
      Serial.printf("OK I %u %.3f\n", m->id, m->req);
    }
  } else if (!strcmp(cmd, "G")) {
    Motor *m = motorArg(a1);
    if (!m) return;
    if (!parseFloat(a2, f)) { Serial.println("ERR usage: G <id> <rad> [vmax]"); return; }
    if (!motionAllowed()) return;
    if (!m->fbSeen) { Serial.println("ERR no position feedback yet"); return; }
    if (fabsf(f) > P_RANGE - 0.05f) { Serial.println("ERR target outside +/-12.5 rad"); return; }
    if (m->lim.on && (f < m->lim.lo || f > m->lim.hi)) {
      Serial.printf("ERR target outside soft limits [%.3f, %.3f]\n", m->lim.lo, m->lim.hi);
      return;
    }
    if (!parseFloat(a3, g) || g <= 0) g = 1.0f;
    m->gotoPos = f;
    m->gotoVmax = clampf(g, 0.05f, velLimit);
    m->gotoHits = 0;
    m->gotoStart = millis();
    m->gotoOn = true;
    m->req = 0;
    if (!m->enabled && !m->cfgStep) startEnable(*m);
    Serial.printf("OK G %u %.3f %.2f\n", m->id, f, m->gotoVmax);
  } else if (!strcmp(cmd, "S")) {
    holdAll();
    Serial.println("OK S");
  } else if (!strcmp(cmd, "EN")) {
    if (!motionAllowed()) return;
    // Already-enabled motors are left alone so EN never makes a loaded joint sag.
    if (a1) { Motor *m = motorArg(a1); if (!m) return; cancelMotion(*m); if (!m->enabled && !m->cfgStep) startEnable(*m); }
    else for (uint8_t i = 0; i < nmot; i++) { cancelMotion(mot[i]); if (!mot[i].enabled && !mot[i].cfgStep && !mot[i].dup) startEnable(mot[i]); }
    Serial.println("OK EN");
  } else if (!strcmp(cmd, "K")) {
    if (a1) { Motor *m = motorArg(a1); if (!m) return; disableMotor(*m); }
    else for (uint8_t i = 0; i < nmot; i++) disableMotor(mot[i]);
    Serial.println("OK K (freewheel)");
  } else if (!strcmp(cmd, "E")) {
    latch(SYS_ESTOP, "HOST_ESTOP");
    Serial.println("OK E");
  } else if (!strcmp(cmd, "F")) {
    latch(SYS_FAULT, a1 ? a1 : "HOST_FLAGGED");
    Serial.println("OK F");
  } else if (!strcmp(cmd, "CLR") || !strcmp(cmd, "R")) {
    sys = SYS_NORMAL;
    strcpy(faultReason, "NONE");
    hostHeld = false;
    // Healthy enabled motors keep holding; only faulted / disabled ones are re-initialised
    // (re-initialising briefly disables the drive, which would let the joint sag).
    for (uint8_t i = 0; i < nmot; i++) {
      Motor &m = mot[i];
      cancelMotion(m);
      if (!m.dup && !m.cfgStep && (!m.enabled || m.fault || m.mode != 2)) startEnable(m);
    }
    Serial.println("OK CLR (all RS02 enabled at 0 rad/s)");
  } else if (!strcmp(cmd, "Z")) {
    Motor *m = motorArg(a1);
    if (!m) return;
    if (m->enabled || m->cfgStep) { Serial.println("ERR disable it first: K <id>  (support the joint!)"); return; }
    sendType(T_SET_ZERO, m->id, 1);
    Serial.printf("OK Z %u (zero set in RAM; SAVE %u to keep it after power-off)\n", m->id, m->id);
  } else if (!strcmp(cmd, "SAVE")) {
    Motor *m = motorArg(a1);
    if (!m) return;
    if (m->enabled || m->cfgStep) { Serial.println("ERR disable it first: K <id>"); return; }
    writeU32(m->id, P_CAN_TIMEOUT, 0);      // don't persist the watchdog into flash
    uint8_t d[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    canSend(mkid(T_SAVE, m->id), d);
    Serial.printf("OK SAVE %u\n", m->id);
  } else if (!strcmp(cmd, "LIM")) {
    Motor *m = motorArg(a1);
    if (!m) return;
    if (a2 && !strcmp(a2, "OFF")) { m->lim.on = 0; saveLimit(*m); Serial.printf("OK LIM %u OFF\n", m->id); return; }
    if (!a2) { Serial.printf("OK LIM %u %s %.3f %.3f\n", m->id, m->lim.on ? "ON" : "OFF", m->lim.lo, m->lim.hi); return; }
    if (!parseFloat(a2, f) || !parseFloat(a3, g) || f >= g) { Serial.println("ERR usage: LIM <id> <lo> <hi>  (lo < hi)"); return; }
    m->lim = {clampf(f, -P_RANGE, P_RANGE), clampf(g, -P_RANGE, P_RANGE), 1};
    saveLimit(*m);
    Serial.printf("OK LIM %u ON %.3f %.3f\n", m->id, m->lim.lo, m->lim.hi);
  } else if (!strcmp(cmd, "RD") || !strcmp(cmd, "WR")) {
    Motor *m = motorArg(a1);
    if (!m || !a2) { if (m) Serial.println("ERR usage: RD <id> <hexidx>"); return; }
    uint16_t idx = (uint16_t)strtoul(a2, nullptr, 16);
    if (cmd[0] == 'R') { readParam(m->id, idx); return; }   // answer arrives as "OK RD ..."
    if (!parseFloat(a3, f)) { Serial.println("ERR usage: WR <id> <hexidx> <float>"); return; }
    writeF(m->id, idx, f);
    Serial.printf("OK WR %u 0x%04X %.4f\n", m->id, idx, f);
  } else if (!strcmp(cmd, "CTRL")) {
    if (a1 && (!strcmp(a1, "SPEED") || !strcmp(a1, "MIT"))) {
      if (anyEnabled()) { Serial.println("ERR disable motors first (K), then CTRL, then EN"); return; }
      ctrlMode = !strcmp(a1, "MIT") ? CTRL_MIT : CTRL_SPEED;
      saveSettings();
    }
    Serial.printf("OK CTRL %s\n", ctrlMode == CTRL_MIT ? "MIT" : "SPEED");
  } else if (!strcmp(cmd, "VMAX")) {
    if (parseFloat(a1, f) && f > 0) { velLimit = clampf(f, 0.05f, V_RANGE); saveSettings(); }
    Serial.printf("OK VMAX %.2f\n", velLimit);
  } else if (!strcmp(cmd, "TEL")) {
    if (a1) telMs = (uint16_t)constrain(atoi(a1), 0, 10000);
    Serial.printf("OK TEL %u\n", telMs);
  } else if (!strcmp(cmd, "HB")) {
    if (a1) hostTimeoutMs = (uint16_t)constrain(atoi(a1), 0, 60000);
    Serial.printf("OK HB %u\n", hostTimeoutMs);
  } else if (!strcmp(cmd, "LIST")) {
    for (uint8_t i = 0; i < nmot; i++) {
      const Motor &m = mot[i];
      Serial.printf("M %u %u %s %s lim=%s[%.2f,%.2f] pos=%.3f\n", i + 1, m.id,
                    m.enabled ? "EN" : (m.cfgStep ? "CFG" : "OFF"), m.dup ? "DUP" : "ok",
                    m.lim.on ? "ON" : "OFF", m.lim.lo, m.lim.hi, m.pos);
    }
    Serial.printf("OK LIST %u\n", nmot);
  } else if (!strcmp(cmd, "SCAN")) {
    if (anyEnabled()) { Serial.println("ERR disable motors first (K) - scanning pauses control"); return; }
    scanBus();
    Serial.println("OK SCAN");
  } else if (!strcmp(cmd, "DIAG") || !strcmp(cmd, "D")) {
    twai_status_info_t st = {};
    twai_get_status_info(&st);
    static const char *const bs[] = {"STOPPED", "RUNNING", "BUS_OFF", "RECOVERING"};
    Serial.printf("# CAN %s tec=%lu rec=%lu txq=%lu rxq=%lu txfail=%lu rxmiss=%lu busoff=%lu txdrop=%lu rx=%lu\n",
                  bs[st.state & 3], (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter,
                  (unsigned long)st.msgs_to_tx, (unsigned long)st.msgs_to_rx, (unsigned long)st.tx_failed_count,
                  (unsigned long)st.rx_missed_count, (unsigned long)busOffCount, (unsigned long)txDrop,
                  (unsigned long)rxCount);
    Serial.printf("# sys=%s reason=%s ctrl=%s vmax=%.2f hb=%u armed=%u\n", kSysName[sys], faultReason,
                  ctrlMode == CTRL_MIT ? "MIT" : "SPEED", velLimit, hostTimeoutMs, hbArmed);
    for (uint8_t i = 0; i < nmot; i++) {
      Motor &m = mot[i];
      if (a1 && m.id != atoi(a1)) continue;
      static const uint16_t q[] = {P_VBUS, P_RUN_MODE, P_LIMIT_CUR, P_LIMIT_TORQUE, P_CAN_TIMEOUT};
      for (uint16_t idx : q) readParam(m.id, idx);   // replies print as "OK RD ..."
    }
    Serial.println("OK DIAG");
  } else {
    Serial.printf("ERR unknown command '%s' (HELP)\n", cmd);
  }
}

static void pollSerial(uint32_t now) {
  static char buf[96];
  static uint8_t n = 0;
  static uint32_t lastChar = 0;
  while (Serial.available()) {
    char c = Serial.read();
    lastChar = now;
    if (c == '\n' || c == '\r' || c == ';') {
      if (n) {
        buf[n] = 0;
        n = 0;
        lastHostMs = now;
        hostHeld = false;
        if ((buf[0] == 'H' || buf[0] == 'h') && buf[1] == 0) hbArmed = true;
        execLine(buf);
      }
    } else if (n < sizeof buf - 1) buf[n++] = c;
  }
  if (n && now - lastChar > 150) {   // Serial Monitor with "No line ending"
    buf[n] = 0;
    n = 0;
    lastHostMs = now;
    execLine(buf);
  }
}

// ---------------------------------- main -------------------------------------
void setup() {
#if !ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxBufferSize(4096);      // telemetry must never stall the control loop
  Serial.setRxBufferSize(512);
#endif
  Serial.begin(SERIAL_BAUD);
#if STATUS_LED_PIN >= 0
  pinMode(STATUS_LED_PIN, OUTPUT);
#endif
#if ESTOP_PIN >= 0
  pinMode(ESTOP_PIN, ESTOP_ACTIVE_LEVEL == HIGH ? INPUT_PULLUP : INPUT_PULLDOWN);
#endif
  delay(300);
  Serial.printf("\n# %s  (RS02 chain bridge, CAN 1 Mbit/s, TX=GPIO%d RX=GPIO%d)\n", FW_VERSION,
                (int)CAN_TX_PIN, (int)CAN_RX_PIN);
  loadSettings();

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g.tx_queue_len = 48;
  g.rx_queue_len = 64;
  g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ERR_PASS |
                     TWAI_ALERT_RX_QUEUE_FULL;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
    for (;;) { Serial.println("!FAULT FATAL TWAI_INIT_FAILED (check pins / transceiver)"); delay(1000); }
  }
  delay(200);
  scanBus();
#if AUTO_ENABLE_ON_BOOT
  for (uint8_t i = 0; i < nmot; i++) startEnable(mot[i]);
#endif
  Serial.printf("READY %s %u\n", FW_VERSION, nmot);
}

void loop() {
  static uint32_t nextTick = 0;
  const uint32_t now = millis();
  pollCan(now);
  pollBusAlerts(now);
  pollSerial(now);
  if ((int32_t)(now - nextTick) >= 0) {
    nextTick = (now - nextTick > CTRL_PERIOD_MS) ? now + CTRL_PERIOD_MS : nextTick + CTRL_PERIOD_MS;
    controlTick(now);
  }
  telemetry(now);
  statusLed(now);
  delay(1);   // yield to the RTOS idle task (keeps the task watchdog happy)
}
