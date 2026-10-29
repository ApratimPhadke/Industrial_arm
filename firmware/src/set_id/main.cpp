// =============================================================================
//  rs02_set_id  —  RobStride RS02 / RS04 CAN-ID setup tool for ESP32
// =============================================================================
//  Works in Arduino IDE (rename to rs02_set_id.ino inside a folder of the same
//  name) and in PlatformIO (env:set_id). One file, no extra libraries.
//
//  Wiring: ESP32 GPIO5 -> transceiver TXD, GPIO4 -> transceiver RXD
//          (SN65HVD230 / TJA1051-3.3V etc.), CAN_H/CAN_L to the motor,
//          120 ohm termination at both ends of the bus, common GND.
//
//  Open the Serial Monitor at 115200 baud (line ending: "Newline" or "Both").
//
//  RECOMMENDED WORKFLOW (one motor at a time, motor powered at 24-48 V):
//      1. Connect ONLY ONE new motor (factory ID is usually 127).
//      2. Type:  assign 1        -> motor gets ID 1, is saved and verified
//      3. Power-cycle the motor, type: scan   -> confirm it now answers on 1
//      4. Unplug it, plug the next motor, type: assign 2   ... and so on.
//
//  Commands:
//      scan                 list every motor on the bus (ID + 64-bit UID)
//      assign <new>         give the ONLY motor on the bus the ID <new>
//      set <old> <new>      change motor <old> to <new> (refuses on duplicates)
//      set <old> <new> force  same, even if several motors share <old>
//      wiggle <id>          spin motor <id> gently back and forth (identify it)
//      zero <id>            store the current shaft angle as mechanical zero
//      help                 show this list
//
//  Optional one-shot mode (what you used to do on the STM32): set
//  AUTO_ASSIGN_ID to 1..127, flash, and the single connected motor is given
//  that ID at boot. Leave it 0 to use the interactive commands above.
// =============================================================================
#include <Arduino.h>
#include "driver/twai.h"

// ------------------------------- settings ------------------------------------
#define CAN_TX_PIN      GPIO_NUM_5
#define CAN_RX_PIN      GPIO_NUM_4
#define HOST_ID         0xFD
#define SERIAL_BAUD     115200
#define AUTO_ASSIGN_ID  0          // 0 = interactive, 1..127 = one-shot assign at boot
#define PROBE_WINDOW_MS 15         // how long to collect replies to a "get ID" request
#define WIGGLE_SPEED    1.0f       // rad/s, output shaft
#define WIGGLE_MS       700
// -----------------------------------------------------------------------------

#if AUTO_ASSIGN_ID < 0 || AUTO_ASSIGN_ID > 127
#error "AUTO_ASSIGN_ID must be 0..127"
#endif

// RobStride private-protocol communication types
enum : uint8_t {
  T_GET_ID = 0, T_MIT = 1, T_FEEDBACK = 2, T_ENABLE = 3, T_STOP = 4,
  T_SET_ZERO = 6, T_SET_ID = 7, T_PARAM_WRITE = 18, T_SAVE = 22
};

struct Found { uint8_t id; uint64_t uid; uint8_t replies; };
static Found found[16];
static uint8_t nfound = 0;

// ------------------------------- CAN helpers ---------------------------------
static bool canSend(uint32_t id29, const uint8_t d[8]) {
  twai_message_t m = {};
  m.extd = 1;
  m.identifier = id29 & 0x1FFFFFFF;
  m.data_length_code = 8;
  memcpy(m.data, d, 8);
  return twai_transmit(&m, pdMS_TO_TICKS(20)) == ESP_OK;
}
static inline uint32_t mkid(uint8_t type, uint8_t motor) {
  return ((uint32_t)type << 24) | ((uint32_t)HOST_ID << 8) | motor;
}
static void sendType(uint8_t type, uint8_t motor, uint8_t b0) {
  uint8_t d[8] = {b0, 0, 0, 0, 0, 0, 0, 0};
  canSend(mkid(type, motor), d);
}
static void writeParamU8(uint8_t motor, uint16_t idx, uint8_t v) {
  uint8_t d[8] = {(uint8_t)idx, (uint8_t)(idx >> 8), 0, 0, v, 0, 0, 0};
  canSend(mkid(T_PARAM_WRITE, motor), d);
}
static void drainRx() {
  twai_message_t r;
  while (twai_receive(&r, 0) == ESP_OK) {}
}
// If nothing on the bus acknowledges our frames (no motor powered), the TWAI
// controller keeps retransmitting forever. Flush it so the tool stays usable.
static void unstickTx() {
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) return;
  if (st.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();
    uint32_t t0 = millis();
    while (millis() - t0 < 200) {
      twai_get_status_info(&st);
      if (st.state == TWAI_STATE_STOPPED) break;
      delay(2);
    }
    twai_start();
  } else if (st.msgs_to_tx > 0) {
    twai_stop();
    twai_start();
  }
}

// Ask <id> for its UID. Returns number of DIFFERENT motors that answered
// (0 = nobody, 1 = good, >1 = several motors share this ID).
static uint8_t probe(uint8_t id, uint64_t *uid_out) {
  drainRx();
  sendType(T_GET_ID, id, 0);
  uint64_t uids[4];
  uint8_t n = 0;
  uint32_t t0 = millis();
  twai_message_t r;
  while (millis() - t0 < PROBE_WINDOW_MS) {
    if (twai_receive(&r, pdMS_TO_TICKS(2)) != ESP_OK) continue;
    if (!r.extd || r.data_length_code != 8) continue;
    uint32_t i = r.identifier;
    if (((i >> 24) & 0x1F) != T_GET_ID || (i & 0xFF) != 0xFE || ((i >> 8) & 0xFF) != id) continue;
    uint64_t u;
    memcpy(&u, r.data, 8);
    bool seen = false;
    for (uint8_t k = 0; k < n; k++) seen |= (uids[k] == u);
    if (!seen && n < 4) uids[n++] = u;
  }
  if (n == 0) unstickTx();
  if (n && uid_out) *uid_out = uids[0];
  return n;
}

static void printUid(uint64_t u) {
  Serial.printf("%08lX%08lX", (unsigned long)(u >> 32), (unsigned long)(u & 0xFFFFFFFF));
}

static void scanBus(bool verbose) {
  nfound = 0;
  if (verbose) Serial.println("Scanning IDs 1..127 ...");
  for (uint8_t id = 1; id <= 127; id++) {
    uint64_t u = 0;
    uint8_t n = probe(id, &u);
    if (n && nfound < 16) found[nfound++] = {id, u, n};
  }
  if (!verbose) return;
  if (!nfound) {
    Serial.println("  No motor answered. Check: motor power (24-48 V), CAN_H/CAN_L not swapped,");
    Serial.println("  120 ohm termination, transceiver on GPIO5(TX)/GPIO4(RX), common GND.");
    return;
  }
  for (uint8_t k = 0; k < nfound; k++) {
    Serial.printf("  ID %3u  UID ", found[k].id);
    printUid(found[k].uid);
    if (found[k].replies > 1)
      Serial.printf("   <-- %u MOTORS SHARE THIS ID! Connect them one at a time.", found[k].replies);
    Serial.println();
  }
  Serial.printf("  %u ID(s) in use.\n", nfound);
}

// Change <oldId> -> <newId>, save to flash and verify. Returns true on success.
static bool changeId(uint8_t oldId, uint8_t newId, bool force) {
  if (newId < 1 || newId > 127) { Serial.println("ERR: new ID must be 1..127"); return false; }
  if (oldId == newId)           { Serial.println("Nothing to do: IDs are equal."); return true; }

  uint64_t uid = 0;
  uint8_t n = probe(oldId, &uid);
  if (n == 0) { Serial.printf("ERR: no motor answers on ID %u\n", oldId); return false; }
  if (n > 1 && !force) {
    Serial.printf("ERR: %u motors answer on ID %u. They would ALL be renamed.\n", n, oldId);
    Serial.println("     Connect only one motor, or add 'force' if that is really what you want.");
    return false;
  }
  if (probe(newId, nullptr)) {
    Serial.printf("ERR: ID %u is already used by another motor on the bus.\n", newId);
    return false;
  }

  Serial.printf("Changing ID %u -> %u (UID ", oldId, newId);
  printUid(uid);
  Serial.println(") ...");
  sendType(T_STOP, oldId, 0);            // ID can only be changed while the motor is disabled
  delay(50);
  uint8_t d[8] = {0};
  canSend(((uint32_t)T_SET_ID << 24) | ((uint32_t)newId << 16) | ((uint32_t)HOST_ID << 8) | oldId, d);
  delay(200);

  uint64_t uid2 = 0;
  uint8_t n2 = probe(newId, &uid2);
  if (n2 == 0 || (!force && (n2 != 1 || uid2 != uid))) {
    Serial.println("ERR: motor did not answer on the new ID. Nothing was saved.");
    return false;
  }
  uint8_t save[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  canSend(mkid(T_SAVE, newId), save);  // persist parameters to flash
  delay(300);
  if (probe(oldId, nullptr)) Serial.printf("WARN: something still answers on old ID %u.\n", oldId);
  Serial.printf("OK: motor now answers on ID %u. Power-cycle it and run 'scan' to confirm it stuck.\n", newId);
  return true;
}

static void wiggle(uint8_t id) {
  if (!probe(id, nullptr)) { Serial.printf("ERR: no motor on ID %u\n", id); return; }
  Serial.printf("Wiggling ID %u (+%.1f / -%.1f rad/s). Keep hands clear.\n", id, WIGGLE_SPEED, WIGGLE_SPEED);
  sendType(T_STOP, id, 1);  delay(30);        // stop + clear faults
  writeParamU8(id, 0x7005, 0); delay(30);     // run_mode 0 = operation control (MIT)
  sendType(T_ENABLE, id, 0);   delay(30);
  // MIT frame: pos=0, vel=v, kp=0, kd=1 -> pure damping velocity tracking
  auto mit = [&](float v) {
    auto f2u = [](float x, float lo, float hi) -> uint16_t {
      x = constrain(x, lo, hi);
      return (uint16_t)((x - lo) * 65535.0f / (hi - lo));
    };
    uint16_t p = f2u(0, -12.566f, 12.566f), vv = f2u(v, -44.0f, 44.0f);
    uint16_t kp = f2u(0, 0, 500.0f), kd = f2u(1.0f, 0, 5.0f), tff = f2u(0, -17.0f, 17.0f);
    uint8_t d[8] = {(uint8_t)(p >> 8), (uint8_t)p, (uint8_t)(vv >> 8), (uint8_t)vv,
                    (uint8_t)(kp >> 8), (uint8_t)kp, (uint8_t)(kd >> 8), (uint8_t)kd};
    canSend(((uint32_t)T_MIT << 24) | ((uint32_t)tff << 8) | id, d);
  };
  const float seq[3] = {WIGGLE_SPEED, -WIGGLE_SPEED, 0.0f};
  for (float v : seq) {
    uint32_t t0 = millis();
    while (millis() - t0 < (v == 0 ? 100u : (uint32_t)WIGGLE_MS)) { mit(v); drainRx(); delay(20); }
  }
  sendType(T_STOP, id, 0);
  Serial.println("Done (motor disabled).");
}

static void setZero(uint8_t id) {
  if (!probe(id, nullptr)) { Serial.printf("ERR: no motor on ID %u\n", id); return; }
  sendType(T_STOP, id, 0); delay(30);
  sendType(T_SET_ZERO, id, 1); delay(50);
  uint8_t save[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  canSend(mkid(T_SAVE, id), save);
  delay(100);
  Serial.printf("OK: current angle of ID %u stored as zero.\n", id);
}

#if AUTO_ASSIGN_ID
// One-shot mode: wait until exactly one motor is present, then assign.
static void autoAssign(uint8_t newId) {
  Serial.printf("AUTO_ASSIGN_ID = %u: waiting for exactly ONE motor on the bus...\n", newId);
  for (;;) {
    scanBus(false);
    if (nfound == 1 && found[0].replies == 1) break;
    Serial.printf("  found %u ID(s) -> connect exactly one motor\n", nfound);
    delay(1500);
  }
  if (found[0].id == newId) Serial.printf("Motor already has ID %u.\n", newId);
  else changeId(found[0].id, newId, false);
  Serial.println("Set AUTO_ASSIGN_ID back to 0 (or keep using the interactive commands).");
}
#endif

static void printHelp() {
  Serial.println();
  Serial.println("==================== RS02 CAN-ID TOOL ====================");
  Serial.println("  scan                  list motors on the bus");
  Serial.println("  assign <new>          set the ONLY connected motor to <new>");
  Serial.println("  set <old> <new>       change one motor's ID");
  Serial.println("  set <old> <new> force change even if IDs are duplicated");
  Serial.println("  wiggle <id>           spin a motor briefly to identify it");
  Serial.println("  zero <id>             save current angle as zero position");
  Serial.println("  help                  this list");
  Serial.println("===========================================================");
}

static void handleLine(char *s) {
  for (char *p = s; *p; p++) *p = tolower((unsigned char)*p);
  char cmd[12] = {0}, opt[8] = {0};
  int a = -1, b = -1;
  int n = sscanf(s, "%11s %d %d %7s", cmd, &a, &b, opt);
  if (n < 1) return;
  Serial.printf("> %s\n", s);
  if ((n >= 2 && (a < 1 || a > 127)) || (n >= 3 && (b < 1 || b > 127))) {
    Serial.println("ERR: motor IDs must be 1..127");
    return;
  }

  if (!strcmp(cmd, "scan")) scanBus(true);
  else if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) printHelp();
  else if (!strcmp(cmd, "assign") && n >= 2) {
    scanBus(false);
    if (nfound == 1 && found[0].replies > 1) {
      Serial.printf("ERR: %u motors share ID %u. Unplug all but one, then try again.\n",
                    found[0].replies, found[0].id);
      return;
    }
    if (nfound != 1) {
      Serial.printf("ERR: need exactly ONE motor on the bus (found %u). Run 'scan'.\n", nfound);
      return;
    }
    if (found[0].id == a) Serial.printf("Motor already has ID %d.\n", a);
    else changeId(found[0].id, (uint8_t)a, false);
  }
  else if (!strcmp(cmd, "set") && n >= 3) changeId((uint8_t)a, (uint8_t)b, n == 4 && !strcmp(opt, "force"));
  else if (!strcmp(cmd, "wiggle") && n >= 2) wiggle((uint8_t)a);
  else if (!strcmp(cmd, "zero") && n >= 2) setZero((uint8_t)a);
  else Serial.println("Unknown command. Type 'help'.");
}

static void pollSerial() {
  static char buf[64];
  static uint8_t n = 0;
  static uint32_t last = 0;
  while (Serial.available()) {
    char c = Serial.read();
    last = millis();
    if (c == '\n' || c == '\r') {
      if (n) { buf[n] = 0; handleLine(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) buf[n++] = c;
  }
  // Serial Monitor set to "No line ending": run the command after 200 ms of silence.
  if (n && millis() - last > 200) { buf[n] = 0; handleLine(buf); n = 0; }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(800);
  Serial.println("\n[rs02_set_id] RobStride CAN-ID tool (ESP32 TWAI, 1 Mbit/s)");

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g.tx_queue_len = 16;
  g.rx_queue_len = 32;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println("FATAL: TWAI init failed (check pins).");
    for (;;) delay(1000);
  }
  delay(300);

#if AUTO_ASSIGN_ID
  autoAssign(AUTO_ASSIGN_ID);
#endif
  scanBus(true);
  printHelp();
}

void loop() {
  pollSerial();
  delay(2);
}
