// === Arduino Nano firmware: Servo steering + ESC throttle ===
// Protocol (from Pi): "C <steer_us> <throttle_us>\n"
// Example:            C 1500 1500
//  - steer_us:     steering servo pulse width in microseconds (e.g. 1100~1900, 1500 = center)
//  - throttle_us:  ESC pulse width in microseconds (1000..2000; 1500 = stop for bidirectional ESC)

#include <Arduino.h>
#include <Servo.h>

// -------- Pins (Nano) --------
const uint8_t PIN_STEER = 9;   // D9  (Timer1, Servo lib)
const uint8_t PIN_ESC   = 10;  // D10 (Timer1, Servo lib)
const uint8_t PIN_LED   = 13;  // On-board LED

// -------- Servo/ESC ranges (match Pi script) --------
const int SERVO_CENTER_US = 1500;
const int SERVO_MIN_US    = 1100;  // steering physical limits (tune for your chassis)
const int SERVO_MAX_US    = 1900;

const int ESC_MIN_US      = 1000;  // bidirectional ESC: 1000 reverse max
const int ESC_STOP_US     = 1500;  // neutral/stop
const int ESC_MAX_US      = 2000;  // forward max

// Optional: invert steering direction (some horn linkages reverse)
const bool SERVO_REVERSE  = false;

// -------- Runtime settings --------
const uint32_t BAUDRATE        = 115200; // must match Pi side
const uint32_t CMD_TIMEOUT_MS  = 500;    // failsafe: no cmd → stop
const uint8_t  SMOOTH_ALPHA    = 64;     // 0..255 (EMA smoothing for servo/throttle), 64≈0.25

Servo servoSteer, esc;
int targetSteer = SERVO_CENTER_US;
int targetThr   = ESC_STOP_US;
int currentSteer = SERVO_CENTER_US;
int currentThr   = ESC_STOP_US;

uint32_t lastCmdMs = 0;
uint32_t lastBeat  = 0;

// Simple integer EMA smoothing
static inline int emaStep(int cur, int tgt, uint8_t alpha /*0..255*/) {
  // cur += alpha*(tgt-cur) / 255
  long diff = (long)tgt - (long)cur;
  return (int)(cur + ((long)alpha * diff) / 255L);
}

static inline int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void applyOutputs() {
  // Smooth
  currentSteer = emaStep(currentSteer, targetSteer, SMOOTH_ALPHA);
  currentThr   = emaStep(currentThr,   targetThr,   SMOOTH_ALPHA);

  servoSteer.writeMicroseconds(currentSteer);
  esc.writeMicroseconds(currentThr);
}

void safeStop() {
  targetSteer = SERVO_CENTER_US;
  targetThr   = ESC_STOP_US;
  applyOutputs();
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(BAUDRATE);
  // Give USB/serial a moment to enumerate
  delay(300);

  servoSteer.attach(PIN_STEER);
  esc.attach(PIN_ESC);

  // Initialize to safe values
  currentSteer = targetSteer = SERVO_CENTER_US;
  currentThr   = targetThr   = ESC_STOP_US;
  applyOutputs();

  // Small arming delay (some ESCs need a neutral pulse at boot)
  delay(800);

  Serial.println(F("READY Nano: send lines like 'C 1500 1500'"));
  lastCmdMs = millis();
}

void loop() {
  // ---- Parse incoming lines ----
  static char buf[32];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[idx] = '\0';
      idx = 0;

      // Expected: C <steer_us> <throttle_us>
      char tag;
      int steer_us, thr_us;
      int matched = sscanf(buf, " %c %d %d", &tag, &steer_us, &thr_us);
      if (matched == 3 && (tag == 'C' || tag == 'c')) {
        // Steering clamp (+ optional reverse)
        steer_us = clampInt(steer_us, SERVO_MIN_US, SERVO_MAX_US);
        if (SERVO_REVERSE) {
          // Mirror around center
          int delta = steer_us - SERVO_CENTER_US;
          steer_us  = SERVO_CENTER_US - delta;
        }
        // Throttle clamp
        thr_us = clampInt(thr_us, ESC_MIN_US, ESC_MAX_US);

        targetSteer = steer_us;
        targetThr   = thr_us;

        lastCmdMs = millis();
        digitalWrite(PIN_LED, HIGH);
      } else {
        // Ignore malformed line, optionally report
        // Serial.print(F("IGN: ")); Serial.println(buf);
      }
    } else {
      if (idx < sizeof(buf) - 1) buf[idx++] = c;
      else idx = 0; // overflow → reset
    }
  }

  // ---- Failsafe if command lost ----
  uint32_t now = millis();
  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    safeStop();
    digitalWrite(PIN_LED, (now >> 7) & 1); // slow blink when idle/failsafe
  } else {
    digitalWrite(PIN_LED, LOW);
  }

  // ---- Output update ----
  applyOutputs();

  // ---- Lightweight heartbeat (2 Hz) ----
  if (now - lastBeat > 500) {
    lastBeat = now;
    Serial.print(F("OK steer=")); Serial.print(currentSteer);
    Serial.print(F(" thr="));     Serial.println(currentThr);
  }

  // With 50 Hz Pi updates, this small delay is fine
  delay(2);
}
