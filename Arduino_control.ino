
#define BAUDRATE 115200
// === 选择驱动板：1=TB6612FNG；0=L298N ===
#define DRIVER_TB6612 1
#if DRIVER_TB6612
// ---- TB6612FNG 引脚（Nano）----
const int STBY = 4;    // 使能脚，高有效
// 左电机（A通道）
const int AIN1 = 7;
const int AIN2 = 8;
const int PWMA = 5;    // PWM
// 右电机（B通道）
const int BIN1 = 9;
const int BIN2 = 10;
const int PWMB = 6;    // PWM
#else
// ---- L298N 引脚（Nano）----
const int ENA = 5;   // 左 PWM
const int IN1 = 7;   // 左方向1
const int IN2 = 8;   // 左方向2
const int ENB = 6;   // 右 PWM
const int IN3 = 9;   // 右方向1
const int IN4 = 10;  // 右方向2
#endif

// 看门狗
const unsigned long CMD_TIMEOUT_MS = 500;
unsigned long lastCmdMs = 0;

void setup() {
  Serial.begin(BAUDRATE);
#if DRIVER_TB6612
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
#else
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
#endif
  lastCmdMs = millis();
  Serial.println("READY");
}

void driveOne(int speed, bool leftMotor) {
  // speed: -255..255；负数=后退
  int v = speed;
  if (v > 255) v = 255;
  if (v < -255) v = -255;
#if DRIVER_TB6612
  if (leftMotor) {
    if (v >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
    else         { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); v = -v; }
    analogWrite(PWMA, v);
  } else {
    if (v >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
    else         { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); v = -v; }
    analogWrite(PWMB, v);
  }
#else
  if (leftMotor) {
    if (v >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
    else         { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); v = -v; }
    analogWrite(ENA, v);
  } else {
    if (v >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
    else         { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); v = -v; }
    analogWrite(ENB, v);
  }
#endif
}

void stopAll() {
#if DRIVER_TB6612
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
#else
  analogWrite(ENA, 0); analogWrite(ENB, 0);
#endif
}

void loop() {
  // 读取一行
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n'); line.trim();
    // 解析格式：S <L> <R>
    if (line.length() > 0 && (line[0] == 'S' || line[0] == 's')) {
      int spL = 0, spR = 0;
      int parsed = sscanf(line.c_str()+1, "%d %d", &spL, &spR);
      if (parsed == 2) {
        driveOne(spL, true);
        driveOne(spR, false);
        lastCmdMs = millis();
      }
    }
  }

  // 超时保护
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopAll();
  }
}
