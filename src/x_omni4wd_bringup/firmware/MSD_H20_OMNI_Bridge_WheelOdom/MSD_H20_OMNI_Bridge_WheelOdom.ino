/*
 * MSD_H20_OMNI_Bridge_WheelOdom.ino
 *
 * Mở rộng MSD_H20_OMNI_Bridge: vừa điều khiển 4 driver MSD_H20, vừa đọc vận tốc encoder.
 *
 * USB (PC/ROS, 19200):
 *   PC -> Arduino:  VXYW <vx_mm_s> <vy_mm_s> <w_mrad_s>  |  STOP
 *   Arduino -> PC:  WVEL <w_fl> <w_rl> <w_rr> <w_fr>     (mrad/s * 1000, tại trục BÁNH)
 *                   READY (khi khởi động)
 *
 * MSD UART network (SoftwareSerial RX=13, TX=12, 19200):
 *   Gửi:  N<addr> v<rad/s_motor> a<accel>   (giống bridge cũ)
 *   Đọc:  N<addr> O G1                       (đọc vận tốc hiện tại 1 lần / driver)
 *   Phản hồi driver (text): ... v<rad/s_motor> ...  -> chia GEAR_RATIO -> ω_bánh
 *
 * Thứ tự bánh trên WVEL: FL, RL, RR, FR (khớp wheel_odometry_node.py).
 * Địa chỉ MSD: FL=10, RL=20, RR=30, FR=40.
 */

#include <SoftwareSerial.h>
#include <string.h>

SoftwareSerial msdSerial(13, 12); // RX=13, TX=12

const long BAUD_USB = 19200;
const long BAUD_MSD = 19200;

const float K_LIN = 0.7071f;
float WHEEL_R = 0.0635f;
float ROBOT_R = 0.2505f;
float GEAR_RATIO = 19.2f;
float MOTOR_MAX_RPM = 14000.0f;
// Giới hạn vận hành (khớp ROS max_wheel_speed): 65% trần bánh ~76.37 rad/s → 50 rad/s.
const float WHEEL_MAX_RAD_S = 50.0f;
const int ACCEL = 320;

int ADDR_FL = 10;
int ADDR_RL = 20;
int ADDR_RR = 30;
int ADDR_FR = 40;

const int ADDR_LIST[4] = {ADDR_FL, ADDR_RL, ADDR_RR, ADDR_FR};

const float INPUT_SCALE_LINEAR = 1.0f / 1000.0f;
const float INPUT_SCALE_ANGULAR = 1.0f / 1000.0f;

const unsigned long WVEL_USB_PERIOD_MS = 50;   // 20 Hz -> PC (sau 1 vòng đọc đủ 4 bánh)
const unsigned long MSD_G1_TIMEOUT_MS = 45;    // chờ 1 dòng phản hồi / driver
const unsigned long POLL_GAP_MS = 8;         // nghỉ giữa 2 lệnh G1 liên tiếp
const unsigned long WHEEL_MEAS_STALE_MS = 250; // hết hạn đo G1 -> dùng cmdWheelRadS

bool hasPendingVXYW = false;
int16_t pending_vx_i = 0;
int16_t pending_vy_i = 0;
int16_t pending_w_i = 0;

float motorWLimRadS = 0.0f;

// ω_bánh rad/s — FL, RL, RR, FR (đo G1)
float wheelRadS[4] = {0, 0, 0, 0};
bool wheelValid[4] = {false, false, false, false};
unsigned long wheelMeasMs[4] = {0, 0, 0, 0};
// ω_bánh từ lệnh VXYW gần nhất (fallback khi G1 chậm / bus MSD bận)
float cmdWheelRadS[4] = {0, 0, 0, 0};

#define LINE_BUF 96
char lineBuf[LINE_BUF];
int lineLen = 0;

#define MSD_RX_BUF 128
char msdRxBuf[MSD_RX_BUF];
uint8_t msdRxLen = 0;

// Poll tuần tự 4 bánh / chu kỳ WVEL (tránh chỉ cập nhật 1 bánh / lần)
int pollActiveIdx = 0;       // bánh đang hỏi G1 (0..3)
int pollCycleWheel = 0;      // tiến độ trong chu kỳ 0..4 (4 = xong)
bool pollWaiting = false;
bool pollCycleActive = false;
unsigned long pollDeadlineMs = 0;
unsigned long lastPollGapMs = 0;
unsigned long lastWvelUsbMs = 0;
unsigned long lastMotorCmdMs = 0;
const unsigned long MOTOR_QUIET_MS = 12;  // nghỉ sau burst lệnh v trước khi hỏi G1

float clampf(float x, float mn, float mx) {
  if (x < mn) return mn;
  if (x > mx) return mx;
  return x;
}

static long roundToLong(float x) {
  return (x >= 0.0f) ? (long)(x + 0.5f) : (long)(x - 0.5f);
}

float motorMaxRadS() {
  return MOTOR_MAX_RPM * 2.0f * 3.14159265f / 60.0f;
}

void msdWarmup() {
  // Workaround SoftwareSerial (tham khảo MSD_XX_Uart_Example.ino)
  msdSerial.println(F("Hello: MSD "));
  delay(80);
  while (msdSerial.available()) {
    msdSerial.read();
  }
}

void flushMsdRx() {
  while (msdSerial.available()) {
    msdSerial.read();
  }
  msdRxLen = 0;
  msdRxBuf[0] = 0;
}

void sendMotorCmd(int addr, long v_motor_rad_s) {
  msdSerial.print(F("N"));
  msdSerial.print(addr);
  msdSerial.print(F(" v"));
  msdSerial.print(v_motor_rad_s);
  msdSerial.print(F(" a"));
  msdSerial.print(ACCEL);
  msdSerial.print('\n');
}

void requestMotorVelocityG1(int addr) {
  msdSerial.print(F("N"));
  msdSerial.print(addr);
  msdSerial.println(F(" O G1"));
}

// AVR (Nano) không có strtof — parse float thủ công
bool parseAsciiFloat(const char *p, float *outVal) {
  if (!p || !outVal || (*p != '-' && *p != '+' && (*p < '0' || *p > '9'))) {
    return false;
  }

  bool neg = false;
  if (*p == '-') {
    neg = true;
    p++;
  } else if (*p == '+') {
    p++;
  }

  long intPart = 0;
  while (*p >= '0' && *p <= '9') {
    intPart = intPart * 10 + (*p - '0');
    p++;
  }

  float fracPart = 0.0f;
  float place = 0.1f;
  if (*p == '.') {
    p++;
    while (*p >= '0' && *p <= '9') {
      fracPart += (*p - '0') * place;
      place *= 0.1f;
      p++;
    }
  }

  float v = (float)intPart + fracPart;
  if (neg) v = -v;
  *outVal = v;
  return true;
}

bool parseMotorVelocityRadS(const char *line, float *outMotorRadS) {
  if (!line || !outMotorRadS) return false;

  // Tìm token vận tốc: 'v' theo sau bởi số (hỗ trợ "v12.3", "v 12.3", "v:12.3")
  const char *p = strchr(line, 'v');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == ':' || *p == '=') {
    p++;
  }
  if (*p == '\0') return false;

  float v = 0.0f;
  if (!parseAsciiFloat(p, &v)) return false;

  *outMotorRadS = v;
  return true;
}

int wheelIndexFromAddr(int addr) {
  for (int i = 0; i < 4; i++) {
    if (ADDR_LIST[i] == addr) return i;
  }
  return -1;
}

bool parseMsdAddrFromLine(const char *line, int *outAddr) {
  if (!line || !outAddr) return false;
  const char *p = strchr(line, 'N');
  if (!p) return false;
  p++;
  if (*p < '0' || *p > '9') return false;

  long addr = 0;
  while (*p >= '0' && *p <= '9') {
    addr = addr * 10L + (*p - '0');
    p++;
  }
  if (addr <= 0L || addr > 255L) return false;
  *outAddr = (int)addr;
  return true;
}

void storeWheelVelocity(int wheelIndex, float motorRadS) {
  if (wheelIndex < 0 || wheelIndex > 3) return;
  float wWheel = motorRadS / GEAR_RATIO;
  wheelRadS[wheelIndex] = wWheel;
  wheelValid[wheelIndex] = true;
  wheelMeasMs[wheelIndex] = millis();
}

float wheelSpeedForPublish(int i, unsigned long nowMs) {
  bool measFresh = wheelValid[i] && (nowMs - wheelMeasMs[i] <= WHEEL_MEAS_STALE_MS);
  if (measFresh) {
    return wheelRadS[i];
  }
  return cmdWheelRadS[i];
}

void publishWvelUsb() {
  // FL RL RR FR — mrad/s * 1000 (int), sau khi hỏi G1 cả 4 driver trong 1 chu kỳ
  unsigned long nowMs = millis();
  Serial.print(F("WVEL "));
  for (int i = 0; i < 4; i++) {
    float w = wheelSpeedForPublish(i, nowMs);
    long scaled = roundToLong(w * 1000.0f);
    if (scaled > 32767) scaled = 32767;
    if (scaled < -32768) scaled = -32768;
    Serial.print(scaled);
    if (i < 3) Serial.print(' ');
  }
  Serial.print('\n');
}

void beginPollWheel(int wheelIndex, bool forceNow) {
  if (wheelIndex < 0 || wheelIndex > 3) return;

  unsigned long now = millis();
  if (!forceNow && (now - lastPollGapMs < POLL_GAP_MS)) return;

  lastPollGapMs = now;
  pollActiveIdx = wheelIndex;
  flushMsdRx();
  requestMotorVelocityG1(ADDR_LIST[wheelIndex]);
  pollWaiting = true;
  pollDeadlineMs = now + MSD_G1_TIMEOUT_MS;
}

void finishPollCycle() {
  pollCycleActive = false;
  pollCycleWheel = 0;
  pollWaiting = false;
  publishWvelUsb();
  lastWvelUsbMs = millis();
}

void advancePollCycle() {
  pollWaiting = false;
  pollCycleWheel++;
  if (pollCycleWheel >= 4) {
    finishPollCycle();
    return;
  }
  beginPollWheel(pollCycleWheel, true);
}

void startPollCycle() {
  if (pollCycleActive || pollWaiting) return;
  if (millis() - lastMotorCmdMs < MOTOR_QUIET_MS) return;

  pollCycleActive = true;
  pollCycleWheel = 0;
  beginPollWheel(0, true);
}

void applyPendingVXYW() {
  int16_t vx_i = pending_vx_i;
  int16_t vy_i = pending_vy_i;
  int16_t w_i = pending_w_i;

  float vx_mps = (float)vx_i * INPUT_SCALE_LINEAR;
  float vy_mps = (float)vy_i * INPUT_SCALE_LINEAR;
  float w_radps = (float)w_i * INPUT_SCALE_ANGULAR;

  float w_fl = (K_LIN * vx_mps - K_LIN * vy_mps - ROBOT_R * w_radps) / WHEEL_R;
  float w_rl = (K_LIN * vx_mps + K_LIN * vy_mps - ROBOT_R * w_radps) / WHEEL_R;
  float w_rr = (-K_LIN * vx_mps + K_LIN * vy_mps - ROBOT_R * w_radps) / WHEEL_R;
  float w_fr = (-K_LIN * vx_mps - K_LIN * vy_mps - ROBOT_R * w_radps) / WHEEL_R;

  cmdWheelRadS[0] = w_fl;
  cmdWheelRadS[1] = w_rl;
  cmdWheelRadS[2] = w_rr;
  cmdWheelRadS[3] = w_fr;

  float w_motor_fl = clampf(w_fl * GEAR_RATIO, -motorWLimRadS, motorWLimRadS);
  float w_motor_rl = clampf(w_rl * GEAR_RATIO, -motorWLimRadS, motorWLimRadS);
  float w_motor_rr = clampf(w_rr * GEAR_RATIO, -motorWLimRadS, motorWLimRadS);
  float w_motor_fr = clampf(w_fr * GEAR_RATIO, -motorWLimRadS, motorWLimRadS);

  sendMotorCmd(ADDR_FL, roundToLong(w_motor_fl));
  sendMotorCmd(ADDR_FR, roundToLong(w_motor_fr));
  sendMotorCmd(ADDR_RL, roundToLong(w_motor_rl));
  sendMotorCmd(ADDR_RR, roundToLong(w_motor_rr));
  lastMotorCmdMs = millis();
}

void parseLine(char *line) {
  int n = strlen(line);
  while (n > 0 && (line[n - 1] == '\r' || line[n - 1] == '\n')) {
    line[--n] = 0;
  }
  if (n == 0) return;

  if (n >= 4 && strncmp(line, "STOP", 4) == 0) {
    pending_vx_i = 0;
    pending_vy_i = 0;
    pending_w_i = 0;
    for (int i = 0; i < 4; i++) {
      cmdWheelRadS[i] = 0.0f;
    }
    hasPendingVXYW = true;
    return;
  }

  if (n >= 4 && strncmp(line, "VXYW", 4) == 0) {
    char *p = line + 4;
    long vals[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++) {
      while (*p == ' ' || *p == '\t') p++;
      if (*p == '\0') return;

      bool neg = false;
      if (*p == '-') {
        neg = true;
        p++;
      }
      if (*p < '0' || *p > '9') return;

      long v = 0;
      while (*p >= '0' && *p <= '9') {
        v = v * 10 + (*p - '0');
        p++;
      }
      if (v > 32767) v = 32767;
      vals[i] = neg ? -v : v;
    }

    pending_vx_i = (int16_t)vals[0];
    pending_vy_i = (int16_t)vals[1];
    pending_w_i = (int16_t)vals[2];
    hasPendingVXYW = true;
    return;
  }
}

void pumpUSB() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        parseLine(lineBuf);
        lineLen = 0;
      }
    } else {
      if (lineLen < LINE_BUF - 1) {
        lineBuf[lineLen++] = c;
      } else {
        lineLen = 0;
      }
    }
  }
}

void onMsdLineComplete() {
  msdRxBuf[msdRxLen] = 0;

  if (!pollWaiting || !pollCycleActive) {
    msdRxLen = 0;
    return;
  }

  float motorV = 0.0f;
  if (parseMotorVelocityRadS(msdRxBuf, &motorV)) {
    int wheelIdx = pollActiveIdx;
    int addr = 0;
    if (parseMsdAddrFromLine(msdRxBuf, &addr)) {
      int mapped = wheelIndexFromAddr(addr);
      if (mapped >= 0) wheelIdx = mapped;
    }
    storeWheelVelocity(wheelIdx, motorV);
  }

  msdRxLen = 0;
  advancePollCycle();
}

void pumpMsdRx() {
  while (msdSerial.available()) {
    char c = (char)msdSerial.read();
    if (c == '\r' || c == '\n') {
      if (msdRxLen > 0) {
        onMsdLineComplete();
      }
      continue;
    }
    if (msdRxLen < MSD_RX_BUF - 1) {
      msdRxBuf[msdRxLen++] = c;
    } else {
      msdRxLen = 0;
    }
  }

  if (pollWaiting && pollCycleActive && millis() >= pollDeadlineMs) {
    // Timeout 1 bánh: chuyển sang bánh tiếp theo, vẫn publish sau đủ 4 lần hỏi
    msdRxLen = 0;
    advancePollCycle();
  }
}

void serviceVelocityPoll() {
  if (pollWaiting) return;

  unsigned long now = millis();
  if (now - lastWvelUsbMs < WVEL_USB_PERIOD_MS) return;

  startPollCycle();
}

void setup() {
  Serial.begin(BAUD_USB);
  msdSerial.begin(BAUD_MSD);
  delay(800);

  motorWLimRadS = WHEEL_MAX_RAD_S * GEAR_RATIO;
  msdWarmup();

  Serial.println(F("READY MSD_H20_OMNI_Bridge_WheelOdom"));
  Serial.println(F("IN:  VXYW <vx_mm_s> <vy_mm_s> <w_mrad_s> | STOP"));
  Serial.println(F("OUT: WVEL <w_fl> <w_rl> <w_rr> <w_fr>  (mrad/s*1000, wheel shaft)"));
}

void loop() {
  pumpUSB();
  pumpMsdRx();

  if (hasPendingVXYW) {
    hasPendingVXYW = false;
    applyPendingVXYW();
  }

  serviceVelocityPoll();
}
