/*
 * MSD_H20_OMNI_Bridge.ino
 * Cầu nối VXYW (vx_mm_s, vy_mm_s, w_mrad_s - scaled int) từ PC/ROS -> 4 driver MSD_H20 qua UART network
 * Cho robot x_omni4wd: 4 bánh omni kiểu X-drive (bánh đặt chéo ±45°, con lăn song song)
 *
 * - PC gửi: "VXYW <vx_mm_s> <vy_mm_s> <w_mrad_s>" (scaled int) qua USB CDC 19200
 * - Uno/Nano gửi cho 4 driver MSD theo địa chỉ: N<addr> v<value> a<accel>
 * - Tham khảo ssmr_bringup/firmware/MSD_H20_OMNI_Bridge (cùng giao thức)
 *
 * HỆ TỌA ĐỘ (ROS REP-103): X=Forward, Y=Left, W=CCW
 *
 * Jacobian chuẩn (ma trận 4x3, K_LIN=1/sqrt(2), cột quay -R):
 *   w_FL = ( K_LIN*vx - K_LIN*vy - ROBOT_R*w) / r
 *   w_RL = ( K_LIN*vx + K_LIN*vy - ROBOT_R*w) / r
 *   w_RR = (-K_LIN*vx + K_LIN*vy - ROBOT_R*w) / r
 *   w_FR = (-K_LIN*vx - K_LIN*vy - ROBOT_R*w) / r
 * Giới hạn bánh: WHEEL_MAX_RAD_S (50 rad/s ≈ 65% trần 76.37 rad/s) → clamp motor |v| ≤ WHEEL_MAX_RAD_S * GEAR_RATIO.
 */

#include <SoftwareSerial.h>

SoftwareSerial msdSerial(13, 12); // RX=13, TX=12

const long BAUD_USB = 19200;
const long BAUD_MSD = 19200;

// Tham số Jacobian chuẩn (khớp cmd_vel_to_omni_wheels.py)
const float K_LIN = 0.7071f;    // 1/sqrt(2)
float WHEEL_R = 0.0635f;        // r - bán kính bánh (~wheel_radius)
float ROBOT_R = 0.2505f;        // R - tâm robot -> tâm bánh (m)
float GEAR_RATIO = 19.2f;

// Trần datasheet motor: 14000 rpm; sau hộp số ≈ 76.37 rad/s tại bánh.
float MOTOR_MAX_RPM = 14000.0f;
// Giới hạn vận hành (khớp ROS max_wheel_speed): 65% trần bánh → 50 rad/s.
const float WHEEL_MAX_RAD_S = 50.0f;

float motorMaxRadS(){
  return MOTOR_MAX_RPM * 2.0f * 3.14159265f / 60.0f;
}

// Driver MSD nhận trường "a" (gia tốc lệnh v): cao = phản ứng nhanh, dễ giật; thử 200–400 nếu cần mượt.
const int ACCEL = 320;

// Địa chỉ MSD_H20 — khớp w_fl, w_fr, w_rl, w_rr (cmd_vel_to_omni_wheels.py)
//   Nhìn từ trên (+X tiến):   FL(10) — FR(40)
//                              |      |
//                            RL(20) — RR(30)
int ADDR_FL = 10;  // Front-left  -> w_fl
int ADDR_RL = 20;  // Rear-left   -> w_rl
int ADDR_RR = 30;  // Rear-right  -> w_rr
int ADDR_FR = 40;  // Front-right -> w_fr

// Tối ưu độ trễ: luôn giữ lệnh VXYW mới nhất đã parse xong,
// sau đó gửi sang 4 driver khi loop rảnh (tránh backlog khi đang bận SoftwareSerial).
bool hasPendingVXYW = false;
int16_t pending_vx_i = 0; // scaled input
int16_t pending_vy_i = 0;
int16_t pending_w_i  = 0;

float motorWLimRadS = 0.0f;

#define LINE_BUF 96
char lineBuf[LINE_BUF];
int lineLen = 0;

float clampf(float x, float mn, float mx){ if(x<mn) return mn; if(x>mx) return mx; return x; }

// Nhận lệnh VXYW dạng số nguyên scaled (từ arduino_interface_omni.py)
// vx, vy: mm/s  -> scaled=1000
// w:       mrad/s -> scaled=1000
const float INPUT_SCALE_LINEAR = 1.0f / 1000.0f;
const float INPUT_SCALE_ANGULAR = 1.0f / 1000.0f;

static long roundToLong(float x){
  return (x >= 0.0f) ? (long)(x + 0.5f) : (long)(x - 0.5f);
}

void sendMotorCmd(int addr, long v_motor_rad_s){
  // Protocol text ngắn để giảm thời gian truyền UART.
  msdSerial.print(F("N"));
  msdSerial.print(addr);
  msdSerial.print(F(" v"));
  msdSerial.print(v_motor_rad_s);
  msdSerial.print(F(" a"));
  msdSerial.print(ACCEL);
  msdSerial.print('\n');
}

void applyPendingVXYW(){
  // Snapshot lệnh để luôn dùng "lệnh mới nhất vừa parse được".
  int16_t vx_i = pending_vx_i;
  int16_t vy_i = pending_vy_i;
  int16_t w_i  = pending_w_i;

  float vx_mps  = (float)vx_i * INPUT_SCALE_LINEAR;
  float vy_mps  = (float)vy_i * INPUT_SCALE_LINEAR;
  float w_radps = (float)w_i  * INPUT_SCALE_ANGULAR;

  // Jacobian chuẩn — khớp cmd_vel_to_omni_wheels.py
  float w_fl = (K_LIN * vx_mps - K_LIN * vy_mps - ROBOT_R * w_radps) / WHEEL_R;
  float w_rl = (K_LIN * vx_mps + K_LIN * vy_mps - ROBOT_R * w_radps) / WHEEL_R;
  float w_rr = (-K_LIN * vx_mps + K_LIN * vy_mps - ROBOT_R * w_radps) / WHEEL_R;
  float w_fr = (-K_LIN * vx_mps - K_LIN * vy_mps - ROBOT_R * w_radps) / WHEEL_R;

  // Đổi sang vận tốc motor để gửi MSD
  float w_motor_fl = w_fl * GEAR_RATIO;
  float w_motor_rl = w_rl * GEAR_RATIO;
  float w_motor_rr = w_rr * GEAR_RATIO;
  float w_motor_fr = w_fr * GEAR_RATIO;

  // Clamp theo giới hạn động cơ (rad/s tại motor)
  w_motor_fl = clampf(w_motor_fl, -motorWLimRadS, motorWLimRadS);
  w_motor_rl = clampf(w_motor_rl, -motorWLimRadS, motorWLimRadS);
  w_motor_rr = clampf(w_motor_rr, -motorWLimRadS, motorWLimRadS);
  w_motor_fr = clampf(w_motor_fr, -motorWLimRadS, motorWLimRadS);

  // Gửi lệnh tới MSD theo thứ tự đan kẽ để giảm moment lệch tức thời.
  // Thứ tự khuyến nghị: FL -> FR -> RL -> RR.
  sendMotorCmd(ADDR_FL, roundToLong(w_motor_fl));
  sendMotorCmd(ADDR_FR, roundToLong(w_motor_fr));
  sendMotorCmd(ADDR_RL, roundToLong(w_motor_rl));
  sendMotorCmd(ADDR_RR, roundToLong(w_motor_rr));
}

void setup(){
  Serial.begin(BAUD_USB);
  msdSerial.begin(BAUD_MSD);
  delay(800);
  Serial.println(F("=== MSD_H20_OMNI_Bridge Ready (x_omni4wd) ==="));
  Serial.println(F("Coordinate system (ROS REP-103): X=Forward, Y=Left, W=CCW"));
  Serial.println(F("Commands: VXYW <vx_mm_s> <vy_mm_s> <w_mrad_s>, STOP"));
  motorWLimRadS = WHEEL_MAX_RAD_S * GEAR_RATIO;
}

void parseLine(char *line){
  int n = strlen(line);
  while(n > 0 && (line[n-1] == '\r' || line[n-1] == '\n')) line[--n] = 0;
  if(n == 0) return;

  // Parse nhanh (không strtok/atof) để giảm overhead khi UART dày.
  // Format kỳ vọng:
  //   VXYW <vx_mm_s> <vy_mm_s> <w_mrad_s>
  //   STOP
  if(n >= 4 && strncmp(line, "STOP", 4) == 0){
    pending_vx_i = 0;
    pending_vy_i = 0;
    pending_w_i  = 0;
    hasPendingVXYW = true;
    return;
  }

  if(n >= 4 && strncmp(line, "VXYW", 4) == 0){
    char *p = line + 4;
    long vals[3] = {0, 0, 0};

    for(int i=0; i<3; i++){
      while(*p == ' ' || *p == '\t') p++;
      if(*p == '\0') return;

      bool neg = false;
      if(*p == '-'){ neg = true; p++; }

      if(*p < '0' || *p > '9') return;

      long v = 0;
      while(*p >= '0' && *p <= '9'){
        v = v * 10 + (*p - '0');
        p++;
      }

      // Clamp vào int16_t để tránh tràn.
      if(v > 32767) v = 32767;
      if(v < -32768) v = -32768;
      vals[i] = neg ? -v : v;
    }

    pending_vx_i = (int16_t)vals[0];
    pending_vy_i = (int16_t)vals[1];
    pending_w_i  = (int16_t)vals[2];
    hasPendingVXYW = true;
    return;
  }
}

void pumpUSB(){
  while(Serial.available()){
    char c=(char)Serial.read();
    if(c=='\r' || c=='\n'){
      if(lineLen>0){ lineBuf[lineLen]=0; parseLine(lineBuf); lineLen=0; }
    }else{
      if(lineLen<LINE_BUF-1) lineBuf[lineLen++]=c; else lineLen=0;
    }
  }
}

void loop(){
  pumpUSB();
  if(hasPendingVXYW){
    hasPendingVXYW = false;
    applyPendingVXYW();
  }
}
