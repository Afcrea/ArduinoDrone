#include <Wire.h>
#include "00_drone.h"
#include "02_gyro.h"
#include "03_balancing.h"
#include "04_remote.h"


#include <Servo.h>
Servo motA, motB, motC, motD;
char data;

mpu6050_t mpu6050 = {
  .i2c_addr = 0x68, 
  .PWR_MGMT_1 = 0x6B, 
  .ACCEL_XOUT_H = 0x3B, 
  .GYRO_XOUT_H = 0x43
};

// 자이로 센서 관련 변수
gyro_raw_t gyro_raw;
gyro_offset_t gyro_offset;
gyro_adj_t gyro_adj;
gyro_rate_t gyro_rate;
gyro_angle_t gyro_angle = { 
  .pitch = 0.0, .roll = 0.0, .yaw = 0.0
};

// 가속도 센서 관련 변수
acc_raw_t acc_raw;
acc_offset_t acc_offset;
acc_adj_t acc_adj;
LPF_accin_t LPF_accin;
LPF_accout_t LPF_accout;
acc_angle_t acc_angle = {
  .pitch = 0.0, .roll = 0.0, .yaw = 0.0
};

angle_error_t angle_error = {
  .pitch = 0.0, .roll = 0.0, .yaw = 0.0
};

angle_error_last_t angle_error_last = {
  .pitch = 0.0, .roll = 0.0, .yaw = 0.0
};

// 온도 센서 변수
temp_raw_t temp_raw;
dt_t dt;

// 무선 제어기 변수
fsi6b_t fsi6b = {
  .Throttle_pin = A8, .Yaw_pin = A9, .Pitch_pin = A10, .Roll_pin = A11, .drive = A12
};
throttle_t throttle = {
  .value = 0.0, .prev = 0.0
};
fsi6b_ch_t fsi6b_ch = {
  .ch_throt = 0, 
  .ch_yaw = 0, 
  .ch_pitch = 0, 
  .ch_roll = 0,
  .drive = 0
};
fsi6b_time_t fsi6b_time;
fsi6b_pwm_t fsi6b_pwm = {
  .pwm_throt = 0,
  .pwm_yaw = 0,
  .pwm_pitch = 0,
  .pwm_roll = 0,
  .pwm_drive = 1000
};

// 모터 관련 변수
motor_t motor = {
};
motor_speed_t motor_speed = {
  .motorA_sp = 0.0, 
  .motorB_sp = 0.0, 
  .motorC_sp = 0.0, 
  .motorD_sp = 0.0
};
balancing_force_t balancing_force = {
  .pitch = 0.0,
  .yaw = 0.0,
  .roll = 0.0
};

target_angle_t target_angle = {
  .pitch = 0.0,
  .roll = 0.0,
  .yaw = 0.0
};
setting_angle_t setting_angle = {         // mpu6050 설치 시 길울어진 값 보상
  .pitch = 0.0,
  .roll = 0.0,
  .yaw = 0.0
};

// PID 계수
double Ki = 0.1;
double Kp = 2.8;
double Kd = 0.75;


void setup() {
  motA.attach(3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motB.attach(8, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motC.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motD.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

  motA.writeMicroseconds(1000);
  motB.writeMicroseconds(1000);
  motC.writeMicroseconds(1000);
  motD.writeMicroseconds(1000);
  
  init(mpu6050);
  get(mpu6050, &gyro_offset, &acc_offset);
  init(&dt);
  init(fsi6b);
}

void loop() {
  static int cnt_loop = 0;

  // 주기 계산
  calc(&dt);
  
  // 자이로 센서에서 현재 정보 읽어오기
  read(mpu6050, &gyro_raw, &temp_raw, &acc_raw);

  // 자이로 센서 각속도 계산하기
  calc(&gyro_adj, gyro_raw, gyro_offset, &acc_adj, acc_raw, acc_offset);
  calc(&gyro_rate, gyro_adj);

  // 가속도 센서 각도 계산하기
  calc(&acc_angle, acc_adj, &LPF_accin, &LPF_accout);

  // 자이로 센서 각도 계산하기
  calc(&gyro_angle, &gyro_rate, dt, &acc_angle);

  // Balancing force 계산하기
  Dual_PID(&balancing_force, target_angle, gyro_angle, acc_angle, dt, gyro_rate);

  // fsi6b 수신기 읽어오기
  read_fsi6(fsi6b_pwm, &target_angle, &throttle);

  // 모터 속도 계산
  calc_motor_speed(&motor_speed, throttle, balancing_force);
  
  if(fsi6b_pwm.pwm_drive > 1900 && fsi6b_pwm.pwm_drive < 2100){
    if(motor_speed.motorA_sp > 1200) motA.writeMicroseconds(constrain((int)motor_speed.motorA_sp,1000,2000));
    else motA.writeMicroseconds(constrain((int)motor_speed.motorA_sp,1000,2000));

    if(motor_speed.motorB_sp > 1200) motB.writeMicroseconds(constrain((int)motor_speed.motorB_sp,1000,2000));
    else motB.writeMicroseconds(constrain((int)motor_speed.motorB_sp,1000,2000));

    if(motor_speed.motorC_sp > 1200) motC.writeMicroseconds(constrain((int)motor_speed.motorC_sp,1000,2000));
    else motC.writeMicroseconds(constrain((int)motor_speed.motorC_sp,1000,2000));

    if(motor_speed.motorD_sp > 1200) motD.writeMicroseconds(constrain((int)motor_speed.motorD_sp,1000,2000));
    else motD.writeMicroseconds(constrain((int)motor_speed.motorD_sp,1000,2000));
  }
  else {
    motA.writeMicroseconds(1000);
    motB.writeMicroseconds(1000);
    motC.writeMicroseconds(1000);
    motD.writeMicroseconds(1000);
  }
}
