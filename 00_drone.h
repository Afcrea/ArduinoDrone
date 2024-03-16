#include <Arduino.h>

#ifndef __00_DRONE_H__
#define __00_DRONE_H__

#define YAW_INIT 0.0
#define PITCH_INIT 0.0
#define ROLL_INIT 0.0

#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs


// 변수 정의
typedef struct { int i2c_addr, PWR_MGMT_1, ACCEL_XOUT_H, GYRO_XOUT_H; } mpu6050_t;

// 자이로 센서 관련 변수
typedef struct { int16_t x, y, z; } gyro_raw_t;
typedef struct { int16_t x, y, z; } gyro_offset_t;
typedef struct { int16_t x, y, z; } gyro_adj_t;
typedef struct { double pitch, roll, yaw; } gyro_rate_t;
typedef struct { double pitch, roll, yaw; } gyro_angle_t;

typedef struct { double pitch, roll, yaw; } target_angle_t;
typedef struct { double pitch, roll, yaw; } setting_angle_t;

// 가속도 센서 관련 변수
typedef struct { int16_t x, y, z; } acc_raw_t;
typedef struct { int16_t x, y, z; } acc_offset_t;
typedef struct { int16_t x, y, z; } acc_adj_t;
typedef struct { int16_t x, y, z; } LPF_accin_t;
typedef struct { int16_t x, y, z; } LPF_accout_t;
typedef struct { double pitch, roll, yaw; } acc_angle_t;

typedef struct { double pitch, roll, yaw; } angle_error_t;
typedef struct { double pitch, roll, yaw; } angle_error_last_t;

// 온도 관련 변수
typedef struct { int16_t value; } temp_raw_t;

// 샘플링 시간 변수
typedef struct { uint32_t t_now, t_prev; double t_period; } dt_t;

// 무선 제어기 관련 변수
typedef struct { int Throttle_pin, Yaw_pin, Pitch_pin, Roll_pin, drive; } fsi6b_t;
typedef struct { double value, prev; } throttle_t;
typedef struct { int ch_throt, ch_yaw, ch_pitch, ch_roll, drive; } fsi6b_ch_t;
typedef struct { uint32_t tmr_base, tmr_throt, tmr_yaw, tmr_pitch, tmr_roll, tmr_drive; } fsi6b_time_t;
typedef struct { int pwm_throt, pwm_yaw, pwm_pitch, pwm_roll, pwm_drive; } fsi6b_pwm_t;

// 모터 관련 변수
typedef struct { int motorA_pin, motorB_pin, motorC_pin, motorD_pin; } motor_t;
typedef struct { double motorA_sp, motorB_sp, motorC_sp, motorD_sp; } motor_speed_t;
typedef struct { double pitch, yaw, roll; } balancing_force_t;

#endif
