#include "00_drone.h"

// 상보필터 적용을 위한 계수
#define ALPHA (1.0 / (1.0+0.04)) // 0.96

#define PITCH_STABLE_KP ROLL_STABLE_KP
#define PITCH_STABLE_KI ROLL_STABLE_KI
#define PITCH_STABLE_KD ROLL_STABLE_KD
#define PITCH_RATE_KP   ROLL_RATE_KP
#define PITCH_RATE_KI   ROLL_RATE_KI
#define PITCH_RATE_KD   ROLL_RATE_KD

#define ROLL_STABLE_KP 2.0
#define ROLL_STABLE_KI 0.0001//0.001
#define ROLL_STABLE_KD 0.005//0.0005
#define ROLL_RATE_KP   3.0
#define ROLL_RATE_KI   0.0001//0.0001//0.001
#define ROLL_RATE_KD   0.005//0.0005

#define YAW_STABLE_KP 2.0
#define YAW_STABLE_KI 0.0
#define YAW_RATE_KP 5.0
#define YAW_RATE_KI 0.0001
#define YAW_RATE_KD 0.0

/*    set OFFSET
         FRONT
  -------------------
           + 
        M1   M2
      -    X    +
        M4   M3
           -
  -------------------
*/

#define OFFSET_PITCH -0.75
#define OFFSET_ROLL -1.2 // 1.6 // 0.8

extern double Ki;
extern double Kp;
extern double Kd;

double angle_L_pitch = 0;
double angle_L_roll = 0;
double angle_L_yaw = 0;

double res_force_pitch = 0;
double res_force_roll = 0;
double res_force_yaw = 0;

double pitch_rate_error_L = 0;
double roll_rate_error_L = 0;
double yaw_rate_error_L = 0;

double pitch_stabilize_Iterm = 0;
double roll_stabilize_Iterm = 0;

//2중 PID
void Dual_PID(balancing_force_t *balancing_force, target_angle_t target_angle, gyro_angle_t gyro_angle, acc_angle_t acc_angle, dt_t dt, gyro_rate_t gyro_rate)
{
  double angle_error_pitch = 0;
  double angle_error_roll = 0;
  double angle_error_yaw = 0;

  double pInput_pitch = 0;
  double pInput_roll = 0;
  double pInput_yaw = 0;

  double Ptemp_pitch = 0;
  double Ptemp_roll = 0;
  double Ptemp_yaw = 0;

  double Dtemp_pitch = 0;
  double Dtemp_roll = 0;
  double Dtemp_yaw = 0;

  extern throttle_t throttle;
  
  double pitch_stabilize_Pterm = 0;
  double roll_stabilize_Pterm = 0;
  double yaw_stabilize_Pterm = 0;

  double pitch_stabilize_Dterm = 0;
  double roll_stabilize_Dterm = 0;

  double pitch_rate_error = 0;
  double roll_rate_error = 0;
  double yaw_rate_error = 0;
  
  angle_error_pitch = (target_angle.pitch - gyro_angle.pitch);// + OFFSET_PITCH;
  angle_error_roll = (target_angle.roll - gyro_angle.roll);// + OFFSET_ROLL;
  angle_error_yaw = (target_angle.yaw - gyro_angle.yaw);

  pitch_stabilize_Pterm = PITCH_STABLE_KP * angle_error_pitch;
  pitch_stabilize_Iterm += angle_error_pitch * PITCH_STABLE_KI * dt.t_period;
  pitch_stabilize_Dterm = (-PITCH_STABLE_KD * (angle_error_pitch - angle_L_pitch) / dt.t_period );

  roll_stabilize_Pterm = ROLL_STABLE_KP * angle_error_roll;
  roll_stabilize_Iterm += angle_error_roll * ROLL_STABLE_KI * dt.t_period;
  roll_stabilize_Dterm = (-ROLL_STABLE_KD * (angle_error_roll - angle_L_roll) / dt.t_period);
  
  yaw_stabilize_Pterm = YAW_STABLE_KP * angle_error_yaw;

  pitch_rate_error = pitch_stabilize_Pterm + pitch_stabilize_Iterm + pitch_stabilize_Dterm - gyro_rate.pitch;

  Ptemp_pitch = PITCH_RATE_KP * pitch_rate_error;
  res_force_pitch += PITCH_RATE_KI * pitch_rate_error * dt.t_period;
  pInput_pitch = pitch_rate_error - pitch_rate_error_L;
  Dtemp_pitch = (-PITCH_RATE_KD * pInput_pitch / dt.t_period);

  roll_rate_error = roll_stabilize_Pterm + roll_stabilize_Iterm + roll_stabilize_Dterm - gyro_rate.roll;

  Ptemp_roll = ROLL_RATE_KP * roll_rate_error;
  res_force_roll += ROLL_RATE_KI * roll_rate_error * dt.t_period;
  pInput_roll = roll_rate_error - roll_rate_error_L;
  Dtemp_roll = (-ROLL_RATE_KD * pInput_roll / dt.t_period);

  yaw_rate_error = yaw_stabilize_Pterm - gyro_rate.yaw;

  Ptemp_yaw = YAW_RATE_KP * yaw_rate_error;
  res_force_yaw += YAW_RATE_KI * yaw_rate_error * dt.t_period;
  //pInput_yaw = yaw_rate_error - yaw_rate_error_L;
  //Dtemp_yaw = (-YAW_RATE_KD * pInput_yaw);

  pitch_rate_error_L = pitch_rate_error;
  roll_rate_error_L = roll_rate_error;
  //yaw_rate_error_L = yaw_rate_error;

  angle_L_pitch = angle_error_pitch;
  angle_L_roll = angle_error_roll;

  if(throttle.value <= MIN_THROTTLE)
  {
    res_force_pitch = 0.0;
    res_force_roll = 0.0;
    res_force_yaw = 0.0;
  }

  balancing_force->pitch = Ptemp_pitch + res_force_pitch + Dtemp_pitch;
  balancing_force->roll = Ptemp_roll + res_force_roll + Dtemp_roll;
  balancing_force->yaw = Ptemp_yaw + res_force_yaw + Dtemp_yaw;
}

// (1) PID 제어
void calc(balancing_force_t *balancing_force, target_angle_t target_angle, gyro_angle_t gyro_angle, acc_angle_t acc_angle, dt_t dt, gyro_rate_t gyro_rate)
{
  double angle_error_pitch;
  double angle_error_roll;
  double angle_error_yaw;

  double pInput_pitch = 0;
  double pInput_roll = 0;
  double pInput_yaw = 0;

  double Ptemp_pitch = 0;
  double Ptemp_roll = 0;
  double Ptemp_yaw = 0;

  double Dtemp_pitch = 0;
  double Dtemp_roll = 0;
  double Dtemp_yaw = 0;

  extern throttle_t throttle;

  //angle_gyro_pitch = ALPHA*(gyro_rate.pitch * dt.t_period) + (1.0 - ALPHA)*acc_angle.pitch;
  //angle_gyro_roll = ALPHA*(gyro_rate.roll * dt.t_period) + (1.0 - ALPHA)*acc_angle.roll;
  //angle_gyro_yaw = gyro_rate.yaw * dt.t_period;

  angle_error_pitch = (target_angle.pitch - gyro_angle.pitch);
  angle_error_roll = (target_angle.roll - gyro_angle.roll);
  angle_error_yaw = (target_angle.yaw - gyro_angle.yaw);

  Ptemp_pitch = Kp * angle_error_pitch;
  Ptemp_roll = Kp * angle_error_roll;
  Ptemp_yaw = Kp * angle_error_yaw;
  
  res_force_pitch += Ki * angle_error_pitch * dt.t_period;
  res_force_roll += Ki * angle_error_roll * dt.t_period;
  //res_force_yaw += Ki * angle_error_yaw * dt.t_period;
  
  
  if(throttle.value <= MIN_THROTTLE)
  {
    res_force_pitch = 0.0;
    res_force_roll = 0.0;
    res_force_yaw = 0.0;
  }
  
  pInput_pitch = gyro_angle.pitch - angle_L_pitch;
  pInput_roll = gyro_angle.roll - angle_L_roll;
  pInput_yaw = gyro_angle.yaw - angle_L_yaw;

  //Serial.print(gyro_angle.pitch);
  //Serial.print("  ||  ");
  //Serial.print(angle_L_pitch);

  Dtemp_pitch = (-Kd * pInput_pitch / dt.t_period); // 주기 나누기
  Dtemp_roll = (-Kd * pInput_roll / dt.t_period);
  Dtemp_yaw = (-Kd * pInput_yaw / dt.t_period);

  angle_L_pitch = gyro_angle.pitch;
  angle_L_roll = gyro_angle.roll;
  angle_L_yaw = gyro_angle.yaw;

  balancing_force->pitch = Ptemp_pitch + res_force_pitch + Dtemp_pitch;
  balancing_force->roll = Ptemp_roll + res_force_roll + Dtemp_roll;
  //balancing_force->yaw = Ptemp_yaw + res_force_yaw + Dtemp_yaw;
  balancing_force->yaw = 0;

}

void calc_motor_speed(motor_speed_t *motor_speed, throttle_t throttle, balancing_force_t balancing_force)
{
  
  if(throttle.value <= MIN_THROTTLE) {
    motor_speed->motorA_sp = MIN_THROTTLE;
  }
  else {
    motor_speed->motorA_sp = throttle.value + balancing_force.roll - balancing_force.pitch - balancing_force.yaw;
  }
  if(throttle.value <= MIN_THROTTLE) {
    motor_speed->motorB_sp = MIN_THROTTLE;
  }
  else {
    motor_speed->motorB_sp = throttle.value - balancing_force.roll - balancing_force.pitch + balancing_force.yaw;
  }
  if(throttle.value <= MIN_THROTTLE) {
    motor_speed->motorC_sp = MIN_THROTTLE;
  }
  else {
    motor_speed->motorC_sp = throttle.value - balancing_force.roll + balancing_force.pitch - balancing_force.yaw;
  }
  if(throttle.value <= MIN_THROTTLE) {
    motor_speed->motorD_sp = MIN_THROTTLE;
  }
  else {
    motor_speed->motorD_sp = throttle.value + balancing_force.roll + balancing_force.pitch + balancing_force.yaw;
  }
}