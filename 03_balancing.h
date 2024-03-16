#ifndef __03_BALANCING_H__
#define __03_BALANCING_H__

void calc_motor_speed(motor_speed_t *motor_speed, throttle_t throttle, balancing_force_t balancing_force);
void calc(balancing_force_t *balancing_force, target_angle_t target_angle, gyro_angle_t gyro_angle, acc_angle_t acc_angle, dt_t dt, gyro_rate_t gyro_rate);
void calc_2(balancing_force_t *balancing_force, target_angle_t target_angle, gyro_angle_t *gyro_angle, gyro_rate_t *gyro_rate, dt_t dt);
void add(balancing_force_t *balancing_force, gyro_rate_t gyro_rate, dt_t dt);
void add(balancing_force_t *balancing_force, target_angle_t target_angle, gyro_angle_t gyro_angle, acc_angle_t acc_angle, dt_t dt);
void Dual_PID(balancing_force_t *balancing_force, target_angle_t target_angle, gyro_angle_t gyro_angle, acc_angle_t acc_angle, dt_t dt, gyro_rate_t gyro_rate);

#endif