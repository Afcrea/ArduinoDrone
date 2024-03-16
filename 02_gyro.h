#ifndef __02_GYRO_H__
#define __02_GYRO_H__

void init(mpu6050_t mpu6050);
void read(mpu6050_t mpu6050, gyro_raw_t *gyro_raw, temp_raw_t *temp_raw, acc_raw_t *acc_raw);
void get(mpu6050_t mpu6050, gyro_offset_t *gyro_offset, acc_offset_t *acc_offset);
void calc(gyro_adj_t *gyro_adj, gyro_raw_t gyro_raw, gyro_offset_t gyro_offset,
          acc_adj_t *acc_adj, acc_raw_t acc_raw, acc_offset_t acc_offset);
void calc(gyro_rate_t *gyro_rate, gyro_adj_t gyro_adj);
void calc(acc_angle_t *acc_angle, acc_adj_t acc_adj, LPF_accin_t *LPF_accin, LPF_accout_t *LPF_accout);
void calc(gyro_angle_t *gyro_angle, gyro_rate_t *gyro_rate, dt_t dt, acc_angle_t *acc_angle);
void calc(dt_t *dt);
void init(dt_t *dt);
#endif
