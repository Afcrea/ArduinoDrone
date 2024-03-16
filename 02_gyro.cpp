#include <Wire.h>
#include "00_drone.h"
#define G_TO_READ   14
#define NSAMPLES     1000
#define DEG_PER_SEC  131

void init(mpu6050_t mpu6050) {
  Wire.begin();
  Wire.beginTransmission(mpu6050.i2c_addr);
  Wire.write(mpu6050.PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void read(mpu6050_t mpu6050, gyro_raw_t *gyro_raw, temp_raw_t *temp_raw, acc_raw_t *acc_raw) 
{
  Wire.beginTransmission(mpu6050.i2c_addr);
  Wire.write(mpu6050.ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu6050.i2c_addr, G_TO_READ, true);

  acc_raw->x = Wire.read()<<8 | Wire.read();
  acc_raw->y = Wire.read()<<8 | Wire.read();
  acc_raw->z = Wire.read()<<8 | Wire.read();

  temp_raw->value = Wire.read()<<8 | Wire.read();
  
  gyro_raw->x = Wire.read()<<8 | Wire.read();
  gyro_raw->y = Wire.read()<<8 | Wire.read();
  gyro_raw->z = Wire.read()<<8 | Wire.read();
}

void get(mpu6050_t mpu6050, gyro_offset_t *gyro_offset, acc_offset_t *acc_offset)
{
  gyro_raw_t gyro_raw = {0, 0, 0};
  acc_raw_t acc_raw = {0, 0, 0};
  temp_raw_t temp_raw = {0};
  int32_t sumGyX = 0, sumGyY = 0, sumGyZ = 0;
  int32_t sumAccX = 0, sumAccY = 0, sumAccZ = 0;

  for(int i=0; i<NSAMPLES; i++)
   {
      read(mpu6050, &gyro_raw, &temp_raw, &acc_raw);

      sumAccX += acc_raw.x;
      sumAccY += acc_raw.y;
      sumAccZ += acc_raw.z; 
            
      sumGyX += gyro_raw.x;
      sumGyY += gyro_raw.y;
      sumGyZ += gyro_raw.z;
   }
   acc_offset->x = sumAccX / NSAMPLES;
   acc_offset->y = sumAccY / NSAMPLES;
   acc_offset->z = sumAccZ / NSAMPLES;
   
   gyro_offset->x = sumGyX / NSAMPLES;
   gyro_offset->y = sumGyY / NSAMPLES;
   gyro_offset->z = sumGyZ / NSAMPLES;
}

void calc(gyro_adj_t *gyro_adj, gyro_raw_t gyro_raw, gyro_offset_t gyro_offset,
    acc_adj_t *acc_adj, acc_raw_t acc_raw, acc_offset_t acc_offset)
{
  acc_adj->x = acc_raw.x - acc_offset.x;                  //roll
  acc_adj->y = acc_raw.y - acc_offset.y;                  //pitch
  acc_adj->z = acc_raw.z + (16384 - acc_offset.z);
  
  gyro_adj->x = gyro_raw.x - gyro_offset.x;               //roll
  gyro_adj->y = gyro_raw.y - gyro_offset.y;               //pitch
  gyro_adj->z = gyro_raw.z - gyro_offset.z;
}
void calc(gyro_rate_t *gyro_rate, gyro_adj_t gyro_adj)
{
  gyro_rate->roll = (double)gyro_adj.y / DEG_PER_SEC;         // 
  gyro_rate->pitch = -(double)gyro_adj.x / DEG_PER_SEC;
  gyro_rate->yaw = -(double)gyro_adj.z / DEG_PER_SEC;
}
void calc(acc_angle_t *acc_angle, acc_adj_t acc_adj, LPF_accin_t *LPF_accin, LPF_accout_t *LPF_accout)
{
  const double RADIANS_TO_DEGREE = 180.0/3.141592;

  LPF_accin->x = acc_adj.x;
  LPF_accout->x = (0.95 * LPF_accin->x) + (1.0-0.95) * LPF_accout->x;
  acc_adj.x = LPF_accout->x;

  LPF_accin->y = acc_adj.y;
  LPF_accout->y = (0.95 * LPF_accin->y) + (1.0-0.95) * LPF_accout->y;
  acc_adj.y = LPF_accout->y;
  
  LPF_accin->z = acc_adj.z;
  LPF_accout->z = (0.95 * LPF_accin->z) + (1.0-0.95) * LPF_accout->z;
  acc_adj.z = LPF_accout->z;

  acc_angle->roll = atan2(acc_adj.x, sqrt(pow(acc_adj.y, 2.0) + pow(acc_adj.z, 2.0))) * -RADIANS_TO_DEGREE;
  acc_angle->pitch = atan2(acc_adj.y, sqrt(pow(acc_adj.x, 2.0) + pow(acc_adj.z, 2.0))) * -RADIANS_TO_DEGREE;
  acc_angle->yaw = 0.0;
}
void calc(gyro_angle_t *gyro_angle, gyro_rate_t *gyro_rate, dt_t dt, acc_angle_t *acc_angle)
{
  double rate_pitch, rate_roll, rate_yaw;

  rate_pitch = gyro_rate->pitch * dt.t_period;
  rate_roll = gyro_rate->roll * dt.t_period;
  rate_yaw = gyro_rate->yaw * dt.t_period;
  
  gyro_angle->pitch = (gyro_angle->pitch + rate_pitch) * 0.95 + acc_angle->pitch * 0.05;
  gyro_angle->roll = (gyro_angle->roll + rate_roll) * 0.95 + acc_angle->roll * 0.05;
  gyro_angle->yaw = (gyro_angle->yaw + rate_yaw);
}
void init(dt_t *dt)
{
  dt->t_prev = micros();
}
void calc(dt_t *dt)
{
  dt->t_now = micros();
  while(micros() - dt->t_prev < 1000); dt->t_now = micros();
  dt->t_period = (dt->t_now - dt->t_prev) * 0.000001F;
  dt->t_prev = dt->t_now;
}
