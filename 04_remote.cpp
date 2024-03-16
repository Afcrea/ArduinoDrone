#include <Wire.h>
#include "00_drone.h"

#define RANGE 30

extern fsi6b_ch_t fsi6b_ch;
extern fsi6b_time_t fsi6b_time;
extern fsi6b_pwm_t fsi6b_pwm;

void init(fsi6b_t fsi6b)
{
  pinMode(fsi6b.Throttle_pin, INPUT);
  pinMode(fsi6b.Yaw_pin, INPUT);
  pinMode(fsi6b.Pitch_pin, INPUT);
  pinMode(fsi6b.Roll_pin, INPUT);
  pinMode(fsi6b.drive, INPUT);

  PCICR = 0x04;
  PCMSK2 |= 0b00011111;
}

ISR(PCINT2_vect) 
{
  fsi6b_time.tmr_base = micros(); // Definition of timer0
  
  /////////// channel 1 ///////////
  // Rising
  if( (fsi6b_ch.ch_roll == 0) && (PINK & B00000001) ) { 
    fsi6b_ch.ch_roll = 1; // change statement
    fsi6b_time.tmr_roll = fsi6b_time.tmr_base; // Time of rising
  }
  // Falling
  else if( (fsi6b_ch.ch_roll == 1) && !(PINK & B00000001) ) {
    fsi6b_ch.ch_roll = 0; // change statement
    fsi6b_pwm.pwm_roll = fsi6b_time.tmr_base - fsi6b_time.tmr_roll; 
  }
  
  /////////// channel 2 ///////////
  // Rising
  if( (fsi6b_ch.ch_pitch == 0) && (PINK & B00000010) ) {
    fsi6b_ch.ch_pitch = 1; // change statement
    fsi6b_time.tmr_pitch = fsi6b_time.tmr_base; // Time of rising
  }
  // Falling
  else if( (fsi6b_ch.ch_pitch == 1) && !(PINK & B00000010) ) {
    fsi6b_ch.ch_pitch = 0; // change statement
    fsi6b_pwm.pwm_pitch = fsi6b_time.tmr_base - fsi6b_time.tmr_pitch; 
  }
  
  /////////// channel 3 ///////////
  // Rising
  if( (fsi6b_ch.ch_throt == 0) && (PINK & B00000100) ) {
    fsi6b_ch.ch_throt = 1; // change statement
    fsi6b_time.tmr_throt = fsi6b_time.tmr_base; // Time of rising
  }
  // Falling
  else if( (fsi6b_ch.ch_throt == 1) && !(PINK & B00000100) ) { 
    fsi6b_ch.ch_throt = 0; // change statement
    fsi6b_pwm.pwm_throt = fsi6b_time.tmr_base - fsi6b_time.tmr_throt; 
  }
  
  /////////// channel 4 ///////////
  // Rising
  if( (fsi6b_ch.ch_yaw == 0) && (PINK & B00001000) ) { 
    fsi6b_ch.ch_yaw = 1; // change statement
    fsi6b_time.tmr_yaw = fsi6b_time.tmr_base; // Time of rising
  }
  // Falling
  else if( (fsi6b_ch.ch_yaw == 1) && !(PINK & B00001000) ) { 
    fsi6b_ch.ch_yaw = 0; // change statement
    fsi6b_pwm.pwm_yaw = fsi6b_time.tmr_base - fsi6b_time.tmr_yaw; 
  }

  if( (fsi6b_ch.drive == 0) && (PINK & B00010000) ) { 
    fsi6b_ch.drive = 1; // change statement
    fsi6b_time.tmr_drive = fsi6b_time.tmr_base; // Time of rising
  }
  // Falling
  else if( (fsi6b_ch.drive == 1) && !(PINK & B00010000) ) { 
    fsi6b_ch.drive = 0; // change statement
    fsi6b_pwm.pwm_drive = fsi6b_time.tmr_base - fsi6b_time.tmr_drive; 
  }
  
  
}

void read_fsi6(fsi6b_pwm_t fsi6b_pwm, target_angle_t *target_angle, throttle_t *throttle )
{
  if(fsi6b_pwm.pwm_pitch > 1450 & fsi6b_pwm.pwm_pitch < 1550)
  {
    fsi6b_pwm.pwm_pitch = 1500;
  }
  if(fsi6b_pwm.pwm_roll > 1450 & fsi6b_pwm.pwm_roll < 1550)
  {
    fsi6b_pwm.pwm_roll = 1500;
  }
  if(fsi6b_pwm.pwm_yaw > 1450 & fsi6b_pwm.pwm_yaw < 1550)
  {
    fsi6b_pwm.pwm_yaw = 1500;
  }
  
  throttle->value = fabs((fsi6b_pwm.pwm_throt - 1000.0) + 1000);
  target_angle->pitch = (fsi6b_pwm.pwm_pitch - 1500.0) * 30 / 500;
  target_angle->roll = (fsi6b_pwm.pwm_roll - 1500.0) * 30 / 500;
  target_angle->yaw = (fsi6b_pwm.pwm_yaw - 1500.0) * 50 / 500;
  
  if(fsi6b_pwm.pwm_pitch > 1490 & fsi6b_pwm.pwm_pitch < 1510)
  {
    fsi6b_pwm.pwm_pitch = 1500;
  }

  if(target_angle->pitch < -RANGE) target_angle->pitch = -RANGE;
  else if(target_angle->pitch > RANGE) target_angle->pitch = RANGE;

  if(target_angle->roll < -RANGE) target_angle->roll = -RANGE;
  else if(target_angle->roll > RANGE) target_angle->roll = RANGE;

  if(target_angle->yaw < -RANGE) target_angle->yaw = -RANGE;
  else if(target_angle->yaw > RANGE) target_angle->yaw = RANGE;
}


