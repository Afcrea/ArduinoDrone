#ifndef __04_REMOTE_H__
#define __04_REMOTR_H__

void init(fsi6b_t fsi6b);
void read_fsi6(fsi6b_pwm_t fsi6b_pwm, target_angle_t *target_angle, throttle_t *throttle );

#endif