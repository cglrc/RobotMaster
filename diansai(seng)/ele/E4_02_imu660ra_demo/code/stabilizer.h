#ifndef _STABILIZER_H_
#define _STABILIZER_H_

#include <stdint.h>
#include <stdbool.h>
#include "zf_common_headfile.h"
/* ------------- YAW 轴 ------------- */
void stab_yaw_init(float kp, float ki, float kd);
void stab_yaw_set_target(float target_deg);
void stab_yaw_run_200Hz(void);

extern float stab_yaw_target_deg;
extern float stab_yaw_current_deg;
extern float stab_yaw_error_deg;
extern float stab_yaw_pid_out;

/* ------------- PITCH 轴 ------------- */
void stab_pitch_init(float kp, float ki, float kd);
void stab_pitch_set_target(float target_deg);
void stab_pitch_run_200Hz(void);

extern float stab_pitch_target_deg;
extern float stab_pitch_current_deg;
extern float stab_pitch_error_deg;
extern float stab_pitch_pid_out;

#endif