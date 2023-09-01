#ifndef __AUTOPARK_H
#define __AUTOPARK_H

#include <stdbool.h>

typedef struct{
	int pos;
	float kp;
	float ki;
	float kd;
	float I_part;
	int erro_pre;
	int setpoint;
	float pwm;

	bool process_flag;
	bool sample_flag;
	int time_sample;

} PID;
typedef struct {
	PID pid;
	
} MOTOR;

void move(MOTOR *motor);

#endif
