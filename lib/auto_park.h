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

typedef struct{
	char receive[64];
	char trans[64];

} DATA;

typedef struct {
	PID pid;
	
} MOTOR;

typedef struct {
	MOTOR up;
	MOTOR roll;
	MOTOR out;

	DATA cdc;
	DATA spi;
} MACHINE;

void process_data(MACHINE*);
void move(MOTOR *motor);

#endif
