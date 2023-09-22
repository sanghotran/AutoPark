#ifndef __AUTOPARK_H
#define __AUTOPARK_H

#include <stdbool.h>
#include <stdint.h>
#include <usbd_cdc_if.h>
#include <stm32f1xx_hal.h>


#define T_SAMPLE  3
#define TS 	0.003

typedef struct{
	int pos;
	float kp;
	float ki;
	float kd;
	float I_part;
	int erro_pre;
	int setpoint;
	float pwm;
	int ERROR;

	bool process_flag;
	bool sample_flag;
	int time_sample;
	bool finish_flag;
	bool home_flag;

} PID;

typedef struct{
	uint32_t *enc;
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	GPIO_TypeDef *gpio_dir;
	uint16_t dir;
	GPIO_TypeDef *gpio_home;
	uint16_t home;
	GPIO_TypeDef *gpio_buzzer;
	uint16_t buzzer;

} PIN;

typedef struct{
	uint8_t receive[64];
	uint8_t trans[64];

} DATA;

typedef struct {
	PID pid;
	PIN pin;
	int target;
	int ref;
	
} MOTOR;

typedef struct {
	MOTOR up;
	MOTOR roll;
	MOTOR out;

	DATA cdc;
	uint8_t id[64];

	uint8_t mode;
	uint8_t slot;
} MACHINE;

void process_data(MACHINE*);
void process_mode(MACHINE*);
void sample(MOTOR*);
//void move(MOTOR *motor);

#endif
