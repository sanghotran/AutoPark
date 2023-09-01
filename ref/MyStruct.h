#ifndef __MYSTRUCT_H
#define __MYSTRUCT_H


#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

#define T_SAMPLE 1
#define Z_MAX 11



typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float I_part;
	int e_pre;
	uint8_t time_sample;
	uint32_t pos;
	float pwm;
	int setpoint;

	int speed;
	
	//TIM_HandleTypeDef* htim_enc;
	uint32_t *enc;
	
	TIM_HandleTypeDef* htim_motor;
	uint32_t CHANNEL;
	
	GPIO_TypeDef* GPIO_DIR;
	uint16_t PIN_DIR;
	
	GPIO_TypeDef* GPIO_HOME;
	uint16_t PIN_HOME;
	
  	float mm_pulse;
	
	bool finish;
	
	bool pid_process;
	bool sample_flag;
	
	bool home;
	
	float next;
	float last;
	float feedback;

	uint8_t ERROR;

} AXIS;

typedef struct MyStruct
{
	TIM_HandleTypeDef* htim_motor;
	uint32_t CHANNEL;
	
	GPIO_TypeDef* GPIO_DIR;
	uint16_t PIN_DIR;

	bool enb;
} DRILL;


typedef struct
{
	char Receive;
	uint8_t index;

	char TransBuff[20];
	char ReceiveBuff[40];
	uint8_t temp;
} DATA;


typedef struct 
{
	AXIS x_axis;
	AXIS y_axis;
	AXIS z_axis;

	uint8_t Mode;
	float step;
	float z_max;
	
	float thickness;
	uint8_t speed;
	DRILL drill;

	DATA data;
} CNC;


#endif
