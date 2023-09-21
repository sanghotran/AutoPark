#include "auto_park.h"

void PWM(MOTOR *motor)
{
	float duty = motor->pid.pwm;
	if (duty > 100) duty = 100;
	else if (duty < -100) duty = -100;

	if(duty > 0)
	{
		__HAL_TIM_SetCompare(motor->pin.htim, motor->pin.channel, 100 - duty);
		HAL_GPIO_WritePin(motor->pin.gpio_dir, motor->pin.dir, 1);
	}
	else if(duty == 0)
	{
		__HAL_TIM_SetCompare(motor->pin.htim, motor->pin.channel, 0);
		HAL_GPIO_WritePin(motor->pin.gpio_dir, motor->pin.dir, 0);
	}
	else if(duty < 0)
	{
		duty *= -1;
		__HAL_TIM_SetCompare(motor->pin.htim, motor->pin.channel, duty);
		HAL_GPIO_WritePin(motor->pin.gpio_dir, motor->pin.dir, 0);
	}
}

void readEncoder(MOTOR *motor)
{
	motor->pid.pos += (int16_t)*motor->pin.enc;
	*motor->pin.enc = 0;
}


void PID_control(MOTOR *motor)
{
	int e;
	e = motor->pid.setpoint - motor->pid.pos;
	motor->pid.I_part += e*TS;
	motor->pid.pwm = motor->pid.kp*e + motor->pid.ki*motor->pid.I_part + motor->pid.kd*(e - motor->pid.erro_pre)/TS;
	motor->pid.erro_pre = e;
	if(abs(e) < motor->pid.ERROR)
	{
		motor->pid.finish_flag = true;
		motor->pid.pwm = 0;
		motor->pid.erro_pre = 0;
	}
}

void home(MOTOR *motor)
{
	if(HAL_GPIO_ReadPin(motor->pin.gpio_home, motor->pin.home) == 1)
	{
		motor->pid.pwm = -1;
		PWM(motor);
		motor->pid.home_flag = false;
	}
	else
	{
		motor->pid.pwm = 0;
		PWM(motor);
		motor->pid.home_flag = true;
	}
}

void move(MOTOR *motor)
{
	if( !motor->pid.process_flag )
	{
		motor->pid.process_flag = true;
	}
	if( motor->pid.sample_flag )
	{
		readEncoder(motor);
		PID_control(motor);
		motor->pid.sample_flag = false;
		motor->pid.time_sample = 0;
	}
	PWM(motor);
	if(motor->pid.finish_flag)
	{
		motor->pid.pwm = 0;
		PWM(motor);
		motor->pid.process_flag = false;
		motor->pid.time_sample = 0;
	}
}

/* global function */
void process_data(MACHINE *machine)
{
	char cmd = machine->cdc.receive[0];
	switch(cmd)
	{
		case 'C':
			machine->mode = 1; // mode connect
			break;
		case 'D':
			machine->mode = 2; // mode disconnect
			break;
		case 'A':
			machine->mode = 3; // mode add card
			break;
		case 'R':
			machine->mode = 4; // mode run motor
			break;

	}
}
void process_mode(MACHINE *machine)
{
	switch(machine->mode)
	{
		case 2: // mode disconnect
			sprintf((char*)machine->cdc.trans, "DISCONNECT\r");
			CDC_Transmit_FS(machine->cdc.trans, strlen((const char*)machine->cdc.trans));
			machine->mode = 0; // mode idle disconnect
			break;
		case 3: // mode add card
			machine->mode = 1; // mode idle connect
			break;
		case 4: // mode run motor
			machine->mode = 1; // mode idle connect
			break;
		case 5: // mode scan
			machine->mode = 1; // mode idle connect
			break;
	}
}

void sample(MOTOR *motor)
{
	if(motor->pid.process_flag)
	{
		motor->pid.time_sample++;
		if(motor->pid.time_sample >= T_SAMPLE)
		{
			motor->pid.time_sample = 0;
			motor->pid.sample_flag = true;
		}
	}
}
