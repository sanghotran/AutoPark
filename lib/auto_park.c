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
		//motor->pid.pwm = 0;
		motor->pid.erro_pre = 0;
	}
}

void home(MOTOR *motor)
{
	while(HAL_GPIO_ReadPin(motor->pin.gpio_home, motor->pin.home) == 1)
	{
		motor->pid.pwm = -100;
		PWM(motor);
	}
	motor->pid.pwm = 0;
	PWM(motor);
	motor->pid.pos = 0;
	*motor->pin.enc = 0;
}

void move_home(MACHINE *machine)
{
	home(&machine->out);
	home(&machine->up);
	home(&machine->roll);
}

void move(MOTOR *motor, int setpoint)
{
	motor->pid.process_flag = true;
	motor->pid.setpoint = setpoint;
	while(motor->pid.process_flag)
	{
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
			//motor->pid.time_sample = 0;
		}
	}
}

void slot_to_pos(MACHINE *machine)
{
	switch(machine->slot)
	{
		case '1':
			break;
		case '2':
			break;
		case '3':
			break;
		case '4':
			break;
		case '5':
			break;
		case '6':
			break;
		case '7':
			break;
		case '8':
			break;

	}
}

void run_in(MACHINE *machine)
{
	slot_to_pos(machine);
	move(&machine->out, machine->out.ref);
	move(&machine->up, machine->up.ref);	
	move(&machine->out, machine->out.ref);
	move(&machine->up, machine->up.target);
	move(&machine->roll, machine->roll.target);
	move(&machine->out, machine->out.target);
	move(&machine->up, machine->up.target - machine->up.ref);
	move_home(machine);
}

void run_out(MACHINE *machine)
{
	slot_to_pos(machine);
	move(&machine->up, machine->up.target - machine->up.ref);
	move(&machine->roll, machine->roll.target);
	move(&machine->out, machine->out.target);
	move(&machine->up, machine->up.target);
	move_home(machine);
	move(&machine->up, machine->up.ref);
	move(&machine->out, machine->out.target);
	move(&machine->up, 0);
	move_home(machine);
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
		case 'I':
			machine->mode = 4; // mode run in motor
			machine->slot = machine->cdc.receive[1];
			break;
		case 'O':
			machine->mode = 5; // mode run out motor
			machine->slot = machine->cdc.receive[1];
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
		case 4: // mode run in motor: up -> out -> up -> roll -> out -> up -> out -> up -> roll -> out
			machine->mode = 1; // mode idle connect
			run_in(machine);
			sprintf((char*)machine->cdc.trans, "DONE\r");
			CDC_Transmit_FS(machine->cdc.trans, strlen((const char*)machine->cdc.trans));
			break;
		case 5: // mode run out motor: out -> up -> roll -> out -> up -> out -> up -> roll -> out -> up
			machine->mode = 1; // mode idle connect
			run_out(machine);
			break;
		case 6: // mode scan
			machine->mode = 1; // mode idle connect
			CDC_Transmit_FS(machine->id, strlen((const char*)machine->id));
			break;
		case 7: // mode go home
			machine->mode = 0; // mode idle disconnect
			move_home(machine);
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
