#include "auto_park.h"

void readEncode(MOTOR *motor)
{
	
}

void move(MOTOR *motor)
{
	if( !motor->pid.process_flag )
	{
		motor->pid.process_flag = true;
	}
	if( motor->pid.sample_flag )
	{
		
	}
}

/* global function */
void process_data(MACHINE *machine)
{
	char cmd = machine->cdc.receive[0];
	switch(cmd)
	{
		case 'D':
			break;
		case 'A':
			break;
		case 'R':
			break;

	}
}
