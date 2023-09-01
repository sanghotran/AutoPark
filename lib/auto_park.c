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
