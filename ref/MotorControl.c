#include "MotorControl.h"
#include "string.h"

void ProcessData(CNC *cnc)
{
	switch (cnc->data.ReceiveBuff[0])
	{
	case 'H':
		cnc->x_axis.home = false;
		cnc->y_axis.home = false;
		cnc->z_axis.home = false;
		cnc->Mode = 1; // mode goto home
		break;
	case 'G':
		if(cnc->data.ReceiveBuff[3] == 'Z')
		{			
			uint8_t temp;
			float z;
			sscanf(cnc->data.ReceiveBuff, "G0%uZ%f ",&temp, &z);
			memset(cnc->data.ReceiveBuff, 0, sizeof(cnc->data.ReceiveBuff));
			if(z >= cnc->z_max)
				cnc->z_axis.next = 0;
			else
				cnc->z_axis.next = cnc->z_max - z;
			cnc->drill.enb = (temp == 1) ? true : false;
			cnc->Mode = 3; // move z
		}
		else
		{
			uint8_t temp;
			float x, y;
			sscanf(cnc->data.ReceiveBuff, "G0%uX%fY%f ",&temp, &x, &y);
			memset(cnc->data.ReceiveBuff, 0, sizeof(cnc->data.ReceiveBuff));
			cnc->x_axis.next = x;
			cnc->y_axis.next = y;
			//*cnc->x_axis.enc = 0;
			cnc->Mode = (temp == 0) ? 4 : 5;
		}
		break;
	/*case 'G':
		strcat(cnc->data.ReceiveBuff, " ");
		float x = 0, y = 0;
		int temp;
		sscanf(cnc->data.ReceiveBuff, "G0%uX%fY%f ", &temp, &x, &y);
		memset(cnc->data.ReceiveBuff, 0, sizeof(cnc->data.ReceiveBuff));
		cnc->x_axis.next = x;
		cnc->y_axis.next = y;
		cnc->data.temp = temp;
		if(cnc->data.temp == 1)
			cnc->drill.enb =true;
		else
			cnc->drill.enb = false;
		cnc->Mode = 3; // check drill
		break;*/	
	case 'E': //stop
		cnc->Mode = 9;
		break;
	case 'Z': // z edit
		if(cnc->data.ReceiveBuff[1] == 'S')
		{
			cnc->z_max = cnc->z_axis.pos / cnc->z_axis.mm_pulse;
			cnc->Mode = 8;
		}
		else
		{
			float z_step;
			sscanf(cnc->data.ReceiveBuff, "Z%f", &z_step);
			memset(cnc->data.ReceiveBuff, 0, sizeof(cnc->data.ReceiveBuff));
			cnc->z_axis.next += z_step;
			cnc->Mode = 10;
		}		
	 	break;
	case 'S': // setting
		if(cnc->data.ReceiveBuff[1] == 'X')
		{
			float ki, kp, kd;
			int speed;
			sscanf(cnc->data.ReceiveBuff, "SXKp%fKi%fKd%fS%d", &kp, &ki, &kd, &speed);
			memset(cnc->data.ReceiveBuff, 0, sizeof(cnc->data.ReceiveBuff));
			cnc->x_axis.Kp = kp;
			cnc->x_axis.Ki = ki;
			cnc->x_axis.Kd = kd;
			cnc->x_axis.speed = speed;
		}
		else if(cnc->data.ReceiveBuff[1] == 'Y')
		{
			float ki, kp, kd;
			int speed;
			sscanf(cnc->data.ReceiveBuff, "SYKp%fKi%fKd%fS%d", &kp, &ki, &kd, &speed);
			memset(cnc->data.ReceiveBuff, 0, sizeof(cnc->data.ReceiveBuff));
			cnc->y_axis.Kp = kp;
			cnc->y_axis.Ki = ki;
			cnc->y_axis.Kd = kd;
			cnc->y_axis.speed = speed;
		}
		else if(cnc->data.ReceiveBuff[1] == 'Z')
		{
			float ki, kp, kd;
			int speed;
			sscanf(cnc->data.ReceiveBuff, "SZKp%fKi%fKd%fS%d", &kp, &ki, &kd, &speed);
			memset(cnc->data.ReceiveBuff, 0, sizeof(cnc->data.ReceiveBuff));
			cnc->z_axis.Kp = kp;
			cnc->z_axis.Ki = ki;
			cnc->z_axis.Kd = kd;
			cnc->z_axis.speed = speed;
		}
		else if(cnc->data.ReceiveBuff[1] == 'E')
		{
			uint8_t err_x, err_y, err_z;
			sscanf(cnc->data.ReceiveBuff, "SEX%dY%dZ%d", &err_x, &err_y, &err_z);
			memset(cnc->data.ReceiveBuff, 0, sizeof(cnc->data.ReceiveBuff));
			cnc->x_axis.ERROR = err_x;
			cnc->y_axis.ERROR = err_y;
			cnc->z_axis.ERROR = err_z;
		}
		else if(cnc->data.ReceiveBuff[1] == 'O')
		{
			float z_max, step;
			sscanf(cnc->data.ReceiveBuff, "SOZ%fS%f", &z_max, &step);
			memset(cnc->data.ReceiveBuff, 0, sizeof(cnc->data.ReceiveBuff));
			cnc->z_max = z_max;
			cnc->step = step;
		}
		else if(cnc->data.ReceiveBuff[1] == 'U')
		{
			int speed;
			float thickness;
			sscanf(cnc->data.ReceiveBuff, "SUT%fS%d", &thickness, &speed);
			memset(cnc->data.ReceiveBuff, 0, sizeof(cnc->data.ReceiveBuff));
			cnc->thickness = thickness;
			cnc->speed = speed;
		}
		cnc->Mode = 8;
		break;
	default:
		break;
	}
	cnc->data.index = 0;
}

void PWM(AXIS *axis)
{
	float duty = axis->pwm;
	if (duty > 1) duty = 1;
	else if(duty ==0)  duty = 0;
	else if (duty < -1) duty = -1;
	int16_t pwm = duty*axis->speed;
		
	if (pwm > 0)
		{
			__HAL_TIM_SetCompare(axis->htim_motor, axis->CHANNEL, 100 - pwm);
			HAL_GPIO_WritePin(axis->GPIO_DIR, axis->PIN_DIR, GPIO_PIN_SET);
		}
	else if (pwm == 0)
		{
			__HAL_TIM_SetCompare(axis->htim_motor, axis->CHANNEL, 0);
			HAL_GPIO_WritePin(axis->GPIO_DIR, axis->PIN_DIR, GPIO_PIN_RESET);
			axis->pwm = 0;
		}
	else if (pwm < 0)
		{
			pwm *= -1;
			__HAL_TIM_SetCompare(axis->htim_motor, axis->CHANNEL, pwm);
			HAL_GPIO_WritePin(axis->GPIO_DIR, axis->PIN_DIR, GPIO_PIN_RESET);

		}
}

// void readEncoder(TIM_HandleTypeDef* htim, int *Pos)
// {
// 	//*Pos += __HAL_TIM_GET_COUNTER(htim);
// 	*Pos += (int16_t)(htim->Instance->CNT);
// 	htim->Instance->CNT=0;
// }

void readEncoder(AXIS *axis)
{
	// axis->pos += (int16_t)(axis->htim_enc->Instance->CNT);
	// axis->htim_enc->Instance->CNT = 0;

	axis->pos += (int16_t)*axis->enc;
	*axis->enc = 0;
}

void sample(AXIS * axis)
{
	if(axis->pid_process)
	{
		axis->time_sample++;
		if( axis->time_sample >= T_SAMPLE )
		{
			//readEncoder(axis->htim_enc, &axis->pos);
			//axis->pos += (int16_t)(axis->htim_enc->Instance->CNT);
			//axis->htim_enc->Instance->CNT = 0;
			//PID_control(axis->setpoint, axis);
			axis->time_sample = 0;			
			axis->sample_flag = true;
		}
	}
}

void PID_control(int sp, AXIS *pid)
{
	int e_pulse;
	float e_mm;
	e_pulse = sp - pid->pos;
	e_mm = e_pulse * pid->mm_pulse;
	pid->I_part += TS*e_mm;
	pid->pwm = pid->Kp*e_mm + pid->Ki*pid->I_part + pid->Kd*(e_mm - pid->e_pre)/TS;
	pid->e_pre = e_mm;
	if(int_abs(e_pulse) < pid->ERROR)
	{
		pid->finish = true;
		pid->pwm = 0;
		pid->e_pre = 0;
		//pid->pid_process = false;
	}
}

void HOME(AXIS *axis)
{
	if( HAL_GPIO_ReadPin(axis->GPIO_HOME, axis->PIN_HOME) == 1)
		{
			axis->pwm = -1;
			PWM(axis);
			axis->home = false;
		}
	else
		{
			axis->pwm = 0;
			PWM(axis);
			axis->home = true;
		}
}
void move(AXIS *axis, float pos)
{
	if( axis->pid_process == false)
	{
		axis->setpoint = (int)(pos*axis->mm_pulse);
	  	axis->pid_process = true;
	}
	if(axis->sample_flag)
	{
		//readEncoder(axis->htim_enc, &axis->pos);
		readEncoder(axis);
		PID_control(axis->setpoint, axis);
		axis->sample_flag = false;
		axis->time_sample = 0;
	}
	PWM(axis);	
	if( axis->finish)
	{
		axis->pwm = 0;
		PWM(axis);
		axis->pid_process = false;		
		axis->time_sample = 0;		
	}
}
void moveGcode(AXIS *pAxis)
{
	pAxis->setpoint = (int)(pAxis->last * pAxis->mm_pulse);
	pAxis->pid_process = true;
	while( pAxis->pid_process)
	{
		if(pAxis->sample_flag)
		{
			//readEncoder(pAxis->htim_enc, &pAxis->pos);
			readEncoder(pAxis);
			PID_control(pAxis->setpoint, pAxis);
			pAxis->sample_flag = false;
		}
		PWM(pAxis);	
		if( pAxis->finish)
		{
			pAxis->pwm = 0;
			PWM(pAxis);
			pAxis->pid_process = false;		
			pAxis->time_sample = 0;		
			pAxis->finish = false;
		}
	}
}
void drawLine(AXIS *pXAxis, AXIS *pYAxis)
{
	// declare variable
	int dx = round((pXAxis->next - pXAxis->last)*100);
	int dy = round((pYAxis->next - pYAxis->last)*100);
	int longest = int_max(int_abs(dx), int_abs(dy));
	int shortest = int_min(int_abs(dx), int_abs(dy));
	int error = - longest;
	int slope = shortest<<1;
	int max = longest<<1;
	int threshold = 0;
	bool swapXY = true;
	
	// swap x and y if dy > dx
	if(int_abs(dx) > int_abs(dy))
		swapXY = false;
	
	// Bresenham Algorithm
	for(int i = 0; i < longest/(STEP * 100) ; i++)
	{
		if(swapXY)
		{
			if( dy > 0)
				pYAxis->last += STEP;
			else
				pYAxis->last -= STEP;
			moveGcode(pYAxis);
		}
		else
		{
			if( dx > 0)
				pXAxis->last += STEP;
			else
				pXAxis->last -= STEP;
			moveGcode(pXAxis);
		}
		error = error + slope;
		if(error >= threshold)
		{
			error = error - max;
			if(!swapXY)
			{
				if( dy > 0)
					pYAxis->last += STEP;
				else
					pYAxis->last -= STEP;
				moveGcode(pYAxis);
			}
			else
			{
				if( dx > 0)
					pXAxis->last += STEP;
				else
					pXAxis->last -= STEP;
				moveGcode(pXAxis);
			}
		}
	}
}

void runDrill(DRILL *drill, float pwm)
{
	if (pwm > 0)
	{
		__HAL_TIM_SetCompare(drill->htim_motor, drill->CHANNEL, 100 - pwm);
		HAL_GPIO_WritePin(drill->GPIO_DIR, drill->PIN_DIR, GPIO_PIN_SET);
	}
	else if (pwm == 0)
	{
		__HAL_TIM_SetCompare(drill->htim_motor, drill->CHANNEL, 0);
		HAL_GPIO_WritePin(drill->GPIO_DIR, drill->PIN_DIR, GPIO_PIN_RESET);
	}
	else if (pwm < 0)
	{
		pwm *= -1;
		__HAL_TIM_SetCompare(drill->htim_motor, drill->CHANNEL, pwm);
		HAL_GPIO_WritePin(drill->GPIO_DIR, drill->PIN_DIR, GPIO_PIN_RESET);
	}
}
