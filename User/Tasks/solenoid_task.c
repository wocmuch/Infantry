#include "cmsis_os.h"
#include "can.h"
#include "dj_motor_driver.h"
#include "stdio.h"
#include "pid.h"
#include "shell.h"
#include "commend.h"
#include <stdarg.h>
#include <stdlib.h>
#include "elog.h"
#include "bsp_sbus.h"
#include "a_led_buzzer_key.h"
#include "my_usart_recieve.h"
		extern PID_T pid1;
		extern PID_T pid2;
		extern float p,i,d;
		extern rc_info_t rc;
		extern my_usart_rc_t my_rc;
		
float auto_yaw_target=0;
float auto_pitch_target=0;
float auto_yaw_kp=1;
float auto_pitch_kp=0.002;
void solenoid_task(void const * argument)
{
	while(1)
	{	 
		if(rc.ch10==3)
		{
				if(rc.ch8==2)
				{
					DJ_Set_Motor_Speed(DJ_CAN1_6020_M6,-my_rc.yaw_speed,DJ_M6020);
				}
				else
				{
					float k_little = rc.ch16/3000.f;
					DJ_Set_Motor_Speed(DJ_CAN1_6020_M6,-my_rc.vw*(152/2/1000.f)*(0.4f+k_little)+-my_rc.yaw_speed,DJ_M6020);
				}	
//		//¸©Ñö
			DJ_Set_Motor_Position(DJ_CAN2_6020_M5,my_rc.pitch_target,DJ_M6020);
		
			osDelay(1);
		}
		else if(rc.ch10!=3&&my_rc.armer_flag==1)
		{
			
			auto_yaw_target   = auto_yaw_kp   *  my_rc.error_x;
			auto_pitch_target = auto_pitch_kp *  my_rc.error_y ;
			DJ_Set_Motor_Position(DJ_CAN2_6020_M5 , my_rc.pitch_target + auto_pitch_target , DJ_M6020);//pitch
			DJ_Set_Motor_Speed(		DJ_CAN1_6020_M6 ,                      auto_yaw_target ,   DJ_M6020);//yaw
			//dj_motor_handler(5,2);
			osDelay(1);
			
			
		}
		osDelay(1);
	}
}






