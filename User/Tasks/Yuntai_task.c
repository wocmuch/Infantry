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
#include "dj_motor_driver.h"
#include "bsp_imu.h"
#include "my_usart_recieve.h"

		extern rc_info_t rc;
		extern imu_t imu;
		extern cimu_t cimu;
		extern my_usart_rc_t my_rc;

		float yaw_target;
		float yaw_current;
		float yaw_offset;
		float yaw_slove;
		float yaw_last;
		
		extern AHRSData_Packet_t AHRSData_Packet;
		
		float auto_yaw_target=0;
float auto_pitch_target=0;
float auto_yaw_kp=0.45;
float auto_pitch_kp=1.2;
uint8_t pitch_up_flag=0;
float pitch_angle_now=0;
void yuntaiTask(void const * argument)
{
	bsp_imu_init();

	while(1){
		
			DJ_Set_Motor_Speed(DJ_CAN1_6020_M6,40,DJ_M6020);
			dj_motor_handler(5,2);
			osDelay(1);
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0){
				uint8_t i =50;
				yaw_offset=(int)DJ_Get_Motor_Position(DJ_CAN1_6020_M6,DJ_M6020)%360;
				while(i--){
					DJ_Set_Motor_Position(DJ_CAN1_6020_M6,yaw_offset,DJ_M6020);
					dj_motor_handler(5,2);
					osDelay(1);
				}
				break;
			}
	}
	while(1)
	{		
		
		if(rc.ch10!=3&&my_rc.armer_flag==1)
		{
			float k_little = rc.ch16/3000.f;
			auto_yaw_target   = -auto_yaw_kp   *  my_rc.error_x;
			auto_pitch_target = -auto_pitch_kp *  my_rc.error_y ;
			DJ_Set_Motor_Position(DJ_CAN2_6020_M5 , my_rc.pitch_target+ auto_pitch_target , 																		 DJ_M6020);//pitch
			DJ_Set_Motor_Speed(		DJ_CAN1_6020_M6 , -my_rc.vw*(152/2/1000.f)*(0.4f+k_little)+ -my_rc.yaw_speed+auto_yaw_target ,   DJ_M6020);//yaw
			//dj_motor_handler(5,2);
			yaw_current=(int)DJ_Get_Motor_Position(DJ_CAN1_6020_M6,DJ_M6020)%360-(int)yaw_offset;
			osDelay(1);
		}
		else
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
//			if(my_rc.pitch_speed==0){
//				if(pitch_up_flag)
//						pitch_angle_now = DJ_Get_Motor_Position(DJ_CAN2_6020_M5,DJ_M6020);
//					DJ_Set_Motor_Position(DJ_CAN2_6020_M5,pitch_angle_now,DJ_M6020);
//					pitch_up_flag=0;
//			}else{
//				DJ_Set_Motor_Speed(DJ_CAN2_6020_M5,my_rc.pitch_speed,DJ_M6020);
//				pitch_up_flag=1;
//			}
			DJ_Set_Motor_Position(DJ_CAN2_6020_M5,my_rc.pitch_target,DJ_M6020);
			yaw_current=(int)DJ_Get_Motor_Position(DJ_CAN1_6020_M6,DJ_M6020)%360-(int)yaw_offset;
			osDelay(1);
		}
		

	}
}









