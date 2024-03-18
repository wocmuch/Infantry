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
#include "my_usart_recieve.h"
		extern rc_info_t rc;
		extern my_usart_rc_t my_rc;
void motor_task(void const * argument)
{
	while(1)
	{	
/*
		���id���ã�
			���̵����can1��1 2 3 4 ע����3�е�С���⣬��ʱ���Ī���鴤
			��̨�����ƫ����can1��6 ������can2��5
			���������can2��3
			��������can2��1 2
*/
//ң����ӳ��
		my_rc.vx=rc.ch2/5;
		my_rc.vy=(-1)*rc.ch1/5;
		
		float k_shot_speed =rc.ch15/3;
		if(rc.ch8==2){
			my_rc.vw=rc.ch14/1.24 ;
		}else if(rc.ch8==1){
			my_rc.vw=1500;
		}else if(rc.ch8==3){
			my_rc.vw=-(1500);
		}
		
		my_rc.pitch_speed=rc.ch3/10;
		if(my_rc.pitch_target>100)my_rc.pitch_target=100;else if(my_rc.pitch_target<-100) my_rc.pitch_target=-100;else my_rc.pitch_target=my_rc.pitch_target;
		my_rc.pitch_target=rc.ch13/10.f;
		
		my_rc.yaw_speed=rc.ch4/10.f;
		
		my_rc.yaw_target=rc.ch4/40.f;
		
		if(rc.ch11==3){
			my_rc.shot_frequency=00;
			my_rc.shot_speed=0;
		}
		else if(rc.ch11==2)
		{
			my_rc.shot_frequency=00;
			my_rc.shot_speed=1;
		}else
		{
			my_rc.shot_frequency=60;
			my_rc.shot_speed=1;
		}
//		//��ת	С���� 
	
		
		
		//���� ������
		if(my_rc.shot_speed==1){

			#define abs(x)        ((x>0)? (x): (-x))
			DJ_Set_Motor_Speed(DJ_CAN2_M0,abs(k_shot_speed),DJ_M3508);
			DJ_Set_Motor_Speed(DJ_CAN2_M1,(-1)*abs(k_shot_speed),DJ_M3508);
			
			static uint8_t shot_disable = 0;
			
			if(abs(DJ_Get_Motor_Speed(DJ_CAN2_M2,DJ_M2006)-0)<5&&my_rc.shot_frequency*0.8>0){
				shot_disable++;
			}else {
				shot_disable = 0;
			}
			
			if(shot_disable>50){
				float temp_pos =DJ_Get_Motor_Position(DJ_CAN2_M2,DJ_M2006);
				uint8_t i =200;
				while(i--){
					DJ_Set_Motor_Position(DJ_CAN2_M2,temp_pos-60,DJ_M2006);
					dj_motor_handler(5,2);
					osDelay(1);
				}
				shot_disable=0;
			}else{
				DJ_Set_Motor_Speed(DJ_CAN2_M2,my_rc.shot_frequency*0.8,DJ_M2006);
			}
		}else
		{
			DJ_Set_Motor_Speed(DJ_CAN2_M0,0,DJ_M3508);
			DJ_Set_Motor_Speed(DJ_CAN2_M1,0,DJ_M3508);		
		}
		
		dj_motor_handler(5,2);
		osDelay(1);
	}
}

