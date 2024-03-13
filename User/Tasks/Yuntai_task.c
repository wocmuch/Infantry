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
		PID_T yaw_pid;
		float yaw_target;
		float yaw_current;
		float yaw_offset;
		float yaw_slove;
		float yaw_last;
		
		extern AHRSData_Packet_t AHRSData_Packet;
void yuntaiTask(void const * argument)
{
	bsp_imu_init();
	PID_Init(&yaw_pid,PID_POSITION_NULL,0.06125f,0.f,0.f);
	PID_Sst_Out_Limit(&yaw_pid,150);
	PID_Sst_Integral_Limit(&yaw_pid,20);
//	while(get_global_time()/5000!=1)
//		osDelay(10);
//	float yaw_now = cimu.Yaw;
//	osDelay(500);
//	float yaw_last = cimu.Yaw;
//	while(1){
//		#define abs(x)        ((x>0)? (x): (-x))
//		if(abs(yaw_now-yaw_last)<0.5){ 
//			yaw_offset=yaw_now;
//			elog_raw_output("%.2f   time_now%d \n",yaw_offset,get_global_time());
//			break;
//		}
//		else{
//			yaw_last = yaw_now;
//			osDelay(500);
//			yaw_now = cimu.Yaw;
//		}
//	}
	while(1)
	{	
		
		yaw_current=(int)DJ_Get_Motor_Position(DJ_CAN1_6020_M6,DJ_M6020)%360;
	//	yaw_current=cimu.Yaw-yaw_offset;
//		yaw_current-=180;
		//elog_raw_output("%.2f,%.2f,%.2f\n",cimu.Yaw,yaw_current,yaw_target);
		//c板 逆时针正
		
		
		//yaw_current=my_rc.yaw_target;
		
		PID_Sst_Target(&yaw_pid,yaw_target);
		PID_Sst_Present(&yaw_pid,yaw_current);
		PID_Hander(&yaw_pid,2);
		
		//DJ_Set_Motor_Speed(DJ_CAN1_6020_M6,-yaw_pid.parameter.out,DJ_M6020);

//		DJ_Set_Motor_Speed(DJ_CAN1_6020_M6,-20,DJ_M6020);//大于0 为逆时针旋转，小于0顺时针
		//这里是云台线程
		osDelay(10);
	}
}









