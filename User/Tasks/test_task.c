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
#include "math.h"
		extern PID_T pid1;
		extern PID_T pid2;
		extern float p,i,d;
		extern rc_info_t rc;
		extern imu_t imu;
		extern my_usart_rc_t my_rc;
		extern cimu_t cimu;
		extern void AHRSData2PC(void);
		extern float yaw_current;
void test_task(void const * argument)
{
	/* 初始化elog */
	elog_init();
	/* 设置每个级别的日志输出格*/
	elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
	//输出日志级别信息和日志TAG
	elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG);
	elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG);
	elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG);
	elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_LVL | ELOG_FMT_TAG);
	//除了时间、进程信息线程信息之外，其余全部输出
	//elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~(ELOG_FMT_TIME | ELOG_FMT_P_INFO | ELOG_FMT_T_INFO));
	
	elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL);
	elog_set_text_color_enabled(true);
	/* 启动elog */
	elog_start();
	dbus_uart_init();
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
	while(1)
	{	
		//AHRSData2PC();
		elog_raw_output("%d\n",HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0));
		//elog_raw_output("%d,%d,%d\n",my_rc.armer_flag,my_rc.error_x,my_rc.error_y);
		//elog_raw_output(" %.2f,%.2f \n",my_rc.pitch_target,DJ_Get_Motor_Position(DJ_CAN2_6020_M5,DJ_M6020));
		osDelay(200);
		
		
	}
}





