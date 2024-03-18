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
		

void solenoid_task(void const * argument)
{
	while(1)
	{	 
		osDelay(10);
	}
}






