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
		extern rc_info_t rc;
void led_task(void const * argument)
{
	while(1)
	{	
		LED_ALL_RUN();
		osDelay(100);
	}
}
