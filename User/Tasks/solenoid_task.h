#ifndef __SOLENOID_TASK_H
#define __SOLENOID_TASK_H

#include "main.h"
#include "struct_typedef.h"
#include "pid.h"
#include "bsp_sbus.h"

/**
  * @brief          led rgb task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          led RGBÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void solenoid_task(void const * argument);

#endif