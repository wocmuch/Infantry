#ifndef __LED_H
#define __LED_H

#include "main.h"
#include "struct_typedef.h"
#include "tim.h"
typedef struct{
	uint16_t seceond;
	uint16_t msec;
}GLOBAL_TIME_t;
void global_time_handle();
uint32_t  get_global_time();
#define POWER_CTRL1_PIN GPIO_PIN_2
#define POWER_CTRL2_PIN GPIO_PIN_3
#define POWER_CTRL3_PIN GPIO_PIN_4
#define POWER_CTRL4_PIN GPIO_PIN_5
#define POWER_CTRL_PORT GPIOH

#define POWER_CTRL1_ENABLE() HAL_GPIO_WritePin(POWER_CTRL_PORT,POWER_CTRL1_PIN,GPIO_PIN_SET)
#define POWER_CTRL1_DISABLE() HAL_GPIO_WritePin(POWER_CTRL_PORT,POWER_CTRL1_PIN,GPIO_PIN_RESET)

#define POWER_CTRL2_ENABLE() HAL_GPIO_WritePin(POWER_CTRL_PORT,POWER_CTRL2_PIN,GPIO_PIN_SET)
#define POWER_CTRL2_DISABLE() HAL_GPIO_WritePin(POWER_CTRL_PORT,POWER_CTRL2_PIN,GPIO_PIN_RESET)

#define POWER_CTRL3_ENABLE() HAL_GPIO_WritePin(POWER_CTRL_PORT,POWER_CTRL3_PIN,GPIO_PIN_SET)
#define POWER_CTRL3_DISABLE() HAL_GPIO_WritePin(POWER_CTRL_PORT,POWER_CTRL3_PIN,GPIO_PIN_RESET)

#define POWER_CTRL4_ENABLE() HAL_GPIO_WritePin(POWER_CTRL_PORT,POWER_CTRL4_PIN,GPIO_PIN_SET)
#define POWER_CTRL4_DISABLE() HAL_GPIO_WritePin(POWER_CTRL_PORT,POWER_CTRL4_PIN,GPIO_PIN_RESET)
/****************************************以下是LED*******************************************************/


void LED_ALL_OFF(void);
void LED_RUN(void);
void LED_ALL_RUN(void);

//LEDA is red   LEDB is green
#define LEDA_GPIO_Port GPIOE
#define LEDA_Pin GPIO_PIN_11
#define LEDB_GPIO_Port GPIOF
#define LEDB_Pin GPIO_PIN_14
#define LED8_Pin GPIO_PIN_8
#define LED8_GPIO_Port GPIOG
#define LED7_Pin GPIO_PIN_7
#define LED7_GPIO_Port GPIOG
#define LED6_Pin GPIO_PIN_6
#define LED6_GPIO_Port GPIOG
#define LED5_Pin GPIO_PIN_5
#define LED5_GPIO_Port GPIOG
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOG
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOG

#define LED1_ON()     HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET)
#define LED1_OFF()    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET)
#define	LED1_TOG() 	  HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin)

#define LED2_ON()     HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET)
#define LED2_OFF()    HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET)
#define	LED2_TOG() 	  HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin)

#define LED3_ON()     HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET)
#define LED3_OFF()    HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET)
#define	LED3_TOG() 	  HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin)

#define LED4_ON()     HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET)
#define LED4_OFF()    HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET)
#define	LED4_TOG() 	  HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin)

#define LED5_ON()     HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET)
#define LED5_OFF()    HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET)
#define	LED5_TOG() 	  HAL_GPIO_TogglePin(LED5_GPIO_Port,LED5_Pin)

#define LED6_ON()     HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_RESET)
#define LED6_OFF()    HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_SET)
#define	LED6_TOG() 	  HAL_GPIO_TogglePin(LED6_GPIO_Port,LED6_Pin)

#define LED7_ON()     HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET)
#define LED7_OFF()    HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET)
#define	LED7_TOG() 	  HAL_GPIO_TogglePin(LED7_GPIO_Port,LED7_Pin)

#define LED8_ON()     HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,GPIO_PIN_RESET)
#define LED8_OFF()    HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,GPIO_PIN_SET)
#define	LED8_TOG() 	  HAL_GPIO_TogglePin(LED8_GPIO_Port,LED8_Pin)

#define LEDA_ON()     HAL_GPIO_WritePin(LEDA_GPIO_Port,LEDA_Pin,GPIO_PIN_RESET)
#define LEDA_OFF()    HAL_GPIO_WritePin(LEDA_GPIO_Port,LEDA_Pin,GPIO_PIN_SET)
#define	LEDA_TOG() 	  HAL_GPIO_TogglePin(LEDA_GPIO_Port,LEDA_Pin)

#define LEDB_ON()     HAL_GPIO_WritePin(LEDB_GPIO_Port,LEDB_Pin,GPIO_PIN_RESET)
#define LEDB_OFF()    HAL_GPIO_WritePin(LEDB_GPIO_Port,LEDB_Pin,GPIO_PIN_SET)
#define	LEDB_TOG() 	  HAL_GPIO_TogglePin(LEDB_GPIO_Port,LEDB_Pin)

/**********************************************************以下是蜂鸣器*****************************************************************************/
void Buzzer_Dji_Start(void);
void Note(int a,float Long);
void Buzzer_Times(int num,int time);

#define buzzer_htim  htim12  //84MHz

#define note_A  220
#define note_3A 110  
#define note_5A 440  
#define note_sA 233  //233.082
#define note_B  247  //246.942
#define note_3B  123  //123.471
#define note_5B  494  //493.883
#define note_C  262  //261.626
#define note_5C  523  //523.251
#define note_sC 277  //277.183
#define note_D  294  //293.665
#define note_sD 311  //311.127
#define note_5D 587  //587.33
#define note_3sD 156  //155.563
#define note_E  330  //329.629
#define note_3E  165  //164.814
#define note_F  349  //349.228
#define note_3F  175  //174.614
#define note_sF 370  //369.994
#define note_3sF 185  //184.997
#define note_G  392  //391.995
#define note_sG 415  //415.305
#define note_3G 196  //195.998
#define note_5sG 831  //830.609

/*************************************************************以下是按键*****************************************************************************************/

#define KEY_GPIO_PIN GPIO_PIN_2
#define KEY_GPIO_PORT GPIOB

#endif
