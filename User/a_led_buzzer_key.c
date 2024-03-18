#include "a_led_buzzer_key.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "bsp_sbus.h"
#include "stdio.h"
#include "elog.h"
static GLOBAL_TIME_t global_time;
void global_time_handle(){
	if(global_time.msec<1000)
			global_time.msec++;
	else{
		global_time.seceond++;
		global_time.msec=0;
	}
}
//返回开机后的时间 单位为ms 
uint32_t get_global_time(){
	return global_time.seceond*1000+global_time.msec;
}

/*	
                  						A板正面示意图                             A板背面示意图
                   **********************************        **********************************
                  |                     SBUS  SW     |      |     A ->PI0          N1->PC4     |
                  |      LED8                        |      |     B ->PH12         N2->PC0     |
                  |      LED7            KEY         |      |     C ->PH11         O1->PC5     |
                  |      LED6      R                 |      |     D ->PH10         O2->PC1     |
                  |      LED5            RESET       |      |     E ->PD15         P1->PA5     |
                  |      LED4            LEDA        |      |     F ->PD14         P2->PA4     |
                  |      LED3            LEDB        |      |     G ->PD13         Q1->PF10    |
                  |      LED2                        |      |     H ->PD12         Q2->PI9     |
                  |      LED1                        |      |     I1->PF1          R1->GND     |
                  |                                  |      |     I2->PF2          R2->+5V     |
                  |                                  |      |     J1->PE5          S ->PA0     |
                  |POWER_OUTPUT * 3   NO NEED ENABLE |      |     J2->PE4          T ->PA1     |
                  |                                  |      |     K1->PE6          U ->PA2     |
                  |                                  |      |     K2->PE12         V ->PA3     |
                  |POWER_OUTPUT4       POWER_OUTPUT2 |      |     L1->PC2          W ->PI5     |
                  |                                  |      |     L2->PB0          X ->PI6     |
                  |POWER_OUTPUT3       POWER_OUTPUT1 |      |     M1->PC3          Y ->PI7     |
                  |                                  |      |     M2->PB1          Z ->PI2     |
                  |           POWER_INPUT            |      |               GPIO               |
                   **********************************        **********************************
                  
*/
/****************************************以下是LED*******************************************************/

void LED_Delay(uint32_t time)
{
    osDelay(time);
}
void LED_ALL_OFF()
{	
	//一排流水灯
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,GPIO_PIN_SET);
}
void LED_ALL_RUN(void)
{

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
		LEDA_TOG();
		LEDB_TOG();
    LED_Delay(100);
	
    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_RESET);
		LEDA_TOG();
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_RESET);
		LEDB_TOG();
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_RESET);
		LEDB_TOG();
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,GPIO_PIN_RESET);	
    LED_Delay(100);

}
void LED_RUN(void){
	
	  LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
    LED_Delay(100);
		
    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,GPIO_PIN_RESET);
    LED_Delay(100);

}
/**********************************************************以下是蜂鸣器*****************************************************************************/
//仿2006的启动声音
void Buzzer_Dji_Start(){

	Note(note_5A,1.5);
	Note(note_5D,1.5);
	Note(note_5sG,1.5);
	HAL_TIM_PWM_Stop(&buzzer_htim,TIM_CHANNEL_1);
	
}
//蜂鸣器响num声,间隔time
void Buzzer_Times(int num,int time){
	
	HAL_TIM_PWM_Start(&buzzer_htim,TIM_CHANNEL_1);
	while(num){
		LEDA_TOG();
		buzzer_htim.Instance->CNT = 0;
		buzzer_htim.Instance->ARR = (21000/800-1)*1u;  //修改重装载值→频率
		buzzer_htim.Instance->CCR1 = (10500/800-1)*1u;
		osDelay(time);
		num--;
	}
	HAL_TIM_PWM_Stop(&buzzer_htim,TIM_CHANNEL_1);
	osDelay(2*time);
	
}
/**
  * @brief				蜂鸣器
  * @param[out]		不同频率及相应节拍
  * @param[in]		note音符，long音长
  * @retval				
*/
void Note(int note,float Long)
{
		__HAL_TIM_DISABLE(&buzzer_htim);									
		buzzer_htim.Instance->CNT = 0;
		buzzer_htim.Instance->ARR = (21000/note-1)*1u;  //修改重装载值→频率
		buzzer_htim.Instance->CCR1 = (10500/note-1)*1u;
		__HAL_TIM_ENABLE(&buzzer_htim);									
	  HAL_TIM_PWM_Start(&buzzer_htim,TIM_CHANNEL_1);  //PH6		
	switch(note)
		{	
			case 0:
			{

			}break;
			
			case note_C:
			case note_sC:
			{
				LED1_ON();		
			}break;
			
			case note_D:
			case note_sD:
			case note_3sD:
			case note_5D:
			{
        LED2_ON();
			}break;

			case note_E:
			case note_3E:
			{
        LED3_ON();
			}break;
			
			case note_F:
			case note_sF:
			case note_3sF:
			{
        LED4_ON();
			}break;

			case note_G:
			case note_sG:
			case note_3G:
			case note_5sG:
			{
        LED5_ON();
			}break;

			case note_A:
			case note_sA:
			case note_5A:
			case note_3A:
			{
        LED6_ON();
			}break;
			
			case note_B:
			case note_3B:
			case note_5B:
			{
        LED7_ON();
			}break;
			
			case note_5C:
			{
        LED8_ON();
			}break;
			
			default: break;
		}
	
		osDelay(Long*200);
}
/*************************************************************以下是按键*****************************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
  if(GPIO_Pin==KEY_GPIO_PIN)
  {
		osDelay(5);
    if(HAL_GPIO_ReadPin(KEY_GPIO_PORT,KEY_GPIO_PIN)==GPIO_PIN_SET)
    {
			LEDA_TOG();
			LEDB_TOG();
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY_GPIO_PIN);
  }

}

