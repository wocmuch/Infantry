#include "my_usart_recieve.h"
#include "stdio.h"

my_usart_rc_t my_rc;
int8_t my_receieve_buff[20];
char cimu_buff[25];
cimu_t cimu;
void my_rc_judge(my_usart_rc_t *rc){
//		if(rc->vw>0)rc->vw=-500;else if(rc->vw>0)rc->vw=500;else rc->vw=0;
		return;
}

void my_usart_solver(int8_t*my_receieve_buff){

		for(uint8_t i =0;i<20;i++){
			if(my_receieve_buff[i]==0x6b){
				my_rc.armer_flag= my_receieve_buff[++i];
				my_rc.error_x= my_receieve_buff[++i];
				my_rc.error_y= my_receieve_buff[++i];
				my_rc_judge(&my_rc);
				break;
			}
			memset(my_receieve_buff,0,20);
			return;
		}
}



void my_usart_handle(UART_HandleTypeDef*my_usart){
		
		if(__HAL_UART_GET_FLAG(my_usart,UART_FLAG_IDLE)!=RESET)
    {
			__HAL_UART_CLEAR_IDLEFLAG(my_usart);//清楚标志位
			HAL_UART_Receive_DMA(my_usart, my_receieve_buff,20);
		  HAL_UART_DMAStop(my_usart); //停止DMA接收，防止数据出错
			my_usart_solver(my_receieve_buff);
      HAL_UART_Receive_DMA(my_usart, my_receieve_buff,20);
		}		
	
}

my_usart_rc_t* get_my_usart_rc_point(void){

 return &my_rc;

}
void cimu_solver(char * cimu_buff){

    /*在字符串 str 中从第二个字符开始解析数据，并尝试匹配格式 "%f,%f:"。如果成功解析两个浮点数并且字符串的格式是正确的（以逗号结尾），则返回值将是2*/
	if (sscanf(cimu_buff, ":%f,%f,%f:", &cimu.Yaw,&cimu.Pitch,&cimu.Roll) != 3)
    {
				memset(cimu_buff,0,20); 
			
        return;
    }
	
		

}
void cimu_handle(UART_HandleTypeDef*my_usart){

	if(__HAL_UART_GET_FLAG(my_usart,UART_FLAG_IDLE)!=RESET)
    {
			__HAL_UART_CLEAR_IDLEFLAG(my_usart);//清楚标志位
			HAL_UART_Receive_DMA(my_usart, cimu_buff,25);
		  HAL_UART_DMAStop(my_usart); //停止DMA接收，防止数据出错
			cimu_solver(cimu_buff);
      HAL_UART_Receive_DMA(my_usart, cimu_buff,25);
		}		
	


}


