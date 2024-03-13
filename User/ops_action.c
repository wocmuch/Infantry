#include "ops_action.h"
#include "stdio.h"

#define OpsUsart_BUFFERSIZE 255	

ops_t ops={
0,0,0,0,0,0,0,0,0,0
};
uint8_t Rx_len_OpsUsart;
uint8_t ReceiveBuff_OpsUsart[OpsUsart_BUFFERSIZE]; 
void opsusart_receive_handler(UART_HandleTypeDef *huart,DMA_HandleTypeDef*hdma_usart_rx){
		 uint32_t temp;//计算串口接收到的数据个数
    static union
    {
        uint8_t date[24];
        float ActVal[6];
    } posture;
			if(RESET != __HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE))//如果为串口2空闲
			{
					__HAL_UART_CLEAR_IDLEFLAG(huart);//清除中断标志
					HAL_UART_DMAStop(huart);//停止DMA接收
					temp  = __HAL_DMA_GET_COUNTER(hdma_usart_rx);//获取DMA当前还有多少未填充 hdma_usart6_rx
					Rx_len_OpsUsart =  OpsUsart_BUFFERSIZE - temp; //计算串口接收到的数据个数 Rx_len_Huart6
					/*************************************************************************/
					//接收数据处理
					if(Rx_len_OpsUsart==28)
					{
							for(int i=0; i<24; i++)
							{
									posture.date[i]=ReceiveBuff_OpsUsart[i+2]; //ReceiveBuff_Huart6
							}
							ops.zangle=-posture.ActVal[0];
							ops.xangle=posture.ActVal[1];
							ops.yangle=posture.ActVal[2];
							ops.pos_x=posture.ActVal[3];
							ops.pos_y=posture.ActVal[4];
							ops.w_z=posture.ActVal[5];
					}
					/*************************************************************************/
					//重新开启下一次接收
					//memset(ReceiveBuff_Huart2,0,sizeof(ReceiveBuff_Huart2));
					Rx_len_OpsUsart=0;//接收数据长度清零
					HAL_UART_Receive_DMA(&huart2,ReceiveBuff_OpsUsart,OpsUsart_BUFFERSIZE);//开启下一次接收

			}
}
ops_t* get_location_control_point(void){
	return &ops;
}