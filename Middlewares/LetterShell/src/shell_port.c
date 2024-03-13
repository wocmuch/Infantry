/**
 * @file shell_port.c
 * @author Letter (NevermindZZT@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-02-22
 * 
 * @copyright (c) 2019 Letter
 * 
 */

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "shell.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "elog.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "semphr.h"

#include "kfifo.h"
#include "my_usart_recieve.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
Shell shell;
char shellBuffer[512];
char shelldata=0;
struct kfifo shell_rxfifo;
//static SemaphoreHandle_t shellMutex;


/**
 * @brief 用户shell写
 * 
 * @param data 数据
 * @param len 数据长度
 * 
 * @return short 实际写入的数据长度
 */
short userShellWrite(char *data, unsigned short len)
{
	HAL_UART_Transmit(&huart6,(uint8_t*)data,len,0xffff);
//		while(huart6.gState != HAL_UART_STATE_READY)
//	{
//		osDelay(1);
//	}
//	HAL_UART_Transmit_IT(&huart6,(uint8_t*)data,len);

	return len;
}


/**
 * @brief 用户shell读
 * 
 * @param data 数据
 * @param len 数据长度
 * 
 * @return short 实际读取到
 */
short userShellRead(char *data, unsigned short len)
{
	
	uint32_t cnt=kfifo_out(&shell_rxfifo,(uint8_t*)data,len);
	if(!cnt)osDelay(5);
	return cnt;
}
/**
 * @brief 用户shell上锁
 * 
 * @param shell shell
 * 
 * @return int 0
 */
int userShellLock(Shell *shell)
{

	extern SemaphoreHandle_t xMutex_Usart1;
	BaseType_t xStatus;
	xStatus = xSemaphoreTakeRecursive(xMutex_Usart1, 10);
	//log_d("shell_get");

    return xStatus;
}

/**
 * @brief 用户shell解锁
 * 
 * @param shell shell
 * 
 * @return int 0       
 */
int userShellUnlock(Shell *shell)
{

	extern SemaphoreHandle_t xMutex_Usart1;
	BaseType_t xStatus;
	xStatus  = xSemaphoreGiveRecursive(xMutex_Usart1);
	//log_d("shell_give");

  return xStatus;
}



//void uartLogWrite(char *buffer, short len)
//{
//    if (uartLog.shell)
//    {
//        shellWriteEndLine(uartLog.shell, buffer, len);
//    }
//}


/**
 * @brief 用户shell初始化
 * 
 */
void userShellInit(void)
{

    shell.write = userShellWrite;
    shell.read = userShellRead;
    shell.lock = userShellLock;
    shell.unlock = userShellUnlock;
    shellInit(&shell, shellBuffer, 512);
		//logRegister(&uartLog, &shell);
		static uint8_t rxbuffer[128]={0};
		kfifo_init(&shell_rxfifo,rxbuffer,sizeof(rxbuffer));
		extern void var_init(void);
		var_init();


		HAL_UART_Receive_IT(&huart6,(uint8_t*)&shelldata,1);

    if (xTaskCreate(shellTask, "shell", 512, &shell, 15, NULL) != pdPASS)
    {
        //logError("shell task creat failed");
    }
}

/* USER CODE BEGIN 1 */
uint8_t Fd_data[64];
uint8_t Fd_rsimu[64];
uint8_t Fd_rsahrs[56];
int rs_imutype =0;
int rs_ahrstype =0;
extern int Time_count;

IMUData_Packet_t IMUData_Packet;
AHRSData_Packet_t AHRSData_Packet;
uint8_t USART_RX_BUF_T[200];
uint8_t aRxBuffer_T[1];//HAL库使用的串口接收缓冲
void AHRSData2PC(void)
{
   elog_raw_output(" %.2f\r\n",(AHRSData_Packet.Heading*57.29578f));

}
float DATA_Trans(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4)
{
  uint32_t transition_32;
	float tmp=0;
	int sign=0;
	int exponent=0;
	float mantissa=0;
  transition_32 = 0;
  transition_32 |=  Data_4<<24;   
  transition_32 |=  Data_3<<16; 
	transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
  sign = (transition_32 & 0x80000000) ? -1 : 1;//符号位
	//先右移操作，再按位与计算，出来结果是30到23位对应的e
	exponent = ((transition_32 >> 23) & 0xff) - 127;
	//将22~0转化为10进制，得到对应的x系数 
	mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
	tmp=sign * mantissa * pow(2, exponent);
	return tmp;
}
long long timestamp(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4)
{
  uint32_t transition_32;
  transition_32 = 0;
  transition_32 |=  Data_4<<24;   
  transition_32 |=  Data_3<<16; 
	transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
	return transition_32;
}

uint8_t TTL_Hex2Dec(uint8_t *Fd_data)
{
  //  uint8 i;
     if(rs_ahrstype==1)
    {
        if(Fd_data[1]==TYPE_AHRS&&Fd_data[2]==AHRS_LEN)
        {
//            AHRSData_Packet.RollSpeed=DATA_Trans(Fd_data[7],Fd_data[8],Fd_data[9],Fd_data[10]);       //????????
//            AHRSData_Packet.PitchSpeed=DATA_Trans(Fd_data[11],Fd_data[12],Fd_data[13],Fd_data[14]);   //?????????
//            AHRSData_Packet.HeadingSpeed=DATA_Trans(Fd_data[15],Fd_data[16],Fd_data[17],Fd_data[18]); //????????

//            AHRSData_Packet.Roll=DATA_Trans(Fd_data[19],Fd_data[20],Fd_data[21],Fd_data[22]);      //?????
//            AHRSData_Packet.Pitch=DATA_Trans(Fd_data[23],Fd_data[24],Fd_data[25],Fd_data[26]);     //??????
            AHRSData_Packet.Heading=DATA_Trans(Fd_data[27],Fd_data[28],Fd_data[29],Fd_data[30]);     //?????

//            AHRSData_Packet.Qw=DATA_Trans(Fd_data[31],Fd_data[32],Fd_data[33],Fd_data[34]);  //?????
//            AHRSData_Packet.Qx=DATA_Trans(Fd_data[35],Fd_data[36],Fd_data[37],Fd_data[38]);
//            AHRSData_Packet.Qy=DATA_Trans(Fd_data[39],Fd_data[40],Fd_data[41],Fd_data[42]);
//            AHRSData_Packet.Qz=DATA_Trans(Fd_data[43],Fd_data[44],Fd_data[45],Fd_data[46]);
            AHRSData_Packet.Timestamp=(unsigned long int)timestamp(Fd_data[47],Fd_data[48],Fd_data[49],Fd_data[50]);   //????
            //AHRSData2PC();
        }
        rs_ahrstype=0;
        //debug("t0\r\n",0);
       // yaw_imu=RAD_TO_ANGLE(AHRSData_Packet.Heading);//????????
    }
    if(rs_imutype==1)
    {
        if(Fd_data[1]==TYPE_IMU&&Fd_data[2]==IMU_LEN)
        {
//            IMUData_Packet.gyroscope_x=DATA_Trans(Fd_data[7],Fd_data[8],Fd_data[9],Fd_data[10]);  //?????
//            IMUData_Packet.gyroscope_y=DATA_Trans(Fd_data[11],Fd_data[12],Fd_data[13],Fd_data[14]);
//            IMUData_Packet.gyroscope_z=DATA_Trans(Fd_data[15],Fd_data[16],Fd_data[17],Fd_data[18]);

//            IMUData_Packet.accelerometer_x=DATA_Trans(Fd_data[19],Fd_data[20],Fd_data[21],Fd_data[22]);  //??????
//            IMUData_Packet.accelerometer_y=DATA_Trans(Fd_data[23],Fd_data[24],Fd_data[25],Fd_data[26]);
//            IMUData_Packet.accelerometer_z=DATA_Trans(Fd_data[27],Fd_data[28],Fd_data[29],Fd_data[30]);

//            IMUData_Packet.magnetometer_x=DATA_Trans(Fd_data[31],Fd_data[32],Fd_data[33],Fd_data[34]);  //??????????
//            IMUData_Packet.magnetometer_y=DATA_Trans(Fd_data[35],Fd_data[36],Fd_data[37],Fd_data[38]);
//            IMUData_Packet.magnetometer_z=DATA_Trans(Fd_data[39],Fd_data[40],Fd_data[41],Fd_data[42]);

//            IMUData_Packet.Timestamp=(unsigned long int)timestamp(Fd_data[55],Fd_data[56],Fd_data[57],Fd_data[58]);   //????
               //IMUData2PC();
        }
        rs_imutype=0;
 }
    //???????

return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static int cnt=0;
	if(huart==&huart6){
		//extern Shell shell;
		//extern uint8_t shelldata;	
		//extern struct kfifo shell_rxfifo;	
		uint8_t data=shelldata;//不要删
		HAL_UART_Receive_IT(&huart6,(uint8_t*)&shelldata,1);//
		cnt++;
		kfifo_in(&shell_rxfifo,&data,1);
	  //shellHandler(&shell, data);

	}

	if(huart == &huart8){
		static uint8_t Count=0;
		static uint8_t last_rsnum=0;
		static uint8_t rsimu_flag=0;
		static uint8_t rsacc_flag=0;
			HAL_UART_Receive_IT(&huart8, (uint8_t *)aRxBuffer_T, 1);
			//Usart_Receive = USART_ReceiveData(UART5);//Read the data //读取数据
			Fd_data[Count]=aRxBuffer_T[0];  //串口数据填入数组
			//usart1_send(Usart_Receive);
			if(((last_rsnum==FRAME_END)&&(aRxBuffer_T[0] == FRAME_HEAD))||Count>0)
			{
			Count++; 
			if((Fd_data[1]==TYPE_IMU)&&(Fd_data[2]==IMU_LEN))
				rsimu_flag=1;
			if((Fd_data[1]==TYPE_AHRS)&&(Fd_data[2]==AHRS_LEN))
				rsacc_flag=1;
			}
			else 
				Count=0;
			last_rsnum=aRxBuffer_T[0];
			
		if(rsimu_flag==1 && Count==IMU_RS) //将本帧数据保存至Fd_rsimu数组中
		{
			Count=0;
			rsimu_flag=0;
			rs_imutype=1;
			if(Fd_data[IMU_RS-1]==FRAME_END) //帧尾校验
			{
					TTL_Hex2Dec(Fd_data);
			}
		}
		if(rsacc_flag==1 && Count==AHRS_RS) //
		{
			Count=0;
			rsacc_flag=0;
			rs_ahrstype=1;
			if(Fd_data[AHRS_RS-1]==FRAME_END)
			{
					TTL_Hex2Dec(Fd_data);
			}
		}
	
		}
	
	
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	
   char a = huart->Instance->DR;
			__HAL_UART_CLEAR_NEFLAG(huart);  //读DR寄存器，就可以清除ORE错误标志位

			__HAL_UART_CLEAR_FEFLAG(huart);  //读DR寄存器，就可以清除ORE错误标志位

			__HAL_UART_CLEAR_OREFLAG(huart);  //读DR寄存器，就可以清除ORE错误标志位
		

	
}

//CEVENT_EXPORT(EVENT_INIT_STAGE2, userShellInit);


