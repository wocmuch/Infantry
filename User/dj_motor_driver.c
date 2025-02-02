#include "can.h"
#include "pid.h"
#include "dj_motor_driver.h"
#include "elog.h"
#include "a_led_buzzer_key.h"
void dj_motor_measure_updata(motor_measure_t *motor, uint8_t Rx_Data[]);
void get_motor_offset(motor_measure_t *motor, uint8_t Rx_Data[]);

#ifndef ABS
#define ABS(x) (((x) > 0) ? (x) : (-x))
#endif
#define DJ_MOTOR_NUMBER 11
// can1的 8个电机
static motor_measure_t motor_can1[DJ_MOTOR_NUMBER] = {0}; // 8个电机
static PID_T motor_can1_speed_pid[DJ_MOTOR_NUMBER] = {0}; // 
static PID_T motor_can1_pos_pid[DJ_MOTOR_NUMBER] = {0}; // 
// can2的 8个电机
static motor_measure_t motor_can2[DJ_MOTOR_NUMBER] = {0}; 
static PID_T motor_can2_speed_pid[DJ_MOTOR_NUMBER] = {0};
static PID_T motor_can2_pos_pid[DJ_MOTOR_NUMBER] = {0};
/**
 * @Func		dj_motor_handler
 * @Brief       电机定时处理
 * @param       cycle 处理间隔，单位ms
 * @param       ExecutionTimes 速度环相较于位置环的周期之比
 * @Data     	2023年7月27日
 * @note        使用pid相关时需调用该函数以保证正常运行 
 */
void dj_motor_handler(uint16_t cycle,uint16_t ExecutionTimes)
{
	static uint32_t __time=0;
	static uint32_t __flag=0;
	__flag++;
	__time++;
	for(int i=0; i<DJ_MOTOR_NUMBER;++i)
	{
		if(__time % ExecutionTimes == 0)
		{
			
		///////////////////位置环更新当前值//////////////////////////////////
		PID_Sst_Present(&motor_can1_pos_pid[i],motor_can1[i].total_angle/8.912f);
		PID_Sst_Present(&motor_can2_pos_pid[i],motor_can2[i].total_angle/8.912f);
		
			//////////////////位置环pid处理//////////////////////////////////
		PID_Hander(&motor_can1_pos_pid[i],cycle);
		PID_Hander(&motor_can2_pos_pid[i],cycle);
			//////////////////位置环加载输出到速度环//////////////////////////
		PID_Sst_Target(&motor_can1_speed_pid[i],motor_can1_pos_pid[i].parameter.out);
		PID_Sst_Target(&motor_can2_speed_pid[i],motor_can2_pos_pid[i].parameter.out);

		}

		///////////////////速度环更新当前值//////////////////////////////////
		PID_Sst_Present(&motor_can1_speed_pid[i],motor_can1[i].speed_rpm);
		PID_Sst_Present(&motor_can2_speed_pid[i],motor_can2[i].speed_rpm);
		   //////////////////速度环pid处理//////////////////////////////////
		PID_Hander(&motor_can1_speed_pid[i],cycle);
		PID_Hander(&motor_can2_speed_pid[i],cycle);
	
	}
	////////////以下define仅仅为了缩短函数的长度/////////////////////////////
#define can1_pid0_out motor_can1_speed_pid[0].parameter.out
#define can1_pid1_out motor_can1_speed_pid[1].parameter.out
#define can1_pid2_out motor_can1_speed_pid[2].parameter.out
#define can1_pid3_out motor_can1_speed_pid[3].parameter.out
#define can1_pid4_out motor_can1_speed_pid[4].parameter.out
#define can1_pid5_out motor_can1_speed_pid[5].parameter.out
#define can1_pid6_out motor_can1_speed_pid[6].parameter.out
#define can1_pid7_out motor_can1_speed_pid[7].parameter.out
#define can1_pid8_out motor_can1_speed_pid[8].parameter.out
#define can1_pid9_out motor_can1_speed_pid[9].parameter.out
#define can1_pid10_out motor_can1_speed_pid[10].parameter.out

#define can2_pid0_out motor_can2_speed_pid[0].parameter.out
#define can2_pid1_out motor_can2_speed_pid[1].parameter.out
#define can2_pid2_out motor_can2_speed_pid[2].parameter.out
#define can2_pid3_out motor_can2_speed_pid[3].parameter.out
#define can2_pid4_out motor_can2_speed_pid[4].parameter.out
#define can2_pid5_out motor_can2_speed_pid[5].parameter.out
#define can2_pid6_out motor_can2_speed_pid[6].parameter.out
#define can2_pid7_out motor_can2_speed_pid[7].parameter.out
#define can2_pid8_out motor_can2_speed_pid[8].parameter.out
#define can2_pid9_out motor_can2_speed_pid[9].parameter.out
#define can2_pid10_out motor_can2_speed_pid[10].parameter.out
	
#define NOT_USE_6020_ID567 0
	/////////////////////////加载输出/////////////////////////////////////
	//注意！！！下面的每个函数里都包含了两个函数，即发送两串指令。。。但是，使一次只能发送两串指令。。。后三个id的6020和其他电机只能动一个。
#if	NOT_USE_6020_ID567

		set_motor_allGroup(&hcan1,can1_pid0_out,can1_pid1_out,can1_pid2_out,
		can1_pid3_out,can1_pid4_out,can1_pid5_out,can1_pid6_out,can1_pid7_out);
	
		set_motor_allGroup(&hcan2,can2_pid0_out,can2_pid1_out,can2_pid2_out,
		can2_pid3_out,can2_pid4_out,can2_pid5_out,can2_pid6_out,can2_pid7_out);

#else

	dj_set_motor_Group_A(&hcan1,can1_pid0_out,can1_pid1_out,can1_pid2_out,can1_pid3_out);
	
	dj_set_motor_Group_A(&hcan2,can2_pid0_out,can2_pid1_out,can2_pid2_out,can2_pid3_out);

    dj_set_6020_Group_B(&hcan1,can1_pid8_out,can1_pid9_out,can1_pid10_out,0);

    dj_set_6020_Group_B(&hcan2,can2_pid8_out,can2_pid9_out,can2_pid10_out,0);
#endif
		motor_state_update();
	
    
}
void motor_state_update(){
	for(int i=0; i<DJ_MOTOR_NUMBER;++i)
	{
		if( get_global_time() - motor_can1[i].time_recieved > 5)
			motor_can1[i].motor_state = NOT_ONlINE;
		else
			motor_can1[i].motor_state = ONlINE;
		if( get_global_time() - motor_can2[i].time_recieved > 5)
			motor_can2[i].motor_state = NOT_ONlINE;
		else
			motor_can2[i].motor_state = ONlINE;
	}
}
//0 速度 1 位置
void dj_set_motor_mode(DJ_Motor_ID id,uint8_t mode)
{
	if(mode == dj_mode_speed)//关闭位置环
	{
		if(id>DJ_MOTOR_NUMBER-1){
			PID_Pause(&motor_can2_pos_pid[(id-DJ_MOTOR_NUMBER)]);
			PID_Enable(&motor_can2_speed_pid[(id-DJ_MOTOR_NUMBER)]);

		}else{
			PID_Pause(&motor_can1_pos_pid[id]);
			PID_Enable(&motor_can1_speed_pid[id]);
		}
		
	}else if(mode == dj_mode_position)//开启位置环
	{
		if(id>DJ_MOTOR_NUMBER){
			PID_Enable(&motor_can2_speed_pid[(id-DJ_MOTOR_NUMBER)]);
			PID_Enable(&motor_can2_pos_pid[(id-DJ_MOTOR_NUMBER)]);

		}else{
			PID_Enable(&motor_can1_speed_pid[id]);
			PID_Enable(&motor_can1_pos_pid[id]);
		}
	
	}else while(1){//不应该进入
	
	};

}




void DJ_Set_Motor_Speed(DJ_Motor_ID id,float speed,DJ_Motor_Type type)
{
	if(type==DJ_M2006){
		speed=speed*36.f;
	}else if(type==DJ_M3508){
		speed=speed*19.f;
	}else
		speed=speed;
	
	if(id>DJ_MOTOR_NUMBER-1){
		PID_Enable(&motor_can2_speed_pid[(id-DJ_MOTOR_NUMBER)]);
		PID_Pause(&motor_can2_pos_pid[(id-DJ_MOTOR_NUMBER)]);
		motor_can2_pos_pid[(id-DJ_MOTOR_NUMBER)].parameter.target = speed;
	}
    else{
		PID_Enable(&motor_can1_speed_pid[(id)]);
		PID_Pause(&motor_can1_pos_pid[id]);
		motor_can1_pos_pid[id].parameter.target = speed;
	}
}

void DJ_Set_Motor_Position(DJ_Motor_ID id,float pos,DJ_Motor_Type type)
{
	if(type==DJ_M2006){
		pos=pos/360*1000*36*0.9152;
	}else if(type==DJ_M3508){
		pos=pos/360*1000*19*0.9118;
	}else
		pos=pos/360*1000*0.8882;
	
	if(id>DJ_MOTOR_NUMBER-1){
		PID_Enable(&motor_can2_speed_pid[(id-DJ_MOTOR_NUMBER)]);
		PID_Enable(&motor_can2_pos_pid[(id-DJ_MOTOR_NUMBER)]);
		motor_can2_pos_pid[(id-DJ_MOTOR_NUMBER)].parameter.target = pos;
 
	}else{
		PID_Enable(&motor_can1_speed_pid[(id)]);
		PID_Enable(&motor_can1_pos_pid[(id)]);
		motor_can1_pos_pid[id].parameter.target = pos;
	}
}


void DJ_Set_Motor_Current(DJ_Motor_ID id,float current,DJ_Motor_Type type)
{
	
	if(type==DJ_M2006){
		current=current*10000/10;
		if(current>10000)current=10000;else if(current<-10000)current=-10000;
	}else if(type==DJ_M3508){
		current=current*16384.f/30;
		if(current>16384)current=16384;else if(current<-16384)current=-16384;
	}else{
		current=current*16384.f/3;
		if(current>16384)current=16384;else if(current<-16384)current=-16384;
	}
	
	if(id>DJ_MOTOR_NUMBER-1){
		PID_Pause(&motor_can2_speed_pid[(id-DJ_MOTOR_NUMBER)]);
		PID_Pause(&motor_can2_pos_pid[(id-DJ_MOTOR_NUMBER)]);
		motor_can2_pos_pid[(id-DJ_MOTOR_NUMBER)].parameter.target = current;
	}
    else{
		PID_Pause(&motor_can1_speed_pid[id]);
		PID_Pause(&motor_can1_pos_pid[id]);
		motor_can1_pos_pid[id].parameter.target = current;
	}
		
}




void set_motor_allGroup(CAN_HandleTypeDef *_hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4,
                                int16_t iq5, int16_t iq6, int16_t iq7, int16_t iq8)
{
    
		dj_set_motor_Group_A(_hcan, iq1, iq2, iq3, iq4);
    dj_set_motor_Group_B(_hcan, iq5, iq6, iq7, iq8);
}

void set_6020_allGroup(CAN_HandleTypeDef *_hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4,
                                int16_t iq5, int16_t iq6, int16_t iq7, int16_t iq8)
{
		//因为标识符重复，避免出问题，注释掉
    //dj_set_6020_Group_A(_hcan, iq1, iq2, iq3, iq4);
    dj_set_6020_Group_B(_hcan, iq5, iq6, iq7, iq8);
}

/**
 * @Func		my_can_filter_init
 * @Brief       CAN1和CAN2滤波器配置
 * @param       _hcan
 * @Data        2022年12月16日
 */
void __can_filter_init_recv_all(CAN_HandleTypeDef *_hcan)
{

    // can1 &can2 use same filter config
    CAN_FilterTypeDef CAN_FilterType = {0};
    /*
    static CanTxMsgTypeDef		Tx1Message;
    static CanRxMsgTypeDef 		Rx1Message;
    */
    CAN_FilterType.FilterBank = 0;
    CAN_FilterType.FilterIdHigh = 0x0000;
    CAN_FilterType.FilterIdLow = 0x0000;
    CAN_FilterType.FilterMaskIdHigh = 0x0000; //(((uint32_t)0x1313<<3)&0xFFFF0000)>>16;
    CAN_FilterType.FilterMaskIdLow = 0x0000;  //(((uint32_t)0x1313<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;
    CAN_FilterType.FilterFIFOAssignment = CAN_RX_FIFO0;
    CAN_FilterType.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterType.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterType.FilterActivation = ENABLE;
    CAN_FilterType.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterType) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(_hcan) != HAL_OK)
    {
        Error_Handler();
    }
}

void __can_filter_recv_special(CAN_HandleTypeDef *_hcan)
{
    CAN_FilterTypeDef CAN_FilterType;
    CAN_FilterType.FilterBank = 14;
    CAN_FilterType.FilterIdHigh = 0x0000;
    CAN_FilterType.FilterIdLow = 0x0000;
    CAN_FilterType.FilterMaskIdHigh = 0x0000; //(((uint32_t)0x1313<<3)&0xFFFF0000)>>16;
    CAN_FilterType.FilterMaskIdLow = 0x0000;  //(((uint32_t)0x1313<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;
    CAN_FilterType.FilterFIFOAssignment = CAN_RX_FIFO0;
    CAN_FilterType.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterType.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterType.FilterActivation = ENABLE;
    CAN_FilterType.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterType) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(_hcan) != HAL_OK)
    {
        Error_Handler();
    }
}


/*******************************************************************************************
 * @Func			void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
 * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
 * @Param
 * @Retval		None
 * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    CAN_RxHeaderTypeDef CAN_RxHeader;
    uint8_t Rx_Data[8];
    uint8_t i;
    HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &CAN_RxHeader, Rx_Data);
    /********  CAN1 *******/
    if (_hcan->Instance == CAN1)
    {
        if(CAN_RxHeader.StdId<0x209)
        i = CAN_RxHeader.StdId - CAN_Motor1_ID;
        else
        i = CAN_RxHeader.StdId - CAN_6020_ID1 +4;
        if (motor_can1[i].msg_cnt <= 50) // 上电后接收50次矫正 50次之后正常接收数据
        {
            motor_can1[i].msg_cnt++;
            get_motor_offset(&motor_can1[i], Rx_Data);
        }
        else
            dj_motor_measure_updata(&motor_can1[i], Rx_Data);
    }
    /********  CAN2 *******/
    else if (_hcan->Instance == CAN2)
    {

        if(CAN_RxHeader.StdId<0x209)
        i = CAN_RxHeader.StdId - CAN_Motor1_ID;
        else
        i = CAN_RxHeader.StdId - CAN_6020_ID1 +4;
        if (motor_can2[i].msg_cnt <= 50)
        {
            motor_can2[i].msg_cnt++;
            get_motor_offset(&motor_can2[i], Rx_Data);
        }
        else
            dj_motor_measure_updata(&motor_can2[i], Rx_Data);
    }
}

int __Can_TxMessage(CAN_HandleTypeDef *_hcan, uint8_t ide, uint32_t id, uint8_t len, uint8_t *data)
{
    uint32_t TxMailbox;
    CAN_TxHeaderTypeDef CAN_TxHeader;
    HAL_StatusTypeDef HAL_RetVal;
    uint16_t i = 0;
    if (ide == 0)
    {
        CAN_TxHeader.IDE = CAN_ID_STD; // 标准帧
        CAN_TxHeader.StdId = id;
    }
    else
    {
        CAN_TxHeader.IDE = CAN_ID_EXT; // 扩展帧
        CAN_TxHeader.ExtId = id;
    }
    CAN_TxHeader.DLC = len;
    CAN_TxHeader.RTR = CAN_RTR_DATA; // 数据帧,CAN_RTR_REMxx	OTE遥控帧
    CAN_TxHeader.TransmitGlobalTime = DISABLE;
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
    {
        i++;
        if (i > 0xfffe)
            return 1;
    }
    HAL_RetVal = HAL_CAN_AddTxMessage(_hcan, &CAN_TxHeader, data, &TxMailbox);
    if (HAL_RetVal != HAL_OK)
        return 1;
    return 0;
}

/**
    Rx_Data[0] 转子机械角度高8位
    Rx_Data[1] 转子机械角度低8位
    Rx_Data[2] 转子转速高8位
    Rx_Data[3] 转子转速低8位
    Rx_Data[4] 实际转矩电流高8位
    Rx_Data[5] 实际转矩电流低8位
    Rx_Data[6] 电机温度
    Rx_Data[7] 空Null

    发送频率 1KHz
    转子机械角度 0 ~ 8191 (对应转子角度0~360)
    转子转速单位 RPM
    电机温度单位 °C

last_angle  上次角度更新
angle       转子机械角度高8位和第八位
speed_rpm   转子转速高8位和低八位
real_current实际输出转矩高8位和低8位
temperature 温度
total_angle 总角度
 */
void dj_motor_measure_updata(motor_measure_t *motor, uint8_t Rx_Data[])
{
    motor->last_angle = motor->angle;                              // 上次角度更新
    motor->angle = (uint16_t)(Rx_Data[0] << 8 | Rx_Data[1]);       // 转子机械角度高8位和第八位
    motor->speed_rpm = (int16_t)(Rx_Data[2] << 8 | Rx_Data[3]);    // 转子转速高8位和低八位
	
    motor->real_current = (int16_t)(Rx_Data[4] << 8 | Rx_Data[5]); // 实际输出转矩高8位和低8位
    motor->temperature = Rx_Data[6];                               // 温度     //Null

    if (motor->angle - motor->last_angle > 4096)
        motor->round_cnt--;
    else if (motor->angle - motor->last_angle < -4096)
        motor->round_cnt++;
    motor->total_angle = motor->round_cnt * 8191 + motor->angle - motor->offset_angle;
		
		motor->time_recieved=get_global_time();
		
}

void get_motor_offset(motor_measure_t *motor, uint8_t Rx_Data[])
{
    motor->offset_angle = motor->angle = (uint16_t)(Rx_Data[0] << 8 | Rx_Data[1]);
}

void dj_motor_BackToZero(motor_measure_t *motor)
{
    motor->offset_angle = motor->angle;
    motor->round_cnt = 0;
}

/**
 * @Func			dj_get_totalAngle(motor_measure_t *p)
 * @Brief    电机上电角度=0， 之后用这个函数  更新电机的相对开机后（为0）的相对角度。
 * @Param
 * @Retval		None
 * @Date     2023/6/20
 */
int32_t dj_get_totalAngle(motor_measure_t *p)
{
    int res1, res2, delta;
    if (p->angle < p->last_angle)
    {                                           // 可能的情况
        res1 = p->angle + 8192 - p->last_angle; // 正转，delta=+
        res2 = p->angle - p->last_angle;        // 反转	delta=-
    }
    else
    {                                           // angle > last
        res1 = p->angle - 8192 - p->last_angle; // 反转	delta -
        res2 = p->angle - p->last_angle;        // 正转	delta +
    }

    //	=DECE_RATIO_ER_TR*8191*36*Read_init_AS5048A[2]/360.0f;
    // 不管正反转，肯定是转的角度小的那个是真的
    if (ABS(res1) < ABS(res2))
        delta = res1;
    else
        delta = res2;

    p->total_angle += delta;
    p->last_angle = p->angle;

    return p->total_angle;
}
/**
2006 C610   1:36
用于向电调发送控制指令控制电调的电流输出，
两个标识符（0x200和0x1FF)各自对应控制4个ID的电调。
控制转矩电流值范围-10000~0~10000，
对应电调输出的转矩电流范围-10~0~10A。
接收频率: 1KHz
转子机械角度值范围:0~8191(对应转子机械角度为0~360° )
转子转速值的单位为:RPM   8191 * 36 = 294876
电机温度的单位为:℃

3508 C620  1:19
用于向电调发送控制指令控制电调的电流输出，
两个标识符（0x200和0x1FF)各自对应控制4个ID的电调。
控制转矩电流值范围-16384~0~16384，
对应电调输出的转矩电流范围-20~0~20A。
接收频率: 1KHz
转子机械角度值范围:0~8191(对应转子机械角度为0~360° )   8191*19 =155629
转子转速值的单位为:RPM
电机温度的单位为:℃

6020  1:1
用于向电调发送控制指令控制电调的电流输出，
ID号：
 1    2     3     4     5     6     7
 0x205 0x206 0x207 0x208 0x209 0x20A 0x20B
两个标识符（0x1FF和0x2FF)各自对应控制4个ID的电调。
id1 2 3 与2006和3508的id标识完全重复 本driver驱动了后三个电机 前四个电机可以当2006和3508驱动
电流给定值范围：-16384~0~16384, 
对应最大转矩电流范围 -3A~0~3A。
接收频率: 1KHz
转子机械角度值范围:0~8191(对应转子机械角度为0~360° )   8191*1 =8191
转子转速值的单位为:RPM
电机温度的单位为:℃
*/
void dj_set_motor_Group_A(CAN_HandleTypeDef *_hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{

    uint8_t data[8] = {0};
    data[0] = iq1 >> 8;
    data[1] = iq1;
    data[2] = iq2 >> 8;
    data[3] = iq2;
    data[4] = iq3 >> 8;
    data[5] = iq3;
    data[6] = iq4 >> 8;
    data[7] = iq4;

    __Can_TxMessage(_hcan, 0, CAN_Motor_ALL_ID, 8, (uint8_t *)data); // 0代表标准正
}
void dj_set_motor_Group_B(CAN_HandleTypeDef *_hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) // 4个电机转向
{
    uint8_t data[8] = {0};
    data[0] = iq1 >> 8;
    data[1] = iq1;
    data[2] = iq2 >> 8;
    data[3] = iq2;
    data[4] = iq3 >> 8;
    data[5] = iq3;
    data[6] = iq4 >> 8;
    data[7] = iq4;

    __Can_TxMessage(_hcan, 0, CAN_Motor_ALL_ID2, 8, (uint8_t *)data); // 0代表标准正
}
void dj_set_6020_Group_A(CAN_HandleTypeDef *_hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{

    uint8_t data[8] = {0};
    data[0] = iq1 >> 8;
    data[1] = iq1;
    data[2] = iq2 >> 8;
    data[3] = iq2;
    data[4] = iq3 >> 8;
    data[5] = iq3;
    data[6] = iq4 >> 8;
    data[7] = iq4;

    __Can_TxMessage(_hcan, 0, CAN_6020_ALL_ID, 8, (uint8_t *)data); // 0代表标准正
}
void dj_set_6020_Group_B(CAN_HandleTypeDef *_hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) // 4个电机转向
{
    uint8_t data[8] = {0};
    data[0] = iq1 >> 8;
    data[1] = iq1;
    data[2] = iq2 >> 8;
    data[3] = iq2;
    data[4] = iq3 >> 8;
    data[5] = iq3;
    data[6] = iq4 >> 8;
    data[7] = iq4;

    __Can_TxMessage(_hcan, 0, CAN_6020_ALL_ID2, 8, (uint8_t *)data); // 0代表标准正
}
/**
 * @brief          返回底盘电机 3508电机数据句柄
 * @param[in]      i: 电机编号,范围[0,7]
 * @retval         电机数据句柄
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_can1[(i & 0x07)];
}

const motor_measure_t *get_chassis_motor2_measure_point(uint8_t i)
{
    return &motor_can2[(i & 0x07)];
}


int dj_motor_init(void)
{
    //can外设初始化，已经在can.c中实现
	__can_filter_init_recv_all(&hcan1);
	__can_filter_recv_special(&hcan2);
	
	for(int i=0;i<DJ_MOTOR_NUMBER;++i)
	{
		//给pid中的参数給一个初值，保证至少可动，后面再根据情况具体调节
		//默认pid 应该都适用。。。较软
		PID_Init(&motor_can1_speed_pid[i],PID_POSITION_NULL,3.0f,0.01f,0.01f);
		PID_Sst_Out_Limit(&motor_can1_speed_pid[i],30000);
		PID_Sst_Integral_Limit(&motor_can1_speed_pid[i],2000);
		PID_Sst_Bias_Dead_Zone(&motor_can1_speed_pid[i],20);

		PID_Init(&motor_can2_speed_pid[i],PID_POSITION_NULL,3.f,0.01f,0.01f);
		PID_Sst_Out_Limit(&motor_can2_speed_pid[i],30000);
		PID_Sst_Integral_Limit(&motor_can2_speed_pid[i],2000);
		PID_Sst_Bias_Dead_Zone(&motor_can2_speed_pid[i],20);

		
		PID_Init(&motor_can1_pos_pid[i],PID_POSITION_NULL,0.125,0.000000395f,0.5f);
		PID_Sst_Out_Limit(&motor_can1_pos_pid[i],20000);
		PID_Sst_Integral_Limit(&motor_can1_pos_pid[i],200);
		
		PID_Init(&motor_can2_pos_pid[i],PID_POSITION_NULL,0.125,0.000000395f,0.5f);
		PID_Sst_Out_Limit(&motor_can2_pos_pid[i],20000);
		PID_Sst_Integral_Limit(&motor_can2_pos_pid[i],200);
		
	}

		
	//////////////////
	//for 6020 pid
		PID_Init(&motor_can1_speed_pid[DJ_CAN1_6020_M6 ],PID_POSITION_NULL,512.5f,0.0f,0.0f);
		PID_Init(&motor_can1_pos_pid[DJ_CAN1_6020_M6],PID_POSITION_NULL,0.125f,0.000000395f,0.5f );
	
		PID_Init(&motor_can2_speed_pid[DJ_CAN2_6020_M5-DJ_MOTOR_NUMBER],PID_POSITION_NULL,475.5f,0.0f,0.05f);
		PID_Init(&motor_can2_pos_pid[DJ_CAN2_6020_M5-DJ_MOTOR_NUMBER],PID_POSITION_NULL,0.175f,0.00003f,0.005f);
	///////////////////
    return 0;
}

float DJ_Get_Motor_Position(DJ_Motor_ID id,DJ_Motor_Type type){
	
	float temp_pos=0;
	
	if(id>DJ_MOTOR_NUMBER-1){
		temp_pos = motor_can2[id-DJ_MOTOR_NUMBER].total_angle;
	}
  else{
		temp_pos = motor_can1[id].total_angle;
	}
	
	if(type==DJ_M2006){
		temp_pos=(temp_pos/(8192*36))*360;
	}else if(type==DJ_M3508){
		temp_pos=(temp_pos/(8192*19))*360;
	}else
		temp_pos=(temp_pos/(8192*1))*360;
	
	return temp_pos;

}

float DJ_Get_Motor_Speed(DJ_Motor_ID id,DJ_Motor_Type type){

	float temp_speed=0;
	
	if(id>DJ_MOTOR_NUMBER-1){
		temp_speed = (float)motor_can2[id-DJ_MOTOR_NUMBER].speed_rpm;
	}
  else{
		temp_speed = (float)motor_can1[id].speed_rpm;
	}
	if(type==DJ_M2006){
		temp_speed=temp_speed/36;
	}else if(type==DJ_M3508){
		temp_speed=temp_speed/19;
	}else
		temp_speed=temp_speed/1;
	
	return temp_speed;
}

float DJ_Get_Motor_RealCurrent(DJ_Motor_ID id,DJ_Motor_Type type){
	
	float temp_Current=0;
	
	if(id>DJ_MOTOR_NUMBER-1){
		temp_Current = motor_can2[id-DJ_MOTOR_NUMBER].real_current;
	}
  else{
		temp_Current = motor_can1[id].real_current;
	}
	if(type==DJ_M2006){
		temp_Current=temp_Current*5.f/10000.f;
	}else if(type==DJ_M3508){
		temp_Current=temp_Current*5.f/16384.f;
	}else
		temp_Current=temp_Current*5.f/16384.f;
	
	return temp_Current;

}

int DJ_Get_Motor_IsOnline(DJ_Motor_ID id){
	if(id>DJ_MOTOR_NUMBER-1){
		return  motor_can2[id-DJ_MOTOR_NUMBER].motor_state;
	}
  else{
		return  motor_can1[id].motor_state;
	}	
}
