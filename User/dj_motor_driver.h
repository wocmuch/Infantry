#ifndef DJ_MOTOR_H
#define DJ_MOTOR_H
#ifdef __cplusplus
extern "C"  
{
#endif

#include <stdint.h>
#include "can.h"


enum{
	dj_mode_speed = 0U,
	dj_mode_position,

};

typedef enum{
	DJ_CAN1_M0 = 0,
	DJ_CAN1_M1,
	DJ_CAN1_M2,
	DJ_CAN1_M3,
	DJ_CAN1_M4,
	DJ_CAN1_M5,
	DJ_CAN1_M6,
	DJ_CAN1_M7,
	DJ_CAN1_6020_M5,
	DJ_CAN1_6020_M6,
	DJ_CAN1_6020_M7,
	
	DJ_CAN2_M0 ,
	DJ_CAN2_M1,
	DJ_CAN2_M2,
	DJ_CAN2_M3,
	DJ_CAN2_M4,
	DJ_CAN2_M5,
	DJ_CAN2_M6,
	DJ_CAN2_M7,
	DJ_CAN2_6020_M5,
	DJ_CAN2_6020_M6,
	DJ_CAN2_6020_M7,
    //6020的id 1234和 2006&3508的id5678重复 以下暂时没用

}DJ_Motor_ID;
/*CAN发送或是接收的ID*/
typedef enum {

    CAN_Motor_ALL_ID = 0x200,
    CAN_Motor1_ID = 0x201,
    CAN_Motor2_ID = 0x202,
    CAN_Motor3_ID = 0x203,
    CAN_Motor4_ID = 0x204,

    CAN_Motor_ALL_ID2 = 0x1FF,
    CAN_Motor5_ID = 0x205,
    CAN_Motor6_ID = 0x206,
    CAN_Motor7_ID = 0x207,
    CAN_Motor8_ID = 0x208,

    CAN_6020_ALL_ID = 0X1FF,
    CAN_6020_ID1 = 0x205,
    CAN_6020_ID2 = 0x206,
    CAN_6020_ID3 = 0x207,
    CAN_6020_ID4 = 0x208,
    //6020的id 1234和 2006&3508的id5678重复
    CAN_6020_ALL_ID2 = 0X2FF,
    CAN_6020_ID5 = 0x209,
    CAN_6020_ID6 = 0x20A,
    CAN_6020_ID7 = 0x20B,
    CAN_6020_ID8 = 0x20C,



} DJ_Motor_Message_ID;

typedef enum {
	DJ_M2006,
	DJ_M3508,
	DJ_M6020,
}DJ_Motor_Type;
/*接收到的电机的参数结构体*/

//last_angle  上次角度更新
//angle       转子机械角度高8位和第八位
//speed_rpm   转子转速高8位和低八位
//real_current实际输出转矩高8位和低8位
//temperature 温度
//total_angle 总角度
#define FILTER_BUF_LEN        5
typedef enum {
	NOT_ONlINE,
	ONlINE
}DJ_Motor_State;
typedef struct {
    int16_t speed_rpm;      //转速
    float real_current;     //实际输出转矩
    int16_t given_current;  //扭矩
    uint8_t temperature;    //温度
    uint16_t angle;         //abs angle range:[0,8191] 角度范围[0,8191]
    uint16_t last_angle;    //abs angle range:[0,8191]
    uint16_t offset_angle;  //偏差
    int32_t round_cnt;      //电机转的圈数
    int32_t total_angle;    //当前总角度 圈数
    uint8_t buf_idx;
    uint16_t angle_buf[FILTER_BUF_LEN];
    uint16_t fited_angle;        //修正角度？
    uint32_t msg_cnt;            //初始化计数 小于50清零
    float angle_pos;  //绝对值角度
		uint32_t time_now;
		uint32_t time_recieved;
		DJ_Motor_State motor_state;
} motor_measure_t;
int dj_motor_init(void);

void dj_set_motor_Group_A(CAN_HandleTypeDef *_hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void dj_set_motor_Group_B(CAN_HandleTypeDef *_hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void dj_motor_handler(uint16_t cycle,uint16_t ExecutionTimes);

void dj_motor_BackToZero(motor_measure_t *motor);

void set_motor_allGroup(CAN_HandleTypeDef *_hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4,
                                int16_t iq5, int16_t iq6, int16_t iq7, int16_t iq8);

void motor_state_update(void);


void DJ_Set_Motor_Position(DJ_Motor_ID id,float pos,DJ_Motor_Type type);

void DJ_Set_Motor_Speed(DJ_Motor_ID id,float speed,DJ_Motor_Type type);

void DJ_Set_Motor_Current(DJ_Motor_ID id,float current,DJ_Motor_Type type);

float DJ_Get_Motor_Position(DJ_Motor_ID id,DJ_Motor_Type type);

float DJ_Get_Motor_Speed(DJ_Motor_ID id,DJ_Motor_Type type);

float DJ_Get_Motor_RealCurrent(DJ_Motor_ID id,DJ_Motor_Type type);

int DJ_Get_Motor_IsOnline(DJ_Motor_ID id);

void dj_set_6020_Group_A(CAN_HandleTypeDef *_hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

void dj_set_6020_Group_B(CAN_HandleTypeDef *_hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);


void set_6020_allGroup(CAN_HandleTypeDef *_hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4,
                                int16_t iq5, int16_t iq6, int16_t iq7, int16_t iq8);






#ifdef __cplusplus
}
#endif
#endif
