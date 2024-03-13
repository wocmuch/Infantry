#ifndef __CHASSIS_BEHAVIOUR_H
#define __CHASSIS_BEHAVIOUR_H
#include "chassis_task.h"
#include "pid.h"
#include "bsp_sbus.h"
#include "dj_motor_driver.h"
#include "ops_action.h"
#include "my_usart_recieve.h"
typedef float fp32;
typedef double fp64;




typedef struct
{
    int id;
    //pid_t pid_pos;      //每个舵轮转向的位置环
    //pid_t pid_speed;    //每个舵轮的速度环
    //float speed;        //运行速度
    float speed_set;    //运行速度
    int16_t give_current; //赋值电流值
    float rpm;          //转多少圈
		int type;
} wheel_speed_t;

typedef struct
{
    int id;

    //float speed_set;    //运行速度
    float angle_set;    //设定角度

    float rpm;          //转多少圈
		int type;
		int16_t give_current;
} wheel_dir_t;




typedef enum
{
    CHASSIS_ZERO_FORCE,                   //底盘无力, 跟没上电那样
    CHASSIS_NO_MOVE,                      //底盘保持不动
    CHASSIS_LOCATING_MOVE,                //底盘定位
    CHASSIS_REMOTE_MOVE,                   //遥控控制
		CHASSIS_NO_SET,												//不set底盘电机
		CHASSIS_DEBUG_MOED										//底盘只能通过set_speed 和set_pos驱动
} chassis_behaviour_mode_e;

/*******底盘模型*********/
typedef enum
{
    Omni3_Car,
    Omni4_Car,
    Wheel3_Car,
    Wheel4_Car,
		Mecanum_Car
} chassis_kinematics_mode_e;

typedef struct
{

    const rc_info_t *Chassis_RC;               //底盘使用的遥控器指针, the point to remote control
		const ops_t *Chassis_OPS;                           //底盘使用的定位
		my_usart_rc_t* my_chassis_rc;
    //const motor_measure_t *Chassis_Motor_Measure[8];
    chassis_behaviour_mode_e Chassis_Mode;               //state machine. 底盘控制状态机
    chassis_behaviour_mode_e Last_Chassis_Mode;          //last state machine.底盘上次控制状态机
    chassis_kinematics_mode_e Chassis_Kinematics_Mode;   //底盘运动学模型
		
		
    wheel_speed_t Wheel_Speed[4] ;				//驱动轮
    wheel_dir_t   Wheel_Dir[4]   ;           //方向轮
    PID_T chassis_angle_pid;
    fp32 Vx;                          //底盘速度 前进方向 前为正，单位 m/s
    fp32 Vy;                          //底盘速度 左右方向 左为正  单位 m/s
    fp32 Vw;                          //底盘旋转角速度，逆时针为正 单位 rad/s
    fp32 Vx_Set;                      //底盘设定速度 前进方向 前为正，单位 m/s
    fp32 Vy_Set;                      //底盘设定速度 左右方向 左为正，单位 m/s
    fp32 Vw_Set;                      //底盘设定旋转角速度，逆时针为正 单位 rad/s

    fp32 Vx_Max_Speed;   //前进方向最大速度 单位m/s
    fp32 Vx_Min_Speed;   //后退方向最大速度 单位m/s
    fp32 Vy_Max_Speed;   //左方向最大速度 单位m/s
    fp32 Vy_Min_Speed;   //右方向最大速度 单位m/s


    fp32 Chassis_Yaw;    //底盘yaw角度
    fp32 Chassis_Yaw_Set;//底盘设定yaw角度
    fp32 Postion_X;   //
    fp32 Postion_Y;
    fp32 Postion_X_Set;   //
    fp32 Postion_Y_Set;
} chassis_move_t;



void chassis_behaviour_mode_set(chassis_move_t *Chassis_Move_Mode);
void chassis_behaviour_control_set(fp32 *Vx_Set, fp32 *Vy_Set, fp32 *Angle_Set, chassis_move_t *Chassis_Move_Rc_To_Vector);



#endif
