
#include "chassis_task.h"
#include "dj_motor_driver.h"
#include "math.h"
#include "bsp_sbus.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "chassis_behaviour.h"
#include "ops_action.h"
#include "elog.h"
#define CHASSIS_CAN hcan1

#define r_x 0.11f			//x方向麦克纳母轮轮距
#define r_y 0.11f			//y方向麦克纳母轮轮距
#define L 0.11f      //全向轮轮距
//全场定位
extern ops_t ops;
typedef float fp32;
typedef double fp64;

static void chassis_init(chassis_move_t *Chassis_Move_Init);
static void chassis_set_mode(chassis_move_t *Chassis_Move_Mode);
static void chassis_feedback_update(chassis_move_t *Chassis_Move_Update);
static void chassis_set_contorl(chassis_move_t *Chassis_Move_Control);
static void  chassis_set_kinematics(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set,chassis_move_t *Chassis_Move_Kinematics);
static void chassis_control_loop(chassis_move_t *Chassis_Move_Control_Loop);
static void chassis_kinematics_omni3_speed(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set, fp32 Wheel_Speed[4]);

static void chassis_info_print(chassis_move_t *Chassis_Info);


void set_pos(float X_target, float Y_target, float Angle_target,
             float X_current, float Y_current, float Angle_current,
							 chassis_move_t *Chassis_Move_Control_Loop) ;	
void Set_speed(int Velocity_X, int Velocity_Y, int W,chassis_move_t *Chassis_Move_Control_Loop);
						 
void set_motors(chassis_move_t *Chassis_Move_Control_Loop,uint32_t is_true);					 
float get_speed(uint8_t i);//获取轮子速度 没写

float get_angle(uint8_t i);//获取轮子角度 没写				 
						 

//底盘运动数据
chassis_move_t Chassis_Move;
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const * argument)
{
	
    chassis_init(&Chassis_Move);//电机id在这里设置 默认1234

//	   Omni3_Car,
//    Omni4_Car,
//    Wheel3_Car,
//    Wheel4_Car
//    Mecanum_Car
    Chassis_Move.Chassis_Kinematics_Mode = Omni4_Car;
	
    while(1)
    {
        //设置底盘控制模式
        chassis_set_mode(&Chassis_Move); //用户自定义怎么选择模式 默认初始化为 NOSET 行驶中不要切换成这个模式！！！
        //底盘数据更新
        chassis_feedback_update(&Chassis_Move);
			
				//判断是否是调试模式 （调试模式下，驱动底盘用set_speed或set_pos函数）
				//定位模式下 只需设置ops.set_pos_x等就行
				if(Chassis_Move.Chassis_Mode!=CHASSIS_DEBUG_MOED){
        //底盘控制量设置
        chassis_set_contorl(&Chassis_Move);
        //运动解算 底盘电机设置
        chassis_control_loop(&Chassis_Move);}
      //  chassis_info_print(&Chassis_Move);
//		 if(count++ >100)
//		 {
//			 HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
//			 count=0;
//		 }
//		 if(count%5==0)
//		 {
//			chassis_info_print(&Chassis_Move);
//		 }                                        
        osDelay(10);

    }
}

/*
1.设置底盘模式:       chassis_set_mode(&chassis_move);

2.底盘数据更新:       chassis_feedback_update(&chassis_move);
		获取当前电机,角度,坐标信息
3.设置底盘控制量:     chassis_set_contorl(&chassis_move);

4.运动解算+设置电机:       	chassis_control_loop(&chassis_move);

delay
*/

#define NO_INFO           0  //无信息
#define INFO_MOTOR        1 //驱动信息
#define INFO_WHEEL_DIR    2 //舵轮角度信息
#define INFO_WHEEL_SPEED  3 //舵轮速度信息
#define INFO_RC           4 //遥控器的信息
#define INFO_V_SET        5 //V_X V_Y V_Z的设置的信息
#define VOFA_WHEEL_DIR    6 //实际角度和计算出来的期望角度
#define VOFA_WHEEL_SPEED  7 //实际速度和计算出来的期望速度

#define INFO_PRINT NO_INFO //无信息


//#define INFO_PRINT  VOFA_WHEEL_DIR  //修改这里的宏定义即可修改打印的信息

static void chassis_info_print(chassis_move_t *Chassis_Info)
{

    int i=0;
#if INFO_PRINT == INFO_MOTOR
    printf("speed_rpm:%6d,angle:%6d,last_angle:%6d,total_angle:%6d 角度:%6d\n",
           (int)Chassis_Info->Chassis_Motor_Measure[i]->speed_rpm,
           (int)Chassis_Info->Chassis_Motor_Measure[i]->angle,
           (int)Chassis_Info->Chassis_Motor_Measure[i]->last_angle,
           (int)Chassis_Info->Chassis_Motor_Measure[i]->total_angle,
           //Motor_Angle_Cal(&motor_can1[0],294912)
           (int)get_total_angle(&motor_can1[i])
          );
#elif  INFO_PRINT == INFO_WHEEL_DIR
    printf("speed_rpm:%6d  angle:%6.2f angle_set:%6.2f  give_current=%6d \r\n",
           (int)Chassis_Info->Wheel_Dir[i].speed,
           Chassis_Info->Wheel_Dir[i].angle,
           Chassis_Info->Wheel_Dir[i].angle_set,
           Chassis_Info->Wheel_Dir[i].give_current);
#elif INFO_PRINT == INFO_WHEEL_SPEED
    printf("speed_rpm:%6d speed_set:%6d give_current=%6d\r\n",
           (int)Chassis_Info->Wheel_Speed[i].speed,
           (int)Chassis_Info->Wheel_Speed[i].speed_set,
           Chassis_Info->Wheel_Speed[i].give_current);
#elif INFO_PRINT == INFO_RC
    printf("ch1:%4d ch2:%4d ch3:%4d ch4:%4d ch5:%4d ch6:%4d \r\n",
           Chassis_Info->Chassis_RC->ch1,
           Chassis_Info->Chassis_RC->ch2,
           Chassis_Info->Chassis_RC->ch3,
           Chassis_Info->Chassis_RC->ch4,
           Chassis_Info->Chassis_RC->ch5,
           Chassis_Info->Chassis_RC->ch6);
#elif INFO_PRINT == INFO_V_SET
    printf("Vx_Set:%6d Vy_Set:%6d Vw_Set:%6d  \r\n",
           (int)Chassis_Info->Vx_Set,
           (int)Chassis_Info->Vy_Set,
           (int)Chassis_Info->Vw_Set);
#elif INFO_PRINT == VOFA_WHEEL_DIR
    printf("%6f,%6f\r\n",
           Chassis_Info->Wheel_Dir[i].angle,
           Chassis_Info->Wheel_Dir[i].angle_set);

#elif INFO_PRINT == VOFA_WHEEL_SPEED
    printf("%6f,%6f\r\n",
           Chassis_Info->Wheel_Speed[i].speed,
           Chassis_Info->Wheel_Speed[i].speed_set);

#endif

}

/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     Chassis_Move_Init:"chassis_move_t"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *Chassis_Move_Init)
{
    if (Chassis_Move_Init == NULL)
    {
        return;
    }
    //底盘速度环pid值
    //const static fp32 motor_speed_pid[3] = {WHEEL_SPEED_PID_SPEED_KP, WHEEL_SPEED_PID_SPEED_KI, WHEEL_SPEED_PID_SPEED_KD};
    //底盘开机状态为原始
    Chassis_Move_Init->Chassis_Mode = CHASSIS_NO_SET;
    //获取遥控器指针
    Chassis_Move_Init->Chassis_RC = get_remote_control_point();
		Chassis_Move_Init->my_chassis_rc = get_my_usart_rc_point();
    //获取定位指针
		Chassis_Move_Init->Chassis_OPS = get_location_control_point();
    //设置底盘电机id 和 type
		
		Chassis_Move_Init->Wheel_Speed[0].id = DJ_CAN1_M0;
		Chassis_Move_Init->Wheel_Speed[1].id = DJ_CAN1_M1;
		Chassis_Move_Init->Wheel_Speed[2].id = DJ_CAN1_M2;
		Chassis_Move_Init->Wheel_Speed[3].id = DJ_CAN1_M3;
		Chassis_Move_Init->Wheel_Speed[0].type = DJ_M3508;
		Chassis_Move_Init->Wheel_Speed[1].type = DJ_M3508;
		Chassis_Move_Init->Wheel_Speed[2].type = DJ_M3508;
		Chassis_Move_Init->Wheel_Speed[3].type = DJ_M3508;
		
		
		Chassis_Move_Init->Wheel_Dir[0].id = DJ_CAN1_M4;
		Chassis_Move_Init->Wheel_Dir[1].id = DJ_CAN1_M5;
		Chassis_Move_Init->Wheel_Dir[2].id = DJ_CAN1_M6;
		Chassis_Move_Init->Wheel_Dir[3].id = DJ_CAN1_M7;
		Chassis_Move_Init->Wheel_Dir[0].type = DJ_M2006;
		Chassis_Move_Init->Wheel_Dir[1].type = DJ_M2006;
		Chassis_Move_Init->Wheel_Dir[2].type = DJ_M2006;
		Chassis_Move_Init->Wheel_Dir[3].type = DJ_M2006;
		
//    for(uint8_t i=0; i<8; i++)
//    {
//        Chassis_Move_Init->Chassis_Motor_Measure[i] = get_chassis_motor_measure_point(i);
//    }
    //初始化底盘pid
	 
//	 //驱动轮速度环
//#define WHEEL_SPEED_PID_SPEED_KP 5.0f
//#define WHEEL_SPEED_PID_SPEED_KI 0.4f
//#define WHEEL_SPEED_PID_SPEED_KD 0.0f
//#define WHEEL_SPEED_PID_SPEED_MAX_OUT 10000
//#define WHEEL_SPEED_PID_SPEED_MAX_IOUT 2000.0f
////转向轮角度环
//#define WHEEL_DIR_PID_POSTION_KP 400.0f
//#define WHEEL_DIR_PID_POSTION_KI 0.0f
//#define WHEEL_DIR_PID_POSTION_KD 10.0f
//#define WHEEL_DIR_PID_POSTION_MAX_OUT 10000
//#define WHEEL_DIR_PID_POSTION_MAX_IOUT 2000.0f
////转向轮速度环
//#define WHEEL_DIR_PID_SPEED_KP 4.5f
//#define WHEEL_DIR_PID_SPEED_KI 0.08f
//#define WHEEL_DIR_PID_SPEED_KD 1.5f
//#define WHEEL_DIR_PID_SPEED_MAX_OUT 10000
//#define WHEEL_DIR_PID_SPEED_MAX_IOUT 2000.0f
//    for (uint8_t i = 0; i < 4; i++)
//    {
////        PID_struct_init(&Chassis_Move_Init->Wheel_Speed[i].pid_speed,POSITION_PID, 
////								WHEEL_SPEED_PID_SPEED_MAX_OUT,            //驱动轮最大输出       10000
////								WHEEL_SPEED_PID_SPEED_MAX_IOUT,				//积分限幅             2000
////                        WHEEL_SPEED_PID_SPEED_KP,                 //驱动轮速度Kp         5.00
////							   WHEEL_SPEED_PID_SPEED_KI,  					//驱动轮速度Ki         0.04
////							   WHEEL_SPEED_PID_SPEED_KD);  					//驱动轮速度Kd         0.00
////		 
////		  PID_struct_init(&Chassis_Move_Init->Wheel_Dir[i].pid_pos,POSITION_PID, 
////								WHEEL_DIR_PID_POSTION_MAX_OUT,             //转向轮最大输出       10000
////								WHEEL_DIR_PID_POSTION_MAX_IOUT,            //积分限幅             2000
////                        WHEEL_DIR_PID_POSTION_KP,                  //转向轮角度Kp         400.00
////							   WHEEL_DIR_PID_POSTION_KI,                  //转向轮角度Ki         0.00
////							   WHEEL_DIR_PID_POSTION_KD);                 //转向轮角度Kd         10.00
////		  PID_struct_init(&Chassis_Move_Init->Wheel_Dir[i].pid_speed,POSITION_PID, 
////								WHEEL_DIR_PID_SPEED_MAX_OUT,               //转向轮最大输出       10000
////								WHEEL_DIR_PID_SPEED_MAX_IOUT,              //积分限幅             2000
////                        WHEEL_DIR_PID_SPEED_KP,                    //转向轮速度Kp         4.50
////							   WHEEL_DIR_PID_SPEED_KI,                    //转向轮速度Ki         0.08
////							   WHEEL_DIR_PID_SPEED_KD);                   //转向轮速度Kd         0.01
//		 

//    }
    //初始化底盘角度PID
    PID_Init(&Chassis_Move_Init->chassis_angle_pid, PID_POSITION_NULL, 10000, 2000,
                    1.50f);

    //最大 最小速度
    Chassis_Move_Init->Vx_Max_Speed = 10;
    Chassis_Move_Init->Vx_Min_Speed = -10;
    Chassis_Move_Init->Vy_Max_Speed = 10;
    Chassis_Move_Init->Vy_Min_Speed = -10;

    //更新一下数据
    chassis_feedback_update(Chassis_Move_Init);


}

/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     Chassis_Move_Mode:"chassis_move_t"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *Chassis_Move_Mode)
{
    if (Chassis_Move_Mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(Chassis_Move_Mode);
}
/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，坐标,机器人速度
  * @param[out]     Chassis_Move_Update:"chassis_move_t"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *Chassis_Move_Update)
{
    if (Chassis_Move_Update == NULL)
    {
        return;
    }

    //底盘位置更新
    Chassis_Move_Update->Postion_X = Chassis_Move_Update->Chassis_OPS->pos_x;
    Chassis_Move_Update->Postion_Y = Chassis_Move_Update->Chassis_OPS->pos_y;

    //底盘角度更新
    Chassis_Move_Update->Chassis_Yaw=Chassis_Move_Update->Chassis_OPS->zangle;


    Chassis_Move_Update->Postion_X_Set = Chassis_Move_Update->Chassis_OPS->set_pos_x;
    Chassis_Move_Update->Postion_Y_Set=Chassis_Move_Update->Chassis_OPS->set_pos_y;
    Chassis_Move_Update->Chassis_Yaw_Set=Chassis_Move_Update->Chassis_OPS->set_zangle;



}


#define sin60 0.866f  //sin60
#define cos60 0.5f    //cos60
#define PI 3.1415926f
#define sin45 0.707107f  //sin45
#define cos45 0.707107f  //cos45
/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     Chassis_Move_Control:"chassis_move_t"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *Chassis_Move_Control)
{
    if (Chassis_Move_Control == NULL)
    {
        return;
    }
    fp32 Vx_Set = 0.0f, Vy_Set = 0.0f, Vw_Set=0.0f;
    //获取三个控制设置值
    chassis_behaviour_control_set(&Vx_Set, &Vy_Set, &Vw_Set, Chassis_Move_Control);  //求得Vx,Vy,Angle

    Chassis_Move_Control->Vx_Set =Vx_Set;
    Chassis_Move_Control->Vy_Set =Vy_Set;
    Chassis_Move_Control->Vw_Set =Vw_Set;


}




////						3	  **************   2
////								*          *        ︿   V_Y
////								 *        *         |
////								  *      *          |             //自转速度 顺时针为正
////									*    *           ----->  V_X
////									 *  *
////									  *
////                           1

static void chassis_kinematics_omni3_speed(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set, fp32 Wheel_Speed[4])
{
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    /*运动学逆解*/
    Wheel_Speed[0] = -Vx_set + L * Vw_Set;
    Wheel_Speed[1] = cos60 * Vx_set - sin60 * Vy_set + L * Vw_Set;
    Wheel_Speed[2] = cos60 * Vx_set + sin60 * Vy_set + L * Vw_Set;
    Wheel_Speed[3] = 0;
}
static void chassis_kinematics_omni4_speed(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set, fp32 Wheel_Speed[4])
{
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    /*运动学逆解*/
    Wheel_Speed[0] = -cos45 * Vx_set - sin45 * Vy_set + L * Vw_Set;
    Wheel_Speed[1] = -cos45 * Vx_set + sin45 * Vy_set + L * Vw_Set;
    Wheel_Speed[2] =  cos45 * Vx_set + sin45 * Vy_set + L * Vw_Set;
    Wheel_Speed[3] =  cos45 * Vx_set - sin45 * Vy_set + L * Vw_Set;
}
#define WHEEL_PERIMETER      (0.2f)  //车轮周长
#define CHASSIS_DECELE_RATIO (19)   //底盘减速比
#define Radius  (40)
static void chassis_kinematics_Mecanum_speed(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set, fp32 Wheel_Speed[4]){

	  Wheel_Speed[0] = Vx_set - Vy_set - (r_x + r_y)*Vw_Set;
    Wheel_Speed[1] = Vx_set + Vy_set + (r_x + r_y)*Vw_Set;
    Wheel_Speed[2] = Vx_set + Vy_set - (r_x + r_y)*Vw_Set;
    Wheel_Speed[3] = Vx_set - Vy_set + (r_x + r_y)*Vw_Set;
}
static void chassis_kinematics_wheel3_speed(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set, fp32 Wheel_Speed[4], fp32 Wheel_Angle[4])
{
	
}
static void chassis_kinematics_wheel4_speed(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set, fp32 Wheel_Speed[4], fp32 Wheel_Angle[4])
{
    float wheel_rpm_ratio; //车轮转速比
    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * CHASSIS_DECELE_RATIO * 1000;
    wheel_rpm_ratio=1;
    //                 60  / (0.2*3.14)  *19*1000z                         pik
    //速度计算
    Wheel_Speed[0] = sqrt(pow(Vy_set + Vw_Set * Radius * 0.707107f, 2)
                          + pow(Vx_set - Vw_Set * Radius * 0.707107f, 2)
                         ) * wheel_rpm_ratio;
    Wheel_Speed[1] = sqrt(pow(Vy_set - Vw_Set * Radius * 0.707107f, 2)
                          + pow(Vx_set - Vw_Set * Radius * 0.707107f, 2)
                         ) * wheel_rpm_ratio;
    Wheel_Speed[2] = sqrt(pow(Vy_set - Vw_Set * Radius * 0.707107f, 2)
                          + pow(Vx_set + Vw_Set * Radius * 0.707107f, 2)
                         ) * wheel_rpm_ratio;
    Wheel_Speed[3] = sqrt(pow(Vy_set + Vw_Set * Radius * 0.707107f, 2)
                          + pow(Vx_set + Vw_Set * Radius * 0.707107f, 2)
                         ) * wheel_rpm_ratio;
    //角度计算

    Wheel_Angle[0] = atan2((Vx_set - Vw_Set * Radius * 0.707107f),(Vy_set + Vw_Set * Radius * 0.707107f))
                     * 180.0f / PI;
    Wheel_Angle[1] = atan2((Vx_set - Vw_Set * Radius * 0.707107f),(Vy_set - Vw_Set * Radius * 0.707107f))
                     * 180.0f / PI;
    Wheel_Angle[2] = atan2((Vx_set + Vw_Set * Radius * 0.707107f),(Vy_set - Vw_Set * Radius * 0.707107f))
                     * 180.0f / PI;
    Wheel_Angle[3] = atan2((Vx_set + Vw_Set * Radius * 0.707107f),(Vy_set + Vw_Set * Radius * 0.707107f))
                     * 180.0f / PI;
										 
		for(int i = 0; i<4 ;i++)    //模型结算后，有些转向看着不舒服，自行加if else判断修改
		{
			if(Wheel_Angle[i] < -90 && Wheel_Angle[i] >-181)
			{
				Wheel_Angle[i] += 180;
				Wheel_Speed[i] = -Wheel_Speed[i];
			}
			else if(Wheel_Angle[i] > 90 && Wheel_Angle[i] <=181)
			{
				Wheel_Angle[i] -= 180;
				Wheel_Speed[i] = -Wheel_Speed[i];
			}
		}
}


static void  chassis_set_kinematics(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set,chassis_move_t *Chassis_Move_Kinematics)
{
    if (Chassis_Move_Kinematics == NULL)
    {
        return;
    }
    float Wheel_Speed[4]= {0};
    float Wheel_Angle[4]= {0};

    if (Chassis_Move_Kinematics->Chassis_Kinematics_Mode == Omni3_Car)
    {
        chassis_kinematics_omni3_speed(Vx_set,Vy_set,Vw_Set,Wheel_Speed);
    }
    else if (Chassis_Move_Kinematics->Chassis_Kinematics_Mode == Omni4_Car)
    {
        chassis_kinematics_omni4_speed(Vx_set,Vy_set,Vw_Set,Wheel_Speed);
    }
    else if (Chassis_Move_Kinematics->Chassis_Kinematics_Mode == Wheel3_Car)
    {
        chassis_kinematics_wheel3_speed(Vx_set,Vy_set,Vw_Set,Wheel_Speed,Wheel_Angle);
    }
    else if (Chassis_Move_Kinematics->Chassis_Kinematics_Mode == Wheel4_Car)
    {
        chassis_kinematics_wheel4_speed(Vx_set,Vy_set,Vw_Set,Wheel_Speed,Wheel_Angle);
    }
		else if(Chassis_Move_Kinematics->Chassis_Kinematics_Mode == Mecanum_Car)
		{
				chassis_kinematics_Mecanum_speed(Vx_set,Vy_set,Vw_Set,Wheel_Speed);
		}
    for(uint8_t i=0; i<4; i++)
    {
        Chassis_Move_Kinematics->Wheel_Speed[i].speed_set = Wheel_Speed[i];
        Chassis_Move_Kinematics->Wheel_Dir[i].angle_set = Wheel_Angle[i];
    }

}
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *Chassis_Move_Control_Loop)
{
    //运动分解
    chassis_set_kinematics(Chassis_Move_Control_Loop->Vx_Set,
                           Chassis_Move_Control_Loop->Vy_Set,
                           Chassis_Move_Control_Loop->Vw_Set,
                           Chassis_Move_Control_Loop);

    uint8_t i = 0;
    //计算pid
    //驱动轮
    //发送控制电流
    if(Chassis_Move_Control_Loop->Chassis_Mode == CHASSIS_ZERO_FORCE)
    {
					set_motors( Chassis_Move_Control_Loop,0);
    }
    else {
					set_motors( Chassis_Move_Control_Loop,1);
    }

}

void set_pos(float X_target, float Y_target, float Angle_target,
             float X_current, float Y_current, float Angle_current,
							 chassis_move_t *Chassis_Move_Control_Loop) 				 
{
		int Kp_V0 = 3;
    int Kp_W = 1500;//40000

    int V0_MAX = 200;
    int V0_MIN = -200;
    int W_MAX = 200;
    int W_MIN = -200;

    //底盘速度
    float Vx = 0; //x方向速度
    float Vy = 0; //y方向速度
    float V0 = 0; //底盘运动方向速度
    float W = 0;  //自转速度 顺时针为正
    int deta=0;
    static float V0_last = 0; //上一时刻底盘运动速度大小,用于速度的增量限幅
    //底盘速度方向角度
    float Angle_speed;          //底盘速度方向与X轴正方向夹角 [0 , 180]
    float Angle_speed_current;  //感知魔块读取的角度(yaw) [-180,180]

    //角度转换为弧度
    //float PI = 3.1415926;
    Angle_target = Angle_target / 180.0 * PI;
    Angle_current = Angle_current / 180.0 * PI;
    //Angle_speed = Angle_current;
    /**计算速度方向*/
    if (X_current == X_target)
        Angle_speed = PI / 2;
    else
        Angle_speed = atan((Y_target - Y_current) / (X_target - X_current));
    if (Angle_speed < 0)
        Angle_speed = Angle_speed + PI;
    if (Y_target > Y_current)
        Angle_speed_current = PI / 2 - (Angle_speed + Angle_current);
    else
        Angle_speed_current = 3 * PI / 2 - (Angle_speed + Angle_current);

#define abs(x)        ((x>0)? (x): (-x))

    /**求V0*/
    V0 = Kp_V0 * sqrt((Y_target - Y_current) * (Y_target - Y_current) + (X_target - X_current) * (X_target - X_current));
    /**V0限幅*/
    deta = V0 - V0_last;
    if (deta > 50) {
        deta = 50 * deta / abs(deta);
    }
    V0 = V0_last + deta;
    V0_last = V0;
    /** 计算Vx Vy W */
    V0 = V0 < V0_MAX ? V0 : V0_MAX;
    V0 = V0 > V0_MIN ? V0 : V0_MIN;

    Vx = V0 * sin(Angle_speed_current);
    Vy = V0 * cos(Angle_speed_current);

    W = Kp_W * (Angle_target - Angle_current);
    W = W < W_MAX ? W : W_MAX;
    W = W > W_MIN ? W : W_MIN;
   // printf("y:%d	x:%d	w:%d\r\n",(int)Vy,(int)Vx,(int)W);
		
   Set_speed(Vx, Vy, W,Chassis_Move_Control_Loop);
}



 void Set_speed(int Velocity_X, int Velocity_Y, int W,chassis_move_t *Chassis_Move_Control_Loop) {
    uint8_t i = 0;
		Chassis_Move_Control_Loop->Vx_Set=Velocity_X;
		Chassis_Move_Control_Loop->Vy_Set=Velocity_Y;
		Chassis_Move_Control_Loop->Vw_Set=W;
	  //运动分解
    chassis_set_kinematics(Chassis_Move_Control_Loop->Vx_Set,
                           Chassis_Move_Control_Loop->Vy_Set,
                           Chassis_Move_Control_Loop->Vw_Set,
                           Chassis_Move_Control_Loop);


		
    //发送控制电流
    if(Chassis_Move_Control_Loop->Chassis_Mode == CHASSIS_ZERO_FORCE)
    {
					set_motors( Chassis_Move_Control_Loop,0);
    }
    else 
		{
					set_motors( Chassis_Move_Control_Loop,1);

    }
	
}


float get_speed(uint8_t i)
{

}

float get_angle(uint8_t i)
{

}

void set_motors(chassis_move_t *Chassis_Move_Control_Loop,uint32_t is_true)
{
	if(Chassis_Move_Control_Loop->Chassis_Mode==CHASSIS_NO_SET){
		elog_raw_output("chassis no set mode!!!\n");
		return;
	}
	else
	{
		if(is_true)
		{
			
			if (Chassis_Move_Control_Loop->Chassis_Kinematics_Mode == Omni3_Car)
				{
					for(uint8_t i=0;i<3;i++)
					{
					 DJ_Set_Motor_Speed(Chassis_Move_Control_Loop->Wheel_Speed[i].id,
														Chassis_Move_Control_Loop->Wheel_Speed[i].speed_set,
														Chassis_Move_Control_Loop->Wheel_Speed[i].type);
					
					}
				}
				else if (Chassis_Move_Control_Loop->Chassis_Kinematics_Mode == Omni4_Car)
				{
					for(uint8_t i=0;i<4;i++)
					{
					
					 DJ_Set_Motor_Speed(Chassis_Move_Control_Loop->Wheel_Speed[i].id,
														Chassis_Move_Control_Loop->Wheel_Speed[i].speed_set,
														Chassis_Move_Control_Loop->Wheel_Speed[i].type);
					
					}
				}
				else if (Chassis_Move_Control_Loop->Chassis_Kinematics_Mode == Wheel3_Car)
				{
					for(uint8_t i=0;i<3;i++)
					{
					 DJ_Set_Motor_Speed(Chassis_Move_Control_Loop->Wheel_Speed[i].id,
														Chassis_Move_Control_Loop->Wheel_Speed[i].speed_set,
														Chassis_Move_Control_Loop->Wheel_Speed[i].type);
					DJ_Set_Motor_Position(Chassis_Move_Control_Loop->Wheel_Dir[i].id,
														Chassis_Move_Control_Loop->Wheel_Dir[i].angle_set,
														Chassis_Move_Control_Loop->Wheel_Dir[i].type);
					
					}
				}
				else if (Chassis_Move_Control_Loop->Chassis_Kinematics_Mode == Wheel4_Car)
				{
					for(uint8_t i=0;i<4;i++)
					{
					 DJ_Set_Motor_Speed(Chassis_Move_Control_Loop->Wheel_Speed[i].id,
														Chassis_Move_Control_Loop->Wheel_Speed[i].speed_set,
														Chassis_Move_Control_Loop->Wheel_Speed[i].type);
					 DJ_Set_Motor_Position(Chassis_Move_Control_Loop->Wheel_Dir[i].id,
														Chassis_Move_Control_Loop->Wheel_Dir[i].angle_set,
														Chassis_Move_Control_Loop->Wheel_Dir[i].type);
					}
					
				}
				else if(Chassis_Move_Control_Loop->Chassis_Kinematics_Mode == Mecanum_Car)
				{
					for(uint8_t i=0;i<4;i++)
					{
					 DJ_Set_Motor_Speed(Chassis_Move_Control_Loop->Wheel_Speed[i].id,
														Chassis_Move_Control_Loop->Wheel_Speed[i].speed_set,
														Chassis_Move_Control_Loop->Wheel_Speed[i].type);
					
					}
				}
		}
		else
		{
				for(uint8_t i=0;i<4;i++)
				{
					DJ_Set_Motor_Current(Chassis_Move_Control_Loop->Wheel_Speed[i].id,
													Chassis_Move_Control_Loop->Wheel_Speed[i].give_current,
													Chassis_Move_Control_Loop->Wheel_Speed[i].type);
					DJ_Set_Motor_Current(Chassis_Move_Control_Loop->Wheel_Dir[i].id,
														Chassis_Move_Control_Loop->Wheel_Dir[i].give_current,
														Chassis_Move_Control_Loop->Wheel_Dir[i].type);		
				}
			
		}
	}
}



















