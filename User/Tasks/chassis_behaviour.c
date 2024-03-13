#include "chassis_behaviour.h"
#include "math.h"
uint16_t Vz=0;
#define CHASSIS_OPEN_RC_SCALE 0.2 // in CHASSIS_OPEN mode, multiply the value. 在chassis_open 模型下，遥控器乘以该比例发送到can上
extern float yaw_current;
#define yaw yaw_current/180*3.14f
float beta=0;
/**
  * @brief          通过判断角速度来判断云台是否到达极限位置
  * @param          对应轴的角速度，单位rad/s
  * @param          计时时间，到达GIMBAL_CALI_STEP_TIME的时间后归零
  * @param          记录的角度 rad
  * @param          反馈的角度 rad
  * @param          记录的编码值 raw
  * @param          反馈的编码值 raw
  * @param          校准的步骤 完成一次 加一
  */
#define chassis_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }





static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
		for(uint8_t i = 0; i<4;i++){
    chassis_move_rc_to_vector->Wheel_Speed[i].give_current = 0.0f;
		chassis_move_rc_to_vector->Wheel_Dir[i].give_current = 0.0f;
		}
}
static void chassis_no_move_control(fp32 *Vx_Set, fp32 *Vy_Set, fp32 *Vw_Set, chassis_move_t *Chassis_Move_Rc_to_Vector)
{
    if (Vx_Set == NULL || Vy_Set == NULL || Vw_Set == NULL || Chassis_Move_Rc_to_Vector == NULL)
    {
        return;
    }
		
        *Vx_Set = 0.0f;
        *Vy_Set = 0.0f;
        *Vw_Set = 0.0f;
}

static void chassis_remote_move_control(fp32 *Vx_Set, fp32 *Vy_Set, fp32 *Vw_Set, chassis_move_t *Chassis_Move_Rc_to_Vector)
{
    if (Vx_Set == NULL || Vy_Set == NULL || Vw_Set == NULL || Chassis_Move_Rc_to_Vector == NULL)
    {
        return;
    }


//    *Vx_Set = Chassis_Move_Rc_to_Vector->Chassis_RC->ch1 *CHASSIS_OPEN_RC_SCALE;
//    *Vy_Set = Chassis_Move_Rc_to_Vector->Chassis_RC->ch2 *CHASSIS_OPEN_RC_SCALE;
//    *Vw_Set = Chassis_Move_Rc_to_Vector->Chassis_RC->ch3 *CHASSIS_OPEN_RC_SCALE;

		*Vx_Set = Chassis_Move_Rc_to_Vector->my_chassis_rc->vx *CHASSIS_OPEN_RC_SCALE*5;
    *Vy_Set = Chassis_Move_Rc_to_Vector->my_chassis_rc->vy *CHASSIS_OPEN_RC_SCALE*5;
    *Vw_Set =- Chassis_Move_Rc_to_Vector->my_chassis_rc->vw *CHASSIS_OPEN_RC_SCALE*5;
//		
//		*Vx_Set =*Vy_Set * cos(yaw_current/180*3.14f) + *Vx_Set * sin(yaw_current/180*3.14f); 
//    *Vy_Set = *Vy_Set * sin(yaw_current/180*3.14f) + *Vx_Set * cos(yaw_current/180*3.14f);
		float vx= *Vx_Set;
		float vy=*Vy_Set;
		*Vx_Set =(-1.f)*vy * sin(yaw_current/180*3.14f) + vx * cos(yaw_current/180*3.14f); 
    *Vy_Set = vy * cos(yaw_current/180*3.14f) + vx * sin(yaw_current/180*3.14f);
//		beta = atan(*Vy_Set/(*Vx_Set));
//		Vz = sqrt(*Vx_Set * (*Vx_Set) + (*Vy_Set) *(*Vy_Set));
//		if(yaw+beta >=0 && yaw+beta <=1.57){
//			*Vx_Set = Vz*cos(yaw+beta);
//			*Vy_Set = Vz*sin(yaw+beta);
//		}else if(yaw +beta >1.57 && yaw +beta <=3.14){
//			*Vx_Set = -1*Vz*cos(3.14f-yaw+beta);
//			*Vy_Set = Vz*sin(3.14f-yaw+beta);
//		}else if(yaw +beta >180 &&yaw +beta<=270){
//			*Vx_Set = -1*Vz*cos(yaw+beta-3.14f);
//			*Vy_Set = -1*Vz*sin(yaw+beta-3.14f);			
//		}else{
//			*Vx_Set = Vz*sin(6.28f-yaw+beta);
//			*Vy_Set = -1*Vz*cos(6.28f-yaw+beta);			
//		}

    return;
}



static void chassis_local_move_control(fp32 *Vx_Set, fp32 *Vy_Set, fp32 *Vw_Set, chassis_move_t *Chassis_Move_Rc_to_Vector)
{
	if (Vx_Set == NULL || Vy_Set == NULL || Vw_Set == NULL || Chassis_Move_Rc_to_Vector == NULL)
    {
        return;
    }
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
		
		float X_target = Chassis_Move_Rc_to_Vector->Postion_X_Set;
	  float Y_target = Chassis_Move_Rc_to_Vector->Postion_Y_Set;
		float Angle_target = Chassis_Move_Rc_to_Vector->Chassis_Yaw_Set;
    float X_current   =Chassis_Move_Rc_to_Vector->Postion_X;
		float Y_current=Chassis_Move_Rc_to_Vector->Postion_Y;
		float Angle_current =Chassis_Move_Rc_to_Vector->Chassis_Yaw;
    //角度转换为弧度
		#define PI  3.1415926f
    Angle_target = Angle_target / 180.0 * PI;
    Angle_current = Angle_current / 180.0 * PI;
    //Angle_speed = Angle_current;
    /**计算速度方向*/
		if (X_current == X_target)
		 Angle_speed = PI / 2.0;
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
		
		*Vx_Set = Vx;
    *Vy_Set = Vy;
    *Vw_Set = W;

    return;
}


//留意，这个底盘行为模式变量
chassis_behaviour_mode_e Chassis_Behaviour_Mode; //= CHASSIS_REMOTE_MOVE;

/**
  * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *Chassis_Move_Mode)
{
    if (Chassis_Move_Mode == NULL)
    {
        return;
    }
		Chassis_Behaviour_Mode = CHASSIS_REMOTE_MOVE;
    //根据行为模式选择一个底盘控制模式
    //添加自己的逻辑判断进入新模式
//    if(Chassis_Move_Mode->Chassis_RC->ch5 == 1)
//    {
//        Chassis_Behaviour_Mode = CHASSIS_ZERO_FORCE;
//    }
//    else if(Chassis_Move_Mode->Chassis_RC->ch5 == 2)
//    {
//        Chassis_Behaviour_Mode = CHASSIS_NO_MOVE;
//    }
//    else if(Chassis_Move_Mode->Chassis_RC->ch5 == 2)
//    {
//        Chassis_Behaviour_Mode = CHASSIS_LOCATING_MOVE;
//    }
//    else if(Chassis_Move_Mode->Chassis_RC->ch5 ==3)
//    {
//        Chassis_Behaviour_Mode = CHASSIS_REMOTE_MOVE;
//    }


    //根据行为模式选择一个底盘控制模式
    if (Chassis_Behaviour_Mode == CHASSIS_ZERO_FORCE) //悬空
    {
        Chassis_Move_Mode->Chassis_Mode = CHASSIS_ZERO_FORCE;
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_NO_MOVE) //固定
    {
        Chassis_Move_Mode->Chassis_Mode = CHASSIS_NO_MOVE;
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_LOCATING_MOVE) //定位
    {
        Chassis_Move_Mode->Chassis_Mode = CHASSIS_LOCATING_MOVE;
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_REMOTE_MOVE) //遥控
    {
        Chassis_Move_Mode->Chassis_Mode = CHASSIS_REMOTE_MOVE;
    }else if (Chassis_Behaviour_Mode == CHASSIS_DEBUG_MOED) //调试
    {
        Chassis_Move_Mode->Chassis_Mode = CHASSIS_DEBUG_MOED;
    }
}

//	 CHASSIS_ZERO_FORCE,                   //底盘无力, 跟没上电那样
//  CHASSIS_NO_MOVE,                      //底盘保持不动
//	 CHASSIS_LOCATING_MOVE,                //底盘定位
//	 CHASSIS_REMOTE_MOVE                   //遥控控制
void chassis_behaviour_control_set(fp32 *Vx_Set, fp32 *Vy_Set, fp32 *Vw_Set, chassis_move_t *Chassis_Move_Rc_To_Vector)
{
    if (Vx_Set == NULL || Vy_Set == NULL || Vw_Set == NULL || Chassis_Move_Rc_To_Vector == NULL)
    {

        return;
    }
    //根据行为模式选择一个底盘控制模式

    if (Chassis_Behaviour_Mode == CHASSIS_ZERO_FORCE)  //底盘无力, 跟没上电那样
    {
        chassis_zero_force_control(Vx_Set, Vy_Set, Vw_Set, Chassis_Move_Rc_To_Vector);
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_NO_MOVE)  //固定底盘 不移动
    {
        chassis_no_move_control(Vx_Set, Vy_Set, Vw_Set, Chassis_Move_Rc_To_Vector);
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_LOCATING_MOVE)//底盘定位，注意哦
    {
			chassis_local_move_control(Vx_Set, Vy_Set, Vw_Set, Chassis_Move_Rc_To_Vector);
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_REMOTE_MOVE)
    {
        chassis_remote_move_control(Vx_Set, Vy_Set, Vw_Set, Chassis_Move_Rc_To_Vector);
    }


    //遥控器设置模式
    //当云台在某些模式下，像初始化， 底盘不动
//    if (gimbal_cmd_to_chassis_stop())
//    {
//        chassis_behaviour_mode = CHASSIS_NO_MOVE;
//    }

}


