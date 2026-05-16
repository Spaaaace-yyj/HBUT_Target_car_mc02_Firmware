#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "algorithm/controller.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"

static Publisher_t *Chassis_Pub;//发布底盘的数据
static Subscriber_t *Chassis_Sub;//用于订阅底盘的控制命令

static  Chassis_Ctrl_Cmd_s Chassis_Cmd_Recv;//底盘接收到的控制命令
static  Chassis_Upload_Data_s Chassis_Feedback_Data;//底盘回传的反馈数据

static DJIMotorInstance *yaw_motor;
static PIDInstance chassis_follow_yaw_controller;

static float Chassis_Target_Velocity = 0,Chassis_Target_Angular_Velocity = 0;//底盘的目标线速度和角速度
static volatile float Chassis_Target_VLF = 0,Chassis_Target_VLB = 0,Chassis_Target_VRF = 0,Chassis_Target_VRB=0;//每个轮子的目标速度

void ChassisInit() {
    //四个轮子的参数是一样的，只是ID和转速有差别从后往前看顺时针左上为ID1
    Motor_Init_Config_s Chassis_Motor_config ={
        .can_init_config = &hcan2, // 修改为对应的CAN接口
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 70.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 5000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter | PID_DerivativeFilter,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.5f,
                .Derivative_LPF_RC = 1.0f,
            },
            .current_PID = {
                .Kp = 1.0, // 0.5
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,
        },
        .motor_type = GM6020,
    };
    Chassis_Motor_config.can_init_config.tx_id = 2;//左前
    Chassis_Motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    yaw_motor = DJIMotorInit(&Chassis_Motor_config);



    Chassis_Sub = SubRegister("Chassis_Cmd",sizeof(Chassis_Ctrl_Cmd_s));
    Chassis_Pub = PubRegister("Chassis_Feed",sizeof(Chassis_Upload_Data_s));

}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    SubGetMessage(Chassis_Sub,&Chassis_Cmd_Recv);//获取底盘命令信息
    DJIMotorEnable(yaw_motor);
    DJIMotorSetRef(yaw_motor, 300);



    PubPushMessage(Chassis_Pub,(void *)&Chassis_Feedback_Data);//发布底盘反馈数据,目前还没有填充数据,后续增加
}