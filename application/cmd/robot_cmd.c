// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "bsp_usart.h"
#include "seasky_protocol.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usb.h"
#include "cmsis_os.h"
// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

static Publisher_t *Chassis_Cmd_Pub;//底盘控制消息发布者
static Subscriber_t *Chassis_Feed_Sub;//底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s Chassis_Cmd_Send; //发送给底盘应用的信息，包括控制信息
static Chassis_Upload_Data_s Chassis_Fetch_Data;//从底盘应用接收的反馈信息与底盘的运动状态

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Robot_Status_e Robot_State;//机器人整体工作状态

BMI088Instance *bmi088_test;
BMI088_Data_t bmi088_data;

USARTInstance *esp32_uart;
ESP32_recv_data_s esp_recv_data;

static float target_yaw = 0;

// OTA升级测试
 void request_ota_update(void)
 {
     HAL_PWR_EnableBkUpAccess();

     RTC->BKP0R = OTA_MAGIC_VALUE;

     __DSB();
     __ISB();

     NVIC_SystemReset();
 }

void ESP32_uart_callback()
{
    uint16_t flag;
    get_protocol_info(esp32_uart->recv_buff, &flag, (uint8_t *)&esp_recv_data.enableCar);
}

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart9); // 视觉通信串口

    USART_Init_Config_s uart_conf = {
        .recv_buff_size = 64,
        .usart_handle = &huart8,
        .module_callback = ESP32_uart_callback,
    };

    esp32_uart = USARTRegister(&uart_conf);

    Chassis_Cmd_Pub = PubRegister("Chassis_Cmd",sizeof(Chassis_Ctrl_Cmd_s));
    Chassis_Feed_Sub = SubRegister("Chassis_Feed",sizeof(Chassis_Upload_Data_s));

    Robot_State
    =ROBOT_READY;

}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 */
static void RemoteControlSet(void)
{
    if (esp_recv_data.enableCar == 1)
    {
        Chassis_Cmd_Send.rotateSpeed = esp_recv_data.rotateSpeed;
        Chassis_Cmd_Send.LinearSpeed = esp_recv_data.linearSpeed;
    }else
    {
        Chassis_Cmd_Send.rotateSpeed = 0.0f;
        Chassis_Cmd_Send.LinearSpeed = 0.0f;
    }
    if (esp_recv_data.moveMode == 0)
    {
        Chassis_Cmd_Send.chassis_move_mode = CHASSIS_POINT_MOVE;
    }else if (esp_recv_data.moveMode == 1)
    {
        Chassis_Cmd_Send.chassis_move_mode = CHASSIS_AB_MODE;
    }

    Chassis_Cmd_Send.ABModeState = esp_recv_data.startABMove;
    if (Chassis_Cmd_Send.rotateSpeed >= 2.0f)
    {
        Chassis_Cmd_Send.rotateSpeed = 2.0f;
    }else if (Chassis_Cmd_Send.rotateSpeed <= -2.0f)
    {
        Chassis_Cmd_Send.rotateSpeed = -2.0f;
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    VisionSend(&vision_send_data);
    RemoteControlSet();
    // OTA升级测试
     if (esp_recv_data.enableCar >= 1.5)
     {
         HAL_Delay(5000);
         request_ota_update();
     }

    SubGetMessage(Chassis_Feed_Sub, (void *)&Chassis_Fetch_Data);
    PubPushMessage(Chassis_Cmd_Pub, (void *)&Chassis_Cmd_Send);

}


