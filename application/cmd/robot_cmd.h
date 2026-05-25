#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

typedef struct
{
    float enableCar;
    float rotateSpeed;     //当停止旋转的时候将速度设置为0,可以少传一点数据
    float linearSpeed;     //同理，具体模式区分在ESP32处理
    float moveMode;        //区分AB平移和点动，AB模式要STM32做处理
    float startABMove ;
}ESP32_recv_data_s;

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

#endif // !ROBOT_CMD_H