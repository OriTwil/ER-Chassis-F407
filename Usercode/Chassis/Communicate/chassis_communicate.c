/**
 * @file controller_data_sender.c
 * @author TITH (1023515576@qq.com)
 * @brief
 * @version 0.1
 * @date 2022-07-22
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "chassis_communicate.h"
#include "cmsis_os.h"
#include "user_config.h"
#include "wtr_mavlink.h"
#include "wtr_uart.h"
#include "chassis_machine.h"
#include "chassis_commen.h"
#include "chassis_remote_control.h"

// 变量定义
mavlink_posture_t mav_posture;
mavlink_control_t control;
mavlink_chassis_to_upper_t chassis_data;

void CommunicateInit()
{
    wtrMavlink_BindChannel(&huart_Computer, MAVLINK_COMM_0); // MAVLINK上位机
    wtrMavlink_BindChannel(&huart_Chassis_to_Upper, MAVLINK_COMM_2); // MAVLINK板间通信
    wtrMavlink_StartReceiveIT(MAVLINK_COMM_0);       // 以mavlink接收上位机通过串口发送的消息
    wtrMavlink_StartReceiveIT(MAVLINK_COMM_2);       // 接收板间通信
}

void CommunicateTask(void const *argument)
{
    while (1) {
        vPortEnterCritical();
        mavlink_posture_t mav_posture_temp           = mav_posture;
        mavlink_chassis_to_upper_t chassis_data_temp = chassis_data;
        mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air;
        vPortExitCritical();

        mavlink_msg_posture_send_struct(MAVLINK_COMM_0, &mav_posture_temp);
        mavlink_msg_chassis_to_upper_send_struct(MAVLINK_COMM_2, &chassis_data_temp);
        mavlink_msg_joystick_air_send_struct(MAVLINK_COMM_2,&msg_joystick_air_temp);
        vTaskDelay(10);
    }
}

void CommunicateStart()
{
    osThreadDef(communicate, CommunicateTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(communicate), NULL);
}
