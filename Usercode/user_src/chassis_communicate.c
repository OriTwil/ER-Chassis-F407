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
#include "state_management.h"
#include "user_config.h"

mavlink_channel_t CtrlDataSendChan = 0;

void CommunicateInit(UART_HandleTypeDef *huart, mavlink_channel_t chan)
{
    // WTR_MAVLink_Init(huart, chan);
    // wtrMavlink_BindChannel(huart, MAVLINK_COMM_1); // MAVLINK初始化
    // wtrMavlink_StartReceiveIT(MAVLINK_COMM_0); // 以mavlink接收上位机通过串口发送的消息
    CtrlDataSendChan = chan;
}

void CommunicateTask(void const *argument)
{
    while (1) {
        vPortEnterCritical();
        // 发送按键通知
        if (1/* 判断按键1是否按下 */) {
            xTaskNotify(g_stateManagementTaskHandle, BUTTON1_NOTIFICATION, eSetBits);
        }
        if (1/* 判断按键2是否按下 */) {
            xTaskNotify(g_stateManagementTaskHandle, BUTTON2_NOTIFICATION, eSetBits);
        }
        if (1/* 判断按键3是否按下 */) {
            xTaskNotify(g_stateManagementTaskHandle, BUTTON3_NOTIFICATION, eSetBits);
        }
        vPortExitCritical();
        osDelay(10);
    }
}

void CommunicateStart()
{
    osThreadDef(communicate, CommunicateTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(communicate), NULL);
}
