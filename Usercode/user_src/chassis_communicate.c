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
// #include "wtr_mavlink.h"
// #include "mavlink_msg_controller.h"

mavlink_channel_t CtrlDataSendChan = 0;

void CtrlDataSender_Init(UART_HandleTypeDef *huart, mavlink_channel_t chan)
{
    // WTR_MAVLink_Init(huart, chan);
    // wtrMavlink_BindChannel(huart, MAVLINK_COMM_1); // MAVLINK初始化
    CtrlDataSendChan = chan;
}

void CtrlDataSender_Task(void const *argument)
{
    // wtrMavlink_StartReceiveIT(MAVLINK_COMM_0); // 以mavlink接收上位机通过串口发送的消息
    while (1) {
        osDelay(10);
    }
}

void CtrlDataSender_Start()
{
    osThreadDef(CtrlSend, CtrlDataSender_Task, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(CtrlSend), NULL);
}
