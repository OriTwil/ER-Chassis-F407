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
#include "wtr_mavlink.h"
#include "wtr_uart.h"

mavlink_posture_t mav_posture_temp;
void CommunicateInit()
{
    //todo 完善通信部分
    // WTR_MAVLink_Init(huart, chan);
    wtrMavlink_BindChannel(&huart4, MAVLINK_COMM_0);        // MAVLINK初始化
    wtrMavlink_BindChannel(&huart5, MAVLINK_COMM_1);        // MAVLINK初始化
    wtrMavlink_StartReceiveIT(MAVLINK_COMM_0);              // 以mavlink接收上位机通过串口发送的消息
    wtrMavlink_StartReceiveIT(MAVLINK_COMM_1);              // 以mavlink接收遥控器
    HAL_UART_Receive_DMA(&huart1, JoyStickReceiveData, 18); // DMA接收AS69
}

void CommunicateTask(void const *argument)
{
    while (1) {
        vPortEnterCritical();
        mav_posture_temp = mav_posture;
        vPortExitCritical();
        
        mavlink_msg_posture_send_struct(MAVLINK_COMM_0,&mav_posture_temp);
        osDelay(10);
    }
}

void CommunicateStart()
{
    osThreadDef(communicate, CommunicateTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(communicate), NULL);
}
