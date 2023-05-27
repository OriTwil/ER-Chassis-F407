/*
 * @Author: szf
 * @Date: 2023-02-23 19:13:13
 * @LastEditTime: 2023-03-29 22:25:16
 * @LastEditors: szf
 * @Description:
 * @FilePath: \ER-Chassis-F407\Usercode\user_src\chassis_perception.c
 * @WeChat:szf13373959031
 */
#include "chassis_perception.h"
#include "wtr_uart.h"
#include "user_config.h"
#include "wtr_ads1256.h"
#include "chassis_commen.h"

PERCEPTION_COMPONENT Perception_component;

/**
 * @description: 线程二：定位系统
 * @author: szf
 * @date:
 * @return {void}
 */
void ChassisPerceptionTask(void const *argument)
{
    vTaskDelay(200);
    for (;;) {
        ADS1256_UpdateDiffData();
        vTaskDelay(10);
    }
}

void PerceptionTaskStart()
{
    osThreadDef(perception, ChassisPerceptionTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(perception), NULL);
}

void PercerptionInit()
{
    // 码盘定位系统通过串口收发信息
    HAL_UART_Receive_IT(&huart_OPS, (uint8_t *)&ch, 1);
    ADS1256_Init(); // DT35距离传感器
}

void PerceptionSwitchState(PERCEPTION_STATE target_perception_state, PERCEPTION_COMPONENT *perception_component)
{
    xSemaphoreTakeRecursive(perception_component->xMutex_perception, (TickType_t)10);
    perception_component->Perception_State = target_perception_state;
    xSemaphoreGiveRecursive(perception_component->xMutex_perception);
}