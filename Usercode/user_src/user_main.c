/**
 * @Author: szf
 * @Date: 2023-02-22 10:59:01
 * @LastEditTime: 2023-02-22 19:27:40
 * @LastEditors: szf
 * @Description: ER
 * @FilePath: \ER\Usercode\user_src\user_main.c
 * @WeChat:szf13373959031
 **/

#include "user_main.h"
#include "wtr_dji.h"
#include "wtr_can.h"
#include "wtr_uart.h"
#include "wtr_mavlink.h"
#include "state_management.h"
#include "wtr_ads1256.h"
#include "chassis_communicate.h"
#include "chassis_machine.h"
#include "chassis_servo.h"
#include "chassis_perception.h"


/**
 * @description: 初始化与开启线程
 * @author: szf
 * @date:
 * @return {void}
 */
void StartDefaultTask(void const *argument)
{
    // 初始化
    StateInit();
    ADS1256_Init(); // DT35距离传感器
    CANFilterInit(&hcan1);
    hDJI[0].motorType = M2006;
    hDJI[1].motorType = M2006;
    hDJI[2].motorType = M2006;
    hDJI[3].motorType = M2006;
    hDJI[4].motorType = M2006;
    hDJI[5].motorType = M2006;
    hDJI[6].motorType = M2006;
    DJI_Init();                                             // 大疆电机初始化
    wtrMavlink_BindChannel(&huart1, MAVLINK_COMM_0);        // MAVLINK初始化
    CtrlDataSender_Init(&huart2, MAVLINK_COMM_1);           // 遥控器初始化
    HAL_UART_Receive_DMA(&huart3, JoyStickReceiveData, 18); // DMA接收AS69
    // 开启线程
    //  OwChassisTaskStart(&ControllerData);// 全向轮底盘控制线程
    PerceptionTaskStart(&ControllerData); // 底盘感知定位线程
    ChassisTaskStart(&ControllerData);    // 舵轮底盘控制线程
    // CtrlDataSender_Start(&ControllerData);// 遥控器线程

    for (;;) {
        osDelay(1);
    }
}

