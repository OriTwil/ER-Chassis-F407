/**
 * @Author: szf
 * @Date: 2023-02-22 10:59:01
 * @LastEditTime: 2023-02-22 19:27:40
 * @LastEditors: szf
 * @Description: ER
 * @FilePath: \ER\Usercode\user_src\usermain.c
 * @WeChat:szf13373959031
 **/

#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "Caculate.h"
#include "wtr_can.h"
#include "DJI.h"
#include "wtr_uart.h"
#include <math.h>
#include "main.h"
#include "usermain.h"
#include "wtr_mavlink.h"
#include "ADS1256.h"
#include <math.h>
#include "usercallback.h"
#include "usercalculate.h"
#include "chassis_control.h"
#include "chassis_driver.h"
#include "chassis_perception.h"
#include "ow_chassis_control.h"
#include "ctrl_data_sender.h"
#include "beep.h"
#include "CLI.h"

// 变量定义
mavlink_control_set_t control;
mavlink_speed_control_status_t v_state;
mavlink_posture_t mav_posture;

PID pid_pos_x;
PID pid_pos_y;
PID pid_vel_w;

PIDType pid_pos_w_pos;
PIDType pid_pos_x_pos;
PIDType pid_pos_y_pos;

uint8_t i = 0;

mavlink_controller_t ControllerData = {0};

/**
 * @description: 初始化与开启线程
 * @author: szf
 * @date:
 * @return {void}
 */
void StartDefaultTask(void const *argument)
{
    //初始化
    ADS1256_Init();// DT35距离传感器
    CANFilterInit(&hcan1);
    hDJI[0].motorType = M2006;
    hDJI[1].motorType = M2006;
    hDJI[2].motorType = M2006;
    hDJI[3].motorType = M2006;
    hDJI[4].motorType = M2006;
	hDJI[5].motorType = M2006;
    hDJI[6].motorType = M2006;
    DJI_Init();// 大疆电机初始化
    wtrMavlink_BindChannel(&huart1, MAVLINK_COMM_0);// MAVLINK初始化
    CtrlDataSender_Init(&huart2, MAVLINK_COMM_1); // 遥控器初始化
    HAL_UART_Receive_DMA(&huart3, JoyStickReceiveData, 18); // DMA接收AS69
    //开启线程
    // OwChassisTaskStart(&ControllerData);// 全向轮底盘控制线程
    PerceptionTaskStart(&ControllerData);  // 底盘感知定位线程
    ChassisTaskStart(&ControllerData); //舵轮底盘控制线程
	// CtrlDataSender_Start(&ControllerData);// 遥控器线程

    for (;;) {
        osDelay(1);
    }
}