/*** 
 * @Author: szf
 * @Date: 2023-02-22 11:56:47
 * @LastEditTime: 2023-02-23 20:32:19
 * @LastEditors: szf
 * @Description: 
 * @FilePath: \ER\Usercode\user_inc\usermain.h
 * @@WeChat:szf13373959031
 */

#ifndef USERMAIN_H
#define USERMAIN_H

#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "wtr_can.h"
#include "wtr_dji.h"
#include "wtr_uart.h"
#include <math.h>
#include "main.h"
#include "wtr_mavlink.h"
#include "user_callback.h"
#include "chassis_machine.h"
#include "chassis_driver.h"
#include "chassis_perception.h"
#include "chassis_communicate.h"
#include "user_beep.h"
#include "CLI.h"
#include "wtr_vesc.h"
#include "user_config.h"
#include "user_calculate.h"
#include "wtr_ads1256.h"

#include "mavlink_msg_chassis_to_upper.h"
#include "mavlink_msg_control.h"
#include "mavlink_msg_posture.h"
#include "mavlink_msg_controller.h"

// #include "state_management.h"

// 宏定义计算中常数及底盘数据
#define pi           3.1415926535898
#define DEC          (pi / 180)
#define r_underpan_3 0.1934
#define r_underpan_4 0.25
#define r_wheel      0.076
// 限幅
#define max(a, b)      (a > b ? a : b)
#define min(a, b)      (a < b ? a : b)
#define range(x, a, b) (min(max(x, a), b))

// static uint8_t i = 0;
extern uint8_t i; // 不确定是不是static

extern mavlink_posture_t mav_posture;
extern mavlink_control_t control;

#endif