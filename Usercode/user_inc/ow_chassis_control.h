/*** 
 * @Author: szf
 * @Date: 2023-02-23 19:03:31
 * @LastEditTime: 2023-02-24 15:17:15
 * @LastEditors: szf
 * @Description: 
 * @FilePath: \ER\Usercode\user_inc\ow_chassis_control.h
 * @@WeChat:szf13373959031
 */
#ifndef OW_CHASSIS_CONTROL_H
#define OW_CHASSIS_CONTROL_H

#include "usermain.h"
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

void OwChassisTaskStart(mavlink_controller_t *ctrl_data);

#endif