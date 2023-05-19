/***
 * @Author: szf
 * @Date: 2023-02-22 11:56:47
 * @LastEditTime: 2023-02-23 20:32:19
 * @LastEditors: szf
 * @Description:
 * @FilePath: \ER\Usercode\user_inc\usermain.h
 * @@WeChat:szf13373959031
 */

#ifndef __USER_MAIN_H__
#define __USER_MAIN_H__

#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

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

#endif