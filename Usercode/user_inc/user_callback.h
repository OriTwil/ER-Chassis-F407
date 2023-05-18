/***
 * @Author: szf
 * @Date: 2023-02-22 12:05:56
 * @LastEditTime: 2023-02-22 12:25:39
 * @LastEditors: szf
 * @Description:
 * @FilePath: \ER\Usercode\user_inc\usercallback.h
 * @@WeChat:szf13373959031
 */
#ifndef _USERCALLBACK_H__
#define _USERCALLBACK_H__

#include "user_main.h"

extern int counter;
extern int test;
extern float w_speed;
extern int16_t crldata[4];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void wtrMavlink_MsgRxCpltCallback(mavlink_message_t *msg);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif