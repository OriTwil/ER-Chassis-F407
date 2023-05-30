/**
 * @file controller_data_sender.h
 * @author TITH (1023515576@qq.com)
 * @brief 发送遥控器原始数据
 * @version 0.1
 * @date 2022-07-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef  __CHASSIS_COMMUNICATE_H__

#include "usart.h"
#include "wtr_mavlink.h"
#include "chassis_commen.h"

void CommunicateInit();

void CommunicateStart();

extern mavlink_posture_t mav_posture;
extern mavlink_control_t control;
extern mavlink_chassis_to_upper_t chassis_data;

#endif
