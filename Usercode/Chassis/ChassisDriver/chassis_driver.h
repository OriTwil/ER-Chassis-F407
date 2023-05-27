/**
 * @file uni_wheel.h
 * @author TITH (1023515576@qq.com)
 * @brief 舵轮驱动
 * @version 0.1
 * @date 2022-07-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __CHASSIS_DRIVER_H__
#define __CHASSIS_DRIVER_H__

#include "chassis_start.h"
#include "wtr_vesc.h"
#include "wtr_dji.h"
#include "chassis_commen.h"

void Chassis_Init(uni_wheel_t *wheel);

void Wheel_Set(uni_wheel_t *wheel, double speed, double rot_pos);

void Wheel_SetXY(uni_wheel_t *wheel, double speed_x, double speed_y);

void Wheels_CalcTransmit(uni_wheel_t wheel[], int num);

void Chassis_SetOrigin(uni_wheel_t *wheel, double origin_x, double origin_y);

void Chassis_SetSpeed(uni_wheel_t *wheel, int num, double vx, double vy, double ang_v);

void Wheel_Hall_Callback(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uni_wheel_t *wheel);

#endif