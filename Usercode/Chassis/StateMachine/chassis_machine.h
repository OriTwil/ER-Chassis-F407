/**
 * @file chassis_control.h
 * @author TITH (1023515576@qq.com)
 * @brief 底盘控制
 * @version 0.1
 * @date 2022-07-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __CHASSIS_MACHINE__
#define __CHASSIS_MACHINE__

#include "chassis_start.h"
#include "chassis_commen.h"

void ChassisTaskStart();

void ChassisSwitchState(CHASSIS_STATE target_chassis_state, CHASSIS_COMPONENT *chassis_component);
void ChassisSwitchPoint(CHASSIS_POINT target_point, CHASSIS_COMPONENT *chassis_component);
void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, float *currentAngle);
void ChassisHallCorrect(float target_angle, WHEEL_COMPONENT *wheel_component);
CHASSIS_COMPONENT ReadChassisComnent(CHASSIS_COMPONENT *chassis_component);

extern CHASSIS_COMPONENT Chassis_component;
extern CHASSIS_CONTROL Chassis_control;
extern CHASSIS_POSITION Chassis_position;
extern WHEEL_COMPONENT Wheel_component;

#endif