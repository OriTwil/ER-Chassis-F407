#ifndef __CHASSIS_SERVO_H__
#define __CHASSIS_SERVO_H__

#include "chassis_start.h"
#include "chassis_commen.h"

void ServoTaskStart();
void MotorInit();
void SetPIDTarget(float target_x, float target_y, float target_w, CHASSIS_PID *chassis_pid);
void SetPIDFeedback(float feedback_x, float feedback_y, float feedback_w, CHASSIS_PID *chassis_pid);
void SetChassisPosition(float position_x, float position_y, float position_w, CHASSIS_POSITION *chassis_position);
void SetChassisControlVelocity(float vx_control, float vy_control, float vw_control, CHASSIS_CONTROL *chassis_control);
void SetChassisControlPosition(float x_control, float y_control, float w_control, CHASSIS_CONTROL *chassis_control);
void CalculateWheels(CHASSIS_CONTROL *chassis_control, WHEEL_COMPONENT *wheel_component);
void SetWheelsRef(int wheel_id, double target_speed, double target_pos, WHEEL_COMPONENT *wheel_component);
void ServoWheels(WHEEL_COMPONENT *wheel_component);
extern CHASSIS_PID Chassis_pid;
#endif