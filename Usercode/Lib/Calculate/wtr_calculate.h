/***
 * @Author: szf
 * @Date: 2023-02-22 12:06:17
 * @LastEditTime: 2023-02-22 12:30:10
 * @LastEditors: szf
 * @Description:
 * @FilePath: \ER\Usercode\user_inc\usercalculate.h
 * @@WeChat:szf13373959031
 */
#pragma once

#include "chassis_start.h"
#include "wtr_mavlink.h"
#include "mavlink_msg_control.h"
#include "wtr_calculate.h"
#include "wtr_dji.h"

#define r_underpan_3 0.1934
#define r_underpan_4 0.25
#define r_wheel      0.076 

/*定义增量式PID结构体*/
typedef __IO struct
{
    float setpoint; // 设定值
    float Kp;       // 比例系数
    float Kd;       // 积分系数
    float Ki;       // 微分系数

    float lasterror; // 前一拍偏差
    float preerror;  // 前两拍偏差
    float result;    // 输出值
    float limit;     // 输出限幅
} PID_Incremwntal;

/*定义位置式PID结构体*/
typedef __IO struct {
    float limit;    // 输出限幅
    float target;   // 目标量
    float feedback; // 反馈量

    float Kp;
    float Ki;
    float Kd;
    float eSum;
    float e0; // 当前误差
    float e1; // 上一次误差
} PID_Pos;

// 定义数组，分别存放四个轮子对应电机的速度
extern double moter_speed[4];
// 声明运动学逆解函数
void CalculateFourWheels(double *moter_speed,
                         double v_x,
                         double v_y,
                         double v_w);

void CalculateThreeWheels_(double *moter_speed,
                           double v_x,
                           double v_y,
                           double v_w);
void CalculateThreeWheels(double *moter_speed,
                          double v_x,
                          double v_y,
                          double v_w);
void CalculateFourWheels(double *moter_speed,
                         double v_x,
                         double v_y,
                         double v_w);
void CalculateFourMecanumWheels(double *moter_speed, double vx, double vy, double vw);

void PIDIncremental(PID_Incremwntal *vPID, float processValue);

float PIDPosition(PID_Pos *p);

void positionServo(float ref, DJI_t *motor);

void speedServo(float ref, DJI_t *motor);

void DeadBand(double x, double y, double *new_x, double *new_y, double threshould);

void DeadBandOneDimensional(double x, double *new_x, double threshould);

mavlink_control_t FrameTransform(mavlink_control_t *control, mavlink_posture_t *posture);
