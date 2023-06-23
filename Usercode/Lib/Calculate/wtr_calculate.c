/*
 * @Author: szf
 * @Date: 2023-02-22 12:04:21
 * @LastEditTime: 2023-03-13 19:54:02
 * @LastEditors: szf
 * @Description: 运动学逆解算及PID计算函数
 * @FilePath: \ER-Chassis-F407\Usercode\user_src\usercalculate.c
 * @WeChat:szf13373959031
 */

#include "wtr_calculate.h"
#include <math.h>
#include "user_config.h"

double moter_speed[4];
// RR麦轮底盘
#define rotate_ratio    0.3615   // (Width + Length)/2
#define wheel_rpm_ratio 2387.324 // 换算线速度到rpm

/**
 * @description:三轮底盘逆解算（第一种方向）
 * @author: szf
 * @date:
 * @return {void}
 */
void CalculateThreeWheels(double *moter_speed,
                          double v_x,
                          double v_y,
                          double v_w)
{
    moter_speed[0] = (-v_x * sin(30 * DEC) - v_y * cos(30 * DEC) + v_w * r_underpan_3) / (2 * M_PI * r_wheel);
    moter_speed[1] = (+v_x + v_w * r_underpan_3) / (2 * M_PI * r_wheel);
    moter_speed[2] = (-v_x * sin(30 * DEC) + v_y * cos(30 * DEC) + v_w * r_underpan_3) / (2 * M_PI * r_wheel);
}

/**
 * @description:三轮底盘逆解算（第二种方向）
 * @author: szf
 * @date:
 * @return {void}
 */
void CalculateThreeWheels_(double *moter_speed,
                           double v_x,
                           double v_y,
                           double v_w)
{
    moter_speed[2] = (-v_y * sin(30 * DEC) + v_x * cos(30 * DEC) + v_w * r_underpan_3) * 60 / (2 * M_PI * r_wheel) * 19;
    moter_speed[1] = (+v_y + v_w * r_underpan_3) * 60 / (2 * M_PI * r_wheel) * 19;
    moter_speed[0] = (-v_y * sin(30 * DEC) - v_x * cos(30 * DEC) + v_w * r_underpan_3) * 60 / (2 * M_PI * r_wheel) * 19;
}

/**
 * @description:四轮底盘逆解算（第一种方向）
 * @author: szf
 * @date:
 * @return {void}
 */
void CalculateFourWheels(double *moter_speed,
                         double v_x,
                         double v_y,
                         double v_w)
{
    moter_speed[0] = (v_y + v_w * r_underpan_4) / (2 * M_PI * r_wheel);
    moter_speed[1] = (-v_x + v_w * r_underpan_4) / (2 * M_PI * r_wheel);
    moter_speed[2] = (-v_y + v_w * r_underpan_4) / (2 * M_PI * r_wheel);
    moter_speed[3] = (v_x + v_w * r_underpan_4) / (2 * M_PI * r_wheel);
}

/**
 * @description:四轮底盘逆解算（第二种方向）
 * @author: szf
 * @date:
 * @return {void}
 */
void CalculateFourWheels_(double *moter_speed,
                          double vx,
                          double vy,
                          double vw)
{
    moter_speed[0] = (vx * sqrt(2) + vy * sqrt(2) + vw * r_underpan_4) / (2 * M_PI * r_wheel);
    moter_speed[1] = (-vx * sqrt(2) + vy * sqrt(2) + vw * r_underpan_4) / (2 * M_PI * r_wheel);
    moter_speed[2] = (-vx * sqrt(2) - vy * sqrt(2) + vw * r_underpan_4) / (2 * M_PI * r_wheel);
    moter_speed[3] = (vx * sqrt(2) - vy * sqrt(2) + vw * r_underpan_4) / (2 * M_PI * r_wheel);
}

/**
 * @description:麦克纳姆轮底盘逆解算
 * @author: szf
 * @date:
 * @return {void}
 */
void CalculateFourMecanumWheels(double *moter_speed, double vx, double vy, double vw)
{
    moter_speed[0] = (vx - vy - vw * rotate_ratio) * wheel_rpm_ratio;
    moter_speed[1] = (vx + vy - vw * rotate_ratio) * wheel_rpm_ratio;
    moter_speed[2] = (-vx + vy - vw * rotate_ratio) * wheel_rpm_ratio;
    moter_speed[3] = (-vx - vy - vw * rotate_ratio) * wheel_rpm_ratio;
}

/**
 * @description: 增量式PID
 * @param {PID} *vPID
 * @param {float} processValue
 * @return {*}
 */
void PIDIncremental(PID_Incremwntal *vPID, float processValue)
{
    float thisError;
    float increment;
    float pError, dError, iError;

    thisError = vPID->setpoint - processValue; // 当前误差等于设定值减去当前值
    // 计算公式中除系数外的三个 乘数
    pError = thisError - vPID->lasterror; // 两次偏差差值err(k)-err(k-1)
    iError = thisError;
    dError = thisError - 2 * (vPID->lasterror) + vPID->preerror;

    increment = vPID->Kp * pError + vPID->Ki * iError + vPID->Kd * dError; // 增量计算

    vPID->preerror  = vPID->lasterror; // 存放偏差用于下次运算
    vPID->lasterror = thisError;

    vPID->result += increment; // 结果是上次结果 加上本次增量
    vPID->result = range(vPID->result, -vPID->limit, vPID->limit);
    // if(vPID->result > 0.6)
    // {
    //     vPID->result = 0.6;
    // }
    // if (vPID->result < -0.6)
    // {
    //     vPID->result = -0.6;
    // }//输出限幅
}

/**
 * @description: 位置式PID
 * @param {PIDType} *p
 * @return {*}
 */
float PIDPosition(PID_Pos *p)
{
    float pe, ie, de;
    float out = 0;

    // 计算当前误差
    p->e0 = p->target - p->feedback;

    // 误差积分
    p->eSum += p->e0;

    // 误差微分
    de = p->e0 - p->e1;

    pe = p->e0;
    ie = p->eSum;

    p->e1 = p->e0;

    out = pe * (p->Kp) + ie * (p->Ki) + de * (p->Kd);
    // 输出限幅
    out = range(out, -p->limit, p->limit);
    return out;
}

// 增量式PID算法
void PID_Calc(PID_t *pid)
{
    pid->cur_error = pid->ref - pid->fdb;
    pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->ref - pid->fdb;
    /*设定输出上限*/
    if (pid->output > pid->outputMax) pid->output = pid->outputMax;
    if (pid->output < -pid->outputMax) pid->output = -pid->outputMax;
}

// 比例算法
void P_Calc(PID_t *pid)
{
    pid->cur_error = pid->ref - pid->fdb;
    pid->output    = pid->KP * pid->cur_error;
    /*设定输出上限*/
    if (pid->output > pid->outputMax) pid->output = pid->outputMax;
    if (pid->output < -pid->outputMax) pid->output = -pid->outputMax;

    if (fabs(pid->output) < pid->outputMin)
        pid->output = 0;
}

// 位置伺服函数
void positionServo(float ref, DJI_t *motor)
{

    motor->posPID.ref = ref;
    motor->posPID.fdb = motor->AxisData.AxisAngle_inDegree;
    PID_Calc(&motor->posPID);

    motor->speedPID.ref = motor->posPID.output;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);
}

// 速度伺服函数
void speedServo(float ref, DJI_t *motor)
{
    motor->speedPID.ref = ref;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);
}

/**
 * @brief 死区
 *
 */
void DeadBand(double x, double y, double *new_x, double *new_y, double threshould)
{
    double length     = sqrt(x * x + y * y);
    double new_length = length - threshould;

    if (new_length <= 0) {
        *new_x = 0;
        *new_y = 0;
        return;
    }

    double k = new_length / length;

    *new_x = x * k;
    *new_y = y * k;
}

void DeadBandOneDimensional(double x, double *new_x, double threshould)
{
    if (fabs(x- threshould) < 0) {
        *new_x = 0;
        return;
    }

    *new_x = x - threshould;
}

/**
 * @description: 速度方向转换
 * @param
 * @return
 * @bug 转换方程还不确定
 */
mavlink_control_t FrameTransform(mavlink_control_t *control, mavlink_posture_t *posture)
{
    mavlink_control_t result;
    vPortEnterCritical();
    result.vx_set = control->vx_set * cos(posture->zangle * DEC) + control->vy_set * sin(posture->zangle * DEC);
    result.vy_set = -control->vx_set * sin(posture->zangle * DEC) + control->vy_set * cos(posture->zangle * DEC);
    result.vw_set = control->vw_set;
    result.w_set  = control->w_set;
    result.x_set  = control->x_set;
    result.y_set  = control->y_set;
    vPortExitCritical();
    return result;
}