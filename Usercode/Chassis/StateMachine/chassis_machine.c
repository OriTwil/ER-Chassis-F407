/**
 * @file chassis_control.c
 * @author TITH (1023515576@qq.com)
 * @brief 底盘控制
 * @version 0.1
 *
 * @date 2022-07-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis_machine.h"
#include "chassis_driver.h"
#include "wtr_mavlink.h"
#include "wtr_uart.h"
#include "user_config.h"
#include "chassis_commen.h"
#include "user_config.h"
#include "wtr_calculate.h"
#include "chassis_communicate.h"
#include "chassis_servo.h"
#include "chassis_remote_control.h"

#define rx_DEADBAND 100.0

WHEEL_COMPONENT Wheel_component;
CHASSIS_COMPONENT Chassis_component;
CHASSIS_CONTROL Chassis_control;
CHASSIS_POSITION Chassis_position;

uni_wheel_t wheels[3];
double HallCorrectingStartPos[3];
uint32_t HallCorrectingStartTick;
double vx_deadbanded = 0;
double vy_deadbanded = 0;

GPIO_PinState state_gpio9  = GPIO_PIN_SET;
GPIO_PinState state_gpio10 = GPIO_PIN_SET;
GPIO_PinState state_gpio11 = GPIO_PIN_SET;

void ChassisTask(void const *argument)
{

    vTaskDelay(200);

    for (;;) {
        vPortEnterCritical();
        mavlink_posture_t mav_posture_temp = mav_posture;
        mavlink_control_t control_temp     = control;
        vPortExitCritical(); // 拷贝
        CHASSIS_COMPONENT Chassis_component_temp = ReadChassisComnent(&Chassis_component);
        switch (Chassis_component_temp.Chassis_State) {
            case Locked:
                SetWheelsRef(Wheel_Front, 0, Wheel_Front_Locked_Pos, &Wheel_component);
                SetWheelsRef(Wheel_Left, 0, Wheel_Left_Locked_Pos, &Wheel_component);
                SetWheelsRef(Wheel_Right, 0, Wheel_Right_Locked_Pos, &Wheel_component);
                break;
            case HallCorrecting:
                ChassisHallCorrect(720, &Wheel_component);
                break;
            case RemoteControl:
                SetChassisPosition(mav_posture_temp.pos_x,
                                   mav_posture_temp.pos_y,
                                   mav_posture_temp.zangle,
                                   &Chassis_position); // 更新底盘位置

                // DJI_Control();
                Joystick_Control();
                vPortEnterCritical();
                DeadBand((double)crl_speed.vx,
                         (double)crl_speed.vy,
                         &vx_deadbanded,
                         &vy_deadbanded,
                         0.1); // 死区控制 DJI遥控器摇杆
                SetChassisControlPosition(Chassis_position.Chassis_Position_x,
                                          Chassis_position.Chassis_Position_y,
                                          Chassis_position.Chassis_Position_w,
                                          &Chassis_control); // 没什么用，反正这个状态用不到PID
                SetChassisControlVelocity(vx_deadbanded,
                                          vy_deadbanded,
                                          crl_speed.vw,
                                          &Chassis_control);
                vPortExitCritical();
                CalculateWheels(&Chassis_control, &Wheel_component); // 用摇杆控制底盘
                break;
            case ComputerControl:

                SetChassisPosition(mav_posture_temp.pos_x,
                                   mav_posture_temp.pos_y,
                                   mav_posture_temp.zangle,
                                   &Chassis_position); // 更新底盘位置
                SetChassisControlPosition(control_temp.x_set,
                                          control_temp.y_set,
                                          control_temp.w_set,
                                          &Chassis_control); // 上位机规划值作为伺服值
                SetChassisControlVelocity(control_temp.vx_set,
                                          control_temp.vy_set,
                                          control_temp.vw_set,
                                          &Chassis_control); // 上位机规划值作为伺服值
                CalculateWheels(&Chassis_control, &Wheel_component);
                break;
            default:
                break;
        }
        vTaskDelay(5);
    }
}

void ChassisTaskStart()
{
    osThreadDef(chassis, ChassisTask, osPriorityBelowNormal, 0, 1024);
    osThreadCreate(osThread(chassis), NULL);
}

void ChassisSwitchState(CHASSIS_STATE target_chassis_state, CHASSIS_COMPONENT *chassis_component)
{
    xSemaphoreTakeRecursive(chassis_component->xMutex_chassis, (TickType_t)10);
    chassis_component->Chassis_State = target_chassis_state;
    xSemaphoreGiveRecursive(chassis_component->xMutex_chassis);
}

void ChassisSwitchPoint(CHASSIS_POINT target_point, CHASSIS_COMPONENT *chassis_component)
{
    xSemaphoreTakeRecursive(chassis_component->xMutex_chassis, (TickType_t)10);
    chassis_component->Chassis_Point = target_point;
    xSemaphoreGiveRecursive(chassis_component->xMutex_chassis);
}

/**
 * @brief T型速度规划函数
 * @param initialAngle 初始角度
 * @param maxAngularVelocity 最大角速度
 * @param AngularAcceleration 角加速度
 * @param targetAngle 目标角度
 * @param currentTime 当前时间
 * @param currentTime 当前角度
 * @todo 转换为国际单位制
 */
void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, float *currentAngle)
{

    float angleDifference = targetAngle - initialAngle;     // 计算到目标位置的角度差
    float sign            = (angleDifference > 0) ? 1 : -1; // 判断角度差的正负(方向)

    float accelerationTime = maxAngularVelocity / AngularAcceleration;                                                      // 加速(减速)总时间
    float constTime        = (fabs(angleDifference) - AngularAcceleration * pow(accelerationTime, 2)) / maxAngularVelocity; // 匀速总时间
    float totalTime        = constTime + accelerationTime * 2;                                                              // 计算到达目标位置所需的总时间

    // 判断能否达到最大速度
    if (constTime > 0) {
        // 根据当前时间判断处于哪个阶段
        if (currentTime <= accelerationTime) {
            // 加速阶段
            *currentAngle = initialAngle + sign * 0.5 * AngularAcceleration * pow(currentTime, 2);
        } else if (currentTime <= accelerationTime + constTime) {
            // 匀速阶段
            *currentAngle = initialAngle + sign * maxAngularVelocity * (currentTime - accelerationTime) + 0.5 * sign * AngularAcceleration * pow(accelerationTime, 2);
        } else if (currentTime <= totalTime) {
            // 减速阶段
            float decelerationTime = currentTime - accelerationTime - constTime;
            *currentAngle          = initialAngle + sign * maxAngularVelocity * constTime + 0.5 * sign * AngularAcceleration * pow(accelerationTime, 2) + sign * (maxAngularVelocity * decelerationTime - 0.5 * AngularAcceleration * pow(decelerationTime, 2));
        } else {
            // 达到目标位置
            *currentAngle = targetAngle;
        }
    } else {
        maxAngularVelocity = sqrt(fabs(angleDifference) * AngularAcceleration);
        accelerationTime   = maxAngularVelocity / AngularAcceleration;
        totalTime          = 2 * accelerationTime;
        constTime          = 0;
        // 根据当前时间判断处于哪个阶段
        if (currentTime <= accelerationTime) {
            // 加速阶段
            *currentAngle = initialAngle + sign * 0.5 * AngularAcceleration * pow(currentTime, 2);
        } else if (currentTime <= totalTime) {
            // 减速阶段
            float decelerationTime = currentTime - accelerationTime; // 减速时间
            *currentAngle          = initialAngle + sign * 0.5 * AngularAcceleration * pow(accelerationTime, 2) + sign * (maxAngularVelocity * decelerationTime - 0.5 * AngularAcceleration * pow(decelerationTime, 2));
        } else {
            // 达到目标位置
            *currentAngle = targetAngle;
        }
    }
}

void ChassisHallCorrect(float target_angle, WHEEL_COMPONENT *wheel_component) //! 要考虑形参volatile
{

    float HallCorrectingStartPos[3] = {0};
    float difference[3]             = {0};
    float currentAngle[3]           = {0};
    for (int i = 0; i < 3; i++) {
        HallCorrectingStartPos[i] = wheel_component->wheels[i].now_rot_pos;
    }
    bool isArrive                      = false;               // 标志是否达到目标位置
    TickType_t HallCorrectingStartTick = xTaskGetTickCount(); // 初始时间
    do {
        TickType_t HallCorrectingNowTick     = xTaskGetTickCount();
        TickType_t HallCorrectingElapsedTick = HallCorrectingNowTick - HallCorrectingStartTick;
        float timeSec                        = (HallCorrectingElapsedTick / (1000.0)); // 获取当前时间/s
        xSemaphoreTakeRecursive(wheel_component->xMutex_wheel, (TickType_t)10);
        // 速度规划
        for (int i = 0; i < 3; i++) {
            VelocityPlanning(HallCorrectingStartPos[i], HallCorrecting_Max_Velocity, HallCorrecting_Acceleration, target_angle, timeSec, &(currentAngle[i]));
            SetWheelsRef(i, 0, currentAngle[i], wheel_component);
        }
        xSemaphoreGiveRecursive(wheel_component->xMutex_wheel); //! 要换成递归互斥锁！！！

        // 判断是否到达目标位置
        if (difference[0] < 0.1 && difference[1] < 0.1 && difference[2] < 0.1) {
            isArrive = true;
        }
        vTaskDelay(2);
    } while (!isArrive);
}

CHASSIS_COMPONENT ReadChassisComnent(CHASSIS_COMPONENT *chassis_component)
{
    CHASSIS_COMPONENT chassis_component_temp;
    xSemaphoreTakeRecursive(chassis_component->xMutex_chassis, (TickType_t)10);
    chassis_component_temp = *chassis_component;
    xSemaphoreGiveRecursive(chassis_component->xMutex_chassis);
    return chassis_component_temp;
}

void Joystick_Control()
{
    crl_speed.vx = ReadJoystickRight_x(msg_joystick_air);
    crl_speed.vy = ReadJoystickRight_y(msg_joystick_air);
    crl_speed.vw = ReadJoystickLeft_x(msg_joystick_air);
}
