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
// #define DJI_CONTROL

WHEEL_COMPONENT Wheel_component;
CHASSIS_COMPONENT Chassis_component;
CHASSIS_CONTROL Chassis_control;
CHASSIS_POSITION Chassis_position;
SPEED_RATIO Speed_ratio;

double HallCorrectingStartPos[3];
uint32_t HallCorrectingStartTick;
double vx_deadbanded = 0;
double vy_deadbanded = 0;
double vw_deadbanded = 0;

GPIO_PinState state_gpio9  = GPIO_PIN_SET;
GPIO_PinState state_gpio10 = GPIO_PIN_SET;
GPIO_PinState state_gpio11 = GPIO_PIN_SET;

void ChassisTask(void const *argument)
{

    vTaskDelay(200);
    // uint32_t PreviousWakeTime = xTaskGetTickCount();

    for (;;) {
        vPortEnterCritical();
        mavlink_posture_t mav_posture_temp = mav_posture;
        mavlink_control_t control_temp     = control;
        vPortExitCritical(); // 拷贝
        CHASSIS_COMPONENT Chassis_component_temp = ReadChassisComnent(&Chassis_component);

        for (int i = 0; i < 4; i++) {
            HallCorrectingStartPos[i] = Wheel_component.wheels[i].now_rot_pos;
        }
        HallCorrectingStartTick = xTaskGetTickCount();

        switch (Chassis_component_temp.Chassis_State) {
            case Locked:
                ChassisLocked();
                break;
            case HallCorrecting:
                ChassisHallCorrect();
                break;
            case RemoteControl:
                SetChassisPosition(mav_posture_temp.pos_x,
                                   mav_posture_temp.pos_y,
                                   mav_posture_temp.zangle,
                                   &Chassis_position); // 更新底盘位置
#ifdef DJI_CONTROL
                DJI_Control();
#else
                Joystick_Control();
#endif
                vPortEnterCritical();
                // vx,vy的死区控制
                DeadBandOneDimensional((double)crl_speed.vx, &vx_deadbanded, 0.25);
                DeadBandOneDimensional((double)crl_speed.vy, &vy_deadbanded, 0.25);
                // DeadBand((double)crl_speed.vx,
                //          (double)crl_speed.vy,
                //          &vx_deadbanded,
                //          &vy_deadbanded,
                //          0.15); // 死区控制 DJI遥控器摇杆
                // vw的死区控制
                DeadBandOneDimensional((double)crl_speed.vw, &vw_deadbanded, 0.1);
                vPortExitCritical();

                xSemaphoreTake(Speed_ratio.xMutex_speed_ratio, portMAX_DELAY);
                double speed_ratio_linear_temp  = Speed_ratio.speed_ratio_linear;
                double speed_ratio_angular_temp = Speed_ratio.speed_ratio_angular;
                xSemaphoreGive(Speed_ratio.xMutex_speed_ratio);

                SetChassisControlPosition(Chassis_position.Chassis_Position_x,
                                          Chassis_position.Chassis_Position_y,
                                          Chassis_position.Chassis_Position_w,
                                          &Chassis_control); // 没什么用，反正这个状态用不到PID
                SetChassisControlVelocity(vx_deadbanded * speed_ratio_linear_temp,
                                          vy_deadbanded * speed_ratio_linear_temp,
                                          vw_deadbanded * speed_ratio_angular_temp,
                                          &Chassis_control);
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

                xSemaphoreTake(Chassis_pid.xMutex_pid, portMAX_DELAY);
                control_temp.vx_set = control_temp.vx_set + PIDPosition(&Chassis_pid.Pid_pos_x);
                control_temp.vy_set = control_temp.vy_set + PIDPosition(&Chassis_pid.Pid_pos_y);
                control_temp.vw_set = PIDPosition(&Chassis_pid.Pid_pos_w);
                xSemaphoreGive(Chassis_pid.xMutex_pid);
                control_temp = FrameTransform(&control_temp, &mav_posture);

                SetChassisControlVelocity(control_temp.vx_set,
                                          control_temp.vy_set,
                                          control_temp.vw_set,
                                          &Chassis_control); // 上位机规划值作为伺服值
                CalculateWheels(&Chassis_control, &Wheel_component);
                break;
            default:
                break;
        }
        vTaskDelay(50);
    }
}

void ChassisTaskStart()
{
    osThreadDef(chassis, ChassisTask, osPriorityNormal, 0, 1024);
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
void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle)
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

void ChassisHallCorrect() //! 要考虑形参volatile
{

    float HallCorrectingStartPos[3] = {0};
    float targetAngle[3]            = {0};
    float currentAngle[3]           = {0};
    float difference[3]             = {0};
    for (int i = 0; i < 3; i++) {
        HallCorrectingStartPos[i] = Wheel_component.wheels[i].now_rot_pos;
    }

    targetAngle[0] = HallCorrectingStartPos[0] - 4 * 1.1 * M_PI;
    targetAngle[1] = HallCorrectingStartPos[1] + 4 * 1.1 * M_PI;
    targetAngle[2] = HallCorrectingStartPos[2] + 4 * 1.1 * M_PI;

    bool isArrive                      = false;               // 标志是否达到目标位置
    TickType_t HallCorrectingStartTick = xTaskGetTickCount(); // 初始时间
    do {
        TickType_t HallCorrectingNowTick     = xTaskGetTickCount();
        TickType_t HallCorrectingElapsedTick = HallCorrectingNowTick - HallCorrectingStartTick;
        float timeSec                        = (HallCorrectingElapsedTick / (1000.0)); // 获取当前时间/s
        // 速度规划
        for (int i = 0; i < 3; i++) {
            VelocityPlanning(HallCorrectingStartPos[i], HallCorrecting_Max_Velocity, HallCorrecting_Acceleration, targetAngle[i], timeSec, &(currentAngle[i]));
            SetWheelsRef(i, 0, currentAngle[i], &Wheel_component);
            difference[i] = fabs(currentAngle[i] - targetAngle[i]);
        }

        // 判断是否到达目标位置
        if (difference[0] < 1 && difference[1] < 1 && difference[2] < 1) {
            isArrive = true;
        }
        vTaskDelay(3);
    } while (!isArrive);

    ChassisSwitchState(Locked, &Chassis_component);
}

void ChassisLocked()
{
    double delta_pos[3];
    for (int i = 0; i < 3; i++) {
        Wheel_ReadNowRotPos(&(Wheel_component.wheels[i]));
    }

    delta_pos[0] = LoopSimplify(2 * M_PI, 0 - Wheel_component.wheels[0].now_rot_pos);
    delta_pos[1] = LoopSimplify(2 * M_PI, 0 - Wheel_component.wheels[1].now_rot_pos);
    delta_pos[2] = LoopSimplify(2 * M_PI, 0 - Wheel_component.wheels[2].now_rot_pos);

    for (int i = 0; i < 3; i++) {
        if (delta_pos[i] > M_PI / 2) {
            delta_pos[i] -= M_PI;
        } else if (delta_pos[i] < -M_PI / 2) {
            delta_pos[i] += M_PI;
        }
    }

    SetWheelsRef(Wheel_Front, 0, Wheel_component.wheels[0].now_rot_pos + delta_pos[0], &Wheel_component);
    SetWheelsRef(Wheel_Left, 0, Wheel_component.wheels[1].now_rot_pos + delta_pos[1], &Wheel_component);
    SetWheelsRef(Wheel_Right, 0, Wheel_component.wheels[2].now_rot_pos + delta_pos[2], &Wheel_component);
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

    crl_speed.vx = ReadJoystickRight_x(&msg_joystick_air);
    crl_speed.vy = ReadJoystickRight_y(&msg_joystick_air);
    crl_speed.vw = ReadJoystickLeft_x(&msg_joystick_air);
}

void SpeedSwitchRatio(double target_speed_ratio_linear, double target_speed_ratio_angular, SPEED_RATIO *Speed_Ratio)
{
    xSemaphoreTake(Speed_ratio.xMutex_speed_ratio, portMAX_DELAY);
    Speed_Ratio->speed_ratio_angular = target_speed_ratio_angular;
    Speed_Ratio->speed_ratio_linear  = target_speed_ratio_linear;
    xSemaphoreGive(Speed_ratio.xMutex_speed_ratio);
}
