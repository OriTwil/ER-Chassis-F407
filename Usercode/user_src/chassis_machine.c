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
#include "mavlink_msg_controller.h"
#include "state_management.h"
#include "user_config.h"
#include "user_calculate.h"

#define rx_DEADBAND 100.0

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

        switch (Chassis_component.Chassis_State) {
            case Locked:
                SetWheelsRef(Wheel_Front, 0, Wheel_Front_Locked_Pos, &Wheel_component);
                SetWheelsRef(Wheel_Left, 0, Wheel_Left_Locked_Pos, &Wheel_component);
                SetWheelsRef(Wheel_Right, 0, Wheel_Right_Locked_Pos, &Wheel_component);
                break;
            case HallCorrecting:
                ChassisHallCorrect(720, &Wheel_component);
                break;
            case RemoteControl:
                vPortEnterCritical(); // todo 如果卡住就改成先拷贝数据
                SetChassisPosition(mav_posture.pos_x,
                                   mav_posture.pos_y,
                                   mav_posture.zangle,
                                   &Chassis_position); // 更新底盘位置
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
                CalculateWheels(&Chassis_control, &Wheel_component); // 用摇杆控制底盘
                vPortExitCritical();
                break;
            case ComputerControl:
                vPortEnterCritical();
                mavlink_posture_t mav_posture_temp = mav_posture;
                mavlink_control_t control_temp     = control;
                vPortExitCritical(); //拷贝

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
