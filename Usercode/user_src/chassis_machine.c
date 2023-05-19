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

#define rx_DEADBAND 100.0

uni_wheel_t wheels[3];
double HallCorrectingStartPos[3];
uint32_t HallCorrectingStartTick;

GPIO_PinState state_gpio9  = GPIO_PIN_SET;
GPIO_PinState state_gpio10 = GPIO_PIN_SET;
GPIO_PinState state_gpio11 = GPIO_PIN_SET;

void ChassisTask(void const *argument)
{
    double vx, vy, vrow;
    double spin_ratio = 1.0 / 1024.0;
    double lx, ly, rx;

    vTaskDelay(200);

    for (;;) {

        switch (Chassis_component.Chassis_State) {
            case Locked:
                SetChassisVelocity(&Chassis_control, &Wheel_component);
                SetWheelsRef(Wheel_Front, 0, Wheel_Front_Locked_Pos, &Wheel_component);
                SetWheelsRef(Wheel_Left, 0, Wheel_Left_Locked_Pos, &Wheel_component);
                SetWheelsRef(Wheel_Right, 0, Wheel_Right_Locked_Pos, &Wheel_component);
                break;
            case HallCorrecting:
                ChassisHallCorrect(720, &Chassis_component);
                break;
            case RemoteControl:
                // Chassis_SetSpeed(wheels, 3, FrameTransform(&control, &mav_posture).vx_set + PID_Position(&pid_pos_x_pos),
                //                  FrameTransform(&control, &mav_posture).vy_set + PID_Position(&pid_pos_y_pos),
                //                  control.vw_set + PID_Position(&pid_pos_w_pos));
                break;
            case ComputerControl:
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
