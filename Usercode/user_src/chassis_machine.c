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

#define rx_DEADBAND 100.0

uni_wheel_t wheels[3];

double HallCorrectingStartPos[3];
uint32_t HallCorrectingStartTick;

enum {
    Normal,
    HallCorrecting,
    Automatic
} RunningState = Automatic;

GPIO_PinState state_gpio9  = GPIO_PIN_SET;
GPIO_PinState state_gpio10 = GPIO_PIN_SET;
GPIO_PinState state_gpio11 = GPIO_PIN_SET;

/**
 * @brief 原地转圈霍尔校准
 *
 */
void Chassis_HallCorrecting(uni_wheel_t *wheel, int num, uint32_t start_tick)
{
    double pos = 0;

    pos = (double)(xTaskGetTickCount() - start_tick) * (2 * M_PI / 1000.0);

    for (int i = 0; i < num; i++) {
        Wheel_Set(&wheel[i], 0, HallCorrectingStartPos[i] + pos);
    }

    if (pos > 1.1 * 2 * M_PI) {
        RunningState = Normal;
    }
    Wheels_CalcTransmit(wheel, num);

    // SetWheelsRef(0,HallCorrectingStartPos[i] + pos,&Wheel_component);
    // if (pos > 1.1 * 2 * M_PI) {
    //     RunningState = Normal;
    // }
    // ServoWheels(&Wheel_component);
}

#define SpeedRatioNum 2

struct
{
    double ratio[SpeedRatioNum];
    int id;
    uint32_t last_tick;
    uint32_t button_min_time;
} SpeedRatio = {
    .ratio[0]        = 1 / 2048.0,
    .ratio[1]        = 1 / 1024.0,
    .button_min_time = 500,
    .last_tick       = 0,
    .id              = 0};

void ChassisTask(void const *argument)
{
    const mavlink_controller_t *ctrl_data = argument;
    Chassis_Init(wheels);
    double vx, vy, vrow;
    double spin_ratio = 1.0 / 1024.0;
    double lx, ly, rx;

    osDelay(200);

    Beep();

    uint32_t PreviousWakeTime = osKernelSysTick();

    // 位置式pid参数设置
    // pid_pos_w_pos.Kp    = -120;
    // pid_pos_w_pos.Ki    = 0.0001;
    // pid_pos_w_pos.Kd    = 0;
    // pid_pos_w_pos.limit = 1;

    // pid_pos_x_pos.Kp    = 4;
    // pid_pos_x_pos.Ki    = 0.0001;
    // pid_pos_x_pos.Kd    = 0;
    // pid_pos_x_pos.limit = 1;

    // pid_pos_y_pos.Kp    = 4;
    // pid_pos_y_pos.Ki    = 0.0001;
    // pid_pos_y_pos.Kd    = 0;
    // pid_pos_y_pos.limit = 1;

    wtrMavlink_StartReceiveIT(MAVLINK_COMM_0); // 以mavlink接收上位机通过串口发送的消息
    osDelay(100);

    for (;;) {
        /* 速度切换 */
        if (ctrl_data->buttons & (1 << 7)) {
            if (SpeedRatio.last_tick + SpeedRatio.button_min_time < xTaskGetTickCount()) {
                SpeedRatio.last_tick = xTaskGetTickCount(); // 按键防抖
                if (SpeedRatio.id == 0) {
                    SpeedRatio.id = 1;
                    Beep();
                    Beep();
                } else {
                    SpeedRatio.id = 0;
                    Beep();
                }
            }
        }
        // 位置pid
        // pid_pos_w_pos.target   = control.w_set;
        // pid_pos_w_pos.feedback = mav_posture.zangle;
        // pid_pos_x_pos.target   = control.x_set;
        // pid_pos_x_pos.feedback = mav_posture.pos_x;
        // pid_pos_y_pos.target   = control.y_set;
        // pid_pos_y_pos.feedback = mav_posture.pos_y;

        // 左摇杆控制
        DeadBand(ctrl_data->left_x, ctrl_data->left_y, &lx, &ly, 100); // 摇杆死区
        rx = ctrl_data->right_x;

        vx = lx * SpeedRatio.ratio[SpeedRatio.id];
        vy = ly * SpeedRatio.ratio[SpeedRatio.id]; // 将左摇杆数据转换为底盘速度

        // 右摇杆控制
        if (fabs(rx) <= rx_DEADBAND) {
            vrow = 0;
        } else {
            if (rx > 0) {
                vrow = (rx - rx_DEADBAND) * spin_ratio;
            } else {
                vrow = (rx + rx_DEADBAND) * spin_ratio;
            }
        }

        // 按键控制
        if ((ctrl_data->buttons & (1 << 0)) && (RunningState == Normal)) {
            RunningState = HallCorrecting;
            for (int i = 0; i < 3; i++) {
                HallCorrectingStartPos[i] = wheels[i].now_rot_pos;
            }

            HallCorrectingStartTick = xTaskGetTickCount();
        }

        switch (RunningState) {
            case Normal:
                Chassis_SetSpeed(wheels, 3, vx, vy, vrow);
                break;
            case HallCorrecting:
                Chassis_HallCorrecting(wheels, 3, HallCorrectingStartTick);
                break;
            case Automatic:
                // Chassis_SetSpeed(wheels, 3, FrameTransform(&control, &mav_posture).vx_set + PID_Position(&pid_pos_x_pos),
                //                  FrameTransform(&control, &mav_posture).vy_set + PID_Position(&pid_pos_y_pos),
                //                  control.vw_set + PID_Position(&pid_pos_w_pos));
                break;
            default:
                break;
        }

        // static int n_ = 0;
        // if (n_++ > 1000) {
        //     n_ = 0;
        // HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14); // A板上的绿灯
        // }
        osDelayUntil(&PreviousWakeTime, 2);
    }
}

void ChassisTestTask(void const *argument)
{
    Chassis_Init(wheels);
    osDelay(200);
    uint32_t PreviousWakeTime = osKernelSysTick();
    RunningState = HallCorrecting;
    double lx = 0, ly = 0, rx = 0;

    for (int i = 0; i < 3; i++) {
        HallCorrectingStartPos[i] = wheels[i].now_rot_pos;
    }
    HallCorrectingStartTick = xTaskGetTickCount();
    for (;;) {
        // Chassis_SetSpeed(wheels, 3, 1, 1, 0);
        switch (RunningState) {
            case Normal:
                DeadBand(crl_speed.vx,crl_speed.vy,&lx,&ly,0.1414);
                Chassis_SetSpeed(wheels, 3, lx * 2, -ly * 2, -crl_speed.vw * 5);
                // Chassis_SetSpeed(wheels,3,0,0,1);
                break;
            case HallCorrecting:
                Chassis_HallCorrecting(wheels, 3, HallCorrectingStartTick);
                break;
            default:
                break;
        }
        osDelayUntil(&PreviousWakeTime, 2);
    }
}

void ChassisTaskStart()
{
    // osThreadDef(chassis, ChassisTask, osPriorityBelowNormal, 0, 1024);
    // osThreadCreate(osThread(chassis), ctrl_data);

    osThreadDef(chassis_test, ChassisTestTask, osPriorityBelowNormal, 0, 256);
    osThreadCreate(osThread(chassis_test), NULL);
}
