#include "chassis_operate_app.h"
#include "semphr.h"
#include "chassis_communicate.h"
#include "chassis_machine.h"
#include "chassis_servo.h"
#include "chassis_perception.h"
#include "user_config.h"
#include "chassis_commen.h"
#include "pickup_state_machine.h"
#include "chassis_remote_control.h"
#include "wtr_uart.h"

/**
 * @description: 操作线程
 * @todo 根据各个传感器、遥控器等设计操作手操作流程
 * @return {void}
 */
void StateManagemantTask(void const *argument)
{
    vTaskDelay(20);
    // ChassisSwitchState(HallCorrecting, &Chassis_component);
    for (;;) {
        JoystickControl();
        vTaskDelay(100);
    }
}

/**
 * @description: 开启线程
 * @return {void}
 */
void StateManagemantTaskStart()
{

    osThreadDef(statemanagement, StateManagemantTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(statemanagement), NULL);
}

// todo 初始化分开,封装函数
void StateInit()
{
    Chassis_component.Chassis_Point  = First_Ring;
    Chassis_component.Chassis_State  = Locked;
    Chassis_component.xMutex_chassis = xSemaphoreCreateRecursiveMutex();

    Perception_component.Perception_State  = Receive;
    Perception_component.xMutex_perception = xSemaphoreCreateRecursiveMutex();

    PIDInit();
    Chassis_pid.xMutex_pid = xSemaphoreCreateRecursiveMutex();

    Chassis_position.Chassis_Position_w = 0;
    Chassis_position.Chassis_Position_x = 0;
    Chassis_position.Chassis_Position_y = 0;
    Chassis_position.Chassis_Angle_y    = 0;
    Chassis_position.xMutex_position    = xSemaphoreCreateRecursiveMutex();

    Chassis_control.Chassis_Control_vw      = 0;
    Chassis_control.Chassis_Control_vx      = 0;
    Chassis_control.Chassis_Control_vy      = 0;
    Chassis_control.Chassis_Control_w       = 0;
    Chassis_control.Chassis_Control_x       = 0;
    Chassis_control.Chassis_Control_y       = 0;
    Chassis_control.Chassis_Control_w_limit = 0;
    Chassis_control.xMutex_control          = xSemaphoreCreateRecursiveMutex();

    Pickup_component.Pickup_Ring   = First_Ring;
    Pickup_component.Pickup_State  = Ready;
    Pickup_component.xMutex_pickup = xSemaphoreCreateRecursiveMutex();

    Wheel_component.xMutex_wheel = xSemaphoreCreateRecursiveMutex();
    Chassis_Init(Wheel_component.wheels);

    msg_joystick_air.xMutex_joystick_air = xSemaphoreCreateRecursiveMutex();
    msg_joystick_air_led.xMutex_joystick_air_led = xSemaphoreCreateRecursiveMutex();
    msg_joystick_air_title_point.xMutex_joystick_air_dashboard_set_title = xSemaphoreCreateRecursiveMutex();
    msg_joystick_air_title_state.xMutex_joystick_air_dashboard_set_title = xSemaphoreCreateRecursiveMutex();
    msg_joystick_air_title_posture.xMutex_joystick_air_dashboard_set_title = xSemaphoreCreateRecursiveMutex();
    msg_joystick_air_title_knob_r.xMutex_joystick_air_dashboard_set_title = xSemaphoreCreateRecursiveMutex();

    msg_joystick_air_msg_point.xMutex_joystick_air_dashboard_set_msg = xSemaphoreCreateRecursiveMutex();
    msg_joystick_air_msg_state.xMutex_joystick_air_dashboard_set_msg = xSemaphoreCreateRecursiveMutex();
    msg_joystick_air_msg_posture.xMutex_joystick_air_dashboard_set_msg = xSemaphoreCreateRecursiveMutex();
    msg_joystick_air_msg_knob_r.xMutex_joystick_air_dashboard_set_msg = xSemaphoreCreateRecursiveMutex();

    msg_joystick_air_delete.xMutex_joystick_air_dashboard_del = xSemaphoreCreateRecursiveMutex();

    Speed_ratio.speed_ratio_angular = 0.5;
    Speed_ratio.speed_ratio_linear  = 0.5;
    Speed_ratio.xMutex_speed_ratio  = xSemaphoreCreateRecursiveMutex();
}

/**
 * @description: PID参数初始化
 * @return {void}
 */
void PIDInit()
{
    // 位置式pid参数设置
    Chassis_pid.Pid_pos_w.Kp    = 0.7;
    Chassis_pid.Pid_pos_w.Ki    = 0.001;
    Chassis_pid.Pid_pos_w.Kd    = 0;
    Chassis_pid.Pid_pos_w.limit = 1;

    Chassis_pid.Pid_pos_x.Kp    = 6;
    Chassis_pid.Pid_pos_x.Ki    = 0.0001;
    Chassis_pid.Pid_pos_x.Kd    = 0;
    Chassis_pid.Pid_pos_x.limit = 0.3;

    Chassis_pid.Pid_pos_y.Kp    = 6;
    Chassis_pid.Pid_pos_y.Ki    = 0.001;
    Chassis_pid.Pid_pos_y.Kd    = 0;
    Chassis_pid.Pid_pos_y.limit = 0.3;
}

void SetChassis_w_Limit(float w_limit, CHASSIS_CONTROL *current_chassis_control)
{
    xSemaphoreTakeRecursive(current_chassis_control->xMutex_control, portMAX_DELAY);
    current_chassis_control->Chassis_Control_w_limit = w_limit / 10.0;
    xSemaphoreGiveRecursive(current_chassis_control->xMutex_control);
}

void JoystickControl()
{
    /*设计操作手的操作*/
    // 微调w
    vPortEnterCritical();
    SetChassis_w_Limit((float)ReadJoystickKnobsRight(&msg_joystick_air), &Chassis_control);
    vPortExitCritical();
    
    // 移动至取环区
    if (ReadJoystickButtons(&msg_joystick_air, Btn_Btn4)) {
        vPortEnterCritical();
        mav_posture.point = Pickup_Point_Left;
        vPortExitCritical();
    }
    if (ReadJoystickButtons(&msg_joystick_air, Btn_Btn5)) {
        vPortEnterCritical();
        mav_posture.point = Pickup_Point_Right;
        vPortExitCritical();
    }
    // 移动至射环区
    if (ReadJoystickButtons(&msg_joystick_air, Btn_RightCrossLeft)) {
        vPortEnterCritical();
        mav_posture.point = Fire_piont_1;
        vPortExitCritical();
    }
    if (ReadJoystickButtons(&msg_joystick_air, Btn_RightCrossUp)) {
        vPortEnterCritical();
        mav_posture.point = Fire_piont_2;
        vPortExitCritical();
    }
    if (ReadJoystickButtons(&msg_joystick_air, Btn_RightCrossMid)) {
        vPortEnterCritical();
        mav_posture.point = Fire_piont_3;
        vPortExitCritical();
    }
    if (ReadJoystickButtons(&msg_joystick_air, Btn_RightCrossRight)) {
        vPortEnterCritical();
        mav_posture.point = Fire_piont_4;
        vPortExitCritical();
    }
    if (ReadJoystickButtons(&msg_joystick_air, Btn_RightCrossDown)) {
        vPortEnterCritical();
        mav_posture.point = Fire_piont_5;
        vPortExitCritical();
    }
    // 切换手动自动模式
    if (ReadJoystickSwitchs(&msg_joystick_air, Right_switch) == 0) {
        ChassisSwitchState(RemoteControl, &Chassis_component);
    } else if (ReadJoystickSwitchs(&msg_joystick_air, Right_switch) == 1) {
        ChassisSwitchState(ComputerControl, &Chassis_component);
    }
    // 霍尔自检
    if (ReadJoystickButtons(&msg_joystick_air, Btn_Btn2)) {
        ChassisSwitchState(HallCorrecting, &Chassis_component);
    }
    // 切换高速低速
    if (ReadJoystickSwitchs(&msg_joystick_air, Left_switch) == 0) {
        SpeedSwitchRatio(0.5, 0.7, &Speed_ratio);
    } else if (ReadJoystickSwitchs(&msg_joystick_air, Left_switch) == 1) {
        SpeedSwitchRatio(1.5, 2.0, &Speed_ratio);
    }
}

//
void Test_Navigation()
{
    /*测试按钮导航*/
    ChassisSwitchState(ComputerControl, &Chassis_component);

    if (ReadJoystickButtons(&msg_joystick_air, Btn_LeftCrossLeft)) {
        // Code for Btn_LeftCrossLeft
        ChassisSwitchPoint(Pickup_Point_Left, &Chassis_component);
    }

    if (ReadJoystickButtons(&msg_joystick_air, Btn_LeftCrossRight)) {
        // Code for Btn_LeftCrossRight
        ChassisSwitchPoint(Pickup_Point_Right, &Chassis_component);
    }

    if (ReadJoystickButtons(&msg_joystick_air, Btn_LeftCrossMid)) {
        // Code for Btn_LeftCrossMid
        ChassisSwitchPoint(Fire_piont_1, &Chassis_component);
    }

    if (ReadJoystickButtons(&msg_joystick_air, Btn_RightCrossUp)) {
        // Code for Btn_RightCrossUp
        ChassisSwitchPoint(Fire_piont_2, &Chassis_component);
    }

    if (ReadJoystickButtons(&msg_joystick_air, Btn_RightCrossDown)) {
        // Code for Btn_RightCrossDown
        ChassisSwitchPoint(Fire_piont_3, &Chassis_component);
    }
    if (ReadJoystickButtons(&msg_joystick_air, Btn_RightCrossDown)) {
        // Code for Btn_RightCrossDown
        ChassisSwitchPoint(Fire_piont_4, &Chassis_component);
    }
    if (ReadJoystickButtons(&msg_joystick_air, Btn_RightCrossDown)) {
        // Code for Btn_RightCrossDown
        ChassisSwitchPoint(Fire_piont_5, &Chassis_component);
    }
}

void Test_RemoteControl()
{
    /*遥控器摇杆控制*/
    ChassisSwitchState(RemoteControl, &Chassis_component);
}

// todo 码盘坐标系转换函数、底盘坐标系转换函数