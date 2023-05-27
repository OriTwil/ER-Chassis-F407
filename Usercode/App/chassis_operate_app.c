#include "chassis_operate_app.h"
#include "mavlink_msg_controller.h"
#include "semphr.h"
#include "chassis_communicate.h"
#include "chassis_machine.h"
#include "chassis_servo.h"
#include "chassis_perception.h"
#include "user_config.h"
#include "chassis_commen.h"
#include "pickup_state_machine.h"

/**
 * @description: 操作线程
 * @todo 根据各个传感器、遥控器等设计操作手操作流程
 * @return {void}
 */
void StateManagemantTask(void const *argument)
{
    vTaskDelay(20);
    for (;;) {
        vTaskDelay(10);
    }
}

/**
 * @description: 测试
 * @return {void}
 */
void StateManagemantTestTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    vTaskDelay(20);
    for (;;) {
        vTaskDelayUntil(&PreviousWakeTime, 5);
    }
}

/**
 * @description: 开启线程
 * @return {void}
 */
void StateManagemantTaskStart()
{

    osThreadDef(statemanagement, StateManagemantTask, osPriorityBelowNormal, 0, 512);
    osThreadCreate(osThread(statemanagement), NULL);

    // osThreadDef(statemanagementtest,StateManagemantTestTask,osPriorityBelowNormal,0,512);
    // osThreadCreate(osThread(statemanagementtest),NULL);
}

void StateInit()
{
    Chassis_component.Chassis_Point  = First_Ring;
    Chassis_component.Chassis_State  = Ready;
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

    Chassis_control.Chassis_Control_vw = 0;
    Chassis_control.Chassis_Control_vx = 0;
    Chassis_control.Chassis_Control_vy = 0;
    Chassis_control.Chassis_Control_w  = 0;
    Chassis_control.Chassis_Control_x  = 0;
    Chassis_control.Chassis_Control_y  = 0;
    Chassis_control.xMutex_control     = xSemaphoreCreateRecursiveMutex();

    Pickup_component.Pickup_Ring   = First_Ring;
    Pickup_component.Pickup_State  = Ready;
    Pickup_component.xMutex_pickup = xSemaphoreCreateRecursiveMutex();

    Wheel_component.xMutex_wheel = xSemaphoreCreateRecursiveMutex();
    Chassis_Init(Wheel_component.wheels);
}

/**
 * @description: PID参数初始化
 * @return {void}
 */
void PIDInit()
{
    // 位置式pid参数设置
    Chassis_pid.Pid_pos_w.Kp    = 40;
    Chassis_pid.Pid_pos_w.Ki    = 0;
    Chassis_pid.Pid_pos_w.Kd    = 0;
    Chassis_pid.Pid_pos_w.limit = 0.5;

    Chassis_pid.Pid_pos_x.Kp    = 5;
    Chassis_pid.Pid_pos_x.Ki    = 0;
    Chassis_pid.Pid_pos_x.Kd    = 0;
    Chassis_pid.Pid_pos_x.limit = 0.5;

    Chassis_pid.Pid_pos_y.Kp    = 5;
    Chassis_pid.Pid_pos_y.Ki    = 0;
    Chassis_pid.Pid_pos_y.Kd    = 0;
    Chassis_pid.Pid_pos_y.limit = 0.5;
}

// todo 码盘坐标系转换函数、底盘坐标系转换函数