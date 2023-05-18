#include "state_management.h"

WHEEL_COMPONENT Wheel_component;
CHASSIS_COMPONENT Chassis_component;
PERCEPTION_COMPONENT Perception_component;
CHASSIS_PID Chassis_pid;
CHASSIS_CONTROL Chassis_control;
PICKUP_COMPONENT Pickup_component;
CHASSIS_POSITION Chassis_position;

/**
 * @description: 操作线程
 * @todo 根据各个传感器、遥控器等设计操作手操作流程
 * @return {void}
 */
void StateManagemantTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    vTaskDelay(20);
    for (;;) {
        vTaskDelayUntil(&PreviousWakeTime, 5);
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
    Chassis_component.xMutex_chassis = xSemaphoreCreateMutex();

    Perception_component.Perception_State  = Receive;
    Perception_component.xMutex_perception = xSemaphoreCreateMutex();

    PIDInit();
    Chassis_pid.xMutex_pid = xSemaphoreCreateMutex();

    Chassis_position.Chassis_Position_w = 0;
    Chassis_position.Chassis_Position_x = 0;
    Chassis_position.Chassis_Position_y = 0;
    Chassis_position.Chassis_Angle_y    = 0;
    Chassis_position.xMutex_position    = xSemaphoreCreateMutex();

    Chassis_control.Chassis_Control_vw = 0;
    Chassis_control.Chassis_Control_vx = 0;
    Chassis_control.Chassis_Control_vy = 0;
    Chassis_control.Chassis_Control_w  = 0;
    Chassis_control.Chassis_Control_x  = 0;
    Chassis_control.Chassis_Control_y  = 0;
    Chassis_control.xMutex_control     = xSemaphoreCreateMutex();

    Pickup_component.Pickup_Ring   = First_Ring;
    Pickup_component.Pickup_State  = Ready;
    Pickup_component.xMutex_pickup = xSemaphoreCreateMutex();

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

void ChassisSwitchState(CHASSIS_STATE target_chassis_state, CHASSIS_COMPONENT *chassis_component)
{
    xSemaphoreTake(chassis_component->xMutex_chassis, (TickType_t)10);
    chassis_component->Chassis_State = target_chassis_state;
    xSemaphoreGive(chassis_component->xMutex_chassis);
}

void ChassisSwitchPoint(CHASSIS_POINT target_point, CHASSIS_COMPONENT *chassis_component)
{
    xSemaphoreTake(chassis_component->xMutex_chassis, (TickType_t)10);
    chassis_component->Chassis_Point = target_point;
    xSemaphoreGive(chassis_component->xMutex_chassis);
}

void PerceptionSwitchState(PERCEPTION_STATE target_perception_state, PERCEPTION_COMPONENT *perception_component)
{
    xSemaphoreTake(perception_component->xMutex_perception, (TickType_t)10);
    perception_component->Perception_State = target_perception_state;
    xSemaphoreGive(perception_component->xMutex_perception);
}

/**
 * @description: 设置PID目标值
 * @param target_ PID目标值
 * @return {void}
 */
void SetPIDTarget(float target_x, float target_y, float target_w, CHASSIS_PID *chassis_pid)
{
    xSemaphoreTake(chassis_pid->xMutex_pid, (TickType_t)10);
    // 位置式PID
    chassis_pid->Pid_pos_w.target = target_w;
    chassis_pid->Pid_pos_x.target = target_x;
    chassis_pid->Pid_pos_y.target = target_y;
    xSemaphoreGive(chassis_pid->xMutex_pid);
}

/**
 * @description: 设置PID反馈值
 * @param feedback_ PID反馈值
 * @return {void}
 */
void SetPIDFeedback(float feedback_x, float feedback_y, float feedback_w, CHASSIS_PID *chassis_pid)
{
    xSemaphoreTake(chassis_pid->xMutex_pid, (TickType_t)10);
    chassis_pid->Pid_pos_x.feedback = feedback_x;
    chassis_pid->Pid_pos_y.feedback = feedback_y;
    chassis_pid->Pid_pos_w.feedback = feedback_w;
    xSemaphoreGive(chassis_pid->xMutex_pid);
}

/**
 * @description: 更新底盘位置
 * @param position_ 底盘位置信息
 * @return {void}
 */
void SetChassisPosition(float position_x, float position_y, float position_w, CHASSIS_POSITION *chassis_position)
{
    xSemaphoreTake(chassis_position->xMutex_position, (TickType_t)10);
    chassis_position->Chassis_Position_x = position_x;
    chassis_position->Chassis_Position_y = position_y;
    chassis_position->Chassis_Position_w = position_w;
    xSemaphoreGive(chassis_position->xMutex_position);
}

/**
 * @description: 设置底盘伺服速度值
 * @param vx_control 底盘速度伺服值
 * @return {void}
 */
void SetChassisControlVelocity(float vx_control, float vy_control, float vw_control, CHASSIS_CONTROL *chassis_control)
{
    xSemaphoreTake(chassis_control->xMutex_control, (TickType_t)10);
    chassis_control->Chassis_Control_vx = vx_control;
    chassis_control->Chassis_Control_vy = vy_control;
    chassis_control->Chassis_Control_vw = vw_control;
    xSemaphoreGive(chassis_control->xMutex_control);
}

/**
 * @description: 设置底盘目标位置
 * @param _control 底盘目标位置
 * @return {void}
 */
void SetChassisControlPosition(float x_control, float y_control, float w_control, CHASSIS_CONTROL *chassis_control)
{
    xSemaphoreTake(chassis_control->xMutex_control, (TickType_t)10);
    chassis_control->Chassis_Control_x = x_control;
    chassis_control->Chassis_Control_y = y_control;
    chassis_control->Chassis_Control_w = w_control;
    xSemaphoreGive(chassis_control->xMutex_control);
}

void PickupSwitchState(PICKUP_STATE target_pickup_state, PICKUP_COMPONENT *pickup_component)
{
    xSemaphoreTake(pickup_component->xMutex_pickup, (TickType_t)10);
    pickup_component->Pickup_State = target_pickup_state;
    xSemaphoreGive(pickup_component->xMutex_pickup);
}

void PickupSwitchRing(PICKUP_RING target_pickup_ring, PICKUP_COMPONENT *pickup_component)
{
    xSemaphoreTake(pickup_component->xMutex_pickup, (TickType_t)10);
    pickup_component->Pickup_Ring = target_pickup_ring;
    xSemaphoreGive(pickup_component->xMutex_pickup);
}


//todo 看似和上面的函数有功能重合的嫌疑
void SetChassisVelocity(CHASSIS_CONTROL *chassis_control,WHEEL_COMPONENT *wheel_component)
{
    xSemaphoreTake(chassis_control->xMutex_control, (TickType_t)10);
    xSemaphoreTake(wheel_component->xMutex_wheel, (TickType_t)10);
    Chassis_SetSpeed(wheel_component->wheels,3,chassis_control->Chassis_Control_vx,chassis_control->Chassis_Control_vy,chassis_control->Chassis_Control_vw);
    xSemaphoreGive(wheel_component->xMutex_wheel);
    xSemaphoreGive(chassis_control->xMutex_control);   
}

void ServoWheels(WHEEL_COMPONENT *wheel_component)
{
    xSemaphoreTake(wheel_component->xMutex_wheel, (TickType_t)10);
    Wheels_CalcTransmit(wheel_component->wheels,3);
    xSemaphoreGive(wheel_component->xMutex_wheel);
}

void SetWheelsRef(double target_speed,double target_pos,WHEEL_COMPONENT *wheel_component)
{
    xSemaphoreTake(wheel_component->xMutex_wheel, (TickType_t)10);
    Wheel_Set(&(wheel_component->wheels[0]),target_speed,target_pos);
    Wheel_Set(&(wheel_component->wheels[1]),target_speed,target_pos);
    Wheel_Set(&(wheel_component->wheels[2]),target_speed,target_pos);
    xSemaphoreGive(wheel_component->xMutex_wheel);
}


