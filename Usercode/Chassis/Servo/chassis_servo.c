#include "chassis_servo.h"
#include "user_config.h"
#include "can.h"
#include "wtr_can.h"
#include "chassis_servo.h"
#include "chassis_commen.h"
#include "chassis_machine.h"
#include "chassis_driver.h"

CHASSIS_PID Chassis_pid;

/**
 * @description: 伺服线程
 * @author: szf
 * @return {void}
 */
void ServoTask(void const *argument)
{
    uint32_t PreviousWakeTime = xTaskGetTickCount();
    vTaskDelay(20);
    for (;;) {
        // 更新PID的目标值和反馈值
        xSemaphoreTakeRecursive(Chassis_control.xMutex_control, portMAX_DELAY);
        SetPIDTarget(Chassis_control.Chassis_Control_x,
                     Chassis_control.Chassis_Control_y,
                     Chassis_control.Chassis_Control_w + Chassis_control.Chassis_Control_w_limit,
                     &Chassis_pid);
        xSemaphoreGiveRecursive(Chassis_control.xMutex_control);

        xSemaphoreTakeRecursive(Chassis_position.xMutex_position, portMAX_DELAY);
        SetPIDFeedback(Chassis_position.Chassis_Position_x,
                       Chassis_position.Chassis_Position_y,
                       Chassis_position.Chassis_Position_w,
                       &Chassis_pid);
        xSemaphoreGiveRecursive(Chassis_position.xMutex_position);

        // 伺服控制
        ServoWheels(&Wheel_component);

        vTaskDelayUntil(&PreviousWakeTime, 3);
    }
}

/**
 * @description: 伺服测试线程，比赛时关掉
 * @author: szf
 * @return {void}
 */
void ServoTestTask(void const *argument)
{
    uint32_t PreviousWakeTime = xTaskGetTickCount();
    vTaskDelay(20);
    for (;;) {
        vTaskDelayUntil(&PreviousWakeTime, 3);
    }
}

/**
 * @description: 创建伺服线程
 * @author: szf
 * @return {void}
 */
void ServoTaskStart()
{
    osThreadDef(servo, ServoTask, osPriorityAboveNormal, 0, 1024);
    osThreadCreate(osThread(servo), NULL);

    // osThreadDef(servo_test,ServoTestTask,osPriorityBelowNormal,0,512);
    // osThreadCreate(osThread(servo_test),NULL);
}

// 电机初始化
void MotorInit()
{
    CANFilterInit(&hcan1);
    hDJI[0].motorType = M2006; //
    hDJI[1].motorType = M2006; //
    hDJI[2].motorType = M2006; //
    hDJI[3].motorType = M2006; //
    hDJI[4].motorType = M2006; //
    hDJI[5].motorType = M2006; //
    hDJI[6].motorType = M2006;
    DJI_Init(); // 大疆电机初始化
}

/**
 * @description: 设置PID目标值
 * @param target_ PID目标值
 * @return {void}
 */
void SetPIDTarget(float target_x, float target_y, float target_w, CHASSIS_PID *chassis_pid)
{
    xSemaphoreTakeRecursive(chassis_pid->xMutex_pid, (TickType_t)10);
    // 位置式PID
    chassis_pid->Pid_pos_w.target = target_w;
    chassis_pid->Pid_pos_x.target = target_x;
    chassis_pid->Pid_pos_y.target = target_y;
    xSemaphoreGiveRecursive(chassis_pid->xMutex_pid);
}

/**
 * @description: 设置PID反馈值
 * @param feedback_ PID反馈值
 * @return {void}
 */
void SetPIDFeedback(float feedback_x, float feedback_y, float feedback_w, CHASSIS_PID *chassis_pid)
{
    xSemaphoreTakeRecursive(chassis_pid->xMutex_pid, (TickType_t)10);
    chassis_pid->Pid_pos_x.feedback = feedback_x;
    chassis_pid->Pid_pos_y.feedback = feedback_y;
    chassis_pid->Pid_pos_w.feedback = feedback_w;
    xSemaphoreGiveRecursive(chassis_pid->xMutex_pid);
}

/**
 * @description: 更新底盘位置
 * @param position_ 底盘位置信息
 * @return {void}
 */
void SetChassisPosition(float position_x, float position_y, float position_w, CHASSIS_POSITION *chassis_position)
{
    xSemaphoreTakeRecursive(chassis_position->xMutex_position, (TickType_t)10);
    chassis_position->Chassis_Position_x = position_x;
    chassis_position->Chassis_Position_y = position_y;
    chassis_position->Chassis_Position_w = position_w;
    xSemaphoreGiveRecursive(chassis_position->xMutex_position);
}

/**
 * @description: 设置底盘伺服速度值
 * @param vx_control 底盘速度伺服值
 * @return {void}
 */
void SetChassisControlVelocity(float vx_control, float vy_control, float vw_control, CHASSIS_CONTROL *chassis_control)
{
    xSemaphoreTakeRecursive(chassis_control->xMutex_control, (TickType_t)10);
    chassis_control->Chassis_Control_vx = vx_control;
    chassis_control->Chassis_Control_vy = vy_control;
    chassis_control->Chassis_Control_vw = vw_control;
    xSemaphoreGiveRecursive(chassis_control->xMutex_control);
}

/**
 * @description: 设置底盘目标位置
 * @param _control 底盘目标位置
 * @return {void}
 */
void SetChassisControlPosition(float x_control, float y_control, float w_control, CHASSIS_CONTROL *chassis_control)
{
    xSemaphoreTakeRecursive(chassis_control->xMutex_control, (TickType_t)10);
    chassis_control->Chassis_Control_x = x_control;
    chassis_control->Chassis_Control_y = y_control;
    chassis_control->Chassis_Control_w = w_control;
    xSemaphoreGiveRecursive(chassis_control->xMutex_control);
}

void CalculateWheels(CHASSIS_CONTROL *chassis_control, WHEEL_COMPONENT *wheel_component)
{
    xSemaphoreTakeRecursive(chassis_control->xMutex_control, (TickType_t)10);
    xSemaphoreTakeRecursive(wheel_component->xMutex_wheel, (TickType_t)10);
    for (int i = 0; i < 3; i++) {
        Wheel_SetXY(&(wheel_component->wheels[i]),
                    chassis_control->Chassis_Control_vx - chassis_control->Chassis_Control_vw * wheel_component->wheels[i].loc_y,
                    chassis_control->Chassis_Control_vy + wheel_component->wheels[i].loc_x * chassis_control->Chassis_Control_vw); // 运动学解算
    }
    xSemaphoreGiveRecursive(wheel_component->xMutex_wheel);
    xSemaphoreGiveRecursive(chassis_control->xMutex_control);
}

void ServoWheels(WHEEL_COMPONENT *wheel_component)
{
    xSemaphoreTakeRecursive(wheel_component->xMutex_wheel, (TickType_t)10);
    Wheels_CalcTransmit(wheel_component->wheels, 3);
    xSemaphoreGiveRecursive(wheel_component->xMutex_wheel);
}

void SetWheelsRef(int wheel_id, double target_speed, double target_pos, WHEEL_COMPONENT *wheel_component)
{
    xSemaphoreTakeRecursive(wheel_component->xMutex_wheel, (TickType_t)10);
    Wheel_Set(&(wheel_component->wheels[wheel_id]), target_speed, target_pos);
    xSemaphoreGiveRecursive(wheel_component->xMutex_wheel);
}
