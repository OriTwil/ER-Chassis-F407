#include "state_management.h"
#include "mavlink_msg_controller.h"
#include "user_config.h"

WHEEL_COMPONENT Wheel_component;
CHASSIS_COMPONENT Chassis_component;
PERCEPTION_COMPONENT Perception_component;
CHASSIS_PID Chassis_pid;
CHASSIS_CONTROL Chassis_control;
PICKUP_COMPONENT Pickup_component;
CHASSIS_POSITION Chassis_position;
// 变量定义
mavlink_posture_t mav_posture;
mavlink_control_t control;
mavlink_controller_t ControllerData;

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

void PerceptionSwitchState(PERCEPTION_STATE target_perception_state, PERCEPTION_COMPONENT *perception_component)
{
    xSemaphoreTakeRecursive(perception_component->xMutex_perception, (TickType_t)10);
    perception_component->Perception_State = target_perception_state;
    xSemaphoreGiveRecursive(perception_component->xMutex_perception);
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

void PickupSwitchState(PICKUP_STATE target_pickup_state, PICKUP_COMPONENT *pickup_component)
{
    xSemaphoreTakeRecursive(pickup_component->xMutex_pickup, (TickType_t)10);
    pickup_component->Pickup_State = target_pickup_state;
    xSemaphoreGiveRecursive(pickup_component->xMutex_pickup);
}

void PickupSwitchRing(PICKUP_RING target_pickup_ring, PICKUP_COMPONENT *pickup_component)
{
    xSemaphoreTakeRecursive(pickup_component->xMutex_pickup, (TickType_t)10);
    pickup_component->Pickup_Ring = target_pickup_ring;
    xSemaphoreGiveRecursive(pickup_component->xMutex_pickup);
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
