#ifndef __STATE_MANAGEMENT_H__
#define __STATE_MANAGEMENT_H__

#include "wtr_dji.h"
#include "chassis_driver.h"
#include "user_calculate.h"
#include "semphr.h"
#include "wtr_mavlink.h"
#include "mavlink_msg_controller.h"

typedef enum {
    Locked,          // 舵轮呈X型锁死
    RemoteControl,   // 遥控器控制
    ComputerControl, // 上位机规划
    HallCorrecting
} CHASSIS_STATE;

typedef enum {
    Receive,
    Transmit
} PERCEPTION_STATE;

typedef enum {
    First_Point = 1,
    Second_Point,
    Third_Point,
    Fourth_Point,
    Fifth_Point,
    Sixth_Point,
    Seventh_Point,
    Eighth_Point,
    Ninth_Point,
    Tenth_Point
} CHASSIS_POINT;

typedef enum {
    Ready,
    Pickup,
    Fire
} PICKUP_STATE;
    
typedef enum {
    First_Ring = 1,
    Second_Ring,
    Third_Ring,
    Fourth_Ring,
    Fifth_Ring
} PICKUP_RING;

// todo 按照什么标准划分结构体？现在是按组件
typedef __IO struct
{
    uni_wheel_t wheels[3];
    double wheel_vx;
    double wheel_vy;
    double rot_pos;
    SemaphoreHandle_t xMutex_wheel;
} WHEEL_COMPONENT;

typedef __IO struct
{
    CHASSIS_STATE Chassis_State;
    CHASSIS_POINT Chassis_Point;
    SemaphoreHandle_t xMutex_chassis;
} CHASSIS_COMPONENT;

typedef __IO struct
{
    PERCEPTION_STATE Perception_State;
    SemaphoreHandle_t xMutex_perception;
} PERCEPTION_COMPONENT;

typedef __IO struct
{
    PID_Pos Pid_pos_w;
    PID_Pos Pid_pos_x;
    PID_Pos Pid_pos_y;
    SemaphoreHandle_t xMutex_pid;
} CHASSIS_PID;

typedef __IO struct
{
    float Chassis_Control_vx;
    float Chassis_Control_vy;
    float Chassis_Control_vw;
    float Chassis_Control_x;
    float Chassis_Control_y;
    float Chassis_Control_w;
    SemaphoreHandle_t xMutex_control;
} CHASSIS_CONTROL;

typedef __IO struct
{
    PICKUP_STATE Pickup_State;
    PICKUP_RING Pickup_Ring;
    SemaphoreHandle_t xMutex_pickup;
} PICKUP_COMPONENT;

typedef __IO struct
{
    float Chassis_Position_x;
    float Chassis_Position_y;
    float Chassis_Position_w;
    float Chassis_Angle_y; // 俯仰
    SemaphoreHandle_t xMutex_position;
} CHASSIS_POSITION;

void StateManagemantTaskStart();
void StateInit();
void PIDInit();
void ChassisSwitchState(CHASSIS_STATE target_chassis_state, CHASSIS_COMPONENT *chassis_component);
void ChassisSwitchPoint(CHASSIS_POINT target_point, CHASSIS_COMPONENT *chassis_component);
void PerceptionSwitchState(PERCEPTION_STATE target_perception_state, PERCEPTION_COMPONENT *perception_component);
void SetPIDTarget(float target_x, float target_y, float target_w, CHASSIS_PID *chassis_pid);
void SetPIDFeedback(float feedback_x, float feedback_y, float feedback_w, CHASSIS_PID *chassis_pid);
void SetChassisPosition(float position_x, float position_y, float position_w, CHASSIS_POSITION *chassis_position);
void SetChassisControlVelocity(float vx_control, float vy_control, float vw_control, CHASSIS_CONTROL *chassis_control);
void SetChassisControlPosition(float x_control, float y_control, float w_control, CHASSIS_CONTROL *chassis_control);
void PickupSwitchState(PICKUP_STATE target_pickup_state, PICKUP_COMPONENT *pickup_component);
void PickupSwitchRing(PICKUP_RING target_pickup_ring, PICKUP_COMPONENT *pickup_component);

void CalculateWheels(CHASSIS_CONTROL *chassis_control, WHEEL_COMPONENT *wheel_component);
void SetWheelsRef(int wheel_id, double target_speed, double target_pos, WHEEL_COMPONENT *wheel_component);
void ServoWheels(WHEEL_COMPONENT *wheel_component);

void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, float *currentAngle);
void ChassisHallCorrect(float target_angle, WHEEL_COMPONENT *wheel_component);
extern CHASSIS_COMPONENT Chassis_component;
extern PERCEPTION_COMPONENT Perception_component;
extern CHASSIS_PID Chassis_pid;
extern CHASSIS_CONTROL Chassis_control;
extern PICKUP_COMPONENT Pickup_component;
extern CHASSIS_POSITION Chassis_position;
extern WHEEL_COMPONENT Wheel_component;
extern mavlink_posture_t mav_posture;
extern mavlink_control_t control;
extern mavlink_controller_t ControllerData;

#endif