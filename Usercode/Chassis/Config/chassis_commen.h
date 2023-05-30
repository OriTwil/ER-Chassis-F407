#ifndef __COMMEN_H__
#define __COMMEN_H__

#include "wtr_calculate.h"
#include "wtr_vesc.h"
#include "wtr_mavlink.h"

typedef __IO struct
{
    double loc_x; // 舵轮在机器人坐标系中的位置
    double loc_y; // 舵轮在机器人坐标系中的位置

    double exp_rot_pos;
    double exp_speed;
    double now_rot_pos;

    double rot_pos_offset; // 角位置偏置，(rot_pos + rot_pos_offset) * rot_pos_ratio = PID 的值
    double rot_pos_ratio;  // 角位置比率，(rot_pos + rot_pos_offset) * rot_pos_ratio = PID 的值
    double speed_ratio;    // 速度比率，speed * speed_ratio = ERPM
    double hall_on_pos;    // 霍尔检测到时的 now_rot_pos
    double hall_on_flag;   // 霍尔检测到时的 now_rot_pos
    double hall_off_pos;   // 霍尔失去时的 now_rot_pos
    double hall_angle;     // 霍尔传感器所在的角度
    DJI_t *hDJI;
    VESC_t hvesc;
} uni_wheel_t;

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

typedef struct
{
    mavlink_joystick_air_led_t msg_joystick_air_led;
    mavlink_joystick_air_dashboard_set_title_t msg_joystick_air_dashboard_set_title;
    mavlink_joystick_air_dashboard_set_msg_t msg_joystick_air_dashboard_set_msg;
    mavlink_joystick_air_dashboard_del_t msg_joystick_air_dashboard_del;
    SemaphoreHandle_t xMutex_joystick;
} JOYSTICK_SEND;

// joystick
typedef enum {
    Left_switch = 0,
    Right_switch
} SWITCHS;

typedef enum {
    Btn_LeftCrossUp     = 2,
    Btn_LeftCrossDown   = 1,
    Btn_LeftCrossLeft   = 13,
    Btn_LeftCrossRight  = 5,
    Btn_LeftCrossMid    = 9,
    Btn_RightCrossUp    = 4,
    Btn_RightCrossDown  = 3,
    Btn_RightCrossLeft  = 7,
    Btn_RightCrossRight = 15,
    Btn_RightCrossMid   = 11,
    Btn_Btn0            = 6,
    Btn_Btn1            = 10,
    Btn_Btn2            = 14,
    Btn_Btn3            = 8,
    Btn_Btn4            = 12,
    Btn_Btn5            = 16,
    Btn_KnobL           = 17,
    Btn_KnobR           = 18,
    Btn_JoystickL       = 19,
    Btn_JoystickR       = 20,
} KEYS;

#endif