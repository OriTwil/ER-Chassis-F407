/***
 * @Author: Xieyang
 * @Date: 2023-02-23 18:51:00
 * @LastEditTime: 2023-02-23 18:51:13
 * @LastEditors: szf
 * @Description:
 * @FilePath: \ER\Usercode\user_inc\useful_constant.h
 * @@WeChat:szf13373959031
 */

/**
 * @todo 还需要做什么?!
 *
 */
// todo strncpy()等函数，参数是volatile,产生warning
// todo 操作手
// todo 舵轮底盘

/**
 * @brief 一些有用的常量，基本上是从 GCC 的 math.h 中复制下来的(因为 ARMCC 的库里貌似没有)
 *
 */

#pragma once
#include "usart.h"
/* Natural log of 2 */
#define _M_LN2   0.693147180559945309417

#define MAXFLOAT 3.40282347e+38F

#define M_E      2.7182818284590452354
#define M_LOG2E  1.4426950408889634074
#define M_LOG10E 0.43429448190325182765
#define M_LN2    _M_LN2
#define M_LN10   2.30258509299404568402
#define M_PI     3.14159265358979323846
#define DEC      (M_PI / 180)

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif // !M_PI_2

#define M_PI_4        0.78539816339744830962
#define M_1_PI        0.31830988618379067154
#define M_2_PI        0.63661977236758134308
#define M_2_SQRTPI    1.12837916709551257390
#define M_SQRT2       1.41421356237309504880
#define M_SQRT1_2     0.70710678118654752440

#define M_TWOPI       (M_PI * 2.0)
#define M_3PI_4       2.3561944901923448370E0
#define M_SQRTPI      1.77245385090551602792981
#define M_LN2LO       1.9082149292705877000E-10
#define M_LN2HI       6.9314718036912381649E-1
#define M_SQRT3       1.73205080756887719000
#define M_IVLN10      0.43429448190325182765 /* 1 / log(10) */
#define M_LOG2_E      _M_LN2
#define M_INVLN2      1.4426950408889633870E0 /* 1 / log(2) */

#define SpeedRatioNum 2

// 限幅
#define max(a, b)      (a > b ? a : b)
#define min(a, b)      (a < b ? a : b)
#define range(x, a, b) (min(max(x, a), b))

/**
 * @brief 自定义参数、id
 *
 */

// 舵轮
#define Wheel_Front                 0
#define Wheel_Left                  1
#define Wheel_Right                 2

#define Wheel_Front_Locked_Pos      0
#define Wheel_Left_Locked_Pos       0
#define Wheel_Right_Locked_Pos      0

#define HallCorrecting_Max_Velocity 1.0472 // 60°
#define HallCorrecting_Acceleration 10.472

#define Hall_Front_GPIN_PIN         GPIO_PIN_13
#define Hall_Left_GPIN_PIN          GPIO_PIN_14
#define Hall_Right_GPIN_PIN         GPIO_PIN_12

//* 舵轮初始化数据在chassis_driver.c中的Chassis_Init()

// 按键通知值定义
#define BUTTON1_NOTIFICATION (1 << 0)
#define BUTTON2_NOTIFICATION (1 << 1)
#define BUTTON3_NOTIFICATION (1 << 2)

/**
 * @brief 自定义端口 *
 */

#define BeepPinGPIOx           GPIOD
#define BeepPinGPIO_Pin        GPIO_PIN_7

#define huart_Computer         huart4
#define huart_Remote_Control   huart6
#define huart_Chassis_to_Upper huart3
#define huart_OPS              huart2
#define huart_AS69             huart1

// wtr_ads1256_config.h
#define ADS1256_DRDY_GPIOx     GPIOA
#define ADS1256_DRDY_GPIO_Pin  GPIO_PIN_4

#define ADS1256_Reset_GPIOx    GPIOB
#define ADS1256_DRDY_GPIOx     GPIOA
#define ADS1256_DRDY_GPIO_Pin  GPIO_PIN_4

#define ADS1256_Reset_GPIOx    GPIOB

#define ADS1256_Reset_GPIO_Pin GPIO_PIN_2

// wtr_time.h
#define HAL_Tick_TIM        TIM7

#define UART_Computer       USART3
#define UART_OPS            USART2
#define UART_AS69           USART1
#define UART_Remote_Control USART6

/**
 * @brief 一些固定参数
 */
// 舵轮直径70mm
// 舵轮齿轮传动比 1:2.6
// 舵轮旋转传动比 1:3
// 舵轮之间的距离 554.6mm
#define rot_diameter 0.07 // meter
