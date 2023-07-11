#ifndef __STATE_MANAGEMENT_H__
#define __STATE_MANAGEMENT_H__

#include "wtr_dji.h"
#include "chassis_driver.h"
#include "wtr_calculate.h"
#include "semphr.h"
#include "wtr_mavlink.h"
#include "chassis_commen.h"

void StateManagemantTaskStart();
void StateInit();
void PIDInit();
void JoystickControl();
void TestControl();

extern int32_t speed;
extern int32_t pitch;
extern int32_t yaw;

#endif