#ifndef __STATE_MANAGEMENT_H__
#define __STATE_MANAGEMENT_H__

#include "wtr_dji.h"
#include "chassis_driver.h"
#include "wtr_calculate.h"
#include "semphr.h"
#include "wtr_mavlink.h"
#include "mavlink_msg_controller.h"
#include "chassis_commen.h"

void StateManagemantTaskStart();
void StateInit();
void PIDInit();

#endif