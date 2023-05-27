#ifndef CHASSIS_PERCEPTION_H
#define CHASSIS_PERCEPTION_H

#include "chassis_start.h"
#include "chassis_commen.h"

void PerceptionTaskStart();
void PercerptionInit();
void PerceptionSwitchState(PERCEPTION_STATE target_perception_state, PERCEPTION_COMPONENT *perception_component);
extern PERCEPTION_COMPONENT Perception_component;
#endif