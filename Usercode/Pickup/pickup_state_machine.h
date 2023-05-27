#ifndef __PICKUP_STATE_MACHINE_H__
#define __PICKUP_STATE_MACHINE_H__

#include "chassis_start.h"
#include "chassis_commen.h"

void PickupSwitchState(PICKUP_STATE target_pickup_state, PICKUP_COMPONENT *pickup_component);
void PickupSwitchRing(PICKUP_RING target_pickup_ring, PICKUP_COMPONENT *pickup_component);

extern PICKUP_COMPONENT Pickup_component;
#endif
