#include "pickup_state_machine.h"
#include "chassis_communicate.h"
#include "chassis_commen.h"

PICKUP_COMPONENT Pickup_component;

// todo 根据机械写状态机
void PickupTask(void const *argument)
{
    vTaskDelay(200);

    for (;;) {
        switch (Pickup_component.Pickup_State) {
            case Ready:
                break;
            case Pickup:
                break;
            case Fire:
                break;
        }
        vTaskDelay(5);
    }
}

void PickupTaskStart()
{
    osThreadDef(Pickup, PickupTask, osPriorityBelowNormal, 0, 1024);
    osThreadCreate(osThread(Pickup), NULL);
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