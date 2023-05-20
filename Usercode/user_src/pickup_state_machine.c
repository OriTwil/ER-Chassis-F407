#include "pickup_state_machine.h"
#include "state_management.h"

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