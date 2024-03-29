/**
 * @file beep.c
 * @author TITH (1023515576@qq.com)
 * @brief 蜂鸣器
 * @version 0.1
 * @date 2022-07-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "chassis_beep.h"
#include "semphr.h"
#include "gpio.h"
#include "chassis_start.h"
#include "user_config.h"
#include "chassis_commen.h"

SemaphoreHandle_t BeepSem;

void Beep()
{
	xSemaphoreGive(BeepSem);
}

void BeepSet(bool is_on)
{
	HAL_GPIO_WritePin(BeepPinGPIOx, BeepPinGPIO_Pin, is_on);
}

void BeepFromISR()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(BeepSem, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void BeepTask(void const *argument)
{
	SemaphoreHandle_t *sem = (SemaphoreHandle_t *)argument;

	while (xSemaphoreTake(*sem, portMAX_DELAY))
	{
		HAL_GPIO_WritePin(BeepPinGPIOx, BeepPinGPIO_Pin, 1);
		osDelay(100);
		HAL_GPIO_WritePin(BeepPinGPIOx, BeepPinGPIO_Pin, 0);
		osDelay(100);
	}
}

void Beep_Init()
{
	BeepSem = xSemaphoreCreateCounting(5, 0);

	osThreadDef(beep_task, BeepTask, osPriorityNormal, 0, 128);
	osThreadCreate(osThread(beep_task), &BeepSem);
}
