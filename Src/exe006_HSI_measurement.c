/*
 * exe006_HSI_measurement.c
 *
 * Created on: Jun 23, 2025
 * Author: Van Tung Dinh
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"


void TimeDelay(uint8_t Div_x) {
	for (uint32_t i = 0; i <= 500000 / Div_x; i += 1) {}
}


int main(void) {

	GPIO_Handle_t GPIO_HSI_CLOCK;

	GPIO_HSI_CLOCK.pGPIOx = GPIOA;
	GPIO_HSI_CLOCK.GPIO_Config.GPIO_PinMode = GPIO_MODE_ALTERNATE_FUNC;
	GPIO_HSI_CLOCK.GPIO_Config.GPIO_PinAltFuncMode = GPIO_MODE_AF_0;
	GPIO_HSI_CLOCK.GPIO_Config.GPIO_PinNumber = GPIO_PIN_8;
	GPIO_HSI_CLOCK.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	/* HSI clock selected */
	RCC->CFGR &= ~(3U << RCC_CFGR_MCO1);
	RCC->CFGR |= (0U << RCC_CFGR_MCO1);

	/* MCO1 prescaler: division by 4 */
	RCC->CFGR &= ~(7U << RCC_CFGR_MCO1PRE);
	RCC->CFGR |= (6U << RCC_CFGR_MCO1PRE);

	TimeDelay(2);
	GPIO_Init(&GPIO_HSI_CLOCK);
	return 0;
}
