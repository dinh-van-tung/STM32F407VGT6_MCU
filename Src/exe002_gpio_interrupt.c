/*
 * exe002_gpio_interrupt.c
 *
 * Created on: Jun 21, 2025
 * Author: Van Tung Dinh
 */

#include <stdio.h>
#include <stdint.h>
#include "stm32f407xx.h"

void TimeDelay(uint8_t Value) {
	for (uint32_t i = 0; i <= 500000 / Value; i += 1) {}
}

uint32_t cnt = 0;

int main(void) {
	GPIO_Handle_t Button, Led;

	Button.pGPIOx = GPIOB;
	Button.GPIO_Config.GPIO_PinNumber = GPIO_PIN_7;
	Button.GPIO_Config.GPIO_PinMode = GPIO_MODE_INTERRUPT_FTRIG;
	Button.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	Led.pGPIOx = GPIOC;
	Led.GPIO_Config.GPIO_PinNumber = GPIO_PIN_7;
	Led.GPIO_Config.GPIO_PinMode = GPIO_MODE_OUTPUT;
	Led.GPIO_Config.GPIO_PinOutSpeed = GPIO_OUT_SPEED_HIGH;
	Led.GPIO_Config.GPIO_PinOutType = GPIO_OUT_TYPE_PUSHPULL;
	Led.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	GPIO_Init(&Button);
	GPIO_Init(&Led);

	GPIO_IRQInterruptConfig(IRQ_NUMBER_EXTI_9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NUMBER_EXTI_9_5, NVIC_IRQ_PRIORITY_7);

	while (1) {
		if (cnt % 3 == 0) {
			GPIO_WriteOutputPin(GPIOC, GPIO_PIN_7, RESET);
		}
		else if (cnt % 3 == 1) {
			GPIO_WriteOutputPin(GPIOC, GPIO_PIN_7, SET);
		}
		else {
			while (cnt % 3 == 2) {
				TimeDelay(8);
				GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_7);
			}
		}
	}

	return 0;
}

void EXTI9_5_IRQHandler(void) {
	TimeDelay(2);
	GPIO_IRQHandling(GPIO_PIN_7);
	++cnt;
}
