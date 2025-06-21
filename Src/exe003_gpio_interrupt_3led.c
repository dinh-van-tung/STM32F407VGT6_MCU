/*
 * exe003_gpio_interrupt_3led.c
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
	GPIO_Handle_t Button, Red, Green, Blue;

	Button.pGPIOx = GPIOB;
	Button.GPIO_Config.GPIO_PinNumber = GPIO_PIN_7;
	Button.GPIO_Config.GPIO_PinMode = GPIO_MODE_INTERRUPT_FTRIG;
	Button.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	Red.pGPIOx = GPIOC;
	Red.GPIO_Config.GPIO_PinNumber = GPIO_PIN_7;
	Red.GPIO_Config.GPIO_PinMode = GPIO_MODE_OUTPUT;
	Red.GPIO_Config.GPIO_PinOutSpeed = GPIO_OUT_SPEED_HIGH;
	Red.GPIO_Config.GPIO_PinOutType = GPIO_OUT_TYPE_PUSHPULL;
	Red.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	Green.pGPIOx = GPIOD;
	Green.GPIO_Config.GPIO_PinNumber = GPIO_PIN_7;
	Green.GPIO_Config.GPIO_PinMode = GPIO_MODE_OUTPUT;
	Green.GPIO_Config.GPIO_PinOutSpeed = GPIO_OUT_SPEED_HIGH;
	Green.GPIO_Config.GPIO_PinOutType = GPIO_OUT_TYPE_PUSHPULL;
	Green.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	Blue.pGPIOx = GPIOE;
	Blue.GPIO_Config.GPIO_PinNumber = GPIO_PIN_7;
	Blue.GPIO_Config.GPIO_PinMode = GPIO_MODE_OUTPUT;
	Blue.GPIO_Config.GPIO_PinOutSpeed = GPIO_OUT_SPEED_HIGH;
	Blue.GPIO_Config.GPIO_PinOutType = GPIO_OUT_TYPE_PUSHPULL;
	Blue.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	GPIO_Init(&Button);
	GPIO_Init(&Red);
	GPIO_Init(&Green);
	GPIO_Init(&Blue);

	GPIO_IRQInterruptConfig(IRQ_NUMBER_EXTI_9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NUMBER_EXTI_9_5, NVIC_IRQ_PRIORITY_7);

	while (1) {
		if (cnt % 4 == 0) {
			GPIO_WriteOutputPin(GPIOE, GPIO_PIN_7, RESET);
		}

		while (cnt % 4 == 1) {
			TimeDelay(8);
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_7);
		}

		while (cnt % 4 == 2) {
			GPIO_WriteOutputPin(GPIOC, GPIO_PIN_7, RESET);
			TimeDelay(4);
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_7);
		}

		while (cnt % 4 == 3) {
			GPIO_WriteOutputPin(GPIOD, GPIO_PIN_7, RESET);
			TimeDelay(2);
			GPIO_ToggleOutputPin(GPIOE, GPIO_PIN_7);
		}
	}

	return 0;
}

void EXTI9_5_IRQHandler(void) {
	GPIO_WriteOutputPin(GPIOC, GPIO_PIN_7, RESET);
	GPIO_WriteOutputPin(GPIOD, GPIO_PIN_7, RESET);
	GPIO_WriteOutputPin(GPIOE, GPIO_PIN_7, RESET);

	TimeDelay(2);
	GPIO_IRQHandling(GPIO_PIN_7);
	++cnt;
}
