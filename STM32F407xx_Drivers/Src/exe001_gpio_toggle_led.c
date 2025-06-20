/*
 * exe001_gpio_toggle_led.c
 * Created on: Jun 20, 2025
 * Author: Van Tung Dinh
 */

#include <stdio.h>
#include <stdint.h>
#include "stm32f407xx.h"

void TimeDelay(void) {
	for (uint32_t i = 0; i <= 500000 / 2; i += 1) {}
}

int main(void) {
	GPIO_Handle_t Internal_Button, Internal_Led;

	Internal_Button.pGPIOx = GPIOA;
	Internal_Button.GPIO_Config.GPIO_PinNumber = GPIO_PIN_0;
	Internal_Button.GPIO_Config.GPIO_PinMode = GPIO_MODE_INPUT;
	Internal_Button.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	Internal_Led.pGPIOx = GPIOD;
	Internal_Led.GPIO_Config.GPIO_PinNumber = GPIO_PIN_15;
	Internal_Led.GPIO_Config.GPIO_PinMode = GPIO_MODE_OUTPUT;
	Internal_Led.GPIO_Config.GPIO_PinOutSpeed = GPIO_OUT_SPEED_VERYHIGH;
	Internal_Led.GPIO_Config.GPIO_PinOutType = GPIO_OUT_TYPE_PUSHPULL;
	Internal_Led.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	GPIO_Init(&Internal_Button);
	GPIO_Init(&Internal_Led);

	GPIO_WriteOutputPin(GPIOD, GPIO_PIN_15, SET);

	uint32_t cnt = 0;
	while (1) {

		while (GPIO_ReadInputPin(GPIOA, GPIO_PIN_0) == LOW) {}

		TimeDelay();
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_15);

		if (GPIO_ReadInputPin(GPIOD, GPIO_PIN_15) == LOW) {
			++cnt;
			printf("Line = %lu: Hello World :v\n", cnt);
		}
	}

	return 0;
}
