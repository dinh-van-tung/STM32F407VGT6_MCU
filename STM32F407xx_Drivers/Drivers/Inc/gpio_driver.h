/*
 * gpio_driver.h
 * Created on: Jun 12, 2025
 * Author: Van Tung Dinh
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32f407xx.h"

typedef struct {
	volatile uint8_t GPIO_PinNumber;
	volatile uint8_t GPIO_PinMode;
	volatile uint8_t GPIO_PinOutSpeed;
	volatile uint8_t GPIO_PinPuPdControl;
	volatile uint8_t GPIO_PinOutType;
	volatile uint8_t GPIO_PinAltFuncMode;
} GPIO_Config_t;

typedef struct {
	volatile GPIO_RegDef_t *pGPIOx;
	volatile GPIO_Config_t GPIO_Config;
} GPIO_Handle_t;

#define GPIO_PIN_0 0
#define GPIO_PIN_1 1
#define GPIO_PIN_2 2
#define GPIO_PIN_3 3
#define GPIO_PIN_4 4
#define GPIO_PIN_5 5
#define GPIO_PIN_6 6
#define GPIO_PIN_7 7
#define GPIO_PIN_8 8
#define GPIO_PIN_9 9
#define GPIO_PIN_10 10
#define GPIO_PIN_11 11
#define GPIO_PIN_12 12
#define GPIO_PIN_13 13
#define GPIO_PIN_14 14
#define GPIO_PIN_15 15

#define GPIO_MODE_INT 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALTFUNC 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_INTERRUPT_FTRIG 4
#define GPIO_MODE_INTERRUPT_RTRIG 5
#define GPIO_MODE_INTERRUPT_RFTRIG 6

#define GPIO_OUT_TYPE_PUSHPULL 0
#define GPIO_OUT_TYPE_OPENDRAIN 1

#define GPIO_OUT_SPEED_LOW 0
#define GPIO_OUT_SPEED_MEDIUM 1
#define GPIO_OUT_SPEED_HIGH 2
#define GPIO_OUT_SPEED_VERYHIGH 3

#define GPIO_NOPULL 0
#define GPIO_PULLUP 1
#define GPIO_PULLDOWN 2

#define GPIO_MODE_AF_0 0
#define GPIO_MODE_AF_1 1
#define GPIO_MODE_AF_2 2
#define GPIO_MODE_AF_3 3
#define GPIO_MODE_AF_4 4
#define GPIO_MODE_AF_5 5
#define GPIO_MODE_AF_6 6
#define GPIO_MODE_AF_7 7
#define GPIO_MODE_AF_8 8
#define GPIO_MODE_AF_9 9
#define GPIO_MODE_AF_10 10
#define GPIO_MODE_AF_11 11
#define GPIO_MODE_AF_12 12
#define GPIO_MODE_AF_13 13
#define GPIO_MODE_AF_14 14
#define GPIO_MODE_AF_15 15

void GPIO_PeriClockControl(volatile GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);

void GPIO_Init(volatile GPIO_Handle_t *pGPIO_Handle);

void GPIO_DeInit(volatile GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadInputPin(volatile GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

uint16_t GPIO_ReadInputPort(volatile GPIO_RegDef_t *pGPIOx);

void GPIO_WriteOutputPin(volatile GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value);

void GPIO_WriteOutputPort(volatile GPIO_RegDef_t *pGPIOx, uint16_t Value);

void GPIO_ToggleOutputPin(volatile GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_GPIO_DRIVER_H_ */
