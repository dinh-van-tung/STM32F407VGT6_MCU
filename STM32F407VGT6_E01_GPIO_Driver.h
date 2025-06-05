/*
 * STM32F407VGT6_E01_GPIO_Driver.h
 *
 *  Created on: Jun 3, 2025
 *      Author: MSII
 */

#ifndef INC_STM32F407VGT6_E01_GPIO_DRIVER_H_
#define INC_STM32F407VGT6_E01_GPIO_DRIVER_H_

#include <stdint.h>
#include <stdio.h>
#include "STM32F407VGT6_E01.h"


typedef struct {
	uint8_t GPIO_PinNumber; // GPIO pin numbers
	uint8_t GPIO_PinMode; // GPIO pin possible modes
	uint8_t GPIO_PinSpeed; // GPIO pin possible output speeds
	uint8_t GPIO_PinPuPdControl; // GPIO pin pull up/down configuration macros
	uint8_t GPIO_PinOutType; // GPIO pin possible output types
	uint8_t GPIO_PinAltFuncMode; // GPIO pin possible alternate function low/high modes
} GPIO_PinConfigStruct;


/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct {
	GPIO_RegisterStruct *pGPIOx; // This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfigStruct GPIO_PinConfig; // This holds GPIO pin configuration settings

} GPIO_HandleStruct;


/*
 * GPIO pin numbers
 */
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

/*
 * GPIO pin possible modes
 */
#define GPIO_MODE_INT 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALTFUNC 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_INTERRUPT_FTRIG 4
#define GPIO_MODE_INTERRUPT_RTRIG 5
#define GPIO_MODE_INTERRUPT_RFTRIG 6


/*
 * GPIO pin possible output types
 */
#define GPIO_OUTTYPE_PUSHPULL 0
#define GPIO_OUTTYPE_OPENDRAIN 1


/*
 * GPIO pin possible output speeds
 */
#define GPIO_OUTSPEED_LOW 0
#define GPIO_OUTSPEED_MEDIUM 1
#define GPIO_OUTSPEED_HIGH 2
#define GPIO_OUTSPEED_VERYHIGH 3


/*
 * GPIO pin pull up/down configuration macros
 */
#define GPIO_PIN_NO_PUPD 0
#define GPIO__PIN_PU 1
#define GPIO_PIN_PD 2


/*
 * GPIO pin possible alternate function low/high modes
 */
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


/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegisterStruct *pGPIOx, uint8_t ENorDI);


/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_HandleStruct *pGPIO_Hande);
void GPIO_DeInit(GPIO_RegisterStruct *pGPIOx);


/*
 * Read and write
 */
uint8_t GPIO_ReadInputPin(GPIO_RegisterStruct *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegisterStruct *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegisterStruct *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteOutputPort(GPIO_RegisterStruct *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegisterStruct *pGPIOx, uint8_t PinNumber);


/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);
void GPIO_ISRHandling(uint8_t PinNumber);


#endif /* INC_STM32F407VGT6_E01_GPIO_DRIVER_H_ */
