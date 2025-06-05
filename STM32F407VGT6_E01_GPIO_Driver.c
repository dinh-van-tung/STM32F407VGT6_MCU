/*
 * STM32F407VGT6_E01_GPIO_Driver.c
 *
 *  Created on: Jun 3, 2025
 *      Author: MSII
 */

#include <stdint.h>
#include <stdio.h>
#include "STM32F407VGT6_E01_GPIO_Driver.h"


/*
 * Peripheral clock setup
 */
/*************************************************************************
 * @fn: GPIO_PeriClockControl
 *
 * @brife: This function enable or disable peripheral clock for the given GPIO port
 *
 * @param[in]: Base address of the GPIO peripheral
 * @param[in]: Enable or Disable macros
 * @param[in]:
 *
 * @return: none
 *
 * @note: none
 *************************************************************************/
void GPIO_PeriClockControl(GPIO_RegisterStruct *pGPIOx_Regis, uint8_t ENorDI){
	if (ENorDI == ENABLE) {
		if (pGPIOx_Regis == GPIOA_Regis) {
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx_Regis == GPIOB_Regis) {
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx_Regis == GPIOC_Regis) {
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx_Regis == GPIOD_Regis) {
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx_Regis == GPIOE_Regis) {
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx_Regis == GPIOF_Regis) {
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx_Regis == GPIOG_Regis) {
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx_Regis == GPIOH_Regis) {
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx_Regis == GPIOI_Regis) {
			GPIOI_PCLK_EN();
		}
	}
	else {
		if (pGPIOx_Regis == GPIOA_Regis) {
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx_Regis == GPIOB_Regis) {
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx_Regis == GPIOC_Regis) {
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx_Regis == GPIOD_Regis) {
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx_Regis == GPIOE_Regis) {
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx_Regis == GPIOF_Regis) {
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx_Regis == GPIOG_Regis) {
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx_Regis == GPIOH_Regis) {
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx_Regis == GPIOI_Regis) {
			GPIOI_PCLK_DI();
		}
	}
}


/*
 * Init and De-Init
 */
/*************************************************************************
 * @fn: GPIO_Init
 *
 * @brife: This function copies data from GPIOx configuration pin to GPIOx register
 *
 * @param[in]: GPIOx handle (GPIOx register, GPIOx configuration pin)
 *
 * @return: none
 *
 * @note: none
 *************************************************************************/
void GPIO_Init(GPIO_HandleStruct *pGPIO_Handle) {
	uint32_t temp = 0; // temp register

	// 1. configure the mode of GPIO pin
	if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// The non interrupt mode
		temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER &= ~(0b11 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER |= temp;
	}
	else {
		// This part will code later (Interrupt mode)
	}

	// 2. configure the speed
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~(0b11 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDR |= temp;

	// 3. configure the pupd settings
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR &= ~(0b11 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR |= temp;

	// 4. configure the output type
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinOutType << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OTYPER &= ~(0b1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OTYPER |= temp;

	// 5. configure the alternate function low/high modes
	if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC) {
		uint32_t firstBIT = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 8;
		temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * firstBIT));
		if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber <= GPIO_PIN_7) {
			pGPIO_Handle->pGPIOx->AFRL &= ~(0b1111 << (4 * firstBIT));
			pGPIO_Handle->pGPIOx->AFRL |= temp;
		}
		else {
			pGPIO_Handle->pGPIOx->AFRH &= ~(0b1111 << (4 * firstBIT));
			pGPIO_Handle->pGPIOx->AFRH |= temp;
		}
	}
}

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
