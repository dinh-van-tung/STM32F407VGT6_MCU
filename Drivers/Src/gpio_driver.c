/**
 * gpio_driver.c
 *
 * Created on: Jun 12, 2025
 * Author: Van Tung Dinh
 */

#include <stdint.h>
#include <stdio.h>
#include "gpio_driver.h"


/**
 * Name:                            GPIO_PeriClockControl
 * Last reviewed and updated:       2025/06/20
 * Parameters:                      1) *pGPIOx:		Pointer to a GPIO port
 * 									2) ENorDI:		ENABLE or DISABLE
 * Return type:                     void
 * Brief description:               Enable or disable pulses on the GPIO ports
 */
void GPIO_PeriClockControl(volatile GPIO_RegDef_t *pGPIOx, uint8_t ENorDI){
	if (ENorDI == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	}
	else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}
}


/**
 * Name:                            GPIO_Init
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      1) *pGPIOx_Handle:	Pointer to a GPIO handle structure
 * Return type:                     void
 * Brief description:               Configure the GPIO port registers based on the data in GPIO_Config_t
 */
void GPIO_Init(volatile GPIO_Handle_t *pGPIOx_Handle) {
	/* Enable pulse input on GPIOx port */
	GPIO_PeriClockControl(pGPIOx_Handle->pGPIOx, ENABLE);

	uint32_t temp = 0;

	if (pGPIOx_Handle->GPIO_Config.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		/* Configure the mode of a GPIO pin */
		temp = (pGPIOx_Handle->GPIO_Config.GPIO_PinMode << (2 * pGPIOx_Handle->GPIO_Config.GPIO_PinNumber));
		pGPIOx_Handle->pGPIOx->MODER &= ~(3U << (2 * pGPIOx_Handle->GPIO_Config.GPIO_PinNumber));
		pGPIOx_Handle->pGPIOx->MODER |= temp;
	}
	else {
		/* Configure interrupt mode for GPIO pin */

		/* Enable pulse input on System configuration controller */
		SYSCFG_PCLK_EN();

		/* Configure SYSCFG external interrupt configuration register 1, 2, 3 or 4 */
		uint32_t temp_div = pGPIOx_Handle->GPIO_Config.GPIO_PinNumber / 4;
		uint32_t temp_mod = pGPIOx_Handle->GPIO_Config.GPIO_PinNumber % 4;
		SYSCFG->EXTICR[temp_div] &= ~(15U << (temp_mod * 4));
		SYSCFG->EXTICR[temp_div] |= GPIO_PORT_CODE(pGPIOx_Handle->pGPIOx) << (temp_mod * 4);

		/* Configure detection of rising edge, falling edge, or both */
		if (pGPIOx_Handle->GPIO_Config.GPIO_PinMode == GPIO_MODE_INTERRUPT_FTRIG) {
			EXTI->RTSR &= ~(1U << pGPIOx_Handle->GPIO_Config.GPIO_PinNumber);
			EXTI->FTSR |= (1U << pGPIOx_Handle->GPIO_Config.GPIO_PinNumber);
		}
		else if (pGPIOx_Handle->GPIO_Config.GPIO_PinMode == GPIO_MODE_INTERRUPT_RTRIG) {
			EXTI->FTSR &= ~(1U << pGPIOx_Handle->GPIO_Config.GPIO_PinNumber);
			EXTI->RTSR |= (1U << pGPIOx_Handle->GPIO_Config.GPIO_PinNumber);
		}
		else if (pGPIOx_Handle->GPIO_Config.GPIO_PinMode == GPIO_MODE_INTERRUPT_RFTRIG) {
			EXTI->RTSR |= (1U << pGPIOx_Handle->GPIO_Config.GPIO_PinNumber);
			EXTI->FTSR |= (1U << pGPIOx_Handle->GPIO_Config.GPIO_PinNumber);
		}

		/* Configure Interrupt mask register */
		EXTI->IMR |= (1U << pGPIOx_Handle->GPIO_Config.GPIO_PinNumber);
	}

	/* Configure the output speed of a GPIO pin */
	temp = (pGPIOx_Handle->GPIO_Config.GPIO_PinOutSpeed << (2 * pGPIOx_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIOx_Handle->pGPIOx->OSPEEDR &= ~(3U << (2 * pGPIOx_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIOx_Handle->pGPIOx->OSPEEDR |= temp;

	/* Configure the mode pull-up / pull-down of a GPIO pin */
	temp = (pGPIOx_Handle->GPIO_Config.GPIO_PinPuPdControl << (2 * pGPIOx_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIOx_Handle->pGPIOx->PUPDR &= ~(3U << (2 * pGPIOx_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIOx_Handle->pGPIOx->PUPDR |= temp;

	/* Configure the output type of a GPIO pin */
	temp = (pGPIOx_Handle->GPIO_Config.GPIO_PinOutType << pGPIOx_Handle->GPIO_Config.GPIO_PinNumber);
	pGPIOx_Handle->pGPIOx->OTYPER &= ~(1U << pGPIOx_Handle->GPIO_Config.GPIO_PinNumber);
	pGPIOx_Handle->pGPIOx->OTYPER |= temp;

	/* Configure the alternate function of a GPIO pin */
	if (pGPIOx_Handle->GPIO_Config.GPIO_PinMode == GPIO_MODE_ALTERNATE_FUNC) {
		uint32_t firstBIT = pGPIOx_Handle->GPIO_Config.GPIO_PinNumber % 8;
		temp = (pGPIOx_Handle->GPIO_Config.GPIO_PinAltFuncMode << (4 * firstBIT));
		if (pGPIOx_Handle->GPIO_Config.GPIO_PinNumber <= GPIO_PIN_7) {
			pGPIOx_Handle->pGPIOx->AFRL &= ~(15U << (4 * firstBIT));
			pGPIOx_Handle->pGPIOx->AFRL |= temp;
		}
		else {
			pGPIOx_Handle->pGPIOx->AFRH &= ~(15U << (4 * firstBIT));
			pGPIOx_Handle->pGPIOx->AFRH |= temp;
		}
	}
}


/**
 * Name:                            GPIO_DeInit
 * Last reviewed and updated:       2025/06/20
 * Parameters:                      1) *pGPIOx:		Pointer to a GPIO port
 * Return type:                     void
 * Brief description:               Reset all the registers of the GPIO port
 */
void GPIO_DeInit(volatile GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_RESET();
	}
	else if (pGPIOx == GPIOB) {
		GPIOB_RESET();
	}
	else if (pGPIOx == GPIOC) {
		GPIOC_RESET();
	}
	else if (pGPIOx == GPIOD) {
		GPIOD_RESET();
	}
	else if (pGPIOx == GPIOE) {
		GPIOE_RESET();
	}
	else if (pGPIOx == GPIOF) {
		GPIOF_RESET();
	}
	else if (pGPIOx == GPIOG) {
		GPIOG_RESET();
	}
	else if (pGPIOx == GPIOH) {
		GPIOH_RESET();
	}
	else if (pGPIOx == GPIOI) {
		GPIOI_RESET();
	}
}


/**
 * Name:                            GPIO_ReadInputPin
 * Last reviewed and updated:       2025/06/20
 * Parameters:                      1) *pGPIOx: 	Pointer to a GPIO port
 * 									2) PinNumber: 	GPIO pin (0, 1, 2, ..., 15)
 * Return type:                     uint8_t
 * Brief description:               Read the state of a GPIO input data pin
 */
uint8_t GPIO_ReadInputPin(volatile GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	return (uint8_t)((pGPIOx->IDR >> PinNumber) & 1U);
}


/**
 * Name:                            GPIO_ReadInputPort
 * Last reviewed and updated:       2025/06/20
 * Parameters:                      1) *pGPIOx: 	Pointer to a GPIO port
 * Return type:                     uint16_t
 * Brief description:               Read states to all 16 pins of a GPIO input data register
 */
uint16_t GPIO_ReadInputPort(volatile GPIO_RegDef_t *pGPIOx) {
	return (uint16_t)(pGPIOx->IDR);
}


/**
 * Name:                            GPIO_WriteOutputPin
 * Last reviewed and updated:       2025/06/20
 * Parameters:                      1) *pGPIOx: 	Pointer to a GPIO port
 * 									2) PinNumber: 	GPIO pin (0, 1, 2, ..., 15)
 * 									3) Value: 		LOW or HIGH (SET or RESET)
 * Return type:                     void
 * Brief description:               Write the state to a GPIO output data pin
 */
void GPIO_WriteOutputPin(volatile GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if (Value == SET) {
		pGPIOx->ODR |= (1U << PinNumber);
	}
	else {
		pGPIOx->ODR &= ~(1U << PinNumber);
	}
}


/**
 * Name:                            GPIO_WriteOutputPin
 * Last reviewed and updated:       2025/06/20
 * Parameters:                      1) *pGPIOx: 	Pointer to a GPIO port
 * 									3) Value: 		16 bits
 * Return type:                     void
 * Brief description:               Write states to all 16 pins of a GPIO output data register
 */
void GPIO_WriteOutputPort(volatile GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}


/**
 * Name:                            GPIO_ToggleOutputPin
 * Last reviewed and updated:       2025/06/20
 * Parameters:                      1) *pGPIOx: 	Pointer to a GPIO port
 * 									2) PinNumber: 	GPIO pin (0, 1, 2, ..., 15)
 * Return type:                     void
 * Brief description:               Toggle the state of a GPIO output data pin
 */
void GPIO_ToggleOutputPin(volatile GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1U << PinNumber);
}


/**
 * Name:                            GPIO_IRQInterruptConfig
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      1) IRQNumber:	IRQ numbers for each type of interrupt
 * 									2) ENorDI: 		ENABLE or DISABLE
 * Return type:                     void
 * Brief description:				On the STM32F407VG, there are only 82 IRQ numbers (from 0 to 81).
 * 									Each IRQ number is used to configure the appropriate bit position in one of the two NVIC registers: ISER or ICER
 * 									Write:
 *								 	0 = no effect
 *									1 = changes interrupt state to pending.
 *							 		Read:
 *							 		0 = interrupt is not pending
 *							 		1 = interrupt is pending.
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI) {
	if (IRQNumber > 81) {
		return;
	}
	uint8_t temp_div = IRQNumber / 32;
	uint8_t temp_mod = IRQNumber % 32;
	if (ENorDI == ENABLE) {
		NVIC->ISER[temp_div] = (1U << temp_mod);
	}
	else {
		NVIC->ICER[temp_div] = (1U << temp_mod);
	}
}


/**
 * Name:                            GPIO_IRQPriorityConfig
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      1) IRQNumber:		IRQ numbers for each type of interrupt
 * 									2) IRQPriority:		Priority IRQ number for the type of interrupt (0, 1, 2, ..., 15)
 * Return type:                     void
 * Brief description:				Write the priority level to the NVIC's IPR register
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	if (IRQNumber > 81) {
		return;
	}
	uint8_t temp_div = IRQNumber / 4;
	uint8_t temp_mod = IRQNumber % 4;
	/* Only the upper 4 bits of a byte are used to store the IRQPriority value */
	NVIC->IPR[temp_div] &= ~(15U << (temp_mod * 8 + 4));
	NVIC->IPR[temp_div] |= ((IRQPriority & 15U) << (temp_mod * 8 + 4));
}


/**
 * Name:                            GPIO_IRQHandling
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      1) PinNumber:		GPIO pin (0, 1, 2, ..., 15)
 * Return type:                     void
 * Brief description:				Clear the interrupt flag in the EXTI pending register by writing a 1
 * 									0: No trigger request occurred
 * 									1: selected trigger request occurred
 * 									This bit is set when the selected edge event arrives on the external interrupt line.
 *									This bit is cleared by programming it to "1".
 */
void GPIO_IRQHandling(uint8_t PinNumber) {
	EXTI->PR = (1U << PinNumber);
}
