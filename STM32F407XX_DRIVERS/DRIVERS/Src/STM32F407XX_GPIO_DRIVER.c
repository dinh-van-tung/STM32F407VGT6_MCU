#include <stdint.h>
#include <stdio.h>
#include "STM32F407XX_GPIO_DRIVER.h"

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

void GPIO_Init(volatile GPIO_Handle_t *pGPIOx_Handle) {
	uint32_t temp = 0;

	if (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOx_Handle->pGPIOx->MODER &= ~(0b11 << (2 * pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOx_Handle->pGPIOx->MODER |= temp;
	}
	else {
		uint32_t temp_div = pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint32_t temp_mod = pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber % 4;
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp_div] &= ~(0b1111 << (temp_mod * 4));
		SYSCFG->EXTICR[temp_div] |= (GPIO_PORT_CODE(pGPIOx_Handle->pGPIOx) << (temp_mod * 4));

		if (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INTERRUPT_FTRIG) {
			EXTI->RTSR &= ~(0b1 << pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (0b1 << pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INTERRUPT_RTRIG) {
			EXTI->FTSR &= ~(0b1 << pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (0b1 << pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INTERRUPT_RFTRIG) {
			EXTI->RTSR |= (0b1 << pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (0b1 << pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}
		EXTI->IMR |= (0b1 << pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOx_Handle->pGPIOx->OSPEEDR &= ~(0b11 << (2 * pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOx_Handle->pGPIOx->OSPEEDR |= temp;

	temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOx_Handle->pGPIOx->PUPDR &= ~(0b11 << (2 * pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOx_Handle->pGPIOx->PUPDR |= temp;

	temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinOutType << pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOx_Handle->pGPIOx->OTYPER &= ~(0b1 << pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOx_Handle->pGPIOx->OTYPER |= temp;

	if (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC) {
		uint32_t firstBIT = pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber % 8;
		temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * firstBIT));
		if (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber <= GPIO_PIN_7) {
			pGPIOx_Handle->pGPIOx->AFRL &= ~(0b1111 << (4 * firstBIT));
			pGPIOx_Handle->pGPIOx->AFRL |= temp;
		}
		else {
			pGPIOx_Handle->pGPIOx->AFRH &= ~(0b1111 << (4 * firstBIT));
			pGPIOx_Handle->pGPIOx->AFRH |= temp;
		}
	}
}

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

uint8_t GPIO_ReadInputPin(volatile GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0b1);
}

uint16_t GPIO_ReadInputPort(volatile GPIO_RegDef_t *pGPIOx) {
	return (uint16_t)(pGPIOx->IDR);
}

void GPIO_WriteOutputPin(volatile GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if (Value == SET) {
		pGPIOx->ODR |= (0b1 << PinNumber);
	}
	else {
		pGPIOx->ODR &= ~(0b1 << PinNumber);
	}
}

void GPIO_WriteOutputPort(volatile GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(volatile GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (0b1 << PinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI) {
	if (IRQNumber >= 82) return;
	uint8_t temp_div = IRQNumber / 32;
	uint8_t temp_mod = IRQNumber % 32;
	if (ENorDI == ENABLE) {
		NVIC->ISER[temp_div] = (0b1 << temp_mod);
	}
	else {
		NVIC->ICER[temp_div] = (0b1 << temp_mod);
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	if (IRQNumber >= 82) return;
	NVIC->IPR[IRQNumber] = (IRQPriority << 4);
}

void GPIO_IRQHandling(uint8_t PinNumber) {
	EXTI->PR = (1U << PinNumber);
}
