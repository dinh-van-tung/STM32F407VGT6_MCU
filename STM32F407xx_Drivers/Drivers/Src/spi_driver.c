/*
 * spi_driver.c
 *
 *  Created on: Jun 16, 2025
 *      Author: Van Tung Dinh
 */

#include <stdio.h>
#include <stdint.h>
#include "spi_driver.h"

uint8_t SPI_GetFlagStatus(volatile SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_PeriClockControl(volatile SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if (ENorDI == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	}
	else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}

void SPI_SSIConfig(volatile SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if (ENorDI == ENABLE) {
		pSPIx->CR1 |= (1U << SPI_CR1_SSI);
	}
	else {
		pSPIx->CR1 &= ~(1U << SPI_CR1_SSI);
	}
}

void SPI_SPEConfig(volatile SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if (ENorDI == ENABLE) {
		pSPIx->CR1 |= (1U << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR1 &= ~(1U << SPI_CR1_SPE);
	}
}

void SPI_SSOEConfig(volatile SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if (ENorDI == ENABLE) {
		pSPIx->CR2 |= (1U << SPI_CR2_SSOE);
	}
	else {
		pSPIx->CR2 &= ~(1U << SPI_CR2_SSOE);
	}
}

void SPI_Init(volatile SPI_Handle_t *pSPI_Handle) {
	SPI_PeriClockControl(pSPI_Handle->pSPIx, ENABLE);
	uint32_t temp_register = 0;

	// 1. Configure the device mode
	temp_register |= (pSPI_Handle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	// 2. Configure the bus config
	if (pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX) {
		temp_register &= ~(1U << SPI_CR1_BIDIMODE);
	}
	else if (pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX) {
		temp_register |= (1U << SPI_CR1_BIDIMODE);
	}
	else if (pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		temp_register &= ~(1U << SPI_CR1_BIDIMODE);
		temp_register |= (1U << SPI_CR1_RXONLY);
	}

	// 3. Configure the SPI serial clock speed (baud rate)
	temp_register |= (pSPI_Handle->SPI_Config.SPI_SCLKSpeed << SPI_CR1_BR);

	// 4. Configure the DFF
	temp_register |= (pSPI_Handle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	// 5. Configure the CPOL
	temp_register |= (pSPI_Handle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	// 6. Configure the CPHA
	temp_register |= (pSPI_Handle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	// 7. Configure the SSM
	temp_register |= (pSPI_Handle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	pSPI_Handle->pSPIx->CR1 = temp_register;
}

void SPI_DeInit(volatile SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_RESET();
	}
	else if (pSPIx == SPI2) {
		SPI2_RESET();
	}
	else if (pSPIx == SPI3) {
		SPI3_RESET();
	}
}

void SPI_SendData(volatile SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t LengthData) {
	while (LengthData > 0) {
		while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET) {}

		if ((pSPIx->CR1 >> SPI_CR1_DFF) & 1U) {
			// 16 BITS DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			LengthData -= 2;
			pTxBuffer++;
			pTxBuffer++;
		}
		else {
			// 8 BITS DFF
			*((volatile uint8_t*)&pSPIx->DR) = *(pTxBuffer);
			LengthData -= 1;
			pTxBuffer++;
		}

		while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_BSY)) {}
	}
}

void SPI_ReceiveData(volatile SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t LengthData);

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

void SPI_IRQHandling(volatile SPI_Handle_t *pSPI_Handle);
