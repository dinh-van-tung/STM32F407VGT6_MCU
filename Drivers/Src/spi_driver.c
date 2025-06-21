/**
 * spi_driver.c
 *
 * Created on: Jun 16, 2025
 * Author: Van Tung Dinh
 */

#include <stdio.h>
#include <stdint.h>
#include "spi_driver.h"

/**
 * Name:                            SPI_GetFlagStatus
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      1) *pSPIx:		Pointer to a SPI peripheral
 * 									2) FlagName:	A flag of the SPI status register
 * Return type:                     uint8_t
 * Brief description:				Returns the flag status from the SPI status register.
 */
uint8_t SPI_GetFlagStatus(volatile SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	return ((pSPIx->SR & FlagName) ? FLAG_SET : FLAG_RESET);
}


/**
 * Name:                            SPI_PeriClockControl
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      1) *pSPIx:		Pointer to a SPI peripheral
 * 									2) ENorDI:		ENABLE or DISABLE
 * Return type:                     void
 * Brief description:				Enable or disable pulses on the SPI peripheral
 */
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


/**
 * Name:                            SPI_SSIConfig
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      1) *pSPIx:		Pointer to a SPI peripheral
 * 									2) ENorDI:		ENABLE or DISABLE
 * Return type:                     void
 * Brief description:				Enable or disable SSI (Internal slave select) bit of the SPI peripheral
 * 									This bit has an effect only when the SSM bit is set. The value of this bit is forced onto the
 *									NSS pin and the IO value of the NSS pin is ignored.
 */
void SPI_SSIConfig(volatile SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if (ENorDI == ENABLE) {
		pSPIx->CR1 |= (1U << SPI_CR1_SSI);
	}
	else {
		pSPIx->CR1 &= ~(1U << SPI_CR1_SSI);
	}
}


/**
 * Name:                            SPI_SPEConfig
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      1) *pSPIx:		Pointer to a SPI peripheral
 * 									2) ENorDI:		ENABLE or DISABLE
 * Return type:                     void
 * Brief description:				Enable or disable SPE (SPI enable) bit of the SPI peripheral
 * 									0: Peripheral disabled
 *									1: Peripheral enabled
 */
void SPI_SPEConfig(volatile SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if (ENorDI == ENABLE) {
		pSPIx->CR1 |= (1U << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR1 &= ~(1U << SPI_CR1_SPE);
	}
}


/**
 * Name:                            SPI_SSOEConfig
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      1) *pSPIx:		Pointer to a SPI peripheral
 * 									2) ENorDI:		ENABLE or DISABLE
 * Return type:                     void
 * Brief description:				Enable or disable SSOE (SS output enable) bit of the SPI peripheral
 * 									0: SS output is disabled in master mode and the cell can work in multi-master configuration
 *									1: SS output is enabled in master mode and when the cell is enabled. The cell cannot work in a multi-master environment.
 */
void SPI_SSOEConfig(volatile SPI_RegDef_t *pSPIx, uint8_t ENorDI) {
	if (ENorDI == ENABLE) {
		pSPIx->CR2 |= (1U << SPI_CR2_SSOE);
	}
	else {
		pSPIx->CR2 &= ~(1U << SPI_CR2_SSOE);
	}
}


/**
 * Name:                            SPI_Init
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      1) *pSPI_Handle:		Pointer to a SPI handle structure
 * Return type:                     void
 * Brief description:				Configure the SPI registers based on the data in SPI_Config_t
 */
void SPI_Init(volatile SPI_Handle_t *pSPI_Handle) {
	/* Enable pulse input on SPI peripheral */
	SPI_PeriClockControl(pSPI_Handle->pSPIx, ENABLE);

	uint32_t temp_register = 0;

	/* 1. Configure the device mode */
	temp_register |= (pSPI_Handle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	/* 2. Configure the bus config */
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

	/* 3. Configure the SPI serial clock speed (baud rate) */
	temp_register |= (pSPI_Handle->SPI_Config.SPI_SCLKSpeed << SPI_CR1_BR);

	/* 4. Configure the DFF */
	temp_register |= (pSPI_Handle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	/* 5. Configure the CPOL */
	temp_register |= (pSPI_Handle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	/* 6. Configure the CPHA */
	temp_register |= (pSPI_Handle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	/* 7. Configure the SSM */
	temp_register |= (pSPI_Handle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	pSPI_Handle->pSPIx->CR1 = temp_register;
}


/**
 * Name:                            SPI_DeInit
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      1) *pSPIx:		Pointer to a SPI peripheral
 * Return type:                     void
 * Brief description:				Reset all the registers of the SPI peripheral
 */
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

/**
 * Name:                            SPI_SendData
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      1) *pSPIx:			Pointer to a SPI peripheral
 * 									2) pTxBuffer:		The pointer is at the beginning of the data
 * 									3) LengthData:		The length (in bytes) of the data
 * Return type:                     void
 * Brief description:				Send data via the SPI protocol
 */
void SPI_SendData(volatile SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t LengthData) {
	while (LengthData > 0) {
		while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET) {}

		if ((pSPIx->CR1 >> SPI_CR1_DFF) & 1U) {
			// 16 BITS DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			LengthData -= 2;
			pTxBuffer += 2;
		}
		else {
			// 8 BITS DFF
			*((volatile uint8_t*)&pSPIx->DR) = *(pTxBuffer);
			LengthData -= 1;
			pTxBuffer += 1;
		}

		/* Wait until the data has been sent (BSY flag = 0) */
		while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_BSY)) {}
	}
}


//void SPI_ReceiveData(volatile SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t LengthData);
//
//void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
//
//void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
//
//void SPI_IRQHandling(volatile SPI_Handle_t *pSPI_Handle);
