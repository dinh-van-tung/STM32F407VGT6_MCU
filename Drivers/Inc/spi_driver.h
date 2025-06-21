/**
 * spi_driver.h
 *
 * Created on: Jun 16, 2025
 * Author: Van Tung Dinh
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32f407xx.h"


/**
 * Name:                            SPI configuration structure
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               None
 */
typedef struct {
	volatile uint8_t SPI_DeviceMode;
	volatile uint8_t SPI_BusConfig;
	volatile uint8_t SPI_DFF;
	volatile uint8_t SPI_CPHA;
	volatile uint8_t SPI_CPOL;
	volatile uint8_t SPI_SSM;
	volatile uint8_t SPI_SCLKSpeed;
} SPI_Config_t;


/**
 * Name:                            SPI handle structure
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               None
 */
typedef struct {
	volatile SPI_RegDef_t *pSPIx;
	volatile SPI_Config_t SPI_Config;
} SPI_Handle_t;


/**
 * Name:                            SPI device mode
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               None
 */
#define SPI_DEVICE_MODE_SLAVE 		0
#define SPI_DEVICE_MODE_MASTER 		1


/**
 * Name:                            SPI bus configuration
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               None
 */
#define SPI_BUS_CONFIG_FULL_DUPLEX 			0
#define SPI_BUS_CONFIG_HALF_DUPLEX 			1
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 		2


/**
 * Name:                            SPI serial clock speed
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               None
 */
#define SPI_SCLK_SPEED_DIV_2 		0
#define SPI_SCLK_SPEED_DIV_4 		1
#define SPI_SCLK_SPEED_DIV_8 		2
#define SPI_SCLK_SPEED_DIV_16 		3
#define SPI_SCLK_SPEED_DIV_32 		4
#define SPI_SCLK_SPEED_DIV_64 		5
#define SPI_SCLK_SPEED_DIV_128 		6
#define SPI_SCLK_SPEED_DIV_256 		7


/**
 * Name:                            SPI data frame format
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               None
 */
#define SPI_DFF_8_BITS 		0
#define SPI_DFF_16_BITS 	1


/**
 * Name:                            SPI clock polarity
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               0: CK to 0 when idle
 *									1: CK to 1 when idle
 */
#define SPI_CPOL_LOW 		0
#define SPI_CPOL_HIGH 		1


/**
 * Name:                            SPI clock phase
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               0: The first clock transition is the first data capture edge
 *									1: The second clock transition is the first data capture edge
 */
#define SPI_CPHA_FIRST_CLK 			0
#define SPI_CPHA_SECOND_CLK 		1


/**
 * Name:                            SPI software slave management
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               When the SSM bit is set, the NSS pin input is replaced with the value from the SSI bit.
 *									0: Software slave management disabled
 * 									1: Software slave management enabled
 */
#define SPI_SSM_SOFTWARE_DI 		0
#define SPI_SSM_SOFTWARE_EN 		1


/**
 * Name:                            SPI flag
 * Last reviewed and updated:       2025/06/21
 * Parameters:                      None
 * Return type:                     None
 * Brief description:               These flags belong to the SPI status register (SPIx_SR, x = 1, 2, 3, ...)
 */
#define SPI_FLAG_TXE 		(1U << SPI_SR_TXE)
#define SPI_FLAG_RXNE 		(1U << SPI_SR_RXNE)
#define SPI_FLAG_BSY 		(1U << SPI_SR_BSY)


uint8_t SPI_GetFlagStatus(volatile SPI_RegDef_t *pSPIx, uint32_t FlagName);

void SPI_PeriClockControl(volatile SPI_RegDef_t *pSPIx, uint8_t ENorDI);

void SPI_SPEConfig(volatile SPI_RegDef_t *pSPIx, uint8_t ENorDI);

void SPI_SSIConfig(volatile SPI_RegDef_t *pSPIx, uint8_t ENorDI);

void SPI_SSOEConfig(volatile SPI_RegDef_t *pSPIx, uint8_t ENorDI);

void SPI_Init(volatile SPI_Handle_t *pSPI_Handle);

void SPI_DeInit(volatile SPI_RegDef_t *pSPIx);

void SPI_SendData(volatile SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t LengthData);

void SPI_ReceiveData(volatile SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t LengthData);

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

void SPI_IRQHandling(volatile SPI_Handle_t *pSPI_Handle);

#endif /* INC_SPI_DRIVER_H_ */
