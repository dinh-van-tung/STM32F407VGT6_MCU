/*
 * spi_driver.h
 * Created on: Jun 16, 2025
 * Author: Van Tung Dinh
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32f407xx.h"

typedef struct {
	volatile uint8_t SPI_DeviceMode;
	volatile uint8_t SPI_BusConfig;
	volatile uint8_t SPI_DFF;
	volatile uint8_t SPI_CPHA;
	volatile uint8_t SPI_CPOL;
	volatile uint8_t SPI_SSM;
	volatile uint8_t SPI_SCLKSpeed;
} SPI_Config_t;

typedef struct {
	volatile SPI_RegDef_t *pSPIx;
	volatile SPI_Config_t SPI_Config;
} SPI_Handle_t;

#define SPI_DEVICE_MODE_SLAVE 0
#define SPI_DEVICE_MODE_MASTER 1

#define SPI_BUS_CONFIG_FULL_DUPLEX 0
#define SPI_BUS_CONFIG_HALF_DUPLEX 1
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 2

#define SPI_SCLK_SPEED_DIV_2 0
#define SPI_SCLK_SPEED_DIV_4 1
#define SPI_SCLK_SPEED_DIV_8 2
#define SPI_SCLK_SPEED_DIV_16 3
#define SPI_SCLK_SPEED_DIV_32 4
#define SPI_SCLK_SPEED_DIV_64 5
#define SPI_SCLK_SPEED_DIV_128 6
#define SPI_SCLK_SPEED_DIV_256 7

#define SPI_DFF_8_BITS 0
#define SPI_DFF_16_BITS 1

#define SPI_CPOL_LOW 0
#define SPI_CPOL_HIGH 1

#define SPI_CPHA_FIRST_CLK 0
#define SPI_CPHA_SECOND_CLK 1

#define SPI_SSM_SOFTWARE_DI 0
#define SPI_SSM_SOFTWARE_EN 1

#define SPI_FLAG_TXE (1U << SPI_SR_TXE)
#define SPI_FLAG_RXNE (1U << SPI_SR_RXNE)
#define SPI_FLAG_BSY (1U << SPI_SR_BSY)

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
