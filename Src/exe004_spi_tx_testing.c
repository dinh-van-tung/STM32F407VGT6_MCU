/*
 * exe004_spi_tx_testing.c
 *
 * Created on: Jun 21, 2025
 * Author: Van Tung Dinh
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"


/**
 * PB12 -> SPI2_NSS
 * PB13	-> SPI2_SCK
 * PB14	-> SPI2_MISO
 * PB15 -> SPI2_MOSI
 */
void SPI2_GPIO_Inits(void) {
	GPIO_Handle_t SPI2_GPIO_Handle;

	SPI2_GPIO_Handle.pGPIOx = GPIOB;
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinMode = GPIO_MODE_ALTFUNC;
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinAltFuncMode = GPIO_MODE_AF_5;
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinOutSpeed = GPIO_OUT_SPEED_VERYHIGH;
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinOutType = GPIO_OUT_TYPE_PUSHPULL;
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	/* NSS */
//	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinNumber = GPIO_PIN_12;
//	GPIO_Init(&SPI2_GPIO_Handle);

	/* SCK */
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPI2_GPIO_Handle);

	/* MISO */
//	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinNumber = GPIO_PIN_14;
//	GPIO_Init(&SPI2_GPIO_Handle);

	/* MOSI */
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPI2_GPIO_Handle);
}


void SPI2_Inits(void) {
	SPI_Handle_t SPI2_Handle;
	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	SPI2_Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPI_Config.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV_2;
	SPI2_Handle.SPI_Config.SPI_DFF = SPI_DFF_8_BITS;
	SPI2_Handle.SPI_Config.SPI_SSM = SPI_SSM_SOFTWARE_EN;
	SPI2_Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;					/* App Logic 2: CPOL = 0 ??? */
	SPI2_Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST_CLK;			/* App Logic 2: CPHA = 1 ??? */
	SPI_Init(&SPI2_Handle);
}


int main(void) {
	SPI2_GPIO_Inits();
	SPI2_Inits();

	GPIO_WriteOutputPin(GPIOB, GPIO_PIN_13, LOW);
	GPIO_WriteOutputPin(GPIOB, GPIO_PIN_15, LOW);

	char user_data[] = "Hello World ?";

	SPI_SSIConfig(SPI2, ENABLE);
	SPI_SPEConfig(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	SPI_SPEConfig(SPI2, DISABLE);

	while(1) {}

	return 0;
}
