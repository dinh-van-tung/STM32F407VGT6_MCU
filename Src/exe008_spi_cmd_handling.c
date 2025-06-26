/**
 * File: exe008_spi_cmd_handling.c
 *
 * Last reviewed and updated: 2025/06/26
 * Author: VanTungDinh
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"


/* Command codes (Slave recognizes) */
#define COMMAND_LED_CTRL			0x50
#define COMMAND_SENSOR_READ			0x51
#define COMMAND_LED_READ			0x52
#define COMMAND_PRINT				0x53
#define COMMAND_ID_READ				0x54

#define LED_ON			1
#define LED_OFF			0


/* Arduino analog pin */
#define ANALOG_PIN_0		0
#define ANALOG_PIN_1		1
#define ANALOG_PIN_2		2
#define ANALOG_PIN_3		3
#define ANALOG_PIN_4		4


/* Arduiono led */
#define	LED_PIN			9


void TimeDelay(uint8_t Div_x) {
	for (uint32_t i = 0; i <= 500000 / Div_x; i += 1) {}
}


/**
 * PB12 -> SPI2_NSS
 * PB13	-> SPI2_SCK
 * PB14	-> SPI2_MISO
 * PB15 -> SPI2_MOSI
 */
void SPI2_GPIO_Inits(void) {
	GPIO_Handle_t SPI2_GPIO_Handle, Button;

	Button.pGPIOx = GPIOA;
	Button.GPIO_Config.GPIO_PinMode = GPIO_MODE_INPUT;
	Button.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;
	Button.GPIO_Config.GPIO_PinNumber = GPIO_PIN_0;

	GPIO_Init(&Button);

	SPI2_GPIO_Handle.pGPIOx = GPIOB;
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinMode = GPIO_MODE_ALTERNATE_FUNC;
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinAltFuncMode = GPIO_MODE_AF_5;
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinOutSpeed = GPIO_OUT_SPEED_VERYHIGH;
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinOutType = GPIO_OUT_TYPE_PUSHPULL;
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	/* NSS */
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPI2_GPIO_Handle);

	/* SCK */
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPI2_GPIO_Handle);

	/* MISO */
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPI2_GPIO_Handle);

	/* MOSI */
	SPI2_GPIO_Handle.GPIO_Config.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPI2_GPIO_Handle);
}


void SPI2_Inits(void) {
	SPI_Handle_t SPI2_Handle;
	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	SPI2_Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPI_Config.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV_32;
	SPI2_Handle.SPI_Config.SPI_DFF = SPI_DFF_8_BITS;
	SPI2_Handle.SPI_Config.SPI_SSM = SPI_SSM_SOFTWARE_DI;
	SPI2_Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST_CLK;
	SPI_Init(&SPI2_Handle);
}


uint8_t SPI_VerifyResponse(uint8_t ack_byte) {
	return ack_byte == 0xF5 ? 1 : 0;
}


int main(void) {
	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read;

	SPI2_GPIO_Inits();
	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	while (1) {
		while (GPIO_ReadInputPin(GPIOA, GPIO_PIN_0) == LOW) {}
		TimeDelay(2);

		SPI_SPEConfig(SPI2, ENABLE);

		/* 1. COMMAND_LED_CTRL		<pin number(1)>		<value(1)> */
		uint8_t cmdcode = COMMAND_LED_CTRL;
		SPI_SendData(SPI2, &cmdcode, 1);

		/* Do dummy read to clear of the RXNE */
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		/* Send some dummy bits (byte) to fetch the response from the slave */
		SPI_SendData(SPI2, &dummy_write, 1);

		/* Read the ACK byte received */
		uint8_t ack_byte;
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		uint8_t array[2];
		if (SPI_VerifyResponse(ack_byte)) {
			/* Send arguments */
			array[0] = LED_PIN;
			array[1] = LED_OFF;
			SPI_SendData(SPI2, array, 2);
		}

		SPI_SPEConfig(SPI2, DISABLE);
	}

	return 0;
}
