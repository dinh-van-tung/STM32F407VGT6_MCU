/*
 * STM32F407VGT6_E01.h
 *
 *  Created on: Jun 3, 2025
 *      Author: MSII
 */


#ifndef INC_STM32F407VGT6_E01_H_
#define INC_STM32F407VGT6_E01_H_

#include <stdint.h>
#include <stdio.h>


/*
 * Base address of FLASH, SRAM, ROM
 */
#define FLASH_BASE 0x08000000UL
#define SRAM1_BASE 0x20000000UL
#define SRAM1_SIZE (112 * 1024) // 112KB * 1024 = ... Byte
#define SRAM2_BASE (SRAM1_BASE + SRAM1_SIZE)
#define SRAM_BASE SRAM1_BASE
#define ROM_BASE 0x1FFF0000UL // System memory


/*
 * Base address of peripherals
 */
#define PERIPHERAL_BASE 0x40000000UL
#define APB1_BASE PERIPHERAL_BASE
#define APB2_BASE 0x40010000UL
#define AHB1_BASE 0x40020000UL
#define AHB2_BASE 0x50000000UL
#define AHB3_BASE 0xA0000000UL


/*
 * Base address of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASE (AHB1_BASE + 0x0000)
#define GPIOB_BASE (AHB1_BASE + 0x0400)
#define GPIOC_BASE (AHB1_BASE + 0x0800)
#define GPIOD_BASE (AHB1_BASE + 0x0C00)
#define GPIOE_BASE (AHB1_BASE + 0x1000)
#define GPIOF_BASE (AHB1_BASE + 0x1400)
#define GPIOG_BASE (AHB1_BASE + 0x1800)
#define GPIOH_BASE (AHB1_BASE + 0x1C00)
#define GPIOI_BASE (AHB1_BASE + 0x2000)
#define RCC_BASE (AHB1_BASE + 0x3800)


/*
 * Base address of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASE (APB1_BASE + 0x5400)
#define I2C2_BASE (APB1_BASE + 0x5800)
#define I2C3_BASE (APB1_BASE + 0x5C00)
#define SPI2_BASE (APB1_BASE + 0x3800)
#define SPI3_BASE (APB1_BASE + 0x3C00)
#define USART2_BASE (APB1_BASE + 0x4400)
#define USART3_BASE (APB1_BASE + 0x4800)
#define UART4_BASE (APB1_BASE + 0x4C00)
#define UART5_BASE (APB1_BASE + 0x5000)


/*
 * Base address of peripherals which are hanging on APB2 bus
 */
#define SPI1_BASE (APB2_BASE + 0x3000)
#define USART1_BASE (APB2_BASE + 0x1000)
#define USART6_BASE (APB2_BASE + 0x1400)
#define EXTI_BASE (APB2_BASE + 0x3C00)
#define SYSCFG_BASE (APB2_BASE + 0x3800)


/*
 * GPIO ports Structure
 */
typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
} GPIO_RegisterStruct;

#define GPIOA_Regis ((GPIO_RegisterStruct*)GPIOA_BASE)
#define GPIOB_Regis ((GPIO_RegisterStruct*)GPIOB_BASE)
#define GPIOC_Regis ((GPIO_RegisterStruct*)GPIOC_BASE)
#define GPIOD_Regis ((GPIO_RegisterStruct*)GPIOD_BASE)
#define GPIOE_Regis ((GPIO_RegisterStruct*)GPIOE_BASE)
#define GPIOF_Regis ((GPIO_RegisterStruct*)GPIOF_BASE)
#define GPIOG_Regis ((GPIO_RegisterStruct*)GPIOG_BASE)
#define GPIOH_Regis ((GPIO_RegisterStruct*)GPIOH_BASE)
#define GPIOI_Regis ((GPIO_RegisterStruct*)GPIOI_BASE)


/*
 * RCC Structure
 */
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
} RCC_RegisterStruct;

#define RCC_Regis ((RCC_RegisterStruct*)RCC_BASE)


/*
 * Clock enable and disable macros for GPIOx peripherals
 */
// enable
#define GPIOA_PCLK_EN() (RCC_Regis->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC_Regis->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC_Regis->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC_Regis->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC_Regis->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC_Regis->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC_Regis->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC_Regis->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC_Regis->AHB1ENR |= (1 << 8))
// disable
#define GPIOA_PCLK_DI() (RCC_Regis->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC_Regis->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC_Regis->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC_Regis->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC_Regis->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC_Regis->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC_Regis->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC_Regis->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() (RCC_Regis->AHB1ENR &= ~(1 << 8))

/*
 * Clock enable and disable macros for I2Cx peripherals
 */
// enable
#define I2C1_PCLK_EN() (RCC_Regis->APB1ENR |= (1 << 21))
//disable
#define I2C1_PCLK_DI() (RCC_Regis->APB1ENR &= ~(1 << 21))


/*
 * Clock enable and disable macros for SPIx peripherals
 */
// enable
#define SPI1_PCLK_EN() (RCC_Regis->APB2ENR |= (1 << 12))
// disable
#define SPI1_PCLK_DI() (RCC_Regis->APB2ENR &= ~(1 << 12))

/*
 * Clock enable and disable macros for USARTx peripherals
 */
// enable
#define USART1_PCLK_EN() (RCC_Regis->APB2ENR |= (1 << 4))
// disable
#define USART1_PCLK_DI() (RCC_Regis->APB2ENR &= ~(1 << 4))


/*
 * Clock enable and disable macros for SYSCFG peripherals
 */
// enable
#define SYSCFG_PCLK_EN() (RCC_Regis->APB2ENR |= (1 << 14))
// disable
#define SYSCFG_PCLK_DI() (RCC_Regis->APB2ENR &= ~(1 << 14))


/*
 * Some generic macros
 */
#define ENABLE 1
#define DISABLE 0
#define SET 1
#define RESET 0
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0


#endif /* INC_STM32F407VGT6_E01_H_ */
