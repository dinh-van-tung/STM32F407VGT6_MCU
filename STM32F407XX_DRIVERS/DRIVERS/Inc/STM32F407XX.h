#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#include <stdint.h>
#include <stdio.h>


/* ----------------------------- BASE ADDRESSES ----------------------------- */
#define NVIC_BASE        0xE000E100UL
#define FLASH_BASE       0x08000000UL
#define SRAM1_BASE       0x20000000UL
#define SRAM1_SIZE       (112 * 1024)
#define SRAM2_BASE       (SRAM1_BASE + SRAM1_SIZE)
#define SRAM_BASE        SRAM1_BASE
#define ROM_BASE         0x1FFF0000UL
#define PERIPHERAL_BASE  0x40000000UL
#define APB1_BASE        PERIPHERAL_BASE
#define APB2_BASE        0x40010000UL
#define AHB1_BASE        0x40020000UL
#define AHB2_BASE        0x50000000UL
#define AHB3_BASE        0xA0000000UL


/* ----------------------------- AHB1 PERIPHERALS ----------------------------- */
#define GPIOA_BASE       (AHB1_BASE + 0x0000)
#define GPIOB_BASE       (AHB1_BASE + 0x0400)
#define GPIOC_BASE       (AHB1_BASE + 0x0800)
#define GPIOD_BASE       (AHB1_BASE + 0x0C00)
#define GPIOE_BASE       (AHB1_BASE + 0x1000)
#define GPIOF_BASE       (AHB1_BASE + 0x1400)
#define GPIOG_BASE       (AHB1_BASE + 0x1800)
#define GPIOH_BASE       (AHB1_BASE + 0x1C00)
#define GPIOI_BASE       (AHB1_BASE + 0x2000)
#define RCC_BASE         (AHB1_BASE + 0x3800)


/* ----------------------------- APB1 PERIPHERALS ----------------------------- */
#define I2C1_BASE        (APB1_BASE + 0x5400)
#define I2C2_BASE        (APB1_BASE + 0x5800)
#define I2C3_BASE        (APB1_BASE + 0x5C00)
#define SPI2_BASE        (APB1_BASE + 0x3800)
#define SPI3_BASE        (APB1_BASE + 0x3C00)
#define USART2_BASE      (APB1_BASE + 0x4400)
#define USART3_BASE      (APB1_BASE + 0x4800)
#define UART4_BASE       (APB1_BASE + 0x4C00)
#define UART5_BASE       (APB1_BASE + 0x5000)


/* ----------------------------- APB2 PERIPHERALS ----------------------------- */
#define SPI1_BASE        (APB2_BASE + 0x3000)
#define USART1_BASE      (APB2_BASE + 0x1000)
#define USART6_BASE      (APB2_BASE + 0x1400)
#define EXTI_BASE        (APB2_BASE + 0x3C00)
#define SYSCFG_BASE      (APB2_BASE + 0x3800)


/* ----------------------------- NVIC Register Structure ----------------------------- */
typedef struct {
	volatile uint32_t ISER[8];        /*!< Interrupt Set Enable Registers               - Offset: 0x000–0x01C */
	volatile uint32_t RESERVED0[24];  /*!< Reserved                                     - Offset: 0x020–0x07C */
	volatile uint32_t ICER[8];        /*!< Interrupt Clear Enable Registers             - Offset: 0x080–0x09C */
	volatile uint32_t RESERVED1[24];  /*!< Reserved                                     - Offset: 0x0A0–0x0FC */
	volatile uint32_t ISPR[8];        /*!< Interrupt Set Pending Registers              - Offset: 0x100–0x11C */
	volatile uint32_t RESERVED2[24];  /*!< Reserved                                     - Offset: 0x120–0x17C */
	volatile uint32_t ICPR[8];        /*!< Interrupt Clear Pending Registers            - Offset: 0x180–0x19C */
	volatile uint32_t RESERVED3[24];  /*!< Reserved                                     - Offset: 0x1A0–0x1FC */
	volatile uint32_t IABR[8];        /*!< Interrupt Active Bit Registers               - Offset: 0x200–0x21C */
	volatile uint32_t RESERVED4[56];  /*!< Reserved                                     - Offset: 0x220–0x2FC */
	volatile uint8_t  IPR[240];       /*!< Interrupt Priority Registers (IPR0–IPR59)    - Offset: 0x300–0x3EF */
	volatile uint32_t RESERVED5[644]; /*!< Reserved                                     - Offset: 0x3F0–0xEFF */
	volatile uint32_t STIR;           /*!< Software Trigger Interrupt Register          - Offset: 0xF00 */
} NVIC_RegDef_t;
#define NVIC ((volatile NVIC_RegDef_t*)NVIC_BASE)


/* ----------------------------- GPIO Register Structure ----------------------------- */
typedef struct {
	volatile uint32_t MODER;   /*!< GPIO port mode register                       - Offset: 0x00 */
	volatile uint32_t OTYPER;  /*!< GPIO port output type register                - Offset: 0x04 */
	volatile uint32_t OSPEEDR; /*!< GPIO port output speed register               - Offset: 0x08 */
	volatile uint32_t PUPDR;   /*!< GPIO port pull-up/pull-down register          - Offset: 0x0C */
	volatile uint32_t IDR;     /*!< GPIO port input data register                 - Offset: 0x10 */
	volatile uint32_t ODR;     /*!< GPIO port output data register                - Offset: 0x14 */
	volatile uint32_t BSRR;    /*!< GPIO port bit set/reset register              - Offset: 0x18 */
	volatile uint32_t LCKR;    /*!< GPIO port configuration lock register         - Offset: 0x1C */
	volatile uint32_t AFRL;    /*!< GPIO alternate function low register (0–7)    - Offset: 0x20 */
	volatile uint32_t AFRH;    /*!< GPIO alternate function high register (8–15)  - Offset: 0x24 */
} GPIO_RegDef_t;
#define GPIOA ((volatile GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB ((volatile GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC ((volatile GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD ((volatile GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE ((volatile GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOF ((volatile GPIO_RegDef_t*)GPIOF_BASE)
#define GPIOG ((volatile GPIO_RegDef_t*)GPIOG_BASE)
#define GPIOH ((volatile GPIO_RegDef_t*)GPIOH_BASE)
#define GPIOI ((volatile GPIO_RegDef_t*)GPIOI_BASE)


/* ----------------------------- RCC Register Structure ----------------------------- */
typedef struct {
	volatile uint32_t CR;           /*!< Clock control register                        - Offset: 0x00 */
	volatile uint32_t PLLCFGR;      /*!< PLL configuration register                    - Offset: 0x04 */
	volatile uint32_t CFGR;         /*!< Clock configuration register                  - Offset: 0x08 */
	volatile uint32_t CIR;          /*!< Clock interrupt register                      - Offset: 0x0C */
	volatile uint32_t AHB1RSTR;     /*!< AHB1 peripheral reset register                - Offset: 0x10 */
	volatile uint32_t AHB2RSTR;     /*!< AHB2 peripheral reset register                - Offset: 0x14 */
	volatile uint32_t AHB3RSTR;     /*!< AHB3 peripheral reset register                - Offset: 0x18 */
	uint32_t RESERVED0;             /*!< Reserved                                      - Offset: 0x1C */
	volatile uint32_t APB1RSTR;     /*!< APB1 peripheral reset register                - Offset: 0x20 */
	volatile uint32_t APB2RSTR;     /*!< APB2 peripheral reset register                - Offset: 0x24 */
	uint32_t RESERVED1[2];          /*!< Reserved                                      - Offset: 0x28–0x2C */
	volatile uint32_t AHB1ENR;      /*!< AHB1 peripheral clock enable register         - Offset: 0x30 */
	volatile uint32_t AHB2ENR;      /*!< AHB2 peripheral clock enable register         - Offset: 0x34 */
	volatile uint32_t AHB3ENR;      /*!< AHB3 peripheral clock enable register         - Offset: 0x38 */
	uint32_t RESERVED2;             /*!< Reserved                                      - Offset: 0x3C */
	volatile uint32_t APB1ENR;      /*!< APB1 peripheral clock enable register         - Offset: 0x40 */
	volatile uint32_t APB2ENR;      /*!< APB2 peripheral clock enable register         - Offset: 0x44 */
	uint32_t RESERVED3[2];          /*!< Reserved                                      - Offset: 0x48–0x4C */
	volatile uint32_t AHB1LPENR;    /*!< AHB1 low power enable register                - Offset: 0x50 */
	volatile uint32_t AHB2LPENR;    /*!< AHB2 low power enable register                - Offset: 0x54 */
	volatile uint32_t AHB3LPENR;    /*!< AHB3 low power enable register                - Offset: 0x58 */
	uint32_t RESERVED4;             /*!< Reserved                                      - Offset: 0x5C */
	volatile uint32_t APB1LPENR;    /*!< APB1 low power enable register                - Offset: 0x60 */
	volatile uint32_t APB2LPENR;    /*!< APB2 low power enable register                - Offset: 0x64 */
	uint32_t RESERVED5[2];          /*!< Reserved                                      - Offset: 0x68–0x6C */
	volatile uint32_t BDCR;         /*!< Backup domain control register                - Offset: 0x70 */
	volatile uint32_t CSR;          /*!< Clock control & status register               - Offset: 0x74 */
	uint32_t RESERVED6[2];          /*!< Reserved                                      - Offset: 0x78–0x7C */
	volatile uint32_t SSCGR;        /*!< Spread spectrum clock generation register     - Offset: 0x80 */
	volatile uint32_t PLLI2SCFGR;   /*!< PLLI2S configuration register                 - Offset: 0x84 */
} RCC_RegDef_t;
#define RCC ((volatile RCC_RegDef_t*)RCC_BASE)


/* ----------------------------- EXTI & SYSCFG Structures ----------------------------- */
typedef struct {
	volatile uint32_t IMR;     /*!< Interrupt mask register                       - Offset: 0x00 */
	volatile uint32_t EMR;     /*!< Event mask register                           - Offset: 0x04 */
	volatile uint32_t RTSR;    /*!< Rising trigger selection register             - Offset: 0x08 */
	volatile uint32_t FTSR;    /*!< Falling trigger selection register            - Offset: 0x0C */
	volatile uint32_t SWIER;   /*!< Software interrupt event register             - Offset: 0x10 */
	volatile uint32_t PR;      /*!< Pending register                              - Offset: 0x14 */
} EXTI_RegDef_t;
#define EXTI ((volatile EXTI_RegDef_t*)EXTI_BASE)


typedef struct {
	volatile uint32_t MEMRMP;     /*!< Memory remap register                         			- Offset: 0x00 */
	volatile uint32_t PMC;        /*!< Peripheral mode configuration register         			- Offset: 0x04 */
	volatile uint32_t EXTICR[4];  /*!< External interrupt configuration registers 1, 2, 3, 4    - Offset: 0x08–0x14 */
	uint32_t RESERVED0[2];        /*!< Reserved                                       			- Offset: 0x18–0x1C */
	volatile uint32_t CMPCR;      /*!< Compensation cell control register            			- Offset: 0x20 */
} SYSCFG_RegDef_t;
#define SYSCFG ((volatile SYSCFG_RegDef_t*)SYSCFG_BASE)


#define GPIO_PORT_CODE(x)	((x == GPIOA) ? 0 : \
						  	(x == GPIOB) ? 1 : \
						    (x == GPIOC) ? 2 : \
						    (x == GPIOD) ? 3 : \
						    (x == GPIOE) ? 4 : \
						    (x == GPIOF) ? 5 : \
						    (x == GPIOG) ? 6 : \
						    (x == GPIOH) ? 7 : \
						    (x == GPIOI) ? 8 : 0)

#define GPIOA_RESET() do { (RCC->AHB1RSTR |= (0b1 << 0)); (RCC->AHB1RSTR &= ~(0b1 << 0)); } while(0);
#define GPIOB_RESET() do { (RCC->AHB1RSTR |= (0b1 << 1)); (RCC->AHB1RSTR &= ~(0b1 << 1)); } while(0);
#define GPIOC_RESET() do { (RCC->AHB1RSTR |= (0b1 << 2)); (RCC->AHB1RSTR &= ~(0b1 << 2)); } while(0);
#define GPIOD_RESET() do { (RCC->AHB1RSTR |= (0b1 << 3)); (RCC->AHB1RSTR &= ~(0b1 << 3)); } while(0);
#define GPIOE_RESET() do { (RCC->AHB1RSTR |= (0b1 << 4)); (RCC->AHB1RSTR &= ~(0b1 << 4)); } while(0);
#define GPIOF_RESET() do { (RCC->AHB1RSTR |= (0b1 << 5)); (RCC->AHB1RSTR &= ~(0b1 << 5)); } while(0);
#define GPIOG_RESET() do { (RCC->AHB1RSTR |= (0b1 << 6)); (RCC->AHB1RSTR &= ~(0b1 << 6)); } while(0);
#define GPIOH_RESET() do { (RCC->AHB1RSTR |= (0b1 << 7)); (RCC->AHB1RSTR &= ~(0b1 << 7)); } while(0);
#define GPIOI_RESET() do { (RCC->AHB1RSTR |= (0b1 << 8)); (RCC->AHB1RSTR &= ~(0b1 << 8)); } while(0);

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (0b1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (0b1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (0b1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (0b1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (0b1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (0b1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (0b1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (0b1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (0b1 << 8))

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(0b1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(0b1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(0b1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(0b1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(0b1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(0b1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(0b1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(0b1 << 7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(0b1 << 8))

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (0b1 << 21))

#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(0b1 << 21))

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (0b1 << 12))

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(0b1 << 12))

#define USART1_PCLK_EN() (RCC->APB2ENR |= (0b1 << 4))

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(0b1 << 4))

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (0b1 << 14))

#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(0b1 << 14))

#define IRQ_NUMBER_EXTI0 6
#define IRQ_NUMBER_EXTI1 7
#define IRQ_NUMBER_EXTI2 8
#define IRQ_NUMBER_EXTI3 9
#define IRQ_NUMBER_EXTI4 10
#define IRQ_NUMBER_EXTI9_5 23
#define IRQ_NUMBER_EXTI15_10 40

#define NVIC_IRQ_PRIORITY0 0
#define NVIC_IRQ_PRIORITY1 1
#define NVIC_IRQ_PRIORITY2 2
#define NVIC_IRQ_PRIORITY3 3
#define NVIC_IRQ_PRIORITY4 4
#define NVIC_IRQ_PRIORITY5 5
#define NVIC_IRQ_PRIORITY6 6
#define NVIC_IRQ_PRIORITY7 7
#define NVIC_IRQ_PRIORITY8 8
#define NVIC_IRQ_PRIORITY9 9
#define NVIC_IRQ_PRIORITY10 10
#define NVIC_IRQ_PRIORITY11 11
#define NVIC_IRQ_PRIORITY12 12
#define NVIC_IRQ_PRIORITY13 13
#define NVIC_IRQ_PRIORITY14 14
#define NVIC_IRQ_PRIORITY15 15

#define ENABLE 1
#define DISABLE 0
#define SET 1
#define RESET 0
#define HIGH 1
#define LOW 0

#include "STM32F407XX_GPIO_DRIVER.h"

#endif /* INC_STM32F407XX_H_ */
