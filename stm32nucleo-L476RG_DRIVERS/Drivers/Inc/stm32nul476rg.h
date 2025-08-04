/*
 * stm32nul476rg.h
 *
 *  Created on: Aug 8, 2024
 *      Author: Lokesh
 */

#ifndef INC_STM32NUL476RG_H_
#define INC_STM32NUL476RG_H_

#include<stdint.h>
#define __vo volatile

/*****************************START:Processor Specific Details**************************
 * 		ARM Cortex M4 processor NVIC ISERx register Address
 */
#define NVIC_ISER0				( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1				( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2				( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3				( (__vo uint32_t*)0xE000E10c )

/*
 * 	ARM Cortex M4 processor NVIC ISERx register Address
 */
#define NVIC_ICER0				( (__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1				( (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2				( (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3				( (__vo uint32_t*)0xE000E18C )

/*
 * 	ARM Cortex M4 processor Processor register Address
 */
#define NVIC_PR_BASE_ADDR		( (__vo uint32_t*)0xE000E400 )


/*
 * 	ARM Cortex M4 processor Processor register Address
 */
#define NO_PR_BITS_IMPLEMENTED		4




/*
 * base address of FLASH  and SRAM memories
 */

#define FLASH_BASEADDR    					0x08000000U					//Flash memory
#define SRAM1_BASEADDR						0x20000000U					//96kb
#define SRAM2_BASEADDR						0X20018000U					//32kb 0x20000000+96KB
#define ROM									0x1FFF0000U
#define SRAM 								SRAM1_BASEADDR


#define PERIPH_BASE							0x40000000U
#define APB1PERIPH_BASEADDR					PERIPH_BASE
#define APB2PERIPH_BASEADDR					0x40010000U
#define AHB1PERIPH_BASEADDR					0x40020000U
#define AHB2PERIPH_BASEADDR					0x48000000U


//Base address of the GPIOS
#define GPIOA_BASEADDR						(AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB2PERIPH_BASEADDR + 0X0400)
#define GPIOC_BASEADDR						(AHB2PERIPH_BASEADDR + 0X0800)
#define GPIOD_BASEADDR						(AHB2PERIPH_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR						(AHB2PERIPH_BASEADDR + 0X1000)
#define GPIOF_BASEADDR						(AHB2PERIPH_BASEADDR + 0X1400)
#define GPIOG_BASEADDR						(AHB2PERIPH_BASEADDR + 0X1800)
#define GPIOH_BASEADDR						(AHB2PERIPH_BASEADDR + 0X1C00)

#define RCC_BASEADDR 						(AHB1PERIPH_BASEADDR + 0x1000)	//0x4002 1000


//Base address of peripherals which are hanging on APB1 bus
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define CAN1_BASEADDR						(APB1PERIPH_BASEADDR + 0x6400)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)


//Base address of peripherals which are hanging on APB2 bus
#define EXTI_BASEADDR 						(APB2PERIPH_BASEADDR + 0X0400)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x0000)
#define	SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)


/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{
	__vo uint32_t MODER;					//__vo is volatalie keyword shortform
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
	__vo uint32_t BRR;
	__vo uint32_t ASCR;				  /*!< GPIO Bit Reset register,               Address offset: 0x28      */

}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t ICSCR;
	__vo uint32_t CFGR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t PLLSAI1CFGR;
	__vo uint32_t PLLSAI2CFGR;
	__vo uint32_t CIER;
	__vo uint32_t CIFR;
	__vo uint32_t CICR;
	uint32_t      RESERVED0;   // Reserved
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t      RESERVED1;   // Reserved
	__vo uint32_t APB1RSTR1;
	__vo uint32_t APB1RSTR2;
	__vo uint32_t APB2RSTR;
	uint32_t      RESERVED2;   // Reserved
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t      RESERVED3;
	__vo uint32_t APB1ENR1;
	__vo uint32_t APB1ENR2;
	__vo uint32_t APB2ENR;
	uint32_t      RESERVED4;
	__vo uint32_t AHB1SMENR;
	__vo uint32_t AHB2SMENR;
	__vo uint32_t AHB3SMENR;
	uint32_t      RESERVED5;
	__vo uint32_t APB1SMENR1;
	__vo uint32_t APB1SMENR2;
	__vo uint32_t APB2SMENR;
	 uint32_t      RESERVED6;
	__vo uint32_t CCIPR;
	uint32_t      RESERVED7;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;

}RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR1;			/*!< EXTI,              					Address offset: 0x00      */
	__vo uint32_t EMR1;			/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__vo uint32_t RTSR1;		/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__vo uint32_t FTSR1;		/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__vo uint32_t SWIER1;		/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__vo uint32_t PR1;			/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	uint32_t      RESERVED1;
	uint32_t      RESERVED2;
	__vo uint32_t IMR2;			/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__vo uint32_t EMR2;			/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__vo uint32_t RTSR2;		/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__vo uint32_t FTSR2;
	__vo uint32_t SWIER2;
	__vo uint32_t PR2;

}EXTI_RegDef_t;


/*
 * SYSTEM CONFIGURE  register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;		/*!< SYSCFG,              					Address offset: 0x00      */
	__vo uint32_t CFGR1;		/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__vo uint32_t EXTICR[4];	/*!< GPIO Bit Reset register,               Address offset: 0x28      */
//	__vo uint32_t EXTICR2;		/*!< GPIO Bit Reset register,               Address offset: 0x28      */
//	__vo uint32_t EXTICR3;		/*!< GPIO Bit Reset register,               Address offset: 0x28      */
//	__vo uint32_t EXTICR4;		/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__vo uint32_t SCSR;			/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__vo uint32_t CFGR2;		/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__vo uint32_t SWPR;			/*!< GPIO Bit Reset register,               Address offset: 0x28      */
	__vo uint32_t SKR;
	__vo uint32_t SWPR2;

}SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
}SPI_RegDef_t;



//peripheral definition (peripheral base type casted to xxx_RegDef_t)
#define GPIOA 		( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB 		( (GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC 		( (GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD 		( (GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE 		( (GPIO_RegDef_t*) GPIOE_BASEADDR )
#define GPIOF 		( (GPIO_RegDef_t*) GPIOF_BASEADDR )
#define GPIOG 		( (GPIO_RegDef_t*) GPIOG_BASEADDR )
#define GPIOH 		( (GPIO_RegDef_t*) GPIOH_BASEADDR )

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI 		((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG 		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1 		( (SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 		( (SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 		( (SPI_RegDef_t*)SPI3_BASEADDR)


// 						ENABLE PCLK MACROS
//Clock enable macros for GPIOX peripherals
#define GPIOA_PCLK_EN()	( RCC->AHB2ENR |= (1<<0) )
#define GPIOB_PCLK_EN()	( RCC->AHB2ENR |= (1<<1) )
#define GPIOC_PCLK_EN()	( RCC->AHB2ENR |= (1<<2) )
#define GPIOD_PCLK_EN()	( RCC->AHB2ENR |= (1<<3) )
#define GPIOE_PCLK_EN()	( RCC->AHB2ENR |= (1<<4) )
#define GPIOF_PCLK_EN()	( RCC->AHB2ENR |= (1<<5) )
#define GPIOG_PCLK_EN()	( RCC->AHB2ENR |= (1<<6) )
#define GPIOH_PCLK_EN()	( RCC->AHB2ENR |= (1<<7) )


//Clock enable macros for I2Cx peripherals
#define I2C1_PCLK_EN()	( RCC->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN()	( RCC->APB1ENR |= (1<<22) )
#define I2C3_PCLK_EN()	( RCC->APB1ENR |= (1<<23) )


//Clock enable macros for I2Cx peripherals
#define SPI1_PCLK_EN() 	( RCC->APB2ENR |= (1<<12) )
#define SPI2_PCLK_EN() 	( RCC->APB1ENR1 |= (1<<14) )
#define SPI3_PCLK_EN() 	( RCC->APB1ENR1 |= (1<<15) )


//Clock enable macros for I2Cx peripherals
#define USART1_PCLK_EN() ( RCC-> APB2ENR |= (1<<14) )
#define USART2_PCLK_EN() ( RCC-> APB1ENR |= (1<<17) )
#define USART3_PCLK_EN() ( RCC-> APB1ENR |= (1<<18) )
#define UART4_PCLK_EN() ( RCC-> APB1ENR |= (1<<19) )
#define UART5_PCLK_EN() ( RCC-> APB1ENR |= (1<<20) )


//Clock Enable macros for SYSCFG peripherals
#define SYSCFG_PCLK_EN() 	( RCC->APB2ENR |= (1<<0) )


//						CLK DISABLE MACROS

//Clock Disable macros for GPIOX peripherals
#define GPIOA_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<4) )
#define GPIOF_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<5) )
#define GPIOG_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<6) )
#define GPIOH_PCLK_DI()	( RCC->AHB2ENR &= ~(1<<7) )

//Clock Disable macros for I2Cx peripherals
#define I2C1_PCLK_DI()	( RCC->APB1ENR &= ~(1<<21) )
#define I2C2_PCLK_DI()	( RCC->APB1ENR &= ~(1<<22) )
#define I2C3_PCLK_DI()	( RCC->APB1ENR &= ~(1<<23) )


//Clock Disable macros for I2Cx peripherals
#define SPI1_PCLK_DI() 	( RCC->APB2ENR &= ~(1<<12) )
#define SPI2_PCLK_DI() 	( RCC->APB1ENR1 &= ~(1<<14) )
#define SPI3_PCLK_DI() 	( RCC->APB1ENR1 &= ~(1<<15) )


//Clock Disable macros for I2Cx peripherals
#define USART1_PCLK_DI() ( RCC-> APB2ENR &= ~(1<<14) )
#define USART2_PCLK_DI() ( RCC-> APB1ENR &= ~(1<<17) )
#define USART3_PCLK_DI() ( RCC-> APB1ENR &= ~(1<<18) )
#define UART4_PCLK_DI() ( RCC-> APB1ENR &= ~(1<<19) )
#define UART5_PCLK_DI() ( RCC-> APB1ENR &= ~(1<<20) )


//Clock Enable macros for SYSCFG peripherals
#define SYSCFG_PCLK_DI() 	( RCC->APB2ENR &= ~(1<<0) )

//Macros to reset GPIOx peripherals
#define GPIOA_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<0));  (RCC->AHB2RSTR &=~(1<<0)); } while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<1));  (RCC->AHB2RSTR &=~(1<<1)); } while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<2));  (RCC->AHB2RSTR &=~(1<<2)); } while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<3));  (RCC->AHB2RSTR &=~(1<<3)); } while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<4));  (RCC->AHB2RSTR &=~(1<<4)); } while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<5));  (RCC->AHB2RSTR &=~(1<<5)); } while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<6));  (RCC->AHB2RSTR &=~(1<<6)); } while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB2RSTR |=(1<<7));  (RCC->AHB2RSTR &=~(1<<7)); } while(0)


#define GPIO_BASEADDR_TO_CODE(x)	(	(x==GPIOA)?0:\
										(x==GPIOB)?1:\
										(x==GPIOC)?2:\
										(x==GPIOD)?3:\
										(x==GPIOE)?4:\
										(x==GPIOF)?5:\
										(x==GPIOG)?6:\
										(x==GPIOH)?7:0  )


/*
 * IRQ (interrupt Request) numbers of the stm32nul476rg
 */
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 *  IRQ PRIORITY
 */
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15




//some generic macros
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_SET		SET
#define FLAG_RESET		RESET


/*
 * BIT position definition of SPI peripheral
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB			7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_CRCL		11
#define SPI_CR1_CRCEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

//CR2
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_NSSP			3
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7
#define SPI_CR2_DS_0   			8  // Bit position for DS0
#define SPI_CR2_DS_1   			9  // Bit position for DS1
#define SPI_CR2_DS_2   			10 // Bit position for DS2
#define SPI_CR2_DS_3   			11 // Bit position for DS3
#define SPI_CR2_FRXTH			12
#define SPI_CR2_LDMA_RX			13
#define SPI_CR2_LDMA_TX			14

//SR
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8
#define SPI_SR_FRLVL			9
#define SPI_SR_FTLVL			11


/*
 * all data size possible
 */
#define SPI_DS_4BITS   0x03
#define SPI_DS_5BITS   0x04
#define SPI_DS_6BITS   0x05
#define SPI_DS_7BITS   0x06
#define SPI_DS_8BITS   0x07
#define SPI_DS_9BITS   0x08
#define SPI_DS_10BITS  0x09
#define SPI_DS_11BITS  0x0A
#define SPI_DS_12BITS  0x0B
#define SPI_DS_13BITS  0x0C
#define SPI_DS_14BITS  0x0D
#define SPI_DS_15BITS  0x0E
#define SPI_DS_16BITS  0x0F



#include "stm32nul476rg_gpio_driver.h"
#include "stm32nul476rg_spi_driver.h"


#endif /* INC_STM32NUL476RG_H_ */
