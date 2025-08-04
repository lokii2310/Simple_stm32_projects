/*
 * stm32nul476rg_spi_driver.h
 *
 *  Created on: Aug 29, 2024
 *      Author: hp
 */

#ifndef INC_STM32NUL476RG_SPI_DRIVER_H_
#define INC_STM32NUL476RG_SPI_DRIVER_H_

#include "stm32nul476rg.h"

typedef struct
{
    uint8_t SPI_DeviceMode;    // Possible values from @SPI_DeviceMode
    uint8_t SPI_BusConfig;     // Possible values from @SPI_BusConfig
    uint8_t SPI_SclkSpeed;     // Possible values from @SPI_SclkSpeed
    uint8_t SPI_DS;           // Possible values from @SPI_DS
    uint8_t SPI_CPOL;          // Possible values from @SPI_CPOL
    uint8_t SPI_CPHA;          // Possible values from @SPI_CPHA
    uint8_t SPI_SSM;           // Possible values from @SPI_SSM
} SPI_Config_t;

typedef struct
{
    SPI_RegDef_t    *pSPIx;    // This holds the base address of the SPIx (x:0,1,2) peripheral
    SPI_Config_t    SPIConfig; // This holds SPI configuration settings
} SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 */
#define SPI_BUSCONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3



/*
 * @SPI_SclkSpeed
 */
#define	SPI_SCLK_SPEED_DIV2		0
#define	SPI_sCLK_SPEED_DIV4		1
#define	SPI_sCLK_SPEED_DIV8		2
#define	SPI_sCLK_SPEED_DIV16	3
#define	SPI_sCLK_SPEED_DIV32	4
#define	SPI_sCLK_SPEED_DIV64	5
#define	SPI_sCLK_SPEED_DIV128	6
#define	SPI_sCLK_SPEED_DIV256	7

///*
// * @SPI_DFF
// */
//#define SPI_DFF_8BITS		0
//#define SPI_DFF_16BITS		1
/** @defgroup SPI_LL_EC_DATAWIDTH Datawidth
  * @{
  */
#define SPI_DATAWIDTH_4BIT              (SPI_CR2_DS_0 | SPI_CR2_DS_1)                               /*!< Data length for SPI transfer:  4 bits */
#define SPI_DATAWIDTH_5BIT              (SPI_CR2_DS_2)                                              /*!< Data length for SPI transfer:  5 bits */
#define SPI_DATAWIDTH_6BIT              (SPI_CR2_DS_2 | SPI_CR2_DS_0)                               /*!< Data length for SPI transfer:  6 bits */
#define SPI_DATAWIDTH_7BIT              (SPI_CR2_DS_2 | SPI_CR2_DS_1)                               /*!< Data length for SPI transfer:  7 bits */
#define SPI_DATAWIDTH_8BIT              (SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0)                /*!< Data length for SPI transfer:  8 bits */
#define SPI_DATAWIDTH_9BIT              (SPI_CR2_DS_3)                                              /*!< Data length for SPI transfer:  9 bits */
#define SPI_DATAWIDTH_10BIT             (SPI_CR2_DS_3 | SPI_CR2_DS_0)                               /*!< Data length for SPI transfer: 10 bits */
#define SPI_DATAWIDTH_11BIT             (SPI_CR2_DS_3 | SPI_CR2_DS_1)                               /*!< Data length for SPI transfer: 11 bits */
#define SPI_DATAWIDTH_12BIT             (SPI_CR2_DS_3 | SPI_CR2_DS_1 | SPI_CR2_DS_0)                /*!< Data length for SPI transfer: 12 bits */
#define SPI_DATAWIDTH_13BIT             (SPI_CR2_DS_3 | SPI_CR2_DS_2)                               /*!< Data length for SPI transfer: 13 bits */
#define SPI_DATAWIDTH_14BIT             (SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_0)                /*!< Data length for SPI transfer: 14 bits */
#define SPI_DATAWIDTH_15BIT             (SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1)                /*!< Data length for SPI transfer: 15 bits */
#define SPI_DATAWIDTH_16BIT             (SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0) /*!< Data length for SPI transfer: 16 bits */

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH 	1
#define SPI_CPHA_LOW 	0

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH 	1
#define SPI_CPOL_LOW 	0
/*
 * @SPI_SSM
 */
#define SPI_SSM_EN		1
#define SPI_SSM_DI		0


/*
 * SPI related status flags de
 */
#define SPI_TXE_FLAG		(1<< SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1<< SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1<< SPI_SR_BSY)




/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and  De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);  // Fixed pTxBuffer type to uint8_t*
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);  // Fixed pRxBuffer type to uint8_t*

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * OTHER pheripherals control API
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIX, uint8_t EnOrDi);

void SPI_SSIConfig(SPI_RegDef_t *pSPIX, uint8_t EnOrDi);

void SPI_SSOEConfig(SPI_RegDef_t *pSPIX, uint8_t EnOrDi);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlageName);

void SPI_SSIConfig(SPI_RegDef_t *pSPIX, uint8_t EnOrDi);


#endif /* INC_STM32NUL476RG_SPI_DRIVER_H_ */
