/*
 * stm32nul476rg_spi_driver.c
 *
 *  Created on: Aug 29, 2024
 *      Author: hp
 */
#include "stm32nul476rg_spi_driver.h"

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}


/*
 * Init and  De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//peri clock enable of spi
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempreg1 = 0;
    uint32_t tempreg2 = 0;


	//configure mode
	tempreg1 |= pSPIHandle->SPIConfig.SPI_DeviceMode<<2;

	//configure the bus
	if(pSPIHandle->SPIConfig.SPI_BusConfig== SPI_BUSCONFIG_FD)
	{
		//BIDI MODE should be cleared
		tempreg1 &= ~(1<<SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig== SPI_BUS_CONFIG_HD)
	{
		//BIDI MODE should be set
		tempreg1 |= (1<<SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig== SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI MODE should be cleared
		tempreg1 &= ~(1<<SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg1 |= (1<<SPI_CR1_RXONLY);
	}

	//configure the SPI serial clock speed (baud rate)
	tempreg1 |= pSPIHandle->SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR;

	//configure the data size
//    tempreg2 |= (pSPIHandle->SPIConfig.SPI_DS << 8);
	// Configure the data size (4-bit to 16-bit)
	if (pSPIHandle->SPIConfig.SPI_DS < 4 || pSPIHandle->SPIConfig.SPI_DS > 16)
	{
		pSPIHandle->SPIConfig.SPI_DS = 8; // Default to 8-bit if invalid value
	}
	tempreg2 |= ((pSPIHandle->SPIConfig.SPI_DS - 1) << 8); // DS[3:0] field in CR2

	//configure the CPOL
	tempreg1 |= pSPIHandle->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL;

	// CONFIGURE THE CPHA
	tempreg1 |= pSPIHandle->SPIConfig.SPI_CPHA<<SPI_CR1_CPHA;

	// Enable Software Slave Management if configured
	if (pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_EN)
	{
	    tempreg1 |= (1 << SPI_CR1_SSM);  // Set SSM
	    tempreg1 |= (1 << SPI_CR1_SSI);  // Set SSI (simulate NSS high)
	}


	//SPE bit
//	tempreg1 |= pSPIHandle->SPIConfig.

	pSPIHandle->pSPIx->CR1=tempreg1;
	pSPIHandle->pSPIx->CR2=tempreg2;

//	pSPIHandle->pSPIx->CR1 |= (1<<SPI_CR1_SPE);

}




void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_PCLK_DI();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_PCLK_DI();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_PCLK_DI();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlageName)
{
	if(pSPIx->SR & FlageName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*
 * DATA SEND and  RECEIVE
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)== FLAG_RESET);

        // Check the data frame format by checking the DS[3:0] bits in the CR2 register
        uint8_t dataSize = (pSPIx->CR2 >> 8) & 0x0F;

        if (dataSize > 7) {
            // 16-bit data frame (DS > 8 bits)
            // Send 16-bit data
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            pTxBuffer += 2; // Move the pointer by 2 bytes since we're sending 16 bits
            Len -= 2;
        } else {
            // 8-bit data frame (DS <= 8 bits)
            // Send 8-bit data
            pSPIx->DR = *pTxBuffer;
            pTxBuffer++; // Move the pointer by 1 byte since we're sending 8 bits
            Len--;
        }
//        // ✅ Wait until RXNE is set (something is received in full-duplex)
//        while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
//
//        // ✅ Read DR to clear RXNE (discard if dummy)
//        volatile uint16_t dummy = pSPIx->DR;
//        (void)dummy; // avoid compiler warnings
	}

}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len>0)
		{
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)== FLAG_RESET);

	        // Check the data frame format by checking the DS[3:0] bits in the CR2 register
	        uint8_t dataSize = (pSPIx->CR2 >> 8) & 0x0F;

	        if (dataSize > 7) {
	            // 16-bit data frame (DS > 8 bits)
	            // Send 16-bit data
	             *((uint16_t*)pRxBuffer)= pSPIx->DR;
	            pRxBuffer += 2; // Move the pointer by 2 bytes since we're sending 16 bits
	            Len -= 2;
	        } else {
	            // 8-bit data frame (DS <= 8 bits)
	            // Send 8-bit data
	             *pRxBuffer=pSPIx->DR ;
	            pRxBuffer++; // Move the pointer by 1 byte since we're sending 8 bits
	            Len--;
	        }
		}

}


/*
 * IRQ Configuration and ISR handle
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling	(SPI_Handle_t *pHandle);

//other pheripherals control
void SPI_PeripheralControl(SPI_RegDef_t *pSPIX, uint8_t EnOrDi)
{
	uint16_t temp = pSPIX->CR1;
	if(EnOrDi == ENABLE)
	{
//		pSPIX->CR1 |= (1<<SPI_CR1_SPE);
		temp |= (1<<SPI_CR1_SPE);
	}
	else
	{
//		pSPIX->CR1 &= ~(1<<SPI_CR1_SPE);
		temp &= ~(1<<SPI_CR1_SPE);
	}
	pSPIX->CR1= temp;


}

void SPI_SSIConfig(SPI_RegDef_t *pSPIX, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIX->CR1 |= (1<<SPI_CR1_SSI);
	}
	else
	{
		pSPIX->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}


void SPI_SSOEConfig(SPI_RegDef_t *pSPIX, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIX->CR2 |= (1<<SPI_CR2_SSOE);
	}
	else
	{
		pSPIX->CR2 &= ~(1<<SPI_CR2_SSOE);
	}

}






