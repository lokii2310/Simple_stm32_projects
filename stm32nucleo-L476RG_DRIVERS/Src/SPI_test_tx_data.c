/*
 * SPI_test_tx_data.c
 *
 *  Created on: Sep 4, 2024
 *      Author: hp
 */
#include "stm32nul476rg.h"
#include<string.h>

/*
 * PA4 -->	NSS
 * PA5 -->	SCLK
 * PA6 -->	MISO
 * PA7 -->  MOSI
 * ALT function mode : AF5
 */
void SPI1_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx=GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdContol = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);
}

void SPI1_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx=SPI1;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUSCONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DS = SPI_DS_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}

int main(void)
{
	char user_data[] = "Hello World";

	SPI1_GPIOInits();

	SPI1_Inits();

	//ENABLE THE SSI to avoid MODF error
	SPI_SSIConfig(SPI1, ENABLE);

	//ENABLE THE SPI PERI
	SPI_PeriClockControl(SPI1, ENABLE);


	SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));

	while(SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));
	//DISABLE THE SPI PERI
	SPI_PeriClockControl(SPI1, DISABLE);
	while(1);

	return 0;
}
