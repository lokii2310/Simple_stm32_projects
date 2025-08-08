/*
 * SPI_ARDUINO_txonly.c
 *
 *  Created on: Sep 6, 2024
 *      Author: hp
 */


#include "stm32nul476rg.h"
#include<string.h>

/*
 * SPI1
 * PA4 -->	NSS
 * PA5 -->	SCLK
 * PA6 -->	MISO
 * PA7 -->  MOSI
 * SPI2
 * PB12 -->	NSS
 * PB13 -->	SCLK
 * PB14 -->	MISO
 * PB15 -->  MOSI
 * ALT function mode : AF5
 */

#define LED_GPIO_PORT GPIOA
#define LED_PIN       5

void LED_Init(void)
{
    GPIO_Handle_t ledgpio;
    ledgpio.pGPIOx = LED_GPIO_PORT;
    ledgpio.GPIO_PinConfig.GPIO_PinNumber = LED_PIN;
    ledgpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    ledgpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    ledgpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    ledgpio.GPIO_PinConfig.GPIO_PinPuPdContol = GPIO_NO_PUPD;
    GPIO_Init(&ledgpio);
}

void LED_On(void)
{
    GPIO_WriteToOutputPin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET);
}

void LED_Off(void)
{
    GPIO_WriteToOutputPin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
}

//small change
void delay(void)
{
	for(uint32_t i=0;i<50000/2;i++);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

//	SPIPins.pGPIOx=GPIOA;
//	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
//	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
//	SPIPins.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
//	SPIPins.GPIO_PinConfig.GPIO_PinPuPdContol = GPIO_NO_PUPD;
//	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//
//	//SCLK
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
//	GPIO_Init(&SPIPins);
//
//	//MISO
////	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
////	GPIO_Init(&SPIPins);
//
//	//MOSI
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
//	GPIO_Init(&SPIPins);
//
//	//NSS
//	SPIPins.pGPIOx=GPIOB;
//	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
//	GPIO_Init(&SPIPins);

		SPIPins.pGPIOx=GPIOB;
		SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
		SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
		SPIPins.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
		SPIPins.GPIO_PinConfig.GPIO_PinPuPdContol = GPIO_NO_PUPD;
		SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		//SCLK
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
		GPIO_Init(&SPIPins);

		//MISO
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
		SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
		GPIO_Init(&SPIPins);

		//MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
		SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
		GPIO_Init(&SPIPins);

		//NSS
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
		GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx=SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUSCONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_sCLK_SPEED_DIV64;
	SPI2handle.SPIConfig.SPI_DS = SPI_DS_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;  // Enable Software Slave Management (SSM)

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;



	//button gpio configuration
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdContol = GPIO_NO_PUPD;   //GPIO_PIN_PU


	GPIO_Init(&GpioBtn);


}

int main(void)
{
	char user_data[] = "Hello World";

	SPI2_GPIOInits();

	GPIO_ButtonInit();
	LED_Init();

//	RCC->APB1RSTR1 |= (1<<21);

	SPI2_Inits();

	SPI_SSIConfig(SPI2, ENABLE);  // Enable SSI to simulate NSS high
//	SPI_SSOEConfig(SPI2, ENABLE);

//	SPI_PeripheralControl(SPI2, ENABLE);
////	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_6, GPIO_PIN_SET);
//
//	SPI_SendData(SPI2, (uint8_t*)user_data,strlen(user_data));
//
//	SPI_PeripheralControl(SPI2, DISABLE);


	while(1)
	{
		while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();
		LED_On();  // Turn on LED to indicate button press
		//ENABLE THE SPI SPE
		SPI_PeripheralControl(SPI2, ENABLE);

//		//first send the length information
		uint8_t dataLen = strlen(user_data);
//
//		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_6, GPIO_PIN_RESET);  // Turn on LED (for testing)
//
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, GPIO_PIN_RESET); // NSS low
		SPI_SendData(SPI2, &dataLen, 1);
		//GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_4, GPIO_PIN_SET);  // NSS low


		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));


//		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_RESET);  // Turn off LED

		//confirm spi is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, GPIO_PIN_SET); // NSS high


//		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_6, GPIO_PIN_SET);  // Turn on LED (for testing)


		//DISABLE THE SPI PERI
		SPI_PeripheralControl(SPI2, DISABLE);


		LED_Off();  // Turn off LED after transmission

	}


	return 0;
}





//
///*
// * SPI_ARDUINO_txonly_updated.c
// *
// * Created on: Sep 6, 2024
// * Author: hp
// */
//
//#include "stm32nul476rg.h"
//#include <string.h>
//
///*
// * PA4 -->  NSS
// * PA5 -->  SCLK
// * PA6 -->  MISO
// * PA7 -->  MOSI
// * ALT function mode : AF5
// */
//
//void delay(void)
//{
//    for (uint32_t i = 0; i < 50000 / 2; i++);
//}
//
//void SPI1_GPIOInits(void)
//{
//    GPIO_Handle_t SPIPins;
//
//    SPIPins.pGPIOx = GPIOA;
//    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
//    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
//    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
//    SPIPins.GPIO_PinConfig.GPIO_PinPuPdContol = GPIO_NO_PUPD;
//    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//
//    // SCLK
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
//    GPIO_Init(&SPIPins);
//
//    // MOSI
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
//    GPIO_Init(&SPIPins);
//
//    // NSS
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
//    GPIO_Init(&SPIPins);
//}
//
//void SPI1_Inits(void)
//{
//    SPI_Handle_t SPI2handle;
//
//    SPI2handle.pSPIx = SPI1;
//    SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUSCONFIG_FD;
//    SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
//    SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_sCLK_SPEED_DIV8;
//    SPI2handle.SPIConfig.SPI_DS = SPI_DS_8BITS;
//    SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
//    SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
//
//    // Enable Software Slave Management (SSM) and set SSI to simulate NSS high
//    SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;
//
//    SPI_Init(&SPI2handle);
//}
//
//void GPIO_ButtonInit(void)
//{
//    GPIO_Handle_t GpioBtn;
//
//    GpioBtn.pGPIOx = GPIOC;
//    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
//    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
//    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdContol = GPIO_NO_PUPD;
//
//    GPIO_Init(&GpioBtn);
//}
//
//int main(void)
//{
//    char user_data[] = "Hello World";
//
//    SPI1_GPIOInits();
//    GPIO_ButtonInit();
//    SPI1_Inits();
//
//    // Enable the SPI and set SSI
//    SPI_PeripheralControl(SPI1, ENABLE);
//    SPI1->CR1 |= (1 << SPI_CR1_SSI);
//
//    while (1)
//    {
//        delay();
//
//        uint8_t dataLen = strlen(user_data);
//
//        SPI_SendData(SPI1, &dataLen, 1);
//        SPI_SendData(SPI1, (uint8_t*)user_data, dataLen);
//
//        // Wait until SPI is not busy before disabling
//        while (SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));
//
//        SPI_PeripheralControl(SPI1, DISABLE);
//    }
//
//    return 0;
//}
//
//




//
//
///*
// * SPI_ARDUINO_txonly_updated.c
// *
// * Created on: Sep 6, 2024
// * Author: hp
// */
//
//#include "stm32nul476rg.h"
//#include <string.h>
//
///*
// * PA4 -->  NSS
// * PA5 -->  SCLK
// * PA6 -->  MISO
// * PA7 -->  MOSI
// * ALT function mode : AF5
// */
//
//void delay(void)
//{
//    for (uint32_t i = 0; i < 50000 / 2; i++);
//}
//
//void SPI1_GPIOInits(void)
//{
//    GPIO_Handle_t SPIPins;
//
//    SPIPins.pGPIOx = GPIOA;
//    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
//    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
//    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
//    SPIPins.GPIO_PinConfig.GPIO_PinPuPdContol = GPIO_NO_PUPD;
//    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//
//    // SCLK
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
//    GPIO_Init(&SPIPins);
//
//    // MOSI
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
//    GPIO_Init(&SPIPins);
//
//    // NSS
//    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
//    GPIO_Init(&SPIPins);
//}
//
//void SPI1_Inits(void)
//{
//    SPI_Handle_t SPI2handle;
//
//    SPI2handle.pSPIx = SPI1;
//    SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUSCONFIG_FD;
//    SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
//    SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_sCLK_SPEED_DIV8;
//    SPI2handle.SPIConfig.SPI_DS = SPI_DS_8BITS;
//    SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
//    SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
//
//    // Enable Software Slave Management (SSM) and set SSI to simulate NSS high
//    SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;
//
//    SPI_Init(&SPI2handle);
//}
//
//void GPIO_ButtonInit(void)
//{
//    GPIO_Handle_t GpioBtn;
//
//    GpioBtn.pGPIOx = GPIOC;
//    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
//    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
//    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdContol = GPIO_NO_PUPD;
//
//    GPIO_Init(&GpioBtn);
//}
//
//int main(void)
//{
//    char user_data[] = "Hello World";
//
//    SPI1_GPIOInits();
//    GPIO_ButtonInit();
//    SPI1_Inits();
//
//    // Enable the SPI and set SSI
////    SPI1->CR1 |= (1 << SPI_CR1_SSI);
//    SPI_PeripheralControl(SPI1, ENABLE);
//
//
//    while (1)
//    {
//        delay();
//
//        uint8_t dataLen = strlen(user_data);
//
//        SPI_SendData(SPI1, &dataLen, 1);
//        SPI_SendData(SPI1, (uint8_t*)user_data, dataLen);
//
//        // Wait until SPI is not busy before disabling
//        while (SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));
//
//        SPI_PeripheralControl(SPI1, DISABLE);
//    }
//
//    return 0;
//}
