/*
 * LED_Button.c
 *
 *  Created on: Aug 14, 2024
 *      Author: hp
 */


#include "stm32nul476rg.h"

void delay(void)
{
	for(uint32_t i=0;i<50000/2;i++);
}

#define LOW 		0
#define BTN_PRESSED LOW


int main(void)
{
	GPIO_Handle_t GpioLed,GpioBtn;

	//led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConig.GPIO_PinPuPdContol = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	//button gpio configuration
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConig.GPIO_PinPuPdContol = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)== BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}


	}
	return 0;
}

