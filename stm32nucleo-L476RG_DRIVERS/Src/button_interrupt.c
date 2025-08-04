/*
 * LED_Button.c
 *
 *  Created on: Aug 14, 2024
 *      Author: hp
 */

#include "stm32nul476rg.h"
#include "string.h"

void delay(void)
{
    for(uint32_t i = 0; i < 50000/2; i++);
}

#define LOW         1
#define BTN_PRESSED LOW

int main(void)
{
    GPIO_Handle_t GpioLed, GpioBtn;

    memset(&GpioLed, 0, sizeof(GpioLed));
    memset(&GpioBtn, 0, sizeof(GpioBtn));

    // LED GPIO configuration
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
//    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdContol = GPIO_NO_PUPD;

//    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GpioLed);

    // Button GPIO configuration
    GpioBtn.pGPIOx = GPIOC;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;  // Interrupt mode, falling edge
//    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdContol = GPIO_NO_PUPD;

//    GPIO_PeriClockControl(GPIOC, ENABLE);
    GPIO_Init(&GpioBtn);

    // IRQ configuration
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
    while(1);
//    while(1){
//    	delay();
//        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);  // Toggle the LED
//
//    }

    return 0;
}

void EXTI15_10_IRQHandler(void)
{
//	if(GPIO_ReadFromInputPin(GPIOC, 13) = BTN_PRESSED)
//	{
		GPIO_IRQHandling(GPIO_PIN_NO_13);  // Clear the interrupt flag
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);  // Toggle the LED
//	}
}
