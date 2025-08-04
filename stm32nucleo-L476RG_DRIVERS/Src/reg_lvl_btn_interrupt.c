/*
 * reg_lvl_btn_interrupt.c
 *
 *  Created on: Sep 2, 2024
 *      Author: hp
 */


#include "stm32nul476rg.h"

void EXTI_init();

void EXTI_init(){
	RCC->AHB2ENR |= (1<<0);  // Enable clock for GPIOA
	RCC->AHB2ENR |= (1<<2);  // Enable clock for GPIOC
	RCC->APB2ENR |= (1<<0); // Enable SYSCFGEN

	GPIOA->MODER |= (1<<10);
	GPIOA->MODER &= ~(1<<11);
	GPIOC->MODER |= 0;

	SYSCFG->EXTICR[3] &= ~(1<<4);  // Clear Interrupt Configuration Register
	SYSCFG->EXTICR[3] &= ~(1<<5);
	SYSCFG->EXTICR[3] &= ~(1<<6);
	SYSCFG->EXTICR[3] &= ~(1<<7);

	SYSCFG->EXTICR[3] |= (1<<5);   // Select PC13 as an Interrupt Trigger

	EXTI->IMR1 |= (1<<13);     // Interrupt Mask Register
	EXTI->FTSR1 |= (1<<13);    // Falling Trigger Selection Register

//	NVIC_EnableIRQ(EXTI15_10_IRQn);
	// IRQ number for EXTI15_10_IRQn is 40
	#define EXTI15_10_IRQn (40)

	// NVIC base address
	#define NVIC_BASE (0xE000E100UL)

	// ISER1 base address (covers IRQ numbers 32 to 63)
	#define NVIC_ISER1 (*(volatile uint32_t *)(NVIC_BASE + 0x004))

	// Enable EXTI15_10 interrupt
	NVIC_ISER1 = (1 << (EXTI15_10_IRQn - 32));

}

void EXTI15_10_IRQHandler(void){

	if((EXTI->PR1 & (1<<13)) != 0){
		GPIOA->ODR ^= (1<<5);
		EXTI-> PR1 |= (1<<13);		// Clear Pending Register
	}

}

int main(void){

	EXTI_init();

	while(1){


	}
}




