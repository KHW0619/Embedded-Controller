/**
******************************************************************************
* @Mod		 2024-9-27 by KHW0619
* @brief   Embedded Controller:  Tutorial EXTI
******************************************************************************
*/



#include "ecSTM32F4v2.h"


#define LED_PIN   PA_5
#define BUTTON_PIN PC_13

int state = 0;

void setup(void);
void LED_toggle(void) {
	state ^= 1;
	GPIO_write(LED_PIN, state);
};

int main(void) {

	// System CLOCK, GPIO Initialiization ----------------------------------------
	setup();


	// EXTI Initialiization ------------------------------------------------------	

	// SYSCFG peripheral clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Connect External Line to the GPIO
	// Button: PC_13 -> EXTICR3(EXTI13)
	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

	// Falling trigger enable (Button: pull-up)
	EXTI->FTSR |= EXTI_PR_PR13;

	// Unmask (Enable) EXT interrupt
	EXTI->IMR |= EXTI_PR_PR13;

	// Interrupt IRQn, Priority
	NVIC_SetPriority(EXTI15_10_IRQn, 0);  		// Set EXTI priority as 0	
	NVIC_EnableIRQ(EXTI15_10_IRQn); 			// Enable EXTI 


	while (1);
}


void EXTI15_10_IRQHandler(void) {
	if ((EXTI->PR & EXTI_PR_PR13) == EXTI_PR_PR13) {
		LED_toggle();
		EXTI->PR |= EXTI_PR_PR13; // cleared by writing '1'
	}
}


// Initialiization 
void setup(void)
{
	RCC_PLL_init();                         // System Clock = 84MHz
	// Initialize GPIOA_5 for Output
	GPIO_init(LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
    GPIO_otype(LED_PIN, PUSH_PULL);
    GPIO_pupd(LED_PIN, NO_PUPD);
	// Initialize GPIOC_13 for Input Button
	GPIO_init(BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(BUTTON_PIN, NO_PUPD);
}

