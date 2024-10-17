/**
******************************************************************************
* @Mod		 2024-10-04 by KHW0619
* @brief   Embedded Controller:  Tutorial TimerInterrupt
******************************************************************************
*/
#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"

PinName_t LED_pin = PA_5;
uint32_t count = 0;
int state = 0;

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	TIM_TypeDef* timerx;
	timerx = TIM2;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	timerx->CR1 &= ~(1 << 4);                     	// counter

	timerx->PSC = 839;				// Timer counter clock: 1MHz(1us)
	timerx->ARR = 99;				// Set auto reload register to maximum (count up to 65535)
	timerx->DIER |= 1 << 0;                    	// Enable Interrupt
	timerx->CR1 |= 1 << 0;                     	// Enable counter
	
	NVIC_SetPriority(TIM2_IRQn, 2);               	// TIM2_IRQHandler Set priority as 2
	NVIC_EnableIRQ(TIM2_IRQn);			// TIM2_IRQHandler Enable
	
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){}
}

// Initialiization 
void setup(void)
{	
	RCC_PLL_init();                       // System Clock = 84MHz
	GPIO_init(LED_pin, OUTPUT);    // calls RCC_GPIOA_enable()	
}

void TIM2_IRQHandler(void){
	if(TIM2->SR & TIM_SR_UIF){ // update interrupt flag
		//Create the code to toggle LED by 1000ms
		count++;
		if(count >= 1000) {
			count = 0;
			state ^= 1;
		}
		GPIO_write(LED_pin, state);

		TIM2->SR &= ~TIM_SR_UIF;                    // clear by writing 0
	}
}
