/**
******************************************************************************
* @Mod		 2024-10-04 by KHW0619
* @brief   Embedded Controller:  Tutorial TU_Timer_PWM
******************************************************************************
*/
#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"
#include "ecSysTick2.h"

#define LED_PIN    PA_5

void setup(void);
	
int main(void) {
	// Initialiization --------------------------------------------------------
	RCC_PLL_init();                         // System Clock = 84MHz
	SysTick_init();                         // for delay_ms()
	GPIO_init(LED_PIN, AF);     // GPIOA 5 ALTERNATE function
	GPIO_ospeed(LED_PIN, HIGH_SPEED);   // GPIOA 5 HIGH SPEED
	GPIO_otype(LED_PIN, PUSH_PULL);
	GPIO_pupd(LED_PIN, NO_PUPD);

	// TEMP: TIMER Register Initialiization --------------------------------------------------------
	TIM_TypeDef *TIMx;
	TIMx = TIM2;

	// GPIO: ALTERNATIVE function setting
	GPIOA->AFR[0] &= ~(0b1111 << 20);           						// AF1 at PA5 = TIM2_CH1 (p.150)
	GPIOA->AFR[0] |= 0b0001 << 20;           						// AF1 at PA5 = TIM2_CH1 (p.150)

	// TIMER: PWM setting
	RCC->APB1ENR |= 1 << 0;              				// Enable TIMER clock

	TIMx->CR1 &= ~(1 << 4);				              		// Direction Up-count

	TIMx->PSC = 839;						              // Set Timer CLK = 100kHz : (PSC + 1) = 84MHz/100kHz --> PSC = ?

	TIMx->ARR = 99;									        // Auto-reload: Upcounting (0...ARR).
	// Set Counter CLK = 1kHz : (ARR + 1) = 100kHz/1kHz --> ARR = ?

	TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;  			// Clear ouput compare mode bits for channel 1
	TIMx->CCMR1 |= 0b110 << 4;                  			// OC1M = 110 for PWM Mode 1 output on ch1
	TIMx->CCMR1	|= TIM_CCMR1_OC1PE;    		// Output 1 preload enable (make CCR1 value changable)

	TIMx->CCR1 = (TIM2->ARR + 1)/2;      										// Output Compare Register for channel 1

	TIMx->CCER &= ~TIM_CCER_CC1P;    			// select output polarity: active high
	TIMx->CCER |= TIM_CCER_CC1E;												// Enable output for ch1

	TIMx->CR1  |= TIM_CR1_CEN;      			// Enable counter
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		//Create the code to change the brightness of LED as 10kHZ (use "delay(1000)")
		for(int i =0;i<10;i++){
			TIM2->CCR1 = i*10;
			delay_ms(100);
		}
		for(int i =0;i<10;i++){
			TIM2->CCR1 = (10-i)*10;
			delay_ms(100);
		}
	}
}

// Initialiization 
void setup(void)
{	
	RCC_PLL_init();       // System Clock = 84MHz
	SysTick_init();       // for delay_ms()
}



