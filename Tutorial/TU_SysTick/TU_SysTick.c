/**
******************************************************************************
* @Mod		 2024-9-27 by KHW0619
* @brief   Embedded Controller:  Tutorial SysTick
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecRCC2.h"
#include "ecGPIO2.h"

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

#define ENABLE 0
#define TICK_INT 1
#define CLKSOURCE 2

volatile uint32_t msTicks = 0;
volatile uint32_t curTicks;
int state = 0;
void setup(void);
void LED_toggle(void) {
	state ^= 1;
	GPIO_write(LED_PIN, state);
};

int main(void) {
	
// System CLOCK, GPIO Initialiization ----------------------------------------
	setup();

// SysTick Initialiization ------------------------------------------------------				
	//  SysTick Control and Status Register
	SysTick->CTRL = 0;				// Disable SysTick IRQ and SysTick Counter

	// Select processor clock
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= 1 << CLKSOURCE;

	// uint32_t MCU_CLK=EC_SYSTEM_CLK
	// SysTick Reload Value Register
	SysTick->LOAD = 0x1481F; 				// 1ms

	// Clear SysTick Current Value 
	SysTick->VAL = 0;

	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	SysTick->CTRL |= 1 << TICK_INT;
		
	// Enable SysTick IRQ and SysTick Timer
	SysTick->CTRL |= 1 << ENABLE;
		
	NVIC_SetPriority(SysTick_IRQn, 1);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC

		
// While loop ------------------------------------------------------				
	msTicks = 0;

	while(1){
		curTicks = msTicks;
		while ((msTicks - curTicks) < 1000);	
		msTicks = 0;
		LED_toggle();
	}
}


void SysTick_Handler(void){
	msTicks++;
}

void setup(void)
{
	RCC_PLL_init();              // System Clock = 84MHz	
	GPIO_init(LED_PIN, OUTPUT);     // calls RCC_GPIOA_enable()
}



