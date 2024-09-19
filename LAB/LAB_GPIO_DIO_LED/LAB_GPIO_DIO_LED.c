/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-09-19 by KHW0619
* @brief   Embedded Controller:  LAB_GPIO_DIO_LED
******************************************************************************
*/


#include "ecRCC2.h"
#include "ecGPIO2.h"

#define LED_PIN PA_5		// LED pin number
#define BUTTON_PIN PC_13	// button pin number

void setup(void);			// Initialiization
	
int main(void) { 
 	setup();
	int button_state = 0;
	int button_state_prev = 0;
	int output_state = LOW;
	int iter = 300;				// iteration for debouncing

	while(1){
		// software debouncing and read the button state
		for(int iter_count = 0; iter_count < iter; iter_count++){};
		button_state = GPIO_read(BUTTON_PIN);

		// check the button state change
		if(button_state_prev && !button_state) {
			// software debouncing and double-check the button state
			for(int iter_count = 0; iter_count < iter; iter_count++){};
			button_state = GPIO_read(BUTTON_PIN);

			// change LED pin's output state
			if(!button_state) output_state = !output_state;
		}

		GPIO_write(LED_PIN, output_state);

		// save the previous value
		button_state_prev = button_state;
	}
}

void setup(void) {
	RCC_HSI_init();
	GPIO_init(BUTTON_PIN, INPUT);
	GPIO_init(LED_PIN, OUTPUT);
	GPIO_otype(LED_PIN, OPEN_DRAIN);
	GPIO_pupd(LED_PIN, PULL_UP);
	GPIO_pupd(BUTTON_PIN, PULL_UP);
	GPIO_ospeed(LED_PIN, MEDIUM_SPEED);
}