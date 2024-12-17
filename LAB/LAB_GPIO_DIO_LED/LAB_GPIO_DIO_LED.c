/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-09-19 by KHW0619
* @brief   Embedded Controller:  LAB_GPIO_DIO_LED
******************************************************************************
*/

#include "ecRCC2.h"
#include "ecGPIO2.h"

#define LED_PIN         PA_5
#define UP_BUTTON_PIN   PA_8
#define DOWN_BUTTON_PIN PA_9

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
		button_state = !GPIO_read(DOWN_BUTTON_PIN);
		if(!button_state) GPIO_write(LED_PIN, LOW);
		else GPIO_write(LED_PIN, HIGH);

		for(int iter_count = 0; iter_count < iter; iter_count++){};
		button_state = !GPIO_read(UP_BUTTON_PIN);
		if(!button_state) GPIO_write(LED_PIN, LOW);
		else GPIO_write(LED_PIN, HIGH);

	}
}

void setup(void) {
	RCC_HSI_init();
	GPIO_init(UP_BUTTON_PIN, INPUT);
	GPIO_pupd(UP_BUTTON_PIN, PULL_UP);
	GPIO_init(DOWN_BUTTON_PIN, INPUT);
	GPIO_pupd(DOWN_BUTTON_PIN, PULL_UP);
	GPIO_init(LED_PIN, OUTPUT);
	GPIO_otype(LED_PIN, OPEN_DRAIN);
	GPIO_pupd(LED_PIN, PULL_UP);
	GPIO_ospeed(LED_PIN, MEDIUM_SPEED);
	GPIO_write(LED_PIN, HIGH);
}