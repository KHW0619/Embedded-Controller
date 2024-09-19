/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-09-19 by KHW0619
* @brief   Embedded Controller:  LAB_GPIO_DIO_multiLED
******************************************************************************
*/

#include "ecRCC2.h"
#include "ecGPIO2.h"

#define BUTTON_PIN PC_13                            // LED pin number
PinName_t LED_PINS[4] = {PA_5, PA_6, PA_7, PB_6};   // button pin number

void setup(void);                                   // Initialiization

int main(void) {
    int button_state = 0;
    int button_state_prev = 0;
    int led_idx = 0;
    int iter = 300;

    setup();

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
            if(!button_state) led_idx++;
        }

        // turn off the previous state and turn on the current state
        GPIO_write(LED_PINS[(led_idx + 3) % 4], LOW);
        GPIO_write(LED_PINS[led_idx % 4], HIGH);

        // save the previous value
        button_state_prev = button_state;
    }
}

void setup(void) {
    RCC_HSI_init();
    GPIO_init(BUTTON_PIN, INPUT);
    for(int led_set_idx = 0; led_set_idx < 4; led_set_idx++) {
        GPIO_init(LED_PINS[led_set_idx], OUTPUT);
        GPIO_otype(LED_PINS[led_set_idx], PUSH_PULL);
        GPIO_pupd(LED_PINS[led_set_idx], PULL_UP);
        GPIO_ospeed(LED_PINS[led_set_idx], MEDIUM_SPEED);
    }
    GPIO_pupd(BUTTON_PIN, PULL_UP);

    // turn off the all LEDs
    for(int led_set_idx = 0; led_set_idx < 4; led_set_idx++) GPIO_write(LED_PINS[led_set_idx], LOW);
}