/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-09-22 by KHW0619
* @brief   Embedded Controller:  LAB_GPIO_7segment problem 1
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"

#define BUTTON_PIN PC_13

// Initialiization
void setup(void);

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();
    unsigned int cnt = 0;
    int button_state = 0;
    int button_state_prev = 0;
    int iter = 500000;

    // Inifinite Loop ----------------------------------------------------------
    while(1){
        for(int i = 0; i < 500000;i++){}  // delay_ms(500);

        // software debouncing and read the button state
        for(int iter_count = 0; iter_count < iter; iter_count++){};
        button_state = GPIO_read(BUTTON_PIN);

        // check the button state change
        if(button_state_prev && !button_state) {
            // software debouncing and double-check the button state
            for(int iter_count = 0; iter_count < iter; iter_count++){};
            button_state = GPIO_read(BUTTON_PIN);

            // change LED pin's output state
            if(!button_state)
                if(GPIO_read(BUTTON_PIN) == 0) cnt++;
            if (cnt > 9) cnt = 0;
        }
        // turn on the 7segment display by using sevensegment_display() function
        sevensegment_display(cnt % 10);

        button_state_prev = button_state;
    }
}

// Initialiization
void setup(void)
{
    RCC_HSI_init();
    GPIO_init(BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    GPIO_pupd(BUTTON_PIN, PULL_UP);
    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);  // Decoder input A,B,C,D
}