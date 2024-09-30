/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-09-30 by KHW0619
* @brief   Embedded Controller:  LAB_EXTI
******************************************************************************
*/

#include "ecSTM32F4v2.h"

#define LED_PIN	PA_5
#define BUTTON_PIN PC_13

unsigned int cnt = 0;

// Initialiization
void setup(void)
{
    RCC_PLL_init();    // System Clock = 84MHz
    SysTick_init();

    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);  // Decoder input A,B,C,D
    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_pupd(BUTTON_PIN, PULL_UP);

    // Priority Highest(0) External Interrupt
    EXTI_init(BUTTON_PIN, FALL, 0);
}

int main(void) {
    setup();

    while (1) {};
}


//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {
    for(int i = 0; i < 500000;i++){}  // delay_ms(500);
    unsigned int button_state = GPIO_read(BUTTON_PIN);

    if (is_pending_EXTI(BUTTON_PIN) && !button_state) {
        for(int i = 0; i < 500000; i++) {};
        button_state = GPIO_read(BUTTON_PIN);

        if(!button_state)
            if(GPIO_read(BUTTON_PIN) == 0) cnt++;
        if (cnt > 9) cnt = 0;

        sevensegment_display(cnt % 10);

        clear_pending_EXTI(BUTTON_PIN);
    }
}