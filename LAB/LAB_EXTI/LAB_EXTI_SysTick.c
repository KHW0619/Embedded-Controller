/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-09-30 by KHW0619
* @brief   Embedded Controller:  LAB_EXTI_SysTick
******************************************************************************
*/
#include "ecSTM32F4v2.h"

int cnt = 0;
// Initialiization
void setup(void)
{
    RCC_PLL_init();
    SysTick_init();

    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_pupd(BUTTON_PIN, PULL_UP);

    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);
    EXTI_init(BUTTON_PIN, FALL, 0);
}

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ----------------------------------------------------------
    while(1){
        sevensegment_display(cnt);
        delay_ms(1000);
        cnt++;
        if (cnt > 9) cnt = 0;
        SysTick_reset();
    }
}


//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {
    unsigned int button_state = GPIO_read(BUTTON_PIN);

    if (is_pending_EXTI(BUTTON_PIN) && !button_state) {
        cnt = -1;

        clear_pending_EXTI(BUTTON_PIN);
    }
}
