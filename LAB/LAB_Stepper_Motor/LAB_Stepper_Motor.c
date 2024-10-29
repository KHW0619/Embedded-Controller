/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-10-22 by KHW0619
* @brief   Embedded Controller:  LAB_Stepper_Motor
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecSTM32F4v2.h"

void setup(void);

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();

    Stepper_step(2048, 1, HALF);  // (Step : 2048, Direction : 0 or 1, Mode : FULL or HALF)

    // Inifinite Loop ----------------------------------------------------------
    while(1){;}
}

// Initialiization
void setup(void){

    RCC_PLL_init();                                     // System Clock = 84MHz
    SysTick_init();                                     // Systick init

    EXTI_init(BUTTON_PIN, FALL,0);  // External Interrupt Setting
    GPIO_init(BUTTON_PIN, INPUT);           // GPIOC pin13 initialization

    Stepper_init(PB_10,PB_4,PB_5,PB_3);                 // Stepper GPIO pin initialization
    Stepper_setSpeed(2);                          	    // set stepper motor speed
}

void EXTI15_10_IRQHandler(void) {
    if (is_pending_EXTI(BUTTON_PIN)) {
        Stepper_stop();
        clear_pending_EXTI(BUTTON_PIN);          // cleared by writing '1'
    }
}
