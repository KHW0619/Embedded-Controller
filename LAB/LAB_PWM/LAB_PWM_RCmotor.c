/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-10-09 by KHW0619
* @brief   Embedded Controller:  LAB_PWM_RCmotor
******************************************************************************
*/

#include "ecSTM32F4v2.h"

#define BUTTON_PIN	PC_13
#define PWM_PIN PA_1

void setup(void);

int main(void) {
    // Initialization --------------------------------------------------
    setup();

    // Infinite Loop ---------------------------------------------------
    while(1){}
}


// Initialization
void setup(void){
    // System Clock = 84MHz
    RCC_PLL_init();

    // SysTick init
    SysTick_init();

    // BUTTON_PIN setting INPUT, PULL_UP
    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_otype(BUTTON_PIN, PULL_UP);

    // PWM_PIN setting 20ms period
    PWM_init(PWM_PIN);
    PWM_period(PWM_PIN, 20);

    // TIM3 setting 500ms period
    TIM_UI_init(TIM3, 500);
    TIM_UI_enable(TIM3);

    EXTI_init(BUTTON_PIN, FALL, 0);
}

// motor rotation interval
float signal_strength = (2.5 - 0.5)/18.0;

// motor state num
int duty_count = 0;

// variable for motor rotation direction
uint32_t direction = 1;

void TIM3_IRQHandler(void) {
    // Check TIM3 UIF
    if(is_UIF(TIM3)){
        PWM_duty(PWM_PIN, (float)(signal_strength * duty_count + 0.5)/20.0);

        // chane the motor state
        duty_count += direction;

        // change the direction
        if(duty_count > 17 || duty_count < 1) direction*= -1;

        // Clear TIM3 UIF
        clear_UIF(TIM3);
    }
}

void EXTI15_10_IRQHandler(void) {
    // Check BUTTON_PIN's pending
    if (is_pending_EXTI(BUTTON_PIN)) {

        // reset the motor state
        duty_count = 0;
        direction = 1;

        // Clear BUTTON_PIN's pending
        clear_pending_EXTI(BUTTON_PIN);
    }
}

