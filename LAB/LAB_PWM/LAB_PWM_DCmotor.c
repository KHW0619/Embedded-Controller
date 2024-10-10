/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-10-10 by KHW0619
* @brief   Embedded Controller:  LAB_PWM_DCmotor
******************************************************************************
*/

#include "ecSTM32F4v2.h"

#define BUTTON_PIN PC_13
#define PWM_PIN PA_0
#define DIR_PIN PC_2

void setup(void);

int main(void) {
    // Initialization --------------------------------------------------
    setup();

    // Infinite Loop ---------------------------------------------------
    while(1){}
}

int DIR_value = LOW;

// Initialization
void setup(void){
    RCC_PLL_init();				// System Clock = 84MHz
    SysTick_init();

    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_otype(BUTTON_PIN, PULL_UP);

    GPIO_init(DIR_PIN, OUTPUT);
    GPIO_otype(DIR_PIN, PUSH_PULL);
    GPIO_write(DIR_PIN, DIR_value);

    PWM_init(PWM_PIN);
    PWM_period(PWM_PIN, 1);

    TIM_UI_init(TIM3, 500);
    TIM_UI_enable(TIM3);

    EXTI_init(BUTTON_PIN, FALL, 0);
}

// counter for 2sec period
int count = 0;

// motor rotation velocity
int signal_strength = 0;

//
static int stop_flag = 0;
float duty_value = 0.0;

void TIM3_IRQHandler(void) {
    // counting the period
    count++;

    // Check TIM3 UIF
    if(is_UIF(TIM3)){
        // making 2sec period
        if(count > 3) {
            // reset counter
            count = 0;

            // when the stop_flag is not set, the duty value
            if(!stop_flag) duty_value = (0.25 + 0.5 * signal_strength);

            // set PWM
            PWM_duty(PWM_PIN,  duty_value);

            // change the motor rotation velocity
            signal_strength ^= 1;
        }

        // Clear TIM3 UIF
        clear_UIF(TIM3);
    }
}


void EXTI15_10_IRQHandler(void) {
    for(int i = 0; i < 500000;i++){}  // delay_ms(500);
    unsigned int button_state = GPIO_read(BUTTON_PIN);

    if (is_pending_EXTI(BUTTON_PIN) && !button_state) {
        for(int i = 0; i < 500000; i++) {};
        button_state = GPIO_read(BUTTON_PIN);

        if(!button_state) {
            // change motor's rotation state
            stop_flag ^= 1;

            // when the stop_flag is set, the duty value
            duty_value = DIR_value;

            // when the stop_flag is not set, the duty value
            if(!stop_flag) duty_value = (0.25 + 0.5 * signal_strength);

            // set PWM
            PWM_duty(PWM_PIN,  duty_value);
        }
        // Clear BUTTON_PIN's pending
        clear_pending_EXTI(BUTTON_PIN);
    }
}