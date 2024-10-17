/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-10-15 by KHW0619
* @brief   Embedded Controller:  EC_TEST1_22200235
******************************************************************************
*/

#include "ecSTM32F4v2.h"

#define BUTTON_PIN PC_13
#define PWM_PIN PA_0
#define DIR_PIN PC_2
#define IR_PIN PB_1

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
    GPIO_otype(BUTTON_PIN, PULL_DOWN);

    GPIO_init(IR_PIN, INPUT);
    GPIO_otype(IR_PIN, PULL_DOWN);

    GPIO_init(DIR_PIN, OUTPUT);
    GPIO_otype(DIR_PIN, PULL_UP);
    GPIO_write(DIR_PIN, DIR_value);

    PWM_init(PWM_PIN);
    PWM_period(PWM_PIN, 1);

    TIM_UI_init(TIM3, 500);
    TIM_UI_enable(TIM3);

    EXTI_init(BUTTON_PIN, FALL, 0);
    EXTI_init(IR_PIN, FALL, 0);
    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);  // Decoder input A,B,C,D
    sevensegment_decoder_init();
}

int count = 0;
int state = 0;
int updown = 1;
//
float duty_value = 0.0;
int sevensegment_state = 0;
float counting_rate = 0.0;

void TIM3_IRQHandler(void) {
    // counting the period
    count++;

    // Check TIM3 UIF
    if(is_UIF(TIM3)){

        if(count > counting_rate && counting_rate != 3) { // counting_rate = 2 (1 sec) counting_rate = 1 (0.5 sec)
            // reset counter

            if(state != 0 && state != 3) sevensegment_display(sevensegment_state % 11);
            //sevensegment
            sevensegment_state += updown;
            if (sevensegment_state > 9) sevensegment_state = 0;
            else if(sevensegment_state < 0) sevensegment_state = 9;

            // set PWM
            PWM_duty(PWM_PIN,  duty_value);
        }

        // Clear TIM3 UIF
        clear_UIF(TIM3);
    }
}

void EXTI1_IRQHandler(void) {

    if(GPIO_read(IR_PIN) == LOW) {
        state = 0;
        sevensegment_display(10);
        GPIO_write(DIR_PIN, 0);
        PWM_duty(PWM_PIN,  0);
    }
    else {
    }
}

void EXTI15_10_IRQHandler(void) {
    for(int i = 0; i < 500000;i++){}  // delay_ms(500);
    unsigned int button_state = GPIO_read(BUTTON_PIN);

    if (is_pending_EXTI(BUTTON_PIN) && !button_state) {
        for(int i = 0; i < 500000; i++) {};
        button_state = GPIO_read(BUTTON_PIN);

        if(!button_state) {

            state++;
            if(state > 5) state = 0;

            switch(state) {
                case 0:
                    DIR_value = 0;
                    duty_value = 0;
                    counting_rate = 3.0;
                    sevensegment_state = 10;
                    sevensegment_display(sevensegment_state);
                    break;
                case 1:
                    DIR_value = 1;
                    duty_value = 0.75;
                    counting_rate = 2.0;
                    updown = 1;
                    sevensegment_state = 0;
                    break;
                case 2:
                    DIR_value = 1;
                    duty_value = 0.25;
                    counting_rate = 1.0;
                    updown = 1;
                    sevensegment_state = 0;
                    break;
                case 3:
                    DIR_value = 0;
                    duty_value = 0;
                    counting_rate = 3.0;
                    sevensegment_state = 10;
                    sevensegment_display(sevensegment_state);
                    break;
                case 4:
                    DIR_value = 0;
                    duty_value = 0.25;
                    counting_rate = 2.0;
                    sevensegment_state = 9;
                    updown = -1;
                    break;
                case 5:
                    DIR_value = 0;
                    duty_value = 0.75;
                    counting_rate = 1.0;
                    sevensegment_state = 9;
                    updown = -1;
                    break;
            }

            GPIO_write(DIR_PIN, DIR_value);
            PWM_duty(PWM_PIN,  duty_value);
            if(state != 0 && state != 3) sevensegment_display(sevensegment_state % 11);
        }
        // Clear BUTTON_PIN's pending
        clear_pending_EXTI(BUTTON_PIN);
    }
}
