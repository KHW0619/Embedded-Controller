/**
******************************************************************************
* @author  Jong-Hyeon Kim / Hee-Won Kim
* @Mod	   2024-12-16
* @brief   PROJECT_Mobile_Elevator
******************************************************************************
*/

#include "ecSTM32F4v2.h"

#define UP_BUTTON_PIN   PA_8 // 3
#define DOWN_BUTTON_PIN  PC_7 // 1

uint8_t BT_string[];

void setup(void);

int main(void) {
    // Initialization --------------------------------------------------
    setup();

    // Infinite Loop ---------------------------------------------------
    while(1) {
        int up_button = !GPIO_read(UP_BUTTON_PIN);
        int down_button = !GPIO_read(DOWN_BUTTON_PIN);

        BT_string[0] = 0;

        if (!up_button) {
            BT_string[0] = 1;
        }
        else if (!down_button) {
            BT_string[0] = 2;
        }

        USART1_write(BT_string,1);

    }
}

// Initialization
void setup(void){
    RCC_PLL_init();				// System Clock = 84MHz

    GPIO_init(UP_BUTTON_PIN, INPUT);
    GPIO_otype(UP_BUTTON_PIN, PUSH_PULL);
    GPIO_init(DOWN_BUTTON_PIN, INPUT);
    GPIO_otype(DOWN_BUTTON_PIN, PUSH_PULL);

    // UART initialization for connection with HC-06
    UART1_init();
    UART1_baud(9600);
}


