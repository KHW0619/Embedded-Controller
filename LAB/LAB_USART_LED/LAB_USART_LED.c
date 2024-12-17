/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-10-25 by KHW0619
* @brief   Embedded Controller:  LAB_USART_LED
******************************************************************************
*/

#include "ecSTM32F4v2.h"
// #include "ecUART.h"



static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;
uint8_t PC_string[]="Loop:\r\n";

void setup(void){
    RCC_PLL_init();
    SysTick_init();

    // USART2: USB serial init
    UART2_init();
    UART2_baud(BAUD_9600);

    // USART1: BT serial init
    UART1_init();
    UART1_baud(BAUD_9600);
}

int main(void){
    setup();
    printf("MCU Initialized\r\n");
    while(1){
        // USART Receive: Use Interrupt only
        // USART Transmit:  Interrupt or Polling
        USART1_write((uint8_t*)&PC_Data, 1);
        delay_ms(2000);
    }
}

void USART2_IRQHandler(){          		// USART2 RX Interrupt : Recommended
    if(is_USART2_RXNE()){
        PC_Data = USART2_read();		// RX from UART2 (PC)
        USART2_write(&PC_Data, 1);		// TX to USART2	 (PC)	 Echo of keyboard typing
    }
}


void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
    if(is_USART1_RXNE()){
        BT_Data = USART1_read();		// RX from UART1 (BT)
        printf("RX: %c \r\n",BT_Data); // TX to USART2(PC)
    }
}

// #include "ecUART2.h"
//
// int main(void) {
//     RCC_PLL_init();
//     UART2_init();
//     UART2_baud(BAUD_9600);
//
//     while(1){printf("Hello World\r\n");}
// }