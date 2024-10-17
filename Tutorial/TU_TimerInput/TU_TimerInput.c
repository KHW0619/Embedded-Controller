//
// Created by dadan on 24. 10. 11.
//

#include <stdio.h>

#include "ecSTM32F4v2.h"
#include "ecUART2_simple.h"

int main(void) {
    RCC_PLL_init();
    UART2_init();

    while(1) {
        printf("1\r\n");
    }

    return 0;
}