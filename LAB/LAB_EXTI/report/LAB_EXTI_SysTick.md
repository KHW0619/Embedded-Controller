# LAB: EXTI & SysTick

**Date:** 2023-09-30

**Author:** Hee-Won Kim

**Github:** [https://github.com/KHW0619/Embedded-Controller/tree/master/LAB/LAB_EXTI](https://github.com/KHW0619/Embedded-Controller/tree/master/LAB/LAB_EXTI)

**Demo Video:** [PROBLEM 1](https://youtu.be/jPIeNC17DnA), [PROBLEM 2](https://youtu.be/rx8KDd-bWiE)

## **Introduction**

> In this lab, two simple programs will be created using interrupt:
>
> (1) displaying the number counting from 0 to 9 with Button Press
> 
> (2) counting at a rate of 1 second

### **Requirement**

### **Hardware**

- MCU
    - NUCLEO-F411RE
- Actuator/Sensor/Others:
    - 4 LEDs and load resistance
    - 7-segment display(5101ASR)
    - Array resistor (330 ohm)
    - breadboard

### **Software**

- Keil uVision, CMSIS, EC_HAL library
---

## **Problem 1: Counting numbers on 7-Segment using EXTI Button**

### **1-1. Create ecEXTI.h library**
ecEXTI.h got below functions.

<br>

```
void EXTI_init(PinName_t pinName, int trig_type, int priority);
```
(In the EXTI_init function, the SYSCFG peripheral clock is enabled, the external line is connected to the GPIO, the trigger edge is configured, the interrupt is enabled, and the NVIC IRQ is set.)

```
void EXTI_enable(uint32_t pin);
```
(In the EXTI_enable function, the pin's EXTI->IMR is set as not masked.)

```
void EXTI_disable(uint32_t pin);
```
(In the EXTI_disable function, the pin's EXTI->IMR is set as masked.)

```
uint32_t  is_pending_EXTI(uint32_t pin);
```
(In the is_pending_EXTI function, the pin is checked its EXTI->PR state.)

```
void clear_pending_EXTI(uint32_t pin);
```
(In the clear_pending_EXTI function, the pin is cleared its EXTI->PR state.)

<br>

### **1-2. Procedure**
>This problem will use the decoder chip (**74LS47**). So, only 4 digital out pins of MCU will be used. Then, a code will be created to display the number counting from 0 to 9 and repeating. The number count up only by pressing the push button.

### **Configuration**

| Digital In for Button (B1) | Digital Out for 7-Segment decoder |
| --- | --- |
| Digital In | Digital Out |
| PC13 | PA7, PB6, PC7, PA9 |
| PULL-UP | Push-Pull, No PullUp-PullDown, Medium Speed |

### **Circuit Diagram**
Circuit Diagram
![Circuit diagram](img/EXTI_SysTick_LAB_problem1_CD.jpg)

### **Discussion**

1. We can use two different methods to detect an external signal: polling and interrupt. What are the advantages and disadvantages of each approach?

    > **polling**
    > 
    > - advantage: This method is good for understanding code.
    > - disadvantage: It can not keep the immediate response then the interrupt method, and it is slow then other one.
    > 
    > **interrupt**
    >
    > - advantage: This is efficient method because the code is operated only when a signal occurs.
    > - disadvantage: It is quietly complex to implement.
    > 

2. What would happen if the EXTI interrupt handler does not clear the interrupt pending flag? Check with your code
    
    > It could checked by removing the clear_pending_EXTI function. The result shows the random numbers displayed to 7-segment display.
    > I guess because of not clear the pending flag, variant cnt's increasing speed could be fast more. That guess could be the reason why display number was random.

### **My Code**

> **Code Explain**
>
> The setup function is initialize all the pins using at this problem. It satisfies configuration.
>
> The while loop prevent the code from ending.
>
> If the button pin pressed, EXTI15_10_IRQHandler function will run. This function change the state at the rate of second.

```
/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-09-30 by KHW0619
* @brief   Embedded Controller:  LAB_EXTI
******************************************************************************
*/

#include "ecSTM32F4v2.h"

#define BUTTON_PIN PC_13

// display number state
unsigned int cnt = 0;

void setup(void)
{
    // setting system clock and initialize SysTick
    RCC_PLL_init();
    SysTick_init();

    // Setting Output Pins (for 7-segment display)
    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);  // Decoder input A,B,C,D

    // Setting Input Pin
    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_pupd(BUTTON_PIN, PULL_UP);

    // initialize EXTI
    EXTI_init(BUTTON_PIN, FALL, 0);
}

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ----------------------------------------------------------
    while (1) {};
}


// EXTI for Pin 13
// this function has software debouncing code
void EXTI15_10_IRQHandler(void) {
    for(int i = 0; i < 500000;i++){}  // delay_ms(500);
    unsigned int button_state = GPIO_read(BUTTON_PIN);

    if (is_pending_EXTI(BUTTON_PIN) && !button_state) {
        for(int i = 0; i < 500000; i++) {};
        button_state = GPIO_read(BUTTON_PIN);

        if(!button_state)
            // change the state
            if(GPIO_read(BUTTON_PIN) == 0) cnt++;
        if (cnt > 9) cnt = 0;

        // set 7-segment display
        sevensegment_display(cnt % 10);

        clear_pending_EXTI(BUTTON_PIN);
    }
}
```

### **Results**
> Through the experiment, the result shows that the display number increases by 1 when the button is pressed.

Experiment Circuit Image
![Circuit image](img/problem_circuit.png)

Experiment Result Image

| ![7segment_0](img/problem1_0.jpg) 0      | ![7segment_1](img/problem1_1.jpg) 1      | ![7segment_2](img/problem1_2.jpg) 2      |
|------------------------------------------|------------------------------------------|------------------------------------------|
| ![7segment_3](img/problem1_3.jpg) 3      | ![7segment_4](img/problem1_4.jpg) 4      | ![7segment_5](img/problem1_5.jpg) 5      |
| ![7segment_6](img/problem1_6.jpg) 6      | ![7segment_7](img/problem1_7.jpg) 7      | ![7segment_8](img/problem1_8.jpg) 8      |
| ![7segment_9](img/problem1_9.jpg) 9      |                                          |                                          |

----

## **Problem 2: Counting numbers on 7-Segment using SysTick**

> The problem shows changing the 7-segment LED at the rate of 1 sec. If the current state is '9', it will reset to '0'. When the button is pressed, the number will reset to '0' and start counting again.

### **2-1. Create ecSysTick.h library**

ecSysTick.h got below functions.

```
void SysTick_init(uint32_t msec);
void delay_ms(uint32_t msec);
uint32_t SysTick_val(void);
void SysTick_reset (void);
void SysTick_enable(void);
void SysTick_disable (void)
```

### **2-2. Procedure**
>This problem will use the decoder chip (**74LS47**). So, only 4 digital out pins of MCU will be used. Then, a code will be created to display the number counting from 0 to 9 and repeating. The number count up only at the rate of 1 second.
> 
> If the button is pressed, it will be reset '0' and counting again.

### **Configuration**

| Digital In for Button (B1) | Digital Out for 7-Segment decoder |
| --- | --- |
| Digital In | Digital Out |
| PC13 | PA7, PB6, PC7, PA9 |
| PULL-UP | Push-Pull, No Pull-up-Pull-down, Medium Speed |

### **Circuit Diagram**

Circuit Diagram
![Circuit diagram](img/EXTI_SysTick_LAB_problem1_CD.jpg)

### **My Code**

> **Code Explain**
> 
> The setup function is initialize all the pins using at this problem. It satisfies configuration.
> 
> In the while loop, 7-segment display change the state at the rate of second.
> 
> If the button pin pressed, EXTI15_10_IRQHandler function will run. This function reset '0' the display state.

```C
/*
******************************************************************************
* @author  Hee-Won Kim
* @Mod	   2024-09-30 by KHW0619
* @brief   Embedded Controller:  LAB_EXTI_SysTick
******************************************************************************
*/
#include "ecSTM32F4v2.h"

// display number state
int cnt = 0;

void setup(void)
{
    // setting system clock and initialize SysTick
    RCC_PLL_init();
    SysTick_init();

    // Setting Input Pin
    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_pupd(BUTTON_PIN, PULL_UP);

    // Setting Output Pins (for 7-segment display)
    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);

    // initialize EXTI
    EXTI_init(BUTTON_PIN, FALL, 0);
}

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ----------------------------------------------------------
    while(1){
        // set 7-segment display
        sevensegment_display(cnt);
        delay_ms(1000);

        // change the state
        cnt++;
        if (cnt > 9) cnt = 0;

        // SysTick reset
        SysTick_reset();
    }
}


//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {
    unsigned int button_state = GPIO_read(BUTTON_PIN);

    if (is_pending_EXTI(BUTTON_PIN) && !button_state) {
        // setting the display to '0'
        cnt = -1;

        clear_pending_EXTI(BUTTON_PIN);
    }
}

```

### **Results**
> Through the experiment, the result shows that the display number increases by 1 as the rate of 1 second. Also, the result shows reset '0' result when the button is pressed.

Experiment Circuit Image
![Circuit image](img/problem_circuit.png)

Experiment Result Image

| ![7segment_0](img/problem2_0.jpg) 0 | ![7segment_1](img/problem2_1.jpg) 1 | ![7segment_2](img/problem2_2.jpg) 2 |
|-------------------------------------|-------------------------------------|-------------------------------------|
| ![7segment_3](img/problem2_3.jpg) 3 | ![7segment_4](img/problem2_4.jpg) 4 | ![7segment_5](img/problem2_5.jpg) 5 |
| ![7segment_6](img/problem2_6.jpg) 6 | ![7segment_7](img/problem2_7.jpg) 7 | ![7segment_8](img/problem2_8.jpg) 8 |
| ![7segment_9](img/problem2_9.jpg) 9 |                                     |                                     |

Experiment Result Image when the button is pressed

| ![buttonpress_0](img/buttonpress_problem2_0.jpg) before the button pressed | ![buttonpress_1](img/buttonpress_problem2_1.jpg) after the button pressed 0 | ![buttonpress_2](img/buttonpress_problem2_2.jpg) after the button pressed 1 | ![buttonpress_3](img/buttonpress_problem2_3.jpg) after the button pressed 2 |
|----------------------------------------------------------------------------|-----------------------------------------------------------------------------|-----------------------------------------------------------------------------|-------------------------------------------------|

## **Reference**

> Zhu, Y. (2018). Embedded systems with ARM Cortex-M microcontrollers in assembly language and C (3rd ed.). E-Man Press LLC.
