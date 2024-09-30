# LAB: EXTI & SysTick

**Date:** 2023-09-30

**Author:** Hee-Won Kim

**Github:** 

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

> Answer discussion questions
> 
1. What would happen if the EXTI interrupt handler does not clear the interrupt pending flag? Check with your code
    
    > Answer discussion questions
    > 

### **My Code**

Explain your source code with the necessary comments.

```
// YOUR MAIN CODE ONLY
// YOUR CODE
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

Explain your source code with necessary comments.

```C
// YOUR MAIN CODE ONLY
// YOUR CODE
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
