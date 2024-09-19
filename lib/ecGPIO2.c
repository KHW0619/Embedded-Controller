/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : SSS LAB
Created          : 05-03-2021
Modified         : 08-23-2024
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/



#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO2.h"

void GPIO_init(PinName_t pinName, uint32_t mode){     
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName, &Port, &pin);
	
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	
	//[TO-DO] YOUR CODE GOES HERE
	// Make it for GPIOB, GPIOD..GPIOH
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	if (Port == GPIOD)
		RCC_GPIOD_enable();
	if (Port == GPIOE)
		RCC_GPIOE_enable();
	if (Port == GPIOH)
		RCC_GPIOH_enable();

	// You can also make a more general function of
	// void RCC_GPIO_enable(GPIO_TypeDef *Port); 

	GPIO_mode(pinName, mode);
}


// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(PinName_t pinName, uint32_t mode){
   GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
	
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |= mode<<(2*pin);    
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(PinName_t pinName, int speed){
	 //[TO-DO] YOUR CODE GOES HERE
	 //[TO-DO] YOUR CODE GOES HERE
	 GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
	
	Port->OSPEEDR &= ~(3UL<<(2*pin));
	Port->OSPEEDR |= speed<<(2*pin); 
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(PinName_t pinName, int type){
   	//[TO-DO] YOUR CODE GOES HERE
	 //[TO-DO] YOUR CODE GOES HERE
	 GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
	
	Port->OTYPER &= ~(1UL<<(pin));
	Port->OTYPER |= type<<(pin); 
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(PinName_t pinName, int pupd){
   	//[TO-DO] YOUR CODE GOES HERE
	//[TO-DO] YOUR CODE GOES HERE
	 GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
	
	Port->PUPDR &= ~(3UL<<(2*pin));
	Port->PUPDR |= pupd<<(2*pin); 
}

int GPIO_read(PinName_t pinName){
   	//[TO-DO] YOUR CODE GOES HERE
	//[TO-DO] YOUR CODE GOES HERE
	 GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
	
	 return Port->IDR&(1<<pin);//[TO-DO] YOUR CODE GOES HERE	
		 
}

void GPIO_write(PinName_t pinName, int Output){
   	//[TO-DO] YOUR CODE GOES HERE
	//[TO-DO] YOUR CODE GOES HERE
	 GPIO_TypeDef * Port;
   unsigned int pin;
   ecPinmap(pinName,&Port,&pin);
	
	Port->ODR &= ~(1<<(pin));
	Port->ODR |= (Output<<(pin)); 
}

