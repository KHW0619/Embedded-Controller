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
PinName_t SEVEN_SEGMENT_PIN[4] = {PA_7, PB_6, PC_7, PA_9};
PinName_t SEVEN_SEGMENT_DECODER_PIN[8] = {PA_5, PA_6, PA_7, PB_6, PC_7, PA_9, PA_8, PB_10};
static int input_pin_state = 0;

void GPIO_init(PinName_t pinName, uint32_t mode){     
	GPIO_TypeDef * Port;
	unsigned int pin;
	ecPinmap(pinName, &Port, &pin);
	
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	
	// Make it for GPIOB, GPIOD..GPIOH
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	if (Port == GPIOD)
		RCC_GPIOD_enable();
	if (Port == GPIOE)
		RCC_GPIOE_enable();
	if (Port == GPIOH)
		RCC_GPIOH_enable();

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
	GPIO_TypeDef * Port;
    unsigned int pin;
    ecPinmap(pinName,&Port,&pin);
	
	Port->OSPEEDR &= ~(3UL<<(2*pin));
	Port->OSPEEDR |= speed<<(2*pin); 
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(PinName_t pinName, int type){
	GPIO_TypeDef * Port;
    unsigned int pin;
    ecPinmap(pinName,&Port,&pin);
	
	Port->OTYPER &= ~(1UL<<(pin));
	Port->OTYPER |= type<<(pin); 
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(PinName_t pinName, int pupd){
	GPIO_TypeDef * Port;
    unsigned int pin;
    ecPinmap(pinName,&Port,&pin);
	
	Port->PUPDR &= ~(3UL<<(2*pin));
	Port->PUPDR |= pupd<<(2*pin); 
}

int GPIO_read(PinName_t pinName){
	GPIO_TypeDef * Port;
    unsigned int pin;
    ecPinmap(pinName,&Port,&pin);

	return Port->IDR&(1<<pin);//[TO-DO] YOUR CODE GOES HERE
}

void GPIO_write(PinName_t pinName, int Output){
	GPIO_TypeDef * Port;
    unsigned int pin;
	ecPinmap(pinName,&Port,&pin);
	
	Port->ODR &= ~(1<<(pin));
	Port->ODR |= (Output<<(pin)); 
}

void sevensegment_display_init(PinName_t pinNameA, PinName_t pinNameB, PinName_t pinNameC, PinName_t pinNameD){
	PinName_t SEVEN_SEGMENT_PIN[4] = {pinNameA, pinNameB, pinNameC, pinNameD};

	for(int i = 0; i < 4; i++){
		GPIO_init(SEVEN_SEGMENT_PIN[i], OUTPUT);
		GPIO_otype(SEVEN_SEGMENT_PIN[i], PUSH_PULL);
		GPIO_pupd(SEVEN_SEGMENT_PIN[i], NO_PUPD);
		GPIO_ospeed(SEVEN_SEGMENT_PIN[i], MEDIUM_SPEED);

		GPIO_write(SEVEN_SEGMENT_PIN[i], LOW);
	}
}

void sevensegment_display(uint8_t num) {
	for(int i = 0; i < 4; i++) {
		GPIO_write(SEVEN_SEGMENT_PIN[3 - i], (num >> i) & ( 1 ));
	}
}

void sevensegment_decoder_init(void) {

	for(int i = 0; i < 8; i++){
		GPIO_init(SEVEN_SEGMENT_DECODER_PIN[i], OUTPUT);
		GPIO_otype(SEVEN_SEGMENT_DECODER_PIN[i], PUSH_PULL);
		GPIO_pupd(SEVEN_SEGMENT_DECODER_PIN[i], NO_PUPD);
		GPIO_ospeed(SEVEN_SEGMENT_DECODER_PIN[i], MEDIUM_SPEED);

		GPIO_write(SEVEN_SEGMENT_DECODER_PIN[i], HIGH);
	}
}

void sevensegment_decoder(uint8_t  num) {
	int seven_segment_state[4] = {0};
	int seven_segment_decoder_state[8] = {0};
	int A = 0, B = 0, C = 0, D = 0;

	for(int i = 0; i < 4; i++) {
		seven_segment_state[i] = (num >> i) & ( 1 );
	}

	A = seven_segment_state[0];
	B = seven_segment_state[1];
	C = seven_segment_state[2];
	D = seven_segment_state[3];

	seven_segment_decoder_state[0] = (~D & ~C & ~B & A) | (~D & C & ~A);
	seven_segment_decoder_state[1] = (C & ~B & A) + (~D & C & B & ~A);
	seven_segment_decoder_state[2] = ~D & ~C & B & ~A;
	seven_segment_decoder_state[3] = (~C & ~B & A) | (~D & C & ~B & ~A) | (C & B & A) | (D & C & B);
	seven_segment_decoder_state[4] = A | (C & ~B) | (D & B);
	seven_segment_decoder_state[5] = (~D & ~C & A) | (~D & ~C & B) | (B & A) | (D & C);
	seven_segment_decoder_state[6] = (~D & ~C & ~B) | (C & B & A) | (D & C & B);


	for(int i = 0; i < 8; i++) {
		GPIO_write(SEVEN_SEGMENT_DECODER_PIN[i], seven_segment_decoder_state[i]);
	}
}

void LED_toggle(void) {
	input_pin_state ^= 1;
	GPIO_write(LED_PIN, input_pin_state);
};