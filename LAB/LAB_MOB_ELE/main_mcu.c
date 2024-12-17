/**
******************************************************************************
* @author  Jong-Hyeon Kim / Hee-Won Kim
* @Mod	   2024-12-16
* @brief   PROJECT_Mobile_Elevator
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecSTM32F4v2.h"

//RC CAR
//TIRE (DC MOTOR)
#define RC_LEFT_DIR 	PA_11
#define RC_LEFT_PWM 	PA_0  // TIM2 CH1
#define RC_RIGHT_DIR 	PA_12
#define RC_RIGHT_PWM 	PA_1  // TIM2 CH2
#define RC_IR_LEFT 		PC_1
#define RC_IR_RIGHT 	PC_0

//ULTRASONIC
#define ULTRASONIC_TRIG PA_6  // TIM3 CH1
#define ULTRASONIC_ECHO PB_6

//ELEVATOR DOOR
#define DOOR_MOTOR 		PC_9

#define CLOSE_BUTTON 	PB_12
#define OPEN_BUTTON		PB_15
#define FLOOR3_BUTTON 	PB_14
#define FLOOR1_BUTTON 	PB_13

// ELEVATOR MOVING
#define ELEV_PWM_PIN 	PB_10
#define ELEV_DIR_PIN 	PC_2
#define UPPER_IR_PIN    PB_1
#define LOWER_IR_PIN    PC_8

#define ALERT			PB_8

// Elevator Up-Down Moving Variable
static int updown_flag		= LOW;
static int UPPER_IR_value	= HIGH;
static int LOWER_IR_value	= HIGH;
static int ELEV_dir_value	= 0;
static float ELEV_duty_value	= 0.0;

uint32_t OPEN = 0;

// ULTRA SONIC
uint32_t ovf_cnt = 0;
float time1 = 0;
float time2 = 0;
float distance = 0;
float prevDist = 1000;
float timeInterval = 0;

char obstacle = HIGH;
char car_moving = 0;

// BLUETOOTH
static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;
char BT_string[200];

// DC MOTOR
float Leftinterval	= 0;
float Rightinterval = 0;

//IR parameter//
uint32_t RC_IR_RIGHT_VALUE, RC_IR_LEFT_VALUE;
PinName_t seqCHn[3] = {PC_0, PC_1, PC_5};
PinName_t SEVEN_SEGMENT_DECODER_PIN_ELE[8] = {PD_2, PC_11, PC_12, PC_10, PA_13, PA_14, PA_15, PB_7};
uint8_t seven_segment_decoder_state_ele[8] = {LOW};

uint32_t PRESSURE_VALUE;
uint8_t OverWeight = 0;

// ELEVATOR FSM
uint32_t CloseDoor		= HIGH;
uint32_t DoorState		= LOW;
uint32_t FloorState		= 1;
uint32_t MoveFloor[2]	= {0};

uint32_t state			= LOW;
uint32_t PlaceDetected	= LOW;

static int count_pressure = 0;

void change_state(void);
void state0(void);
void state1(void);
void state2(void);
void state3(void);
void state4(void);
void car_move(void);
void sevensegment_decoder_set(uint8_t  num);
void setup(void);

int main(void) {
	// Initialization --------------------------------------------------
	setup();

	// Infinite Loop ---------------------------------------------------
	while(1){};
}

void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){

		// 7 segment
		if(FloorState && !car_moving) {
			sevensegment_decoder_set(FloorState);
		}
		else {
			seven_segment_decoder_state_ele[0] = HIGH;
			seven_segment_decoder_state_ele[1] = LOW;
			seven_segment_decoder_state_ele[2] = LOW;
			seven_segment_decoder_state_ele[3] = LOW;
			seven_segment_decoder_state_ele[4] = LOW;
			seven_segment_decoder_state_ele[5] = LOW;
			seven_segment_decoder_state_ele[6] = LOW;
			seven_segment_decoder_state_ele[7] = LOW;

			if(ELEV_dir_value == LOW) { // DOWN
				seven_segment_decoder_state_ele[5] = HIGH;
			}
			if(ELEV_dir_value == HIGH) { // UP
				seven_segment_decoder_state_ele[6] = HIGH;
			}

			for(int i = 0; i < 8; i++) {
				GPIO_write(SEVEN_SEGMENT_DECODER_PIN_ELE[i], seven_segment_decoder_state_ele[i]);
			}
		}

		// If weight is over. Caution!!!
		if(PRESSURE_VALUE > 700) {
			GPIO_write(ALERT,1);
			OverWeight = 1;
		}
		else {
			GPIO_write(ALERT,0);
			OverWeight = 0;
		}

		// ELEVATOR FSM CONTROL TOWER
		change_state();

		car_move();

		// if UPPER_IR is detected input data is '0'
		UPPER_IR_value = GPIO_read(UPPER_IR_PIN);
		LOWER_IR_value = !GPIO_read(LOWER_IR_PIN);

		FloorState = 0;

		if(UPPER_IR_value == LOW && ELEV_dir_value == HIGH) {
			updown_flag = 0;
			FloorState = 3;
		}

		if(LOWER_IR_value == LOW && ELEV_dir_value == LOW) {
			updown_flag = 0;
			FloorState = 1;
		}

		if (updown_flag == 0) {
			GPIO_write(ELEV_DIR_PIN, ELEV_dir_value);
			PWM_duty(ELEV_PWM_PIN,  ELEV_dir_value);
		}
		else PWM_duty(ELEV_PWM_PIN,  ELEV_duty_value);

		clear_UIF(TIM2); 		// Clear UI flag by writing 0
	}
}

// 차량 정지 여부
//Timer Interrupt for INPUT CAPTURE ULTRA SONIC
void TIM4_IRQHandler(void) {
	if(is_UIF(TIM4)){													// Update interrupt
		ovf_cnt++;														// overflow count
		clear_UIF(TIM4);  												// clear update interrupt flag
	}

	if(is_CCIF(TIM4, 1)){ 										// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1 = ICAP_capture(TIM4,1);								// Capture TimeStart
		clear_CCIF(TIM4, 1);										// clear capture/compare interrupt flag
	}
	else if(is_CCIF(TIM4,2)){ 									// TIM4_Ch1 (IC2) Capture Flag. Falling Edge Detect
		time2 = ICAP_capture(TIM4,2);								// Capture TimeEnd
		timeInterval = ((time2-time1)+ovf_cnt*(TIM4->ARR+1.0))*0.01; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		ovf_cnt = 0;													// overflow reset
		clear_CCIF(TIM4,2);										// clear capture/compare interrupt flag
	}
	distance = (float)timeInterval * 340.0 / 2.0 / 10.0;

	if(distance < 30.0) obstacle = LOW;
	else obstacle = HIGH;
}

// 문 닫힘 DoorClosed = HIGH
// 문 열림 DoorClosed = LOW
// Servo Motor Operating for Door open ELE
void TIM1_BRK_TIM9_IRQHandler(void){
	static uint32_t count = 0;

	if(is_UIF(TIM9)) {
		PWM_duty(DOOR_MOTOR, (float)((0.5+(1.0/60.0)*count)/10.0));

		if (CloseDoor && count > 0) count--;
		else if (!CloseDoor && count < 50) count++;

		if(count <= 0) {
			DoorState = 1;
		}
		else if(count >= 50) {
			DoorState = 0;
		}
 		else {
 			DoorState = 2;
 		}

		clear_UIF(TIM9); 		// Clear UI flag by writing 0
	}
}

// 내부 버튼
void EXTI15_10_IRQHandler(void) {
	if(is_pending_EXTI(OPEN_BUTTON)) {
		if(state != 1 && state != 2 && !car_moving) {
			CloseDoor = 0;

			state = 3;
		}

		clear_pending_EXTI(OPEN_BUTTON);
	}
	else if(is_pending_EXTI(CLOSE_BUTTON)) {
		if(state != 1 && state != 2 && !car_moving) {
			CloseDoor = 1;

			state = 4;
		}

		clear_pending_EXTI(CLOSE_BUTTON);
	}
	else if(is_pending_EXTI(FLOOR1_BUTTON)) {
		if(FloorState != 1 && !car_moving) {
			MoveFloor[0] = 1;

			// 우선순위 지정
			if(MoveFloor[1]) MoveFloor[0]++;
		}

		clear_pending_EXTI(FLOOR1_BUTTON);
	}
	else if(is_pending_EXTI(FLOOR3_BUTTON)) {
		if(FloorState != 3 && !car_moving) {
			MoveFloor[1] = 1;

			// 우선순위 지정
			if(MoveFloor[0]) MoveFloor[1]++;
		}

		clear_pending_EXTI(FLOOR3_BUTTON);
	}
}

// 외부 버튼
void USART1_IRQHandler(){
	if(is_USART1_RXNE()){
		BT_Data = USART1_read();

		if(BT_Data == 1) { // 1 -> 3

			if(FloorState != 1 && state != 1 && state != 2) {
				MoveFloor[0] = 1;

				// 우선순위 지정
				if(MoveFloor[1]) MoveFloor[0]++;
			}
			else if(FloorState == 1 && state != 1 && state != 2) {
				state = 3;
			}
		}
		else if(BT_Data == 2) { // 3 -> 1

			if(FloorState != 3 && state != 1 && state != 2) {
				MoveFloor[1] = 1;

				// 우선순위 지정
				if(MoveFloor[0]) MoveFloor[1]++;
			}
			else if(FloorState == 3 && state != 1 && state != 2) {
				state = 3;
			}
		}
	}
}

// 차량 조향
// RC CAR IR VALUE
void ADC_IRQHandler(void){
	if(is_ADC_OVR())
		clear_ADC_OVR();

	if(is_ADC_JEOC()){		// after finishing sequence
		RC_IR_RIGHT_VALUE	= JADC_read(1);
		RC_IR_LEFT_VALUE	= JADC_read(2);
		PRESSURE_VALUE		= JADC_read(3);

		if(RC_IR_RIGHT_VALUE > 1500 && RC_IR_LEFT_VALUE > 1500) { // stop
			Leftinterval	= 1;
			Rightinterval	= 1;
			if(FloorState)
				car_moving = 0;
			else car_moving = 1;
		}
		else if(RC_IR_RIGHT_VALUE > 1500 && RC_IR_LEFT_VALUE < 1500) {// turn right
			Leftinterval	= 0;
			Rightinterval	= 1;
			car_moving		= 1;
		}
		else if(RC_IR_RIGHT_VALUE < 1500 && RC_IR_LEFT_VALUE > 1500) { // turn left
			Leftinterval	= 1;
			Rightinterval	= 0;
			car_moving		= 1;
		}
		else { // go straight
			Leftinterval	= 1;
			Rightinterval	= 1;
			car_moving		= 1;
		}

		clear_ADC_JEOC();
	}
}

// state set
void change_state(void) {

	switch(state) {
		case 0: // stop state
			state0();

		break;
		case 1: // move to 3 floor
			state1();

		break;
		case 2: // move to 1 floor
			state2();

		break;
		case 3: // open door
			state3();

		break;
		case 4: // close door
			state4();

		break;
		default:
			break;
	}
}

// stop elevator
void state0(void) {
	CloseDoor = 1; //닫힘
	updown_flag = 0;

	state = 0;
	if(MoveFloor[0] == 1 && MoveFloor[1] == 2) {
		MoveFloor[0] = 0;
		MoveFloor[1] = 1;
		state = 2;
	}
	else if(MoveFloor[0] == 1 && MoveFloor[1] == 0) {
		MoveFloor[0] = 0;
		MoveFloor[1] = 0;
		state = 2;
	}
	else if(MoveFloor[1] == 1 && MoveFloor[0] == 2) {
		MoveFloor[0] = 1;
		MoveFloor[1] = 0;
		state = 1;
	}
	else if(MoveFloor[1] == 1 && MoveFloor[0] == 0) {
		MoveFloor[0] = 0;
		MoveFloor[1] = 0;
		state = 1;
	}
}

// move elevator to the 3 floor
void state1(void) {
	CloseDoor = 1;
	ELEV_dir_value = HIGH;
	ELEV_duty_value = 0.5;
	updown_flag = 0;

	GPIO_write(ELEV_DIR_PIN, ELEV_dir_value);
	if(DoorState == 1) updown_flag = 1;
	if(FloorState == 3 && !car_moving) state = 3;
}

// move elevator to the 1 floor
void state2(void) {
	CloseDoor = 1;
	ELEV_dir_value = LOW;
	ELEV_duty_value = 0.5;
	updown_flag = 0;

	GPIO_write(ELEV_DIR_PIN, ELEV_dir_value);
	if(DoorState == 1) updown_flag = 1;
	if(FloorState == 1 && !car_moving) state = 3;
}

// open elevator door
void state3(void) {
	CloseDoor = 0;
	updown_flag = 0;

	if(OverWeight) GPIO_write(ALERT, HIGH);
	else GPIO_write(ALERT, LOW);

	count_pressure ++;

	if(!OverWeight && DoorState == 0 && count_pressure > 5000) {
		state = 4;
	}

	if(count_pressure > 5000) count_pressure = 0;
}

// close elevator door
void state4(void) {
	updown_flag = 0;
	CloseDoor = 1;

	if(OverWeight) {
		state = 3;
	}

	if(DoorState == 1) {
		delay_ms(2000);
		state = 0;
	}
}

// 차량 정지 car_moving = LOW
// 차량 동작 car_moving = HIGH
void car_move(void) {
	PWM_duty(RC_LEFT_PWM, 1 - ( 0.7 * Rightinterval) * obstacle * car_moving);
	PWM_duty(RC_RIGHT_PWM, 1 - ( 0.7 * Leftinterval) * obstacle * car_moving);
}

void sevensegment_decoder_set(uint8_t  num) {
	int seven_segment_state[4] = {0};
	int A = 0, B = 0, C = 0, D = 0;

	for(int i = 0; i < 4; i++) {
		seven_segment_state[i] = (num >> i) & ( 1 );
	}

	A = seven_segment_state[0];
	B = seven_segment_state[1];
	C = seven_segment_state[2];
	D = seven_segment_state[3];

	seven_segment_decoder_state_ele[0] = (~D & ~C & ~B & A) | (~D & C & ~A);
	seven_segment_decoder_state_ele[1] = (C & ~B & A) + (~D & C & B & ~A);
	seven_segment_decoder_state_ele[2] = ~D & ~C & B & ~A;
	seven_segment_decoder_state_ele[3] = (~C & ~B & A) | (~D & C & ~B & ~A) | (C & B & A) | (D & C & B);
	seven_segment_decoder_state_ele[4] = A | (C & ~B) | (D & B);
	seven_segment_decoder_state_ele[5] = (~D & ~C & A) | (~D & ~C & B) | (B & A) | (D & C);
	seven_segment_decoder_state_ele[6] = (~D & ~C & ~B) | (C & B & A) | (D & C & B);


	for(int i = 0; i < 8; i++) {
		GPIO_write(SEVEN_SEGMENT_DECODER_PIN_ELE[i], seven_segment_decoder_state_ele[i]);
	}
}

// Setting setup
void setup(void) {
	RCC_PLL_init();				// System Clock Initiallization
    SysTick_init();

	UART1_init();
	UART1_baud(BAUD_9600);

	// ADC Init  Default: HW triggered by TIM3 counter @ 1msec
	JADC_init(PC_0);
	JADC_init(PC_1);
	JADC_init(PC_5);

	// ADC channel sequence setting
	JADC_sequence(seqCHn, 3);

	GPIO_init(ALERT,OUTPUT);

	// Timer Update Interrupt
	// TIM3: set Update interrupt by Timer 3
	// 500: timeperiod = 500m second
	TIM_UI_init(TIM2, 1);
    TIM_UI_init(TIM9, 30);

	EXTI_init(OPEN_BUTTON, FALL,1);
	EXTI_init(CLOSE_BUTTON, FALL,1);
	EXTI_init(FLOOR1_BUTTON, FALL,1);
	EXTI_init(FLOOR3_BUTTON, FALL,1);
	GPIO_init(OPEN_BUTTON, INPUT);
	GPIO_init(CLOSE_BUTTON, INPUT);
	GPIO_init(FLOOR1_BUTTON, INPUT);
	GPIO_init(FLOOR3_BUTTON, INPUT);
	GPIO_pupd(OPEN_BUTTON, PULL_UP);
	GPIO_pupd(CLOSE_BUTTON,  PULL_UP);
	GPIO_pupd(FLOOR1_BUTTON, PULL_UP);
	GPIO_pupd(FLOOR3_BUTTON, PULL_UP);

	GPIO_init(UPPER_IR_PIN, INPUT);
	GPIO_init(LOWER_IR_PIN, INPUT);

	GPIO_init(RC_LEFT_DIR, OUTPUT);
	GPIO_init(RC_RIGHT_DIR, OUTPUT);
	GPIO_otype(RC_LEFT_DIR, OPEN_DRAIN);
	GPIO_otype(RC_RIGHT_DIR,OPEN_DRAIN);
	GPIO_write(RC_LEFT_DIR, 1);
	GPIO_write(RC_RIGHT_DIR, 1);


	// RC_RIGHT_PWM of 1 msec: TIM2_CH1 (PA_1 AFmode)
	// RC_LEFT_PWM of 1 msec:  TIM2_CH2 (PA_0 AFmode)
	PWM_init(RC_LEFT_PWM);
	PWM_init(RC_RIGHT_PWM);
	PWM_period(RC_LEFT_PWM, LOW);
	PWM_period(RC_RIGHT_PWM, LOW);

	/* INPUT CAPTURE FOR ULTRA SONIC */
	// PWM Trigger configuration ---------------------------------------------------------------------
	PWM_init(ULTRASONIC_TRIG);								// PA_6: Ultrasonic trig pulse
	PWM_period_us(ULTRASONIC_TRIG, 1000);				// PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(ULTRASONIC_TRIG, 10);		// PWM pulse width of 10us

	// Input Capture configuration -----------------------------------------------------------------------
	ICAP_init(ULTRASONIC_ECHO);    							// PB_6 as input caputre
	ICAP_counter_us(ULTRASONIC_ECHO, 10);   				// ICAP counter step time as 10us
	ICAP_setup(ULTRASONIC_ECHO, 1, IC_RISE);		// TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ULTRASONIC_ECHO, 2, IC_FALL);		// TIM4_CH2 as IC2 , falling edge detect

	// DOOR_MOTOR PWM of 1 msec:  TIM3_CH4 (PC_9 AFmode)
	PWM_init(DOOR_MOTOR);
	PWM_period(DOOR_MOTOR, 10);   // 10 msec PWM period

	GPIO_init(ELEV_DIR_PIN, OUTPUT);
	GPIO_otype(ELEV_DIR_PIN, PUSH_PULL);
	GPIO_write(ELEV_DIR_PIN, ELEV_dir_value);

	PWM_init(ELEV_PWM_PIN);
	PWM_duty(ELEV_PWM_PIN,  0.0);

	for(int i = 0; i < 8; i++){
		GPIO_init(SEVEN_SEGMENT_DECODER_PIN_ELE[i], OUTPUT);
		GPIO_otype(SEVEN_SEGMENT_DECODER_PIN_ELE[i], PUSH_PULL);
		GPIO_pupd(SEVEN_SEGMENT_DECODER_PIN_ELE[i], NO_PUPD);
		GPIO_ospeed(SEVEN_SEGMENT_DECODER_PIN_ELE[i], MEDIUM_SPEED);

		GPIO_write(SEVEN_SEGMENT_DECODER_PIN_ELE[i], HIGH);
	}
}