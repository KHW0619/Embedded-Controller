/**
******************************************************************************
* @author  Jong-Hyeon Kim / Mu-Yeol HWANG
* @Mod	   2024-11-12
* @brief   PROJECT_RC_CAR
******************************************************************************
*/

#include "stm32f411xe.h"
#include "math.h"

#include "ecSTM32F4v2.h"


// Definition Button Pin & PWM Port, Pin
#define LEFT_DIR_PIN 	PA_11
#define LEFT_PWM_PIN 	PA_0  // TIM2 CH1
#define RIGHT_DIR_PIN 	PA_12
#define RIGHT_PWM_PIN 	PA_1  // TIM2 CH2
#define IR_LEFT 		PC_1
#define IR_RIGHT 		PC_0
#define TRIG 			PA_6  // TIM3 CH1
#define ECHO 			PB_6
#define LED_PIN 		PA_5

//#define PRESSURE 		PC_2
#define RC_SERVO 		PC_9
#define BUTTON_PIN 		PC_13//Find BTN Port&Pin and Fill the blank


uint32_t count = 0;
uint32_t OPEN = 0;

// ULTRA SONIC
uint32_t ovf_cnt = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;
float prevDist=1000;
char ctr = 1;

// BLUETOOTH
static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;
char BT_string[100];

// DC MOTOR
char DIR = 0;
float Lduty;
float Rduty;
float MRduty=0;
float MLduty=0;
float Leftinterval = 0;
float Rightinterval = 0;

//IR parameter//
uint32_t Rightvalue1, Leftvalue2;
PinName_t seqCHn[2] = {PC_0, PC_1};
float distance=0;


// State
char SPD_cnt=0;
char ctrM=0;
int STR_cnt=0;
char manual=0;
uint32_t autoCtr=0;
float dutySpd=0;
char Go_flg=0;
char STR_flg=0;
char prtDIR;
char prtMode='M';
int prtSTR=0;
char prtVEL=0;



void setup(void);

int main(void) {
	// Initialization --------------------------------------------------
	setup();


	// Infinite Loop ---------------------------------------------------
	while(1){

		if (DIR) {
			prtDIR = 'F';
		} else prtDIR = 'B';

		if(autoCtr) {
			prtMode='A';
			prtVEL=3;
			if(Leftinterval) prtSTR=3;
			else if(Rightinterval) prtSTR=-3;
			else prtSTR=0;
		}
		else {
			prtMode='M';
			prtSTR=STR_cnt;
			prtVEL=SPD_cnt;
			if(Go_flg) prtSTR=0;

		}
		if ((ctr && ctrM)==0) {
			prtVEL=0;
			prtSTR=0;
		}
		sprintf(BT_string, "MOD: %c DIR: %c STR: %d VEL: %d \r\n", prtMode, prtDIR, prtSTR, prtVEL);
		USART1_write(BT_string, 100);

		delay_ms(1000);
	}
}

/* Timer3 interrupt ISR
	* With the DIR = 0, it is depends on PWM. For the 25%, 75% cycle, set the interval to change the ratio values.(25%->75%->25%...)
Timer3 has 500ms period. So, for the 2 second, we can make 500ms*4 = 2 second by if (count==4) and reset the counter to 0.
*/
void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){			// Check UIF(update interrupt flag)
		if(autoCtr) {
			float LtargetPWM =  (0+ 0.4*Rightinterval); // pwm for motor input
			float RtargetPWM =  (0+ 0.4*Leftinterval);

			PWM_duty(LEFT_PWM_PIN, ctr*LtargetPWM);
			PWM_duty(RIGHT_PWM_PIN, ctr*RtargetPWM);
		}
		else {
			float MLtarget = STR_flg*MLduty*dutySpd + Go_flg*dutySpd;
			float MRtarget = STR_flg*MRduty*dutySpd + Go_flg*dutySpd;

			Lduty=fabs(DIR-MLtarget); // duty with consideration of DIR=1 or 0
			Rduty=fabs(DIR-MRtarget);

			PWM_duty(LEFT_PWM_PIN, ctrM*Lduty);
			PWM_duty(RIGHT_PWM_PIN, ctrM*Rduty);
		}

		//GPIO_write(LEFT_DIR_PIN,ctr*ctrM*DIR);
		//GPIO_write(RIGHT_DIR_PIN,ctr*ctrM*DIR);

		GPIO_write(LEFT_DIR_PIN,ctrM*DIR);
		GPIO_write(RIGHT_DIR_PIN,ctrM*DIR);

		clear_UIF(TIM2); 		// Clear UI flag by writing 0
	}
}

void TIM1_BRK_TIM9_IRQHandler(void){
	if(is_UIF(TIM9)) {
		// Check UIF(update interrupt flag)
		//PWM_duty(PWM_PIN, (float)0.8);
		PWM_duty(RC_SERVO, (float)((0.5+(1.0/60.0)*count)/10.0));

		if (OPEN==0) {
			if(count>0)
				count--;
			//if(count==0)
			//PWM_duty(PWM_PIN, (float)(0.05));
		}
		if (OPEN==1) {
			if (count<50)
				count++;
			//if(count==50)
			//	PWM_duty(PWM_PIN, (float)(0.15));
		}

		//GPIO_write(LED_PIN,1);
		//GPIOA->ODR ^= (1UL << LED_PIN);

		clear_UIF(TIM9); 		// Clear UI flag by writing 0
	}
}



//Timer Interrupt for INPUT CAPTURE ULTRA SONIC
void TIM4_IRQHandler(void) {
	if(is_UIF(TIM4)){                     // Update interrupt
		ovf_cnt ++;													// overflow count
		clear_UIF(TIM4);  							    // clear update interrupt flag
	}
	if(is_CCIF(TIM4, 1)){ 								// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1 = ICAP_capture(TIM4,1);				// Capture TimeStart
		clear_CCIF(TIM4, 1);                // clear capture/compare interrupt flag
	}
	else if(is_CCIF(TIM4,2)){ 						// TIM4_Ch1 (IC2) Capture Flag. Falling Edge Detect
		time2 = ICAP_capture(TIM4,2);				// Capture TimeEnd
		timeInterval = ((time2-time1)+ovf_cnt*(TIM4->ARR+1.0))*0.01; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		ovf_cnt = 0;                        // overflow reset
		clear_CCIF(TIM4,2);								  // clear capture/compare interrupt flag
	}
	distance =  (float)timeInterval * 340.0 / 2.0 / 10.0;

	if(distance < 30.0) ctr=0;
	else ctr = 1;
}


void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		OPEN=!OPEN;
		clear_pending_EXTI(BUTTON_PIN); // Clear EXTI flag
	}
}


void ADC_IRQHandler(void){
	if(is_ADC_OVR())
		clear_ADC_OVR();

	if(is_ADC_JEOC()){		// after finishing sequence
		Rightvalue1 = JADC_read(1);
		Leftvalue2 = JADC_read(2);

		if(Rightvalue1 > 1500) Leftinterval=1;
		else Leftinterval=0;

		if(Leftvalue2 > 1500) Rightinterval=1;
		else Rightinterval=0;

		clear_ADC_JEOC();
	}
}

// Setting setup
void setup(void) {
	RCC_PLL_init();				// System Clock Initiallization
    SysTick_init();

	UART1_init();
	UART1_baud(BAUD_9600);
	UART2_init();			// UART2 Init

	// ADC Init  Default: HW triggered by TIM3 counter @ 1msec
	JADC_init(PC_0);
	JADC_init(PC_1);

	// ADC channel sequence setting
	JADC_sequence(seqCHn, 2);

	GPIO_init(LED_PIN,OUTPUT);
	GPIO_otype(LED_PIN,PUSH_PULL);

/* Timer Update Interrupt
		*TIM3: set Update interrupt by Timer 3
		*500: timeperiod = 500m second
*/
	TIM_UI_init(TIM2, 10);
    TIM_UI_init(TIM9, 30);

	EXTI_init(BUTTON_PIN, FALL,1);


	GPIO_init(BUTTON_PIN, INPUT);
	GPIO_pupd(BUTTON_PIN, PULL_UP);


	GPIO_init(LEFT_DIR_PIN, OUTPUT);
	GPIO_init(RIGHT_DIR_PIN, OUTPUT);
	GPIO_otype(LEFT_DIR_PIN, OPEN_DRAIN);
	GPIO_otype(RIGHT_DIR_PIN, OPEN_DRAIN);

	// PWM of 20 msec:  TIM2_CH1 (PA_1 AFmode)
	PWM_init(LEFT_PWM_PIN);
	PWM_init(RIGHT_PWM_PIN);
	PWM_period(LEFT_PWM_PIN, 1);   // 1 msec PWM period
	PWM_period(RIGHT_PWM_PIN, 1);


	/* INPUT CAPTURE FOR ULTRA SONIC */
	// PWM Trigger configuration ---------------------------------------------------------------------
	PWM_init(TRIG);								 // PA_6: Ultrasonic trig pulse
	PWM_period_us(TRIG, 1000);    // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG, 10);   // PWM pulse width of 10us

	// Input Capture configuration -----------------------------------------------------------------------
	ICAP_init(ECHO);    					 // PB_6 as input caputre
	ICAP_counter_us(ECHO, 10);   	 // ICAP counter step time as 10us
	ICAP_setup(ECHO, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect

	// PWM of 20 msec:  TIM2_CH1 (PA_1 AFmode)
	PWM_init(RC_SERVO);
	PWM_period(RC_SERVO, 10);   // 20 msec PWM period

}




void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	static uint8_t k = 0;
	if(is_USART1_RXNE()){
		BT_Data = USART1_read();		// RX from UART1 (BT)
		//printf("RX: %c \r\n",BT_Data); // TX to USART2(PC)
		if(BT_Data == 27)                  // if ch is the escape sequence with num code 27, k turns 1 to signal the next
			k = 1;
		if(BT_Data == 91 && k == 1)       // if the previous char was 27, and the current 91, k turns 2 for further use
			k = 2;
		if(BT_Data == 65 && k == 2) {
			ctrM=1;
			if(SPD_cnt<3)
				SPD_cnt++;
		}// finally, if the last char of the sequence matches, you've got a key !
		if(BT_Data == 66 && k == 2) {
			ctrM=1;
			if(SPD_cnt>0)
				SPD_cnt--;
		}
		if(BT_Data == 67 && k == 2) {
			ctrM=1;
			STR_flg=1;
			Go_flg=0;
			if(STR_cnt<3)
				STR_cnt++;
		}

		if(BT_Data == 68 && k == 2) {
			ctrM=1;
			STR_flg=1;
			Go_flg=0;
			if(STR_cnt>-3)
				STR_cnt--;
		}
		if(BT_Data != 27 && BT_Data != 91)      // if ch isn't either of the two, the key pressed isn't up/down so reset k
			k = 0;

		switch(BT_Data) {
			case 's':
				ctrM=0;
			break;
			case 'f':
				ctrM=1;
				STR_flg=0;
                Go_flg=1;
				DIR=1;
			break;
			case 'b':
				ctrM=1;
				STR_flg=0;
                Go_flg=1;
				DIR=0;
			break;
			case 'm':
				GPIO_write(LED_PIN,1);
				ctrM=1;
				manual=1;
				autoCtr=0;
				DIR=0;
				STR_flg=0;
				STR_cnt=0;
				SPD_cnt=0;
			break;
			case 'a':
				GPIO_write(LED_PIN,0);
				ctrM=1;
				manual=0;
				autoCtr=1;
				DIR=1;
			break;
		}

		switch(SPD_cnt) {
			case 0:
				dutySpd=0;
			break;
			case 1:
				dutySpd=0.3;
			break;
			case 2:
				dutySpd=0.7;
			break;
			case 3:
				dutySpd=1;
			break;
		}

		switch(STR_cnt) {
			case -3:
				MLduty=0.0; MRduty=1.0;
			break;
			case -2:
				MLduty=0.4; MRduty=1.0;
			break;
			case -1:
				MLduty=0.7; MRduty=1.0;
			break;
			case 0:
				MLduty=0,0; MRduty=0.0;
			break;
			case 1:
				MLduty=1.0; MRduty=0.7;
			break;
			case 2:
				MLduty=1.0; MRduty=0.4;
			break;
			case 3:
				MLduty=1.0; MRduty=0.0;
			break;
		}
	}
}
