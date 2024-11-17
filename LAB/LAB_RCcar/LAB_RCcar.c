/*
******************************************************************************
* @author  Yuha Park && Heewon Kim
* @created 2024.11.05
* @Mod     2024.11-07 by Yuha Park && Heewon Kim
* @brief   Embedded Controller: LAB_RCcar
******************************************************************************
*/

#include "ecSTM32F4v2.h"
#include "math.h"
#include "ecRCmotor.h"

// DC motor-----------------------------------------------------
PinName_t PWM_PIN[2] = {PA_0, PA_6}; // TIM2, TIM3
PinName_t DIR_PIN[2] = {PC_2, PC_3};
float PWM_value[2] = { 1.0, 0.0 };
uint32_t DIR_value = 1;  //1 or 0

volatile uint8_t RC_mode = 1;
volatile uint8_t RC_stop_flag = 0;

// IR------------------------------------------------------------
PinName_t IR_PIN[2] = {PB_0, PB_1};
uint32_t IR_value[2] = { 0, 0 };

//Input Capture-Ultrasonic---------------------------------------
float ICAP_time1 = 0;
float ICAP_time2 = 0;
float ICAP_distance = 0;
float ICAP_timeInterval = 0;
uint32_t ICAP_ovf_cnt = 0;

//want to STOP!
float distance_buffer[DISTANCE_BUFFER_SIZE] = {0};
int buffer_index = 0;
int ICAP_Large_State = 0;
int ICAP_Small_State = 0;

// USART - bluetooth--------------------------------------------------
static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;

int velocity_state = 0;
//int SteerState[7] = {-3, -2, -1, 0 ,1, 2, 3};
float PWM_SteerState[7][2] = {{0.4, 1.0}, {0.4, 0.8}, {0.2, 0.4}, {0.0, 0.0}, {0.4, 0.2}, {0.8, 0.4}, {1.0, 0.4}};
int PreessedNum = STEER_INIT;
int ICAP_stop_flag = 0;

//int mode = 0;                   // 0: Auto Mode, 1: Manual Mode
//int speed_level = 0;          // Speed levels: V0, V1, V2, V3
//int steer_angle = 0;          // Steer angles from -3 to 3

/*------------Print State-------------*/
char BT_string[50]={0};

char STR_Print = '0';
char STR_Print_Func(void){
   if(PreessedNum < STEER_INIT) STR_Print = 'L';
   else if(PreessedNum > STEER_INIT) STR_Print = 'R';
   else STR_Print = 'C';
   return STR_Print;
}
/*-----------End Print State-----------*/

	
void setup(void);
void control_vehicle(char cmd);

int main(void){

    setup();
    while(1){
			 STR_Print_Func();
			 sprintf(BT_string, "MOD: %d, DIR:%d, STR: %c, VEL: %d\r\n", RC_mode, DIR_value, STR_Print, velocity_state);  
			 USART1_write(BT_string, 50);
       delay_ms(1000);
			 
       if (RC_mode) GPIO_write(LED_PIN, HIGH);   // LED ON in Manual Mode
       else LED_toggle();                                    // LED toggle in Auto Mode
    }
}

void control_vehicle(char cmd) {
   switch (cmd) {
      case MANUAL_MODE:
      case MANUAL_MODE + ALP_SMALL:
         RC_mode = 1;
      break;

      case AUTO_MODE:
      case AUTO_MODE + ALP_SMALL:
         RC_mode = 0;
      break;

      case SPEED_INCREASE:
      case SPEED_INCREASE + ALP_SMALL:
         velocity_state++;
         if (velocity_state > MAX_SPEED) velocity_state = MAX_SPEED;
         PWM_value[LEFT]  = fabs(DIR_value - velocity_state * VELOCITY_RATIO);
         PWM_value[RIGHT] = fabs((1-DIR_value)- velocity_state * VELOCITY_RATIO);
      break;

      case SPEED_DECREASE:
      case SPEED_DECREASE + ALP_SMALL:
         velocity_state--;
         if (velocity_state < MIN_SPEED) velocity_state = MIN_SPEED;
         PWM_value[LEFT]  = fabs(DIR_value - velocity_state * VELOCITY_RATIO);
         PWM_value[RIGHT] = fabs((1-DIR_value) - velocity_state * VELOCITY_RATIO);
      break;

      case STEER_RIGHT:
      case STEER_RIGHT + ALP_SMALL:
         if(PreessedNum < 6) PreessedNum++;
         PWM_value[LEFT] = fabs(DIR_value - PWM_SteerState[PreessedNum][LEFT]);
         PWM_value[RIGHT] = fabs((1-DIR_value)- PWM_SteerState[PreessedNum][RIGHT]);

         if(PreessedNum == STEER_INIT) {
            PWM_value[LEFT]  = fabs(DIR_value - velocity_state * VELOCITY_RATIO);
            PWM_value[RIGHT] = fabs((1-DIR_value) - velocity_state * VELOCITY_RATIO);
         }
      break;

      case STEER_LEFT:
      case STEER_LEFT + ALP_SMALL:
         if(PreessedNum > 0) PreessedNum--;
         PWM_value[LEFT] = fabs(DIR_value - PWM_SteerState[PreessedNum][LEFT]);
         PWM_value[RIGHT] = fabs((1-DIR_value)-PWM_SteerState[PreessedNum][RIGHT]);

         if(PreessedNum == STEER_INIT) {
            PWM_value[LEFT]  = fabs(DIR_value - velocity_state * VELOCITY_RATIO);
            PWM_value[RIGHT] = fabs((1-DIR_value) - velocity_state * VELOCITY_RATIO);
         }
      break;

      case FORWARD:
      case FORWARD + ALP_SMALL:
					
					DIR_value = 1;
					velocity_state = 2;

					if(PreessedNum == STEER_INIT) {
            PWM_value[LEFT]  = fabs(DIR_value - velocity_state * VELOCITY_RATIO);
            PWM_value[RIGHT] = fabs((1-DIR_value) - velocity_state * VELOCITY_RATIO);
          }
					else{
						PWM_value[LEFT] = fabs(DIR_value - PWM_value[LEFT]);
						PWM_value[RIGHT] = fabs(DIR_value - PWM_value[RIGHT]);
					}
      break;

      case BACKWARD:
      case BACKWARD + ALP_SMALL:
					DIR_value = 0;
					velocity_state = 2;

					if(PreessedNum == STEER_INIT) {
            PWM_value[LEFT]  = fabs(DIR_value - velocity_state * VELOCITY_RATIO);
            PWM_value[RIGHT] = fabs((1-DIR_value) - velocity_state * VELOCITY_RATIO);
          }
					else{
						PWM_value[LEFT] = fabs((1-DIR_value) - PWM_value[LEFT]);
						PWM_value[RIGHT] = fabs((1-DIR_value) - PWM_value[RIGHT]);
					}
      break;

      case EMERGENCY_STOP: // Emergency Stop
      case EMERGENCY_STOP + ALP_SMALL:
         PWM_value[LEFT] = fabs((1-DIR_value) - 1);
         PWM_value[RIGHT] = fabs((1-DIR_value)- 0);
      break;

      default:
      break;
   }

   GPIO_write(DIR_PIN[LEFT], DIR_value);
   GPIO_write(DIR_PIN[RIGHT], DIR_value^1);
   PWM_duty(PWM_PIN[LEFT], PWM_value[LEFT]);
   PWM_duty(PWM_PIN[RIGHT], PWM_value[RIGHT]);
}

void USART1_IRQHandler(){                // USART2 RX Interrupt : Recommended
    static int specific_key_state = 0;
    int specific_key = 0;
    if(is_USART1_RXNE()){
        BT_Data = USART1_read();      // RX from UART1 (BT)
        USART1_write(&BT_Data, 1);

        if(BT_Data == 27) specific_key_state = 1;
        else if(specific_key_state == 1) {
            if(BT_Data == 91) specific_key_state = 2;
            else specific_key_state = 0;
        }
        else if(specific_key_state == 2) {
            switch(BT_Data) {
                case SPECIFIC_KEY_A:
                   specific_key = 25;
                break;
                case SPECIFIC_KEY_B:
                   specific_key = 8;
                break;
                case SPECIFIC_KEY_C:
                    specific_key = 22;
                break;
                case SPECIFIC_KEY_D:
                    specific_key = 16;
                break;
                default:
                    specific_key_state = -1;
                break;
            }

            if(specific_key_state != -1) {
               BT_Data = 60 + specific_key;
               control_vehicle(BT_Data);
               specific_key_state = 0;
            }
        }
        else if(!specific_key_state) control_vehicle(BT_Data);
    }
}

void ADC_IRQHandler(void){
   if(is_ADC_OVR())
      clear_ADC_OVR();

   if(is_ADC_JEOC()){      // after finishing sequence
      IR_value[0] = JADC_read(1);
      IR_value[1] = JADC_read(2);
      clear_ADC_JEOC();
   }

   if(!RC_mode && !ICAP_stop_flag) {
      DIR_value = 1;

      if(IR_value[LEFT] < IR_THRESHOLD && IR_value[RIGHT] < IR_THRESHOLD) {            // straight
				 PreessedNum = 3;
				 velocity_state = 5;
         PWM_value[LEFT]  = DIR_value - AUTO_STRAIGHT * VELOCITY_RATIO;
         PWM_value[RIGHT] = AUTO_STRAIGHT * VELOCITY_RATIO;
      }
      else if(IR_value[LEFT] > IR_THRESHOLD && IR_value[RIGHT] < IR_THRESHOLD) {         // turn left
				 PreessedNum = 2;
         PWM_value[LEFT]  = DIR_value - PWM_SteerState[STEER_INIT - AUTO_STEER_LEFT][LEFT];
         PWM_value[RIGHT] = PWM_SteerState[STEER_INIT - AUTO_STEER_LEFT][RIGHT];
      }
      else if(IR_value[LEFT] < IR_THRESHOLD && IR_value[RIGHT] > IR_THRESHOLD){         // turn right
				 PreessedNum = 4;
         PWM_value[LEFT]  = DIR_value - PWM_SteerState[STEER_INIT + AUTO_STEER_RIGHT][LEFT];
         PWM_value[RIGHT] = PWM_SteerState[STEER_INIT + AUTO_STEER_RIGHT][RIGHT];
      }
      else {
         PWM_value[LEFT]  = fabs((1-DIR_value) - 1);
         PWM_value[RIGHT] = fabs((1-DIR_value) - 0);
      }
   }

   GPIO_write(DIR_PIN[LEFT], DIR_value);
   GPIO_write(DIR_PIN[RIGHT], DIR_value ^ 1);
   PWM_duty(PWM_PIN[LEFT], PWM_value[LEFT]);
   PWM_duty(PWM_PIN[RIGHT], PWM_value[RIGHT]);
}

void TIM4_IRQHandler(void){
   if(is_UIF(TIM4)){                                                            // Update interrupt
      ICAP_ovf_cnt++;                                                                     // overflow count
      clear_UIF(TIM4);                                                            // clear update interrupt flag
   }
   if(is_CCIF(TIM4, IC_1)){                                                   // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
      ICAP_time1 = ICAP_capture(TIM4, IC_1);                                          // Capture TimeStart
      clear_CCIF(TIM4, IC_1);                                                   // clear capture/compare interrupt flag
   }
   else if(is_CCIF(TIM4, IC_2)){                                                // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
      ICAP_time2 = ICAP_capture(TIM4, IC_2);                                          // Capture TimeEnd
      ICAP_timeInterval = ((ICAP_time2 - ICAP_time1) + ICAP_ovf_cnt * (TIM4->ARR + 1)) * 0.01;      // (10us * counter pulse -> [msec] unit) Total time of echo pulse
      ICAP_ovf_cnt = 0;                                                            // overflow reset
      clear_CCIF(TIM4,IC_2);                                                   // clear capture/compare interrupt flag
   }

   ICAP_distance = (float) ICAP_timeInterval * 340.0 / 2.0 / 10.0;

	 if (ICAP_distance > STOP_DISTANCE) {
       ICAP_Large_State++;
       ICAP_Small_State = 0;
   }
   else if (ICAP_distance <= STOP_DISTANCE) {
       ICAP_Small_State++;
       ICAP_Large_State = 0;
    }
	 
	 if (!RC_mode) { 
       if (ICAP_Small_State >= ICAP_MIN_STOP_NUM) { 
           DIR_value = 1;
           ICAP_stop_flag = 1;
           PWM_value[LEFT]  = fabs((1 - DIR_value) - 1);
           PWM_value[RIGHT] = fabs((1 - DIR_value) - 0);
       } 
       else if (ICAP_Large_State >= ICAP_MIN_STOP_NUM) { 
           ICAP_stop_flag = 0;
       }
   }
}

void USART2_IRQHandler(){               // USART2 RX Interrupt : Recommended
   if(is_USART2_RXNE()){
      PC_Data = USART2_read();            // RX from UART2 (PC)
      USART2_write(&PC_Data,1);      // TX to USART2    (PC)    Echo of keyboard typing
   }
}

void setup(void){
    RCC_PLL_init();
    SysTick_init();

   // ==================================================================================================================== //
   //                                                   Blue Tooth
   // ==================================================================================================================== //
    // USART2: USB serial init
    UART2_init();
    UART2_baud(BAUD_9600);

    // USART1: BT serial init
    UART1_init();
    UART1_baud(BAUD_9600);

   // ==================================================================================================================== //
    //                                                       IR
    // ==================================================================================================================== //
    // ADC Init  Default: HW triggered by TIM5 counter @ 1msec
    JADC_init(IR_PIN[0]);
    JADC_init(IR_PIN[1]);

    // ADC channel sequence setting
    JADC_sequence(IR_PIN, 2);

   // ==================================================================================================================== //
   //                                                      ICAP
   // ==================================================================================================================== //
    // PWM configuration
    PWM_init(TRIG);                        // PA_8: Ultrasonic trig pulse
    PWM_period_us(TRIG, 50000);            // PWM of 50ms period. Use period_us()
    PWM_pulsewidth_us(TRIG, 10);      // PWM pulse width of 10us

    // Input Capture configuration
    ICAP_init(ECHO);                        // PB_6 as input caputre
    ICAP_counter_us(ECHO, 10);            // ICAP counter step time as 10us
    ICAP_setup(ECHO, 1, IC_RISE);   // TIM4_CH1 as IC1 , rising edge detect
    ICAP_setup(ECHO, 2, IC_FALL);   // TIM4_CH2 as IC2 , falling edge detect

   // ==================================================================================================================== //
   //                                                     MOTOR
   // ==================================================================================================================== //
    for(int i = 0; i < 2; i++){
      GPIO_init(DIR_PIN[i], OUTPUT);
      GPIO_pupd(DIR_PIN[i], PUSH_PULL);

      PWM_init(PWM_PIN[i]);
      PWM_period_ms(PWM_PIN[i], 20);
    }

    GPIO_write(DIR_PIN[LEFT], DIR_value);       //SET DIR Pin Low(CW)
    GPIO_write(DIR_PIN[RIGHT], 1 - DIR_value);
    GPIO_init(PA_5, OUTPUT);

   PWM_duty(PWM_PIN[LEFT], 1);
   PWM_duty(PWM_PIN[RIGHT], 0);
}