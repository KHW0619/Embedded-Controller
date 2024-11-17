//
// Created by dadan on 24. 11. 7.
//

#ifndef ECRCCAR_H
#define ECRCCAR_H

// Define RC Car control modes and settings
#define MANUAL_MODE      'M'
#define AUTO_MODE      'A'
#define SPEED_INCREASE   'U'
#define SPEED_DECREASE   'D'
#define STEER_RIGHT      'R'
#define STEER_LEFT      'L'
#define FORWARD         'F'
#define BACKWARD      'B'
#define EMERGENCY_STOP   'S'
#define ALP_SMALL       (char) (32)

#define MODE_M 0
#define MODE_A 1

#define LEFT  0
#define RIGHT 1
#define LED_PIN PA_5

#define MAX_SPEED 4
#define MID_SPEED 2.5
#define MIN_SPEED 1

#define TRIG    PA_8 //TIM1
#define ECHO    PB_6 //TIM4
#define TXD_PIN PA_9
#define RXD_PIN PA_10

#define VELOCITY_RATIO 0.2
#define STEER_INIT (int)(3)

#define STOP_DISTANCE   (float)( 15.0 )
#define IR_THRESHOLD    (float)(1000.0)
#define DISTANCE_BUFFER_SIZE 5 

#define AUTO_STRAIGHT    (int)(5)
#define AUTO_STEER_LEFT  (int)(3)
#define AUTO_STEER_RIGHT (int)(3)

#define SPECIFIC_KEY_A 'A'
#define SPECIFIC_KEY_B 'B'
#define SPECIFIC_KEY_C 'C'
#define SPECIFIC_KEY_D 'D'

#define ICAP_MIN_STOP_NUM 5

#endif //ECRCCAR_H
