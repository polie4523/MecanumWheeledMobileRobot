#ifndef __FM_H
#define __FM_H


#include "main.h"
#include <stdint.h>

#ifndef WHEEL_NUM
#define WHEEL_NUM 4
#endif

#ifndef SAMPLE_TIME
#define SAMPLE_TIME 0.05
#endif

#define CNT2RAD(X) (X/576.0f*2.0f*3.14159f)
typedef struct
{
  volatile float Speed[WHEEL_NUM];
  volatile uint32_t PWMdutycycle[WHEEL_NUM]; //=CCR/ARR, ARR=255
  volatile signed char Direction[WHEEL_NUM];
  TIM_HandleTypeDef *Encoder_timer[WHEEL_NUM];
} Four_motor_t;

void Motor_Init(Four_motor_t *const four_wheel);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_Drive(Four_motor_t *const four_wheel);
void GetSpeed(Four_motor_t *const four_wheel);


#endif //__FM_H