/*
 * four_motor.c
 *
 *  Created on: Jul 11, 2024
 *      Author: polie
 */


#include "main.h"
#include "four_motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim2;

void Motor_Init(Four_motor_t *const four_wheel)
{
  four_wheel->Encoder_timer[0] = &htim1;
  four_wheel->Encoder_timer[1] = &htim8;
  four_wheel->Encoder_timer[2] = &htim4;
  four_wheel->Encoder_timer[3] = &htim3;

  four_wheel->PWMdutycycle[0] = 0;
  four_wheel->Direction[0] = 1;
  four_wheel->PWMdutycycle[1] = 0;
  four_wheel->Direction[1] = 1;
  four_wheel->PWMdutycycle[2] = 0;
  four_wheel->Direction[2] = 1;
  four_wheel->PWMdutycycle[3] = 0;
  four_wheel->Direction[3] = 1;
}

void Motor_Start(void)
{
  htim2.Instance->CCR1 = 0;
  htim2.Instance->CCR2 = 0;
  htim2.Instance->CCR3 = 0;
  htim2.Instance->CCR4 = 0;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // STBY ON
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // STBY ON
}

void Motor_Stop(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // STBY OFF
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // STBY OFF
  htim2.Instance->CCR1 = 0;
  htim2.Instance->CCR2 = 0;
  htim2.Instance->CCR3 = 0;
  htim2.Instance->CCR4 = 0;
}

void Motor_Drive(Four_motor_t *const four_wheel)
{
  htim2.Instance->CCR1 = four_wheel->PWMdutycycle[0];
  if (four_wheel->Direction[0]==1)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
  }
  htim2.Instance->CCR2 = four_wheel->PWMdutycycle[1];
  if (four_wheel->Direction[1]==1)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
  }
  htim2.Instance->CCR3 = four_wheel->PWMdutycycle[2];
  if (four_wheel->Direction[2]==1)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  htim2.Instance->CCR4 = four_wheel->PWMdutycycle[3];
  if (four_wheel->Direction[3]==1)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  }
}

void GetSpeed(Four_motor_t *const four_wheel) {
  short count = 0;
  float theta_dot = 0.0f;
  for (short i = 0; i < WHEEL_NUM; i++) {
    count = (short) (*(four_wheel->Encoder_timer[i])).Instance->CNT;
    (*(four_wheel->Encoder_timer[i])).Instance->CNT = 0;
    theta_dot = count/SAMPLE_TIME;
    four_wheel->Speed[i] = CNT2RAD(theta_dot);
  }
}
