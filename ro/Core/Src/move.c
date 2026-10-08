#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "servo.h"
#include "move.h"
void run(float speed1, float speed2,float* target1,float* target2)
{
	*target1 = 120;
	*target2 = 120;
  if (speed1 >= 0)
  {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  else
  {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  }

  if (speed2 >= 0)
  {
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, speed2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  }
  else
  {
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, -speed2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  }
}
void turn_left(float speed1, float speed2,float* target1,float* target2)
{
	*target1 = 120;
	*target2 = 120;
  if (speed1 >= 0)
  {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  else
  {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  }

  if (speed2 >= 0)
  {
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, speed2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  }
  else
  {
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, -speed2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  }
}
void turn_right(float speed1, float speed2,float* target1,float* target2)
{
	*target1 = 120;
	*target2 = 120;
  if (speed1 >= 0)
  {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
  }
  else
  {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
  }

  if (speed2 >= 0)
  {
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, speed2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  }
  else
  {
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, -speed2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  }

}