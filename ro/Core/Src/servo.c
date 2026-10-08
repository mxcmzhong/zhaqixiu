#include "servo.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

void servo_init(void)
{
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}
void servo_set_angle(int angle)
{
	int true_angle = 0;
	true_angle = (angle * 10000) / 180; // Convert angle to pulse width in microseconds
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, true_angle);
 
}