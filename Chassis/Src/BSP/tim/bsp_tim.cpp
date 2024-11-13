#include "bsp_tim.hpp"
#include "stm32f4xx_it.h"

/**
 * @brief 这是hal库的定时器中断函数
 * @attention 每次用MX生成代码时都会在stm32f4xx_it文件中生成这个函数，这里将这个函数的定义放在这里以便管理和开发，因此每次MX重新生成代码时需要将stm32f4xx_it中这个函数的定义注释掉
 */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

void PWM_Init(void)
{
  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
}

void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  HAL_TIM_PWM_Start(htim, Channel);
}

void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  HAL_TIM_PWM_Stop(htim, Channel);
}

void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period)
{
  __HAL_TIM_SetAutoreload(htim, period * ((168000) / (htim->Init.Prescaler + 1))); // tim1和tim8的时钟频率是168M
}

void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel)
{
  __HAL_TIM_SetCompare(htim, channel, dutyratio * (htim->Instance->ARR));
}

void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  AHRS::Instance()->AHRS_Update();
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}
