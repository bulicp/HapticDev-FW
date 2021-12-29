/*
 * stm32f7xx_hal_msp_tim.c
 *
 *  Created on: Dec 26, 2021
 *      Author: patricio
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim) {
  GPIO_InitTypeDef   GPIO_InitStruct;

  /* Configure GPIOC AF2 on PIN 8: TIM3, Channel 3 */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure the TIM3 IRQ priority */
  HAL_NVIC_SetPriority(TIM3_IRQn, TICK_INT_PRIORITY, 0U);

  /* Enable the TIM3 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  /* Enable TIM3 clock */
  __HAL_RCC_TIM3_CLK_ENABLE();
}



void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {
  GPIO_InitTypeDef   GPIO_InitStruct;

  // Configure GPIOC AF2 on PIN 8: TIM3, Channel 3
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure the TIM3 IRQ priority
  HAL_NVIC_SetPriority(TIM3_IRQn, TICK_INT_PRIORITY, 0U);

  // Enable the TIM3 global Interrupt
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  // Enable TIM3 clock
  __HAL_RCC_TIM3_CLK_ENABLE();
}


