/*
 * GPIO.c
 *
 *  Created on: Dec 28, 2021
 *      Author: patricio
 */

#include "main.h"

void GPIOE_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Configure GPIOE PIN 7 (Motor direction): and PIN8 (8255 Enable) TIM3, Channel 3 */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Disable 8255 at init
  HAL_GPIO_WritePin(GPIOE, nENABLE_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOE, DIRECTION_PIN, GPIO_PIN_SET);
}


