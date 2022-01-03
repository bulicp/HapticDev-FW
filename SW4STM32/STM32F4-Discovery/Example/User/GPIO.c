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

  __HAL_RCC_GPIOE_CLK_ENABLE();

  // Configure GPIOE PIN 7 (Motor direction): and PIN8 (8255 Enable) TIM3, Channel
  // Configure GPIOE PIN 9 (M0), PIN 11 (M1), Pin13 (M2)
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // Disable 8255 at init
  HAL_GPIO_WritePin(GPIOE, nENABLE_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOE, DIRECTION_PIN, GPIO_PIN_SET);

  // Initially set 1/32 microstepping:
  HAL_GPIO_WritePin(GPIOE, M0_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, M1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, M2_PIN, GPIO_PIN_SET);
}

void SetMicrostepping(uint16_t ubMicrostep){
  switch(ubMicrostep){
  case MICROSTEP_1:
	  HAL_GPIO_WritePin(GPIOE, M0_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, M1_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, M2_PIN, GPIO_PIN_RESET);
	  break;
  case MICROSTEP_2:
	  HAL_GPIO_WritePin(GPIOE, M0_PIN, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, M1_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, M2_PIN, GPIO_PIN_RESET);
	  break;
  case MICROSTEP_4:
	  HAL_GPIO_WritePin(GPIOE, M0_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, M1_PIN, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, M2_PIN, GPIO_PIN_RESET);
	  break;
  case MICROSTEP_8:
	  HAL_GPIO_WritePin(GPIOE, M0_PIN, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, M1_PIN, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, M2_PIN, GPIO_PIN_RESET);
	  break;
  case MICROSTEP_16:
	  HAL_GPIO_WritePin(GPIOE, M0_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, M1_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, M2_PIN, GPIO_PIN_SET);
	  break;
  case MICROSTEP_32:
	  HAL_GPIO_WritePin(GPIOE, M0_PIN, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, M1_PIN, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, M2_PIN, GPIO_PIN_SET);
	  break;
  default:
	  HAL_GPIO_WritePin(GPIOE, M0_PIN, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, M1_PIN, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, M2_PIN, GPIO_PIN_SET);
  }
}


