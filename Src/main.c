/**
  ******************************************************************************
  * @file    HAL/HAL_TimeBase/Src/main.c 
  * @author  MCD Application Team
  * @brief   This example describes how to configure HAL time base using
  *          the STM32F4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup HAL_TimeBase
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t uwIncrementState = 0;
TIM_HandleTypeDef TIM3Handle;
// UART handler declaration
UART_HandleTypeDef UartHandle;
/* Buffer used for transmission */
const uint8_t aTxBuffer[] = "\n**Motor OK**\n";
const uint8_t aHelloBuffer[] = "\n\n\n**Motor Ready : Enter speed as a 3 digit number**\n\n";
const uint8_t aBeltBuffer[] = "belt";
/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
__IO ITStatus UartRXReady = RESET;  // UART RX Callback sets this flag
__IO ITStatus UartTXReady = RESET;  // UART TX Callback sets this flag

uint32_t uwRxData;
int32_t wNewSpeed = 0;
int32_t wCurrentSpeed = 0;
//uint32_t uwMotorPulse = 123;
uint32_t uwMotorPulse = 32000;
int32_t wDeltaSpeed = 0;

uint32_t uwCurrentFreq = 0;

uint8_t bDirectionChange = DIRCHANGE_FALSE;




float fMotorPeriod;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void ChangeSpeed(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* This sample code shows how to configure The HAL time base source base with a 
  dedicated  Tick interrupt priority.
  A general purpose timer(TIM6) is used instead of Systick as source of time base.  
  Time base duration is fixed to 1ms since PPP_TIMEOUT_VALUEs are defined and 
  handled in milliseconds basis.
  */
  
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 168 MHz */
  SystemClock_Config();
  
  /* Configure LED3, LED4, LED5 and LED6 */
  BSP_LED_Init(LED3); // orange
  BSP_LED_Init(LED4); // green
  BSP_LED_Init(LED5); // red
  BSP_LED_Init(LED6); // blue
  
  // Init and set high nENABLE and DIRECTION pins
  GPIOE_Init();

  USART_Init(&UartHandle);
  //HAL_Delay(1000);

  /* Configure USER Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  //Init_TIM3_PWM(&TIM3Handle);
  Init_TIM3_OC(&TIM3Handle);

  /*
  // send ACK:
  if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aHelloBuffer, HELLLOBUFFERSIZE )!= HAL_OK)
  {
    Error_Handler();
  }
  //Wait for the end of the transfer ###################################
  while (UartTXReady != SET)
  {
  }
  // Reset transmission flag
  UartTXReady = RESET;
  */

  uwMotorPulse = 0;
  uwCurrentFreq = 0;
  wCurrentSpeed = 0;

  SetMicrostepping(MICROSTEPS);

  /* Infinite loop */  
  while (1)
  {

    //##- Put UART peripheral in reception process ###########################
    if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
    {
      Error_Handler();
    }
    //##- Wait for the end of the transfer ###################################
    while (UartRXReady != SET) // UART RX Callback sets this flag
    {
    }
    // Reset transmission flag
    UartRXReady = RESET;
#ifdef __TERMINAL_DEBUG
	wNewSpeed = atoi((char *)aRxBuffer);
#else
	uwRxData = aRxBuffer[0];
	wNewSpeed = uwRxData;
#endif

	if (wNewSpeed == 255) { // Ping from PC
	  BSP_LED_On(LED_GREEN); // Green LED On: device connected via COM and response with "belt"
	  if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aBeltBuffer, BELTBUFFERSIZE)!= HAL_OK)
	  {
	    Error_Handler();
	  }
	  //Wait for the end of the transfer ###################################
	  while (UartTXReady != SET)
	  {
	  }
	  // Reset transmission flag
	  UartTXReady = RESET;
	  wNewSpeed = wCurrentSpeed;
	}
	else if (wNewSpeed > MAX_SPEED || wNewSpeed < MIN_SPEED) { // If new speed exceeds the limit, keep current speed
	  wNewSpeed = wCurrentSpeed;
	}
	else {
	  wNewSpeed = wNewSpeed - SPEED_OFFSET; // remove offset

	  // determine direction (if new speed == 0, keep the current direction):
	  if (wNewSpeed < 0) {
		wNewSpeed = -wNewSpeed;
		  HAL_GPIO_WritePin(GPIOE, DIRECTION_PIN, GPIO_PIN_RESET);
	  }
	  else if ((wNewSpeed > 0)) {
	    HAL_GPIO_WritePin(GPIOE, DIRECTION_PIN, GPIO_PIN_SET);
	  }
	}

	  // Flash LED upon successful UART reception
    BSP_LED_On(LED_ORANGE);
    HAL_Delay(60);
    BSP_LED_Off(LED_ORANGE);

    // Process new speed and set new frequency in Timer CCR:
    wDeltaSpeed = wNewSpeed - wCurrentSpeed;
    wCurrentSpeed = wNewSpeed;

    ChangeSpeed();
  } // while
}



/*
 * Hange speed with a=500 mm/s2
 */

static void ChangeSpeed(void){

  // accelerate with 500 mm/s2:
  if (wDeltaSpeed > 0){
    HAL_GPIO_WritePin(GPIOE, nENABLE_PIN, GPIO_PIN_RESET);
    for(int i = 0; i<wDeltaSpeed; i++){
      uwCurrentFreq += UINT_ROUND(DELTAFREQ);
      fMotorPeriod = 1.0/ (float)uwCurrentFreq;
      uwMotorPulse = (TIM3_CNTCLK * fMotorPeriod)/2;
      HAL_Delay(DELTA_T);
    }
    uwMotorPulse = UINT_ROUND((TIM3_CNTCLK * (((float)CRANKSHAFTDIAMETER * PI / 2.0) / ((float) MOTORSTEPS * (float) MICROSTEPS * (float) wNewSpeed))));
  }
  //decelerate with 500mm/s2
  else if (wDeltaSpeed < 0) {
    for(int i = 0; i< -(wDeltaSpeed); i++){
      uwCurrentFreq -= UINT_ROUND(DELTAFREQ);
      fMotorPeriod = 1.0/ (float)uwCurrentFreq;
      uwMotorPulse = (TIM3_CNTCLK * fMotorPeriod)/2;
      HAL_Delay(DELTA_T);
    }
    uwMotorPulse = UINT_ROUND((TIM3_CNTCLK * (((float)CRANKSHAFTDIAMETER * PI / 2.0) / ((float) MOTORSTEPS * (float) MICROSTEPS * (float) wNewSpeed))));
    if (wNewSpeed == 0){
      HAL_GPIO_WritePin(GPIOE, nENABLE_PIN, GPIO_PIN_SET);
    }
  }
}


/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == KEY_BUTTON_PIN)
  {
    if (uwIncrementState == 0)
    {
      /* Suspend tick increment */
      HAL_SuspendTick();
      
      /* Change the Push button state */
      uwIncrementState = 1;
    }
    else
    {
      /* Resume tick increment */
      HAL_ResumeTick();
      
      /* Change the Push button state */
      uwIncrementState = 0;
    }
  }  
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED5 on */
  BSP_LED_On(LED5);
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
