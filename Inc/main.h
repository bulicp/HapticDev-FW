/**
  ******************************************************************************
  * @file    HAL/HAL_TimeBase/Inc/main.h 
  * @author  MCD Application Team
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#define __USART2
//#define __USART6
#define __TERMINAL_DEBUG

//#define USE_TIM3_BASE
//#define USE_TIM3_OC
//#define USE_TIM3_PWM

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define INT_ROUND(x)        ((x)>=0?(int32_t)((x)+0.5):(int32_t)((x)-0.5))
#define UINT_ROUND(x)       (uint32_t)((x)+0.5)
/* Exported functions ------------------------------------------------------- */
HAL_StatusTypeDef Init_TIM3(TIM_HandleTypeDef* TIMHandle);
HAL_StatusTypeDef Init_TIM3_OC(TIM_HandleTypeDef* TIMHandle);
HAL_StatusTypeDef Init_TIM3_PWM(TIM_HandleTypeDef* TIMHandle);
void USART_Init(UART_HandleTypeDef* pUartHandle);
void Error_Handler(void);
void GPIOE_Init(void);

/* Private define ------------------------------------------------------------*/
#define TIM3_PERIOD         65536U-1U
#define TIM3_CNTCLK         1000000U
#define TIM3_CCR3           15U-1U
#define DATASIZE            4096*8

// Motor related constants
#define CRANKSHAFTDIAMETER  30        // diameter in mm
//#define DELTA_T             2         // delta t in ms - time period in which we change motor freqency while accelerating

// Acceleration in mm/s. Should be one of the values: 100,200,250,500,1000
//#define ACCELERATION        100.0     // acceleration in mm/s2
//#define ACCELERATION        200.0     // acceleration in mm/s2
#define ACCELERATION        250.0     // acceleration in mm/s2
//#define ACCELERATION        500.0     // acceleration in mm/s2
//#define ACCELERATION        1000.0     // acceleration in mm/s2
// Set the delay so that the delta_speed is always 1ms and delta frerqency is always 68:
#define DELTA_T 			UINT_ROUND((1000.0 / ACCELERATION))

#define DELTA_SPEED         (ACCELERATION * DELTA_T) / 1000
#define MOTORSTEPS          200
#define MICROSTEPS          32
#define PI                  3.14159265359
#define DELTAFREQ           DELTA_SPEED / ( (CRANKSHAFTDIAMETER*PI) / (MOTORSTEPS*MICROSTEPS) )
//#define DELTAFREQ           68        // delta freq for 2ms interval and acceleration a=500mm/s2

#define SPEED_60            123
#define SPEED_167           441
#define SPEED_1032          71




//#define USE_TIM3_BASE
#define USE_TIM3_OC
//#define USE_TIM3_PWM

#define LED_GREEN     LED4
#define LED_BLUE      LED6
#define LED_ORANGE    LED3
#define LED_RED       LED5

#define nENABLE_PIN 		  GPIO_PIN_8
#define DIRECTION_PIN 		GPIO_PIN_7

#define DIRCHANGE_TRUE 1
#define DIRCHANGE_FALSE 0

#ifdef __TERMINAL_DEBUG
#define MAX_SPEED       999
#define MIN_SPEED       0
#define SPEED_OFFSET    500
#else
#define MAX_SPEED       250
#define MIN_SPEED       0
#define SPEED_OFFSET   125
#endif


#ifdef __USART2
//#define USART_SPEED                   115200
#define USART_SPEED                     9600
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __USART2_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART2

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_STREAM              DMA1_Stream6
#define USARTx_RX_DMA_STREAM              DMA1_Stream5
#define USARTx_TX_DMA_CHANNEL             DMA_CHANNEL_4
#define USARTx_RX_DMA_CHANNEL             DMA_CHANNEL_4


/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Stream6_IRQn
#define USARTx_DMA_RX_IRQn                DMA1_Stream5_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Stream6_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA1_Stream5_IRQHandler

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler
#endif

#ifdef __USART6
#define USART_SPEED 9600
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART6
#define USARTx_CLK_ENABLE()              __USART6_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART6_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART6_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_6
#define USARTx_TX_GPIO_PORT              GPIOC
#define USARTx_TX_AF                     GPIO_AF8_USART6
#define USARTx_RX_PIN                    GPIO_PIN_7
#define USARTx_RX_GPIO_PORT              GPIOC
#define USARTx_RX_AF                     GPIO_AF8_USART6

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_STREAM              DMA2_Stream6
#define USARTx_RX_DMA_STREAM              DMA2_Stream1
#define USARTx_TX_DMA_CHANNEL             DMA_CHANNEL_5
#define USARTx_RX_DMA_CHANNEL             DMA_CHANNEL_5


/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA2_Stream6_IRQn
#define USARTx_DMA_RX_IRQn                DMA2_Stream1_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA2_Stream6_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA2_Stream1_IRQHandler

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART6_IRQn
#define USARTx_IRQHandler                USART6_IRQHandler
#endif

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
#define HELLLOBUFFERSIZE                      (COUNTOF(aHelloBuffer) - 1)
#define BELTBUFFERSIZE                      (COUNTOF(aBeltBuffer) - 1)
/* Size of Reception buffer */
//#define RXBUFFERSIZE                      TXBUFFERSIZE

#ifdef __TERMINAL_DEBUG
#define RXBUFFERSIZE                      3
#else
#define RXBUFFERSIZE                      1
#endif

/* Exported macro ------------------------------------------------------------*/
#define COUNT_OF_EXAMPLE(x)    (sizeof(x)/sizeof(BSP_DemoTypedef))

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
