/*
 * timer.c
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
extern uint32_t uwMotorPulse;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/*
 * Init and start TIM3 in Base Interrupt mode
 *
 * */

HAL_StatusTypeDef Init_TIM3(TIM_HandleTypeDef* TIMHandle) {

  RCC_ClkInitTypeDef    clkconfig;
  uint32_t uwTimclock = 0U;
  uint32_t uwPrescalerValue = 0U;
  uint32_t uwAPB1Prescaler = 0U;
  uint32_t SysClockFreq;
  uint32_t HCLKFreq;
  uint32_t APB1Freq;
  uint32_t APB2Freq;
  uint32_t              pFLatency;

  /*Configure the TIM3 IRQ priority */
  HAL_NVIC_SetPriority(TIM3_IRQn, TICK_INT_PRIORITY, 0U);
  /* Enable the TIM3 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* Enable TIM6 clock */
  __HAL_RCC_TIM3_CLK_ENABLE();

  // Find out the TIM3 CLK frequency:
  /* Get clock configuration */
  //SysClockFreq = HAL_RCC_GetSysClockFreq();
  //HCLKFreq = HAL_RCC_GetHCLKFreq();
  APB1Freq = HAL_RCC_GetPCLK1Freq();
  //APB2Freq = HAL_RCC_GetPCLK2Freq();
  /* Get APB1 prescaler */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  uwAPB1Prescaler = clkconfig.APB1CLKDivider;
  /* Compute TIMERs clock */
  if (uwAPB1Prescaler == RCC_HCLK_DIV1)
  {
    uwTimclock = APB1Freq; // if PPRE1 bits in RCC_CFGR are 0XX (APB1 Prescaler = 1), then TIMs run at APB1 clk)
  }
  else
  {
    uwTimclock = 2*APB1Freq; // if PPRE1 bits in RCC_CFGR are 1XX (APB1 Prescaler > 1), then TIMs run at 2*APB1 clk)
  }

  /* Compute the prescaler value to have TIM3 counter clock equal to TIM3_CNTCLK in Hz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / TIM3_CNTCLK) - 1U);
  /* Initialize TIM3 */
  TIMHandle->Instance = TIM3;
  TIMHandle->Init.Period = TIM3_PERIOD;
  TIMHandle->Init.Prescaler = uwPrescalerValue;
  TIMHandle->Init.ClockDivision = 0;
  TIMHandle->Init.CounterMode = TIM_COUNTERMODE_UP;
  TIMHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if(HAL_TIM_Base_Init(TIMHandle) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start(TIMHandle);
  }
  /* Return function status */
  return HAL_ERROR;
}



/*
 * Init and start TIM3 in OC Interrupt mode
 *
 * */

HAL_StatusTypeDef Init_TIM3_OC(TIM_HandleTypeDef* TIMHandle) {

  RCC_ClkInitTypeDef    clkconfig;
  uint32_t uwTimclock;
  uint32_t uwPrescalerValue = 0U;
  uint32_t uwAPB1Prescaler = 0U;
  uint32_t SysClockFreq;
  uint32_t HCLKFreq;
  uint32_t APB1Freq;
  uint32_t APB2Freq;
  uint32_t              pFLatency;


  // Find out the TIM3 CLK frequency:
  //SysClockFreq = HAL_RCC_GetSysClockFreq();
  //HCLKFreq = HAL_RCC_GetHCLKFreq();
  APB1Freq = HAL_RCC_GetPCLK1Freq();
  //APB2Freq = HAL_RCC_GetPCLK2Freq();

  /* Get APB1 prescaler */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  uwAPB1Prescaler = clkconfig.APB1CLKDivider;
  /* Compute TIMERs clock */
  if (uwAPB1Prescaler == RCC_HCLK_DIV1)
  {
    uwTimclock = APB1Freq; // if PPRE1 bits in RCC_CFGR are 0XX (APB1 Prescaler = 1), then TIMs run at APB1 clk)
  }
  else
  {
    uwTimclock = 2*APB1Freq; // if PPRE1 bits in RCC_CFGR are 1XX (APB1 Prescaler > 1), then TIMs run at 2*APB1 clk),
  }

  /* Compute the prescaler value to have TIM3 counter clock equal to TIM3_CNTCLK in Hz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / TIM3_CNTCLK) - 1U);

  /* Initialize TIM3 */
  TIMHandle->Instance = TIM3;
  TIMHandle->Init.Period = TIM3_PERIOD;
  TIMHandle->Init.Prescaler = uwPrescalerValue;
  TIMHandle->Init.ClockDivision = 0;
  TIMHandle->Init.CounterMode = TIM_COUNTERMODE_UP;
  TIMHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;



  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef OCConfig;
  /* Configure the Output Compare channels */
  OCConfig.OCMode     = TIM_OCMODE_TOGGLE;
  OCConfig.OCPolarity = TIM_OCPOLARITY_LOW;
  OCConfig.Pulse = uwMotorPulse;
  OCConfig.OCFastMode = TIM_OCFAST_DISABLE;

  if(HAL_TIM_OC_Init(TIMHandle) == HAL_OK)
  {
    if (HAL_TIM_OC_ConfigChannel(TIMHandle, &OCConfig, TIM_CHANNEL_3) == HAL_OK)
    {
      /* Start the TIM3 OC in interrupt mode */
      return HAL_TIM_OC_Start_IT(TIMHandle, TIM_CHANNEL_3);
    }

  }

  /* Return function status */
  return HAL_ERROR;

}




/*
 * Init and start TIM3 in PWM Interrupt mode
 *
 * */
HAL_StatusTypeDef Init_TIM3_PWM(TIM_HandleTypeDef* TIMHandle) {

  RCC_ClkInitTypeDef    clkconfig;
  uint32_t uwTimclock;
  uint32_t uwPrescalerValue = 0U;
  uint32_t uwAPB1Prescaler = 0U;
  uint32_t SysClockFreq;
  uint32_t HCLKFreq;
  uint32_t APB1Freq;
  uint32_t APB2Freq;
  uint32_t              pFLatency;


  // Find out the TIM3 CLK frequency:
  //SysClockFreq = HAL_RCC_GetSysClockFreq();
  //HCLKFreq = HAL_RCC_GetHCLKFreq();
  APB1Freq = HAL_RCC_GetPCLK1Freq();
  //APB2Freq = HAL_RCC_GetPCLK2Freq();

  /* Get APB1 prescaler */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  uwAPB1Prescaler = clkconfig.APB1CLKDivider;
  /* Compute TIMERs clock */
  if (uwAPB1Prescaler == RCC_HCLK_DIV1)
  {
    uwTimclock = APB1Freq; // if PPRE1 bits in RCC_CFGR are 0XX (APB1 Prescaler = 1), then TIMs run at APB1 clk)
  }
  else
  {
    uwTimclock = 2*APB1Freq; // if PPRE1 bits in RCC_CFGR are 1XX (APB1 Prescaler > 1), then TIMs run at 2*APB1 clk)
  }

  /* Compute the prescaler value to have TIM3 counter clock equal to TIM3_CNTCLK in Hz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / TIM3_CNTCLK) - 1U);

  /* Initialize TIM3 */
  TIMHandle->Instance = TIM3;
  TIMHandle->Init.Period = TIM3_PERIOD;
  TIMHandle->Init.Prescaler = uwPrescalerValue;
  TIMHandle->Init.ClockDivision = 0;
  TIMHandle->Init.CounterMode = TIM_COUNTERMODE_UP;
  //TIMHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  TIMHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;  // should be enabled in PWM mode
  TIMHandle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef OCConfig;
  /* Configure the Output Compare channels in PWM1 mode */
  OCConfig.OCMode     = TIM_OCMODE_PWM1;
  OCConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  OCConfig.Pulse = TIM3_CCR3;
  OCConfig.OCFastMode = TIM_OCFAST_DISABLE;

  if(HAL_TIM_PWM_Init(TIMHandle) == HAL_OK)
  {
    if (HAL_TIM_PWM_ConfigChannel(TIMHandle, &OCConfig, TIM_CHANNEL_3) == HAL_OK)
    {
      /* Start the TIM3 PWM in interrupt mode */
      return HAL_TIM_PWM_Start_IT(TIMHandle, TIM_CHANNEL_3);
    }

  }
  /* Return function status */
  return HAL_ERROR;

}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {

  uint32_t ccr;

  // check if TIM3 asserted the interrupt:
  if(htim->Instance == TIM3) {
    // check if Channel 3 asserted the OC interrupt
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
      // Read and update CCR1
      ccr = htim->Instance->CCR3;
      ccr = ccr + uwMotorPulse;
      if (ccr >= htim->Instance->ARR) {
        htim->Instance->CCR3 = uwMotorPulse;
      }
      else htim->Instance->CCR3 = ccr;

      BSP_LED_Toggle(LED_BLUE);
    }
  }
}



/*
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){

  uint32_t ccr;

  // check if TIM3 asserted the interrupt:
  if(htim->Instance == TIM3) {
    // check if Channel 3 asserted the OC interrupt
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
      // Read and update CCR1
      ccr = htim->Instance->CCR3;
      ccr = ccr + uwMotorPulse;
      if (ccr >= htim->Instance->ARR) {
        htim->Instance->CCR3 = uwMotorPulse;
      }
      else htim->Instance->CCR3 = ccr;

      BSP_LED_Toggle(LED_BLUE);
    }
  }
}
*/



