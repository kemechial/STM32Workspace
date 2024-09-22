/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t state=4,debounceflag,led_flash,backward;
uint16_t tim1_cnt=999,tim2_cnt=999,tim3_cnt=999,tim4_cnt=999;
uint8_t tim1_ena,tim2_ena,tim3_ena,tim4_ena;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	if(debounceflag==0)
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			led_flash=0;
			debounceflag=1;
			state++;
			if(state==5)
			{
				state=1;
			}
			switch(state)
			{
			case 1:

				tim1_cnt=1000-1;
				tim2_cnt=2000-1;



				tim1_ena=1;
				tim2_ena=1;


				TIM1->CR1|=(TIM_CR1_CEN);
				TIM2->CR1|=(TIM_CR1_CEN);
				TIM3->CR1&=~(TIM_CR1_CEN);
				TIM4->CR1&=~(TIM_CR1_CEN);
				break;

			case 2:

				tim1_cnt=1000;
				tim2_cnt=2000;
				tim3_cnt=100;
				tim4_cnt=1;



				tim1_ena=1;
				tim2_ena=1;
				tim3_ena=1;
				tim4_ena=1;

				TIM1->CR1|=(TIM_CR1_CEN);
				TIM2->CR1|=(TIM_CR1_CEN);
				TIM3->CR1|=(TIM_CR1_CEN);
				TIM4->CR1|=(TIM_CR1_CEN);
				break;

			case 3:

				tim1_cnt=50;
				tim2_cnt=50;
				tim3_cnt=50;
				tim4_cnt=50;



				tim1_ena=1;
				tim2_ena=0;
				tim3_ena=0;
				tim4_ena=0;

				TIM1->CR1|=(TIM_CR1_CEN);
				TIM2->CR1|=(TIM_CR1_CEN);
				TIM3->CR1|=(TIM_CR1_CEN);
				TIM4->CR1|=(TIM_CR1_CEN);
				break;

			case 4:

				tim1_cnt=25;
				tim2_cnt=25;
				tim3_cnt=25;
				tim4_cnt=25;



				tim1_ena=1;
				tim2_ena=0;
				tim3_ena=0;
				tim4_ena=0;

				TIM1->CR1|=(TIM_CR1_CEN);
				TIM2->CR1|=(TIM_CR1_CEN);
				TIM3->CR1|=(TIM_CR1_CEN);
				TIM4->CR1|=(TIM_CR1_CEN);
				break;

			}


		}
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BUTTON_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
	TIM1->ARR=tim1_cnt;
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
  if(tim1_ena==1)
    {
  	switch(state)
  	{
  	case 1:
  		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  		break;

  	case 2:
  		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  		break;

  	case 3:
  		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

  		led_flash++;
  		if(led_flash>1)
  		{
  			led_flash=0;
  			tim1_ena=0;
  			tim2_ena=1;

  			TIM2->CR1|=(TIM_CR1_CEN);
  			TIM1->CR1&=~(TIM_CR1_CEN);
  		}

  		break;

  	case 4:
  		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);


			if(backward==0)
			{
				tim1_ena=0;
				tim2_ena=1;

				TIM2->CR1|=(TIM_CR1_CEN);
				TIM1->CR1&=~(TIM_CR1_CEN);
			}
			backward=0;
			break;
  		}
    }
  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	TIM2->ARR=tim2_cnt;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  if(tim2_ena==1)
    {
  	switch(state)
  	{
  	case 1:
  		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
  		break;

  	case 2:
  		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
  		break;

  	case 3:
  		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

  		led_flash++;
  		if(led_flash>1)
  		{
  			led_flash=0;
  			tim2_ena=0;
  			tim3_ena=1;

  			TIM3->CR1|=(TIM_CR1_CEN);
  			TIM2->CR1&=~(TIM_CR1_CEN);
  		}

  		break;

  	case 4:
  		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

				if(backward==0)
				{
					tim2_ena=0;
					tim3_ena=1;
					TIM3->CR1|=(TIM_CR1_CEN);
					TIM2->CR1&=~(TIM_CR1_CEN);
				}
				else
				{
					tim1_ena=1;
					tim2_ena=0;
					TIM1->CR1|=(TIM_CR1_CEN);
					TIM2->CR1&=~(TIM_CR1_CEN);
				}

				break;
			}
    	}
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	TIM3->ARR=tim3_cnt;
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  if(tim3_ena==1)
    {
  	switch(state)
  	{
  	case 1:

  		break;

  	case 2:
  		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
  		break;

  	case 3:
  		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

  		led_flash++;
  		if(led_flash>1)
  		{
  			led_flash=0;
  			tim3_ena=0;
  			tim4_ena=1;
  			TIM4->CR1|=(TIM_CR1_CEN);
  			TIM3->CR1&=~(TIM_CR1_CEN);
  		}

  		break;

  	case 4:
  		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);


				if(backward==0)
				{
					tim3_ena=0;
					tim4_ena=1;
					TIM4->CR1|=(TIM_CR1_CEN);
					TIM3->CR1&=~(TIM_CR1_CEN);
				}
				else
				{
					tim3_ena=0;
					tim2_ena=1;
					TIM2->CR1|=(TIM_CR1_CEN);
					TIM3->CR1&=~(TIM_CR1_CEN);
				}
				break;
			}
    	}
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	TIM4->ARR=tim4_cnt;
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  if(tim4_ena==1)
    {
  	switch(state)
  	{
  	case 1:
  		break;

  	case 2:
  		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
  		break;

  	case 3:
  		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);

  		led_flash++;
  		if(led_flash>1)
  		{
  			led_flash=0;
  			tim4_ena=0;
  			tim1_ena=1;
  			TIM1->CR1|=(TIM_CR1_CEN);
  			TIM4->CR1&=~(TIM_CR1_CEN);
  		}

  		break;

  	case 4:
  		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);


				if(backward==1)
				{
				tim4_ena=0;
				tim3_ena=1;
				TIM3->CR1|=(TIM_CR1_CEN);
				TIM4->CR1&=~(TIM_CR1_CEN);
				}
				backward=1;
				break;
			}
    	}
  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
