/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#include "stm32f1xx_hal.h"
int main()
{
__IO uint32_t i;
RCC->APB2ENR |= (1u<<5);
GPIOA->CRH &= ~(0x00F00000);
GPIOA->CRH |= (0x00200000);
while(1)
{
GPIOA->BSRR |= (1u<<13);
for(i=0;i<20000;i++);
GPIOA->BRR |= (1u<<13);
//GPIOD->BSRR |= ((1u<<13)<<16); //alternatif
for(i=0;i<20000;i++);
}
return 0;
}
