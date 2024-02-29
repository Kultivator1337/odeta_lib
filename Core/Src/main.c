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
#include "main.h"
#include "odeta_lib.h"

int main(void)
{
  init();

  uint32_t state = 0;

  setPinsAsOutput(PIN(C,13), OUTPUT_PUSHPULL|OUTPUT_NOPULL);
  while (1)
  {
    for(uint32_t i = 0; i < 10000000; i++) __NOP();
    state = getOutput(PIN(C,13));
    setOutput(PIN(C,13), !state);

  }

}


void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}