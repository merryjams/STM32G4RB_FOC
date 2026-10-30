/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/

/******************************************* 功能：按键外部触发、软件消抖 *********************************************/
#include <stdio.h>
#include "usart.h"
#include "tim.h"
#include "foc.h"


// 定义2个按键
Key_TypeDef keys[] = {
    {Key1_GPIO_Port, Key1, KEY_IDLE}, 
    {Key2_GPIO_Port, Key2, KEY_IDLE}   
};


// 按键外部中断回调函数 *****************************************************
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 遍历所有按键，判断是哪个按键触发中断
    for (uint8_t i = 0; i < KEY_NUM; i++)
    {
      if (GPIO_Pin == keys[i].GPIO_Pin)
      {
        // 仅标记按键进入检测状态，启动20ms消抖定时器
        keys[i].state = KEY_DETECTED;
        HAL_TIM_Base_Start_IT(&Key_Delay_Timer); // 启动定时器
        break;
      }
    }
}


// 定时器中断处理20ms消抖在tim.c文件 ****************************************


// 按键1功能函数 ***********************************************************
void Key1_Callback(void)
{
    motor_control_context.type = control_type_null;
    printf("control_type: %d\n", motor_control_context.type);	
    HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1);
}


// 按键2功能函数 ***********************************************************
void Key2_Callback(void)
{
    motor_control_context.type = control_type_position;
    motor_control_context.position = deg2rad(720);
    // motor_control_context.type = control_type_speed;
    // motor_control_context.speed = deg2rad(360); 
    printf("control_type: %d\n", motor_control_context.type);	
    HAL_GPIO_TogglePin(Led2_GPIO_Port, Led2);
}


// 多按键处理函数 ***********************************************************
void Keys_Process(void)
{
    // 遍历所有按键，处理消抖完成的按键
    for (uint8_t i = 0; i < KEY_NUM; i++)
    {
      if (keys[i].state == KEY_READY)
      {
        switch(i)
        {
          case 0: Key1_Callback(); break; 
          case 1: Key2_Callback(); break; 
        }
        // 处理完成后重置状态
        keys[i].state = KEY_IDLE;
      }
    }
}


// 开机Led闪两下 **************************************************************
void Leds_Blink(void)
{
    HAL_GPIO_WritePin(Led1_GPIO_Port, Led1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Led2_GPIO_Port, Led2, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(Led1_GPIO_Port, Led1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Led2_GPIO_Port, Led2, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(Led1_GPIO_Port, Led1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Led2_GPIO_Port, Led2, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(Led1_GPIO_Port, Led1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Led2_GPIO_Port, Led2, GPIO_PIN_RESET);
    HAL_Delay(1000);
}

/* USER CODE END 2 */
