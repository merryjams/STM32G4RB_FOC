/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/********************************************************************************************************************/
/********************************************************************************************************************/

#define Key1 GPIO_PIN_4
#define Key2 GPIO_PIN_12
#define Key1_GPIO_Port GPIOA
#define Key2_GPIO_Port GPIOB
#define Led1 GPIO_PIN_11
#define Led2 GPIO_PIN_5
#define Led1_GPIO_Port GPIOB
#define Led2_GPIO_Port GPIOA
#define Key_Delay_Timer htim4

// 按键相关全局变量
typedef enum {
    KEY_IDLE = 0,        // 空闲状态
    KEY_DETECTED = 1,    // 检测到按键触发（等待消抖）
    KEY_READY = 2        // 消抖完成，可处理
} Key_StateTypeDef;

// 按键结构体
typedef struct {
    GPIO_TypeDef* GPIOx;      // 按键GPIO端口
    uint16_t GPIO_Pin;        // 按键引脚
    Key_StateTypeDef state;   // 按键状态
} Key_TypeDef;

#define KEY_NUM 2

// 函数声明
void Keys_Process(void);
void Leds_Blink(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

