/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */

/********************************************************************************************************************/
/********************************************************************************************************************/

#define ADC1_CH_NUM  3  // ADC1：三相电流采样
#define ADC2_CH_NUM  3  // ADC2：2路温度 + 1路电位器

// 过流检测防误触
#define FILTER_TIMES      5      // 滑动均值滤波次数（5次，兼顾精度和响应）
extern uint8_t over_curr_flag;   // 过流标志位，0 = 无过流，1 = 过流

extern uint16_t adc1_buf[ADC1_CH_NUM];  // ADC1-三相电流采样值
extern uint16_t adc2_buf[ADC2_CH_NUM];  // ADC2-温度/电位器采样值

// 2路温度、1路电位器
extern float temperature1;
extern float temperature2;
extern float temperature_final;
extern float potentiometer;

void Over_Current_Recovery(void);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

