/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED=0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC1 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA1     ------> ADC1_IN2
    PA2     ------> ADC1_IN3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel4;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC2 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PC1     ------> ADC2_IN7
    PA7     ------> ADC2_IN4
    PC4     ------> ADC2_IN5
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA1_Channel5;
    hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_NORMAL;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc2);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA1     ------> ADC1_IN2
    PA2     ------> ADC1_IN3
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
  /* USER CODE BEGIN ADC1:ADC1_2_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
  /* USER CODE END ADC1:ADC1_2_IRQn disable */

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PC1     ------> ADC2_IN7
    PA7     ------> ADC2_IN4
    PC4     ------> ADC2_IN5
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_4);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC2 interrupt Deinit */
  /* USER CODE BEGIN ADC2:ADC1_2_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
  /* USER CODE END ADC2:ADC1_2_IRQn disable */

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  ******************************************************************************
  * @attention
  * 
  * 功能：adc采样、q轴电流计算、滤波、过流保护等
  * 作者: B站UP 点灯子曰 -- https://space.bilibili.com/3546977746815362
  * 授权: 移植/再开发请注明原作者，商业行为请联系原作者
  *
  ******************************************************************************
  */ 

/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/

// 在stm32g431里，转换时间 = 采样时间 + 12.5个ADC时钟周期(cubemx配置为170MHz4分频)
// 三相电流配置采样时间 = ADC_SAMPLETIME_47CYCLES_5 对应 转换时间 = (47.5+12.5)/(170M/4）= 1.4us

#include "foc.h"
#include "arm_math.h"   // ARM DSP库（针对arm架构优化的数学库），提供sin/cos/park等函数
#include <stdio.h>
#include "gpio.h"
#include "open_serial_debugger.h"
#include <stdlib.h>
#include "tim.h"

uint16_t adc1_buf[ADC1_CH_NUM] = {0};  // ADC1-三相电流采样值
uint16_t adc2_buf[ADC2_CH_NUM] = {0};  // ADC2-温度/电位器采样值

// 2路温度、1路电位器
float temperature1 = 0;
float temperature2 = 0;
float temperature_final = 0;
float potentiometer = 0;  // 电位器比例（0-1）

uint8_t over_curr_flag = 0;  // 过流标志位，0 = 无过流，1 = 过流
// 滑动均值滤波缓冲区
float i_u_filter_buf[FILTER_TIMES] = {0};
float i_v_filter_buf[FILTER_TIMES] = {0};
float i_bus_filter_buf[FILTER_TIMES] = {0};
uint8_t filter_index = 0;  // 滤波缓冲区索引（循环覆盖）
uint8_t over_curr_cnt_con = 0; // 持续过流计数（连续超过阈值的次数）
uint8_t over_curr_cnt_ins = 0; // 瞬间大电流过流计数
// 滤波后的电流值
float i_u_filt = 0, i_v_filt = 0, i_bus_filt = 0;

static void Current_Slide_Filter(float i_u, float i_v, float i_bus);  // 滑动均值滤波
static uint8_t Over_Current_Detect(void);  // 过流检测（带延时确认）
static void Over_Current_Protect(void);    // 过流保护执行

static void NTC_Get_Temperature(uint16_t* adc2_buf, float* temp1, float* temp2, float* temp_final);  // 温度计算


// ADC DMA传输完成回调函数 ***********************************************************************************
// 获得3相电流，通过克拉克变换、帕克变换得到 d/q 轴电流
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
      // 根据电路，电流计算公式 Vadc = 1.65+0.005*I*50
      float u_1 = ADC_VOLTAGE * ((float)(adc1_buf[0]) / (ADC_RESOLUTION - 1) - 0.5f);
      float u_2 = ADC_VOLTAGE * ((float)(adc1_buf[1]) / (ADC_RESOLUTION - 1) - 0.5f);
      float u_3 = ADC_VOLTAGE * ((float)(adc1_buf[2]) / (ADC_RESOLUTION - 1) - 0.5f + BUS_VREF_CORRECT);
      float i_1 = u_1 / R_SHUNT / OP_GAIN + MOTOR_I_U_CORRECT;
      float i_2 = u_2 / R_SHUNT / OP_GAIN + MOTOR_I_V_CORRECT;
      float i_3 = u_3 / R_SHUNT / OP_GAIN;

      // 滑动均值滤波
      Current_Slide_Filter(i_1, i_2, i_3);

      // 过流检测（带延时确认）
      Over_Current_Detect(); 
      // 过流保护执行
      Over_Current_Protect();

      motor_i_u = i_u_filt;                  // 对应MA
      motor_i_v = i_v_filt;                  // 对应MB
      motor_i_w = -(motor_i_u + motor_i_v);  // 对应MC
      motor_i_bus = i_bus_filt;              // 对应母线

      // 测试
      // motor_i_u = i_1;                  
      // motor_i_v = i_2;                 
      // motor_i_w = -(motor_i_u + motor_i_v); 
      // motor_i_bus = i_3;  

      float i_alpha = 0;
      float i_beta = 0;
      // 克拉克变换
      // i_alpha = i_u
      // i_beta  = (1/sqrt(3)) * i_u + (2/sqrt(3)) * i_v
      arm_clarke_f32(motor_i_u, motor_i_v, &i_alpha, &i_beta);

      float sin_value = arm_sin_f32(rotor_electrical_angle);
      float cos_value = arm_cos_f32(rotor_electrical_angle);
      float _motor_i_d = 0;
      float _motor_i_q = 0;
      // 帕克变换
      // id ​= iα * ​cosθ + iβ * ​sinθ
      // iq ​= −iα * ​sinθ + iβ * ​cosθ
      // i_d：直轴电流（励磁电流），控制电机磁链，FOC 通常设为 0（弱磁控制除外）
      // i_q：交轴电流（转矩电流），直接决定电机输出转矩，是转速环的控制目标
      arm_park_f32(i_alpha, i_beta, &_motor_i_d, &_motor_i_q, sin_value, cos_value);

      // 一阶低通滤波
      motor_i_d = low_pass_filter(_motor_i_d, motor_i_d, i_d_low_pass_filter_alpha);
      motor_i_q = low_pass_filter(_motor_i_q, motor_i_q, i_q_low_pass_filter_alpha);

      // //printf("pos:%.2f, speed:%.2f, electrical_angle:%.2f \r\n", cumulative_encoder_angle, motor_speed, rotor_electrical_angle);   
      // //printf("target_position:%.2f, target_speed:%.2f \r\n", motor_control_context.position, motor_control_context.speed);

      // 运行电机相应闭环控制
      switch (motor_control_context.type)
      {
        case control_type_null:
          set_pwm_duty(0, 0, 0, 0.9f);
          pid_i_reset();
          break;
        case control_type_position:
          set_motor_pid_single(&pid_position, Position_P, Position_I, Position_D);
          lib_position_control(motor_control_context.position);
          break;
        case control_type_speed:
          set_motor_pid_single(&pid_speed, Speed_P, Speed_I, Speed_D);
          lib_speed_control(motor_control_context.speed);
          break;
        case control_type_torque:
          set_motor_pid_single(&pid_torque_q, Torque_Q_P, Torque_Q_I, Torque_Q_D);
          lib_torque_control(motor_control_context.current_d, motor_control_context.current_q);
          break;
        case control_type_position_speed:
          set_motor_pid_single(&pid_position_PS_Ctrl, Position_P, Position_I, Position_D);
          set_motor_pid_single(&pid_speed_PS_Ctrl, Speed_P, Speed_I, Speed_D);
          lib_position_speed_control(motor_control_context.position, motor_control_context.speed);
          break;
        case control_type_position_torque:
          set_motor_pid_single(&pid_position_PT_Ctrl, Position_P, Position_I, Position_D);
          set_motor_pid_single(&pid_torque_q_PT_Ctrl, Torque_Q_P, Torque_Q_I, Torque_Q_D);
          lib_position_torque_control(motor_control_context.position, motor_control_context.current_q);
          break;
        case control_type_speed_torque:
          set_motor_pid_single(&pid_speed_ST_Ctrl, Speed_P, Speed_I, Speed_D);
          set_motor_pid_single(&pid_torque_q_ST_Ctrl, Torque_Q_P, Torque_Q_I, Torque_Q_D);
          lib_speed_torque_control(motor_control_context.speed, motor_control_context.current_q);
          break;
        case control_type_position_speed_torque:
          set_motor_pid_single(&pid_position_PST_Ctrl, Position_P, Position_I, Position_D);
          set_motor_pid_single(&pid_speed_PST_Ctrl, Speed_P, Speed_I, Speed_D);
          set_motor_pid_single(&pid_torque_q_PST_Ctrl, Torque_Q_P, Torque_Q_I, Torque_Q_D);
          lib_position_speed_torque_control(motor_control_context.position, motor_control_context.speed, motor_control_context.current_q);
          break;
        default:
          break;
      }

      HAL_ADC_Start_DMA(hadc, (uint32_t*)adc1_buf, ADC1_CH_NUM);  //开启下一次
    }

    else if(hadc->Instance == ADC2)
    {
        // 电位器比例（0-1）
        potentiometer = (float)adc2_buf[0] / ADC_RESOLUTION;         

        // 根据电路，3.3/(10k+R)=Vadc/R
        // 根据ntc芯片手册，(25-50℃)1/(t+273.15)=1/298.15+1/3380*ln(R/10k)   (25-85℃)1/(t+273.15)=1/298.15+1/3434*ln(R/10k)
        NTC_Get_Temperature(adc2_buf, &temperature1, &temperature2, &temperature_final);
        
        // 过温保护（超过阈值触发PWM降额/停机）
        //if(temperature1 > TEMPREATURE_THRESHOLD || temperature2 > TEMPREATURE_THRESHOLD)
        if(temperature2 > TEMPREATURE_THRESHOLD)  // 第一个温度采样异常，我懒得拆了
        {
            printf("Out of Temperature \r\n");
            PWM_Stop();
            HAL_GPIO_WritePin(Led1_GPIO_Port, Led1, GPIO_PIN_SET);  // 亮灯提示
        }

        HAL_ADC_Start_DMA(hadc, (uint32_t*)adc2_buf, ADC2_CH_NUM);
    }

}

// ADC错误回调
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        printf("ADC1 Error! Error Code: 0x%04X\r\n", hadc->ErrorCode);
    }
    else if(hadc->Instance == ADC2)
    {
        printf("ADC2 Error! Error Code: 0x%04X\r\n", hadc->ErrorCode);
    }
    
    HAL_ADC_Stop_DMA(hadc);
    if(hadc->Instance == ADC1)
    {
        HAL_ADC_Start_DMA(hadc, (uint32_t*)adc1_buf, ADC1_CH_NUM);
    }
    else if(hadc->Instance == ADC2)
    {
        HAL_ADC_Start_DMA(hadc, (uint32_t*)adc2_buf, ADC2_CH_NUM);
    }
}


// 滑动均值滤波,对三相电流做滑动均值滤波，输出平滑的电流值 ***************************************************
// 滑动均值：每次新增一个采样值，剔除最旧的，计算平均值（兼顾实时性和滤波效果）
static void Current_Slide_Filter(float i_u, float i_v, float i_bus)
{
    // 循环覆盖滤波缓冲区
    i_u_filter_buf[filter_index] = i_u;
    i_v_filter_buf[filter_index] = i_v;
    i_bus_filter_buf[filter_index] = i_bus;
    // 索引自增，超过次数则清零（循环）
    filter_index = (filter_index + 1) % FILTER_TIMES;
    
    // 计算滑动平均值
    float sum_u = 0, sum_v = 0, sum_bus = 0;
    for (uint8_t i = 0; i < FILTER_TIMES; i++)
    {
        sum_u += i_u_filter_buf[i];
        sum_v += i_v_filter_buf[i];
        sum_bus += i_bus_filter_buf[i];
    }
    // 存储滤波后的电流值
    i_u_filt = sum_u / FILTER_TIMES;
    i_v_filt = sum_v / FILTER_TIMES;
    i_bus_filt = sum_bus / FILTER_TIMES;
}


// 过流检测（带延时确认，防误触） ****************************************************************************
static uint8_t Over_Current_Detect(void)
{   
    // 过流判断：取三相电流的绝对值
    float i_u_abs = abs(i_u_filt);
    float i_v_abs = abs(i_v_filt);
    float i_bus_abs = abs(i_bus_filt);
    float i_max = max(i_u_abs, max(i_v_abs, i_bus_abs)); // 取三相最大电流
    //float i_max = motor_i_q; 
    
    // 延时确认：连续N次超过阈值，才判定为真实过流
    if (i_max > OVER_CURR_THRESH_Con)
    {
        over_curr_cnt_con++; // 超过阈值，计数+1
        // 连续OVER_CURR_CNT次超过阈值，判定为真实过流
        if (over_curr_cnt_con >= OVER_CURR_CNT_Con)
        {
            over_curr_flag = 1;
            return 1; // 触发过流保护
        }
    }
    else
    {
        over_curr_cnt_con = 0; // 未超过阈值，计数清零
        over_curr_flag = 0;
    }

    // 延时确认：连续N次超过阈值，才判定为真实过流
    if (i_max > OVER_CURR_THRESH_Ins)
    {
        over_curr_cnt_ins++; // 超过阈值，计数+1
        // 连续OVER_CURR_CNT次超过阈值，判定为真实过流
        if (over_curr_cnt_ins >= OVER_CURR_CNT_Ins)
        {
            over_curr_flag = 1;
            return 1; // 触发过流保护
        }
    }
    else
    {
        over_curr_cnt_ins = 0; // 未超过阈值，计数清零
        over_curr_flag = 0;
    }

    return 0; // 无过流/瞬时毛刺，不触发
}


// 过流保护执行函数 *******************************************************************************************
static void Over_Current_Protect(void)
{
    if (over_curr_flag)
    {
        // 硬件级紧急关断PWM
        TIM8->BDTR &= ~TIM_BDTR_MOE;
        PWM_Stop();
        printf("OverCurrent Protection is Triggered \r\n");
        HAL_GPIO_WritePin(Led2_GPIO_Port, Led2, GPIO_PIN_SET);  // 亮灯提示
    }
}


// 过流恢复运行函数 *******************************************************************************************
void Over_Current_Recovery(void)
{
    over_curr_flag = 0;
    HAL_GPIO_WritePin(Led2_GPIO_Port, Led2, GPIO_PIN_RESET);
    for (uint8_t i = 0; i < FILTER_TIMES; i++)
    {
      i_u_filter_buf[filter_index] = 0;
      i_v_filter_buf[filter_index] = 0;
      i_bus_filter_buf[filter_index] = 0;
    }
    TIM8->BDTR |= TIM_BDTR_MOE; // 硬件级重新使能MOE位
    PWM_Start();
    printf("Over Current Recovery\r\n");
}


// NCP18XH103F03RB 参数 **************************************************************************************
#define R25         10000.0f // 25℃标称阻值10kΩ
#define T25         298.15f  // 25℃转开尔文温度
#define B_25_50     3380.0f  // 25/50℃ B值
#define B_25_85     3434.0f  // 25/85℃ B值
#define K2C         273.15f  // 开尔文转摄氏温度常数

// 温度计算函数
// 根据电路，3.3/(10k+R)=Vadc/R
// 根据ntc芯片手册，(25-50℃)1/(t+273.15)=1/298.15+1/3380*ln(R/10k)   (25-85℃)1/(t+273.15)=1/298.15+1/3434*ln(R/10k)
static void NTC_Get_Temperature(uint16_t* adc2_buf, float* temp1, float* temp2, float* temp_final)
{
    float Vadc1 = ADC_VOLTAGE * (float)adc2_buf[1] / ADC_RESOLUTION;
    float Vadc2 = ADC_VOLTAGE * (float)adc2_buf[2] / ADC_RESOLUTION;

    float ntc1 = Vadc1 / (ADC_VOLTAGE - Vadc1);
    float ntc2 = Vadc2 / (ADC_VOLTAGE - Vadc2);

    float ln_R1, ln_R2;
    arm_vlog_f32(&ntc1, &ln_R1, 1);
    arm_vlog_f32(&ntc2, &ln_R2, 1);

    // 25~50℃精准计算
    float T1_K = 1.0f / ( (1.0f/T25) + (ln_R1 / B_25_50) );
    *temp1 = T1_K - K2C; // 开尔文转摄氏温度

    // 25~85℃精准计算
    float T2_K = 1.0f / ( (1.0f/T25) + (ln_R2 / B_25_85) );
    *temp2 = T2_K - K2C;

    // 50℃以下用3380，50℃以上用3434，全温段±1℃精度
    //*temp_final = (*temp1 <= 50.0f) ? *temp1 : *temp2;
    *temp_final = *temp2; // 第一个温度采样异常，我懒得拆了
}


/* USER CODE END 1 */
