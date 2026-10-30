/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    hdma_spi1_rx.Instance = DMA2_Channel2;
    hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);

    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA2_Channel3;
    hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);

    /* SPI1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  ******************************************************************************
  * @attention
  * 
  * 功能：mt6816磁编码相关函数
  * 作者: B站UP 点灯子曰 -- https://space.bilibili.com/3546977746815362
  * 授权: 移植/再开发请注明原作者，商业行为请联系原作者
  *
  ******************************************************************************
  */ 

/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/

/******************************************* 功能：mt6816磁编码读取角度 ***********************************************/

#include "foc.h"
#include <stdio.h>

// 编码器计算后得出的弧度值
float encoder_angle = 0;  
// 电机多圈累加弧度值 
float cumulative_encoder_angle = 0;
// 复位标志，用于只赋一次值
uint8_t motor_angle_reset_flag = 1;

uint32_t raw_angle = 0;
uint8_t Spi_TxData[4] = {0x83, 0xff, 0xff, 0xff}; 
uint8_t Spi_RxData[4] = {0};
//uint8_t spi1_dma_done_flag = 0;

#define MT6816_NCS_LOW()  HAL_GPIO_WritePin(MT6816_NCS_GPIO_Port, MT6816_NCS_Pin, GPIO_PIN_RESET)
#define MT6816_NCS_HIGH() HAL_GPIO_WritePin(MT6816_NCS_GPIO_Port, MT6816_NCS_Pin, GPIO_PIN_SET)
#define MT6816_ANGLE_MAX  16383U    // 14位最大值

static void multi_circle_angle_accumulation(void);


// 非阻塞启动 SPI1 DMA 全双工 *******************************************************************
void MT6816_Start_DMA_Read(void)
{
    //spi1_dma_done_flag = 0; // 清零DMA完成标志，准备新一次收发
    MT6816_NCS_LOW();       // CS拉低
    HAL_SPI_TransmitReceive_DMA(&hspi1, Spi_TxData, Spi_RxData, 0x03);  //模式3
}


// SPI DMA 收发完成回调 *************************************************************************
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        //spi1_dma_done_flag = 1;  // 置DMA完成标志
        MT6816_NCS_HIGH();       // 拉高CS
        raw_angle = (((Spi_RxData[1]&0x00ff)<<8)|(Spi_RxData[2]&0x00fc))>>2;  //2字节拼接得到16位数据再右移2位得到14位数据
        encoder_angle = raw_angle / (float)MT6816_ANGLE_MAX * 2.0f * PI;   // 编码器计算后得出的弧度值
        multi_circle_angle_accumulation();   // 电机多圈累加弧度值 
    }
}


// 电机多圈累加弧度值计算函数 **********************************************************************
static void multi_circle_angle_accumulation(void)
{
    static float encoder_angle_last = 0;

    // 防止浮点计算或采集误差，超过2PI范围
    encoder_angle = min(encoder_angle, 2*PI);
    encoder_angle = max(encoder_angle, -2*PI);
    
    // encoder_angle_last默认值是0，不能用于计算，需要赋初值
    if (motor_angle_reset_flag)
    {
      // 初始化关键变量（复位后重新赋值，而非依赖static初值）
      encoder_angle_last = encoder_angle;    // 赋当前编码器实时角度为初值
      // 清零复位标志（仅初始化一次）
      motor_angle_reset_flag = 0;
    }
      
    float _encoder_angle = encoder_angle;
    // 角度差值，用于累计多圈角度
    float diff_angle = cycle_diff(_encoder_angle - encoder_angle_last, 2 * PI);
    // 电角度反向了，这里差值也反向一下
    diff_angle = -diff_angle;
    encoder_angle_last = _encoder_angle;
    // 实现周期操作，将cumulative_encoder_angle转到周期内
    cumulative_encoder_angle = cycle_diff(cumulative_encoder_angle + diff_angle, position_cycle);
}

/* USER CODE END 1 */
