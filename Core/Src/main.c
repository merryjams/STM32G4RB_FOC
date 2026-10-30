/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "foc.h"
#include "open_serial_debugger.h"
#include "kalman_filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */


  // 上电led闪两下
  Leds_Blink();

  // CAN配置、使能
  my_FDCAN1_init();

  // 启动PWM输出
  PWM_Start();
  HAL_Delay(1);

  // 测试pwm输出是否正常，uvw相接口输出是否正常
  //set_pwm_duty(0.2, 0.2, 0.2, 0.9);

  // 转子电气零点校准 
  set_rotor_zero_angle();

  // 开启adc采集温度、电位器电压的触发定时器
  HAL_TIM_Base_Start_IT(&htim6);

  // ADC校准
  HAL_ADCEx_Calibration_Start(&hadc1, 0);
  HAL_ADCEx_Calibration_Start(&hadc2, 0);
  // 启动ADC1+DMA：三相电流采样
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, ADC1_CH_NUM);
  // 启动ADC2+DMA：温度+电位器采样
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC2_CH_NUM);


  // 开启速度计算定时器
  HAL_TIM_Base_Start_IT(&htim7);

  // 卡尔曼滤波初始化
  kalman_filter_init(&motor_speed_kalman_filter, KF_R, KF_Q);

  #ifdef MODULE_OPEN_SERIAL_DEBUGGER_EN
    // 使用open_serial_debugger/vofa上位机
    open_serial_debugger_init(&huart1);
  #endif

  uint32_t last_print_time = 0;

  // 使用vofa justFloat格式
  float data_print[8] = {0};
  uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};  // 帧尾

  // 积分项累积值显式清零
  pid_i_reset();

  // // 全部调试好后使用
  // set_motor_pid_single(&pid_position, POSITION_MODE_POS_P, POSITION_MODE_POS_I, POSITION_MODE_POS_D);
  // set_motor_pid_single(&pid_speed, SPEED_MODE_SPEED_P, SPEED_MODE_SPEED_I, SPEED_MODE_SPEED_D);
  // set_motor_pid_single(&pid_torque_q, TORQUE_MODE_Q_P, TORQUE_MODE_Q_I, TORQUE_MODE_Q_D);
  // set_motor_pid_single(&pid_position_PS_Ctrl, POS_SPD_MODE_POS_P, POS_SPD_MODE_POS_I, POS_SPD_MODE_POS_D);
  // set_motor_pid_single(&pid_speed_PS_Ctrl, POS_SPD_MODE_SPEED_P, POS_SPD_MODE_SPEED_I, POS_SPD_MODE_SPEED_D);
  // set_motor_pid_single(&pid_position_PT_Ctrl, POS_TOR_MODE_POS_P, POS_TOR_MODE_POS_I, POS_TOR_MODE_POS_D);
  // set_motor_pid_single(&pid_torque_q_PT_Ctrl, POS_TOR_MODE_Q_P, POS_TOR_MODE_Q_I, POS_TOR_MODE_Q_D);
  // set_motor_pid_single(&pid_speed_ST_Ctrl, SPD_TOR_MODE_SPD_P, SPD_TOR_MODE_SPD_I, SPD_TOR_MODE_SPD_D);
  // set_motor_pid_single(&pid_torque_q_ST_Ctrl, SPD_TOR_MODE_Q_P, SPD_TOR_MODE_Q_I, SPD_TOR_MODE_Q_D);
  // set_motor_pid_single(&pid_position_PST_Ctrl, POS_SPD_TOR_MODE_POS_P, POS_SPD_TOR_MODE_POS_I, POS_SPD_TOR_MODE_POS_D);
  // set_motor_pid_single(&pid_speed_PST_Ctrl, POS_SPD_TOR_MODE_SPD_P, POS_SPD_TOR_MODE_SPD_I, POS_SPD_TOR_MODE_SPD_D);
  // set_motor_pid_single(&pid_torque_q_PST_Ctrl, POS_SPD_TOR_MODE_Q_P, POS_SPD_TOR_MODE_Q_I, POS_SPD_TOR_MODE_Q_D);


  // 测试can
  //uint8_t send_data[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08}; // 待发送数据

  // 测试电流采样是否正常
  // set_pwm_duty(0.2, 0, 0, 0.9);
  // HAL_Delay(500);
  // printf("%f,%f,%f\n", motor_i_u, motor_i_v, motor_i_w);
  // set_pwm_duty(0, 0.2, 0, 0.9);
  // HAL_Delay(500);
  // printf("%f,%f,%f\n", motor_i_u, motor_i_v, motor_i_w);
  // set_pwm_duty(0, 0, 0.2, 0.9);
  // HAL_Delay(500);
  // printf("%f,%f,%f\n", motor_i_u, motor_i_v, motor_i_w);
  // set_pwm_duty(0, 0, 0, 0.9);

  // 位置环测试
  // motor_control_context.position = deg2rad(3600); 
  // motor_control_context.type = control_type_position;
  // // 速度环测试
  // motor_control_context.speed = deg2rad(120); 
  // motor_control_context.type = control_type_speed;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    #ifdef MODULE_OPEN_SERIAL_DEBUGGER_EN
      if (Serial_available() >= PACKET_LEN)
      {
        uint8_t packet[PACKET_LEN];
        Serial_readBytes(packet, PACKET_LEN);
        handle_packet(packet);
      }
    #endif

    // 按键
    Keys_Process();

    // 测试打印、磁编码
    if (HAL_GetTick() - last_print_time > 2)  //数字是多少就多少ms打印一次
    { 
      // can测试
      // my_FDCAN1_Transmit(send_data);
      // send_data[0]++; 

      // 磁编码mt6816测试
      //MT6816_Start_DMA_Read();
      //printf("encoder_angle = %.4f   cumulative_encoder_angle = %.4f\r\n", encoder_angle, cumulative_encoder_angle);

      // adc测试
      //printf("potentiometer = %.2f   temperature1 = %.1f   temperature2 = %.1f\r\n", potentiometer, temperature1, temperature2);
      //printf("motor_i_u motor_i_v motor_i_bus: %.2f, %.2f, %.2f\r\n", motor_i_u, motor_i_v, motor_i_bus);

      // vofa fireWater格式 显示曲线
      //printf("set_pos  real_pos: %.1f, %.1f\r\n", rad2deg(motor_control_context.position), rad2deg(motor2load(cumulative_encoder_angle)));
      //printf("set_speed  real_speed: %.1f, %.1f\r\n", rad2deg(motor_control_context.speed), rad2deg(motor2load(motor_speed)));

      // 使用vofa justFloat格式，打印曲线更稳定
      // 分别为 设定位置 实际位置 设定速度 实际速度 设定q轴电流 实际q轴电流 温度
      data_print[0] = rad2deg(motor_control_context.position);
      data_print[1] = rad2deg(motor2load(cumulative_encoder_angle));
      data_print[2] = rad2deg(motor_control_context.speed);
      data_print[3] = rad2deg(motor2load(motor_speed));
      data_print[4] = motor_control_context.current_q;
      data_print[5] = motor_i_q;
      // data_print[7] = motor_i_u;
      // data_print[8] = motor_i_v;
      data_print[6] = temperature_final;
      data_print[7] = -rad2deg(motor_control_context.speed);
      Serial_write_DMA(&huart1, (uint8_t*)data_print, sizeof(float) * 8); 
      Serial_write_DMA(&huart1, tail, 4);

      last_print_time = HAL_GetTick();
    }

    // // can回环测试
    // if(fdcan1_RXFlag == 1)
    // {
    //   fdcan1_RXFlag = 0; 
    //   char fdcan_print_buf[64]; // 用于拼接FDCAN接收数据，足够容纳8个0xXX + 空格
    //   int idx = 0;
    //   // 循环拼接字符串到临时缓冲区，不操作DMA打印缓冲区
    //   for(int i = 0; i < 8; i++)
    //   {
    //     idx += sprintf(&fdcan_print_buf[idx], "0x%02X ", fdcan1RxData[i]);
    //   }
    //   // 整行一次性printf，仅一次写入DMA缓冲区，一次发送
    //   printf("%s\r\n", fdcan_print_buf);
    // }

    // 开环测试，看是否转动
    // for (uint16_t i = 0; i < 360; i += 20)
    // {
    //   HAL_Delay(10);
    //   foc_forward(0, 8, deg2rad(i));
    //   //printf("encoder_angle: %.1f   motor_speed: %.1f \r\n", encoder_angle, motor_speed);
    // }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
