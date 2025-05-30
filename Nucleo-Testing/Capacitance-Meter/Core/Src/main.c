/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>

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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int Time_Constant_Capacitance_Measurement(void);
void Ohmmeter(void);
void Inductance_Meter(void);
void Print_Averaged_ADC_Value(void);
int Get_ADC_Value(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM16_Init(void);
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
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim16); // Start the timer for time measurement

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
    Print_Averaged_ADC_Value();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TogglePower_GPIO_Port, TogglePower_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TogglePower_Pin */
  GPIO_InitStruct.Pin = TogglePower_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TogglePower_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int iters = 0;
int itersMax = 100;
uint32_t adcValues[100] = {0}; 
uint16_t times[1] = {0};

// Output in nC
int Time_Constant_Capacitance_Measurement(void) {
  // First charge for 0.1s
  // Record for 1s and output over uart

  if (HAL_GetTick() < 1000){
    HAL_GPIO_WritePin(TogglePower_GPIO_Port, TogglePower_Pin, GPIO_PIN_RESET);
    return 0;
  }

  HAL_GPIO_WritePin(TogglePower_GPIO_Port, TogglePower_Pin, GPIO_PIN_SET);

  iters++;

  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
  adcValues[iters - 1] = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);

  times[iters - 1] = __HAL_TIM_GET_COUNTER(&htim16);

  int total_length_written = 0;
  if (iters % itersMax == 0){
    char buffer[2000];
    for (int i = 0; i < itersMax; i++){
      int length_written = snprintf(buffer + total_length_written, sizeof(buffer) - total_length_written, "%u,%lu\r\n", times[i], adcValues[i]);
      total_length_written += length_written;
    }
    iters = 0;

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, total_length_written, HAL_MAX_DELAY);
  }
}

void Ohmmeter(void){
  HAL_GPIO_WritePin(TogglePower_GPIO_Port, TogglePower_Pin, GPIO_PIN_RESET);

  iters++;

  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
  adcValues[iters - 1] = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);

  times[iters - 1] = __HAL_TIM_GET_COUNTER(&htim16);

  int total_length_written = 0;
  if (iters % itersMax == 0){
    char buffer[2000];
    for (int i = 0; i < itersMax; i++){
      int length_written = snprintf(buffer + total_length_written, sizeof(buffer) - total_length_written, "%u,%lu\r\n", times[i], adcValues[i]);
      total_length_written += length_written;
    }
    iters = 0;

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, total_length_written, HAL_MAX_DELAY);
  }
}

void Inductance_Meter(void){
  if (HAL_GetTick() < 1000){
    HAL_GPIO_WritePin(TogglePower_GPIO_Port, TogglePower_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(TogglePower_GPIO_Port, TogglePower_Pin, GPIO_PIN_SET);
  }

  iters++;

  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
  adcValues[iters - 1] = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);

  times[iters - 1] = __HAL_TIM_GET_COUNTER(&htim16);

  int total_length_written = 0;
  if (iters % itersMax == 0){
    char buffer[2000];
    for (int i = 0; i < itersMax; i++){
      int length_written = snprintf(buffer + total_length_written, sizeof(buffer) - total_length_written, "%u,%lu\r\n", times[i], adcValues[i]);
      total_length_written += length_written;
    }
    iters = 0;

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, total_length_written, HAL_MAX_DELAY);
  }
}

void Print_Averaged_ADC_Value(void) {
  // First, collect 100 samples in array
  // when full, calculate average and print to UART

  iters++;
  adcValues[iters - 1] = Get_ADC_Value();

  if (iters % itersMax == 0){
    // Calculate average
    uint32_t sum = 0;
    for (int i = 0; i < itersMax; i++){
      sum += adcValues[i];
    }
    int average = sum / itersMax;

    // Print to UART
    char buffer[50];
    int length_written = snprintf(buffer, sizeof(buffer), "Average ADC Value: %d\r\n", average);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, length_written, HAL_MAX_DELAY);

    iters = 0; // Reset the counter
  }
}

int Get_ADC_Value(void){
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
  int64_t adcValue = (int64_t)HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);

  if (adcValue == 0) {
    return 0;
  }

  //int adcErrorTerm = (int)(2.994 + (1.730E-01 * adcValue) - (1.295E-04 * pow(adcValue, 2)) + (1.613E-08 * pow(adcValue, 3)));

  // All constant multiplied by 1E11 to avoid floating point arithmetic
  int64_t preadcErrorTerm = (int64_t)(299400000000 + (17300000000 * adcValue) - (12950000 * adcValue * adcValue) + (1613 * adcValue * adcValue * adcValue));
  //adcErrorTerm = adcErrorTerm / 1E11;
  //int64_t adcErrorTerm = 0;

  int32_t adcErrorTerm = (int32_t)(preadcErrorTerm / 1E11);

  // max: 922337200000000000
  int64_t term1 = (int64_t)2994000000000LL; 
  int64_t term2 = (int64_t)17300000000LL * (int64_t)adcValue;
  int64_t term3 = (int64_t)12950000LL * (int64_t)adcValue * (int64_t)adcValue;
  int64_t term4 = (int64_t)1613LL * (int64_t)adcValue * (int64_t)adcValue * (int64_t)adcValue;
  
  // Log only each 64-bit component
  char buffer[200];
  int length_written = snprintf(buffer, sizeof(buffer),
    "ADC: %ld, T1: %lu|%lu, T2: %lu|%lu, T3: %lu|%lu, T4: %lu|%lu\r\n",
    adcErrorTerm,
    (uint32_t)(term1 >> 32), (uint32_t)(term1),
    (uint32_t)(term2 >> 32), (uint32_t)(term2),
    (uint32_t)(term3 >> 32), (uint32_t)(term3),
    (uint32_t)(term4 >> 32), (uint32_t)(term4)
  );
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, length_written, HAL_MAX_DELAY);


  if (adcValue - adcErrorTerm > 5000){
    return 0;
  }

  return adcValue - adcErrorTerm;
}

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

#ifdef  USE_FULL_ASSERT
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
