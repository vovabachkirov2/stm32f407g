/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ctype.h>
#include <stdio.h>

#include "usbd_cdc_if.h"

#ifdef USE_LL_DRIVER
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dac.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef USER_ADC_BUFFER_SIZE
#error  USER_ADC_BUFFER_SIZE macro is not defined in STM32CubeMX project file
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/** Значения АЦП (два буфера ПДП) */
static uint16_t USER_ADC_Buffer[2 * USER_ADC_BUFFER_SIZE];

/** Накопленная сумма вторичного усреднения */
static uint32_t USER_ADC_Summa = 0;

/** Количество значений вторичного усреднения */
static uint32_t USER_ADC_Count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
#ifdef USE_LL_DRIVER
static void USER_DMAConvCplt(DMA_HandleTypeDef *hdma);
static void USER_DMAHalfConvCplt(DMA_HandleTypeDef *hdma);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Основная функция программы
  * @retval Без возврата
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
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
#ifdef USE_LL_DRIVER
	/* Запуск работы АЦП и ЦАП с использованием функций нижнего уровня */

  /* Enable the ADC peripheral */
	LL_ADC_Enable(ADC1);
    
  /* Enable ADC DMA mode */
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	
  /* Delay for ADC stabilization time */
  HAL_Delay(1);

  /* Set the DMA transfer complete callback */
  hdma_adc1.XferCpltCallback = USER_DMAConvCplt;

  /* Set the DMA half transfer complete callback */
  hdma_adc1.XferHalfCpltCallback = USER_DMAHalfConvCplt;
    
  /* Start the DMA channel for regular ADC channel with double-sized buffer */
  HAL_DMA_Start_IT(&hdma_adc1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA), (uint32_t)USER_ADC_Buffer, 2 * USER_ADC_BUFFER_SIZE);
	
  /* Enable the DAC peripheral */
	LL_DAC_Enable(DAC1, DAC_CHANNEL_1);
    
	fprintf(stderr, "ADC1 channel 0 and DAC1 channel 1, started via LL\r\n");
#else
	/* Запуск работы АЦП и ЦАП с использованием функций уровня абстракции периферии */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)USER_ADC_Buffer, 2 * USER_ADC_BUFFER_SIZE);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    
	fprintf(stderr, "ADC1 Channel 0 and DAC1 channel 1, started via HAL\r\n");
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	/* Включаем генерацию запросов преобразования */
  HAL_TIM_Base_Start(&htim8);
	
  /* Ждем окончания первого вторичного усреднения */
  HAL_Delay(1000);
  
  /* Включаем периодический вывод на консоль */
  HAL_TIM_Base_Start_IT(&htim7);
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(5000); // Ждем 5 секунд, т.к. больше все-равно нечего делать.
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7200;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 72;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
/**
  * @brief Усреднение данных в принятом буфере.
  *
  * Вычисляет среднее значение и добавляет его к сумме вторичного усреднения.
  *
  * @param Указатель на буфер
  * @retval Среднее значение
  */
static uint16_t USER_ADC_ProcessData(const uint16_t *data)
{
	int i;
	uint32_t sum = 0;
	uint16_t average;
	
	// Суммируем
	
	for (i = 0; i < USER_ADC_BUFFER_SIZE; i++)
	{
		sum += (uint32_t) *data++;
	}
	
	average = sum / USER_ADC_BUFFER_SIZE;
	
	// Добавляем к сумме вторичного усреднения
	
	USER_ADC_Summa += average;
	USER_ADC_Count ++;
	
	return average;
}

	
#ifdef USE_LL_DRIVER
/**
  * @brief  Функция обратного вызова ПДП (для первой половины буфера).
  *
  * Данная функция вызывается модулем HAL_DMA при заполнении первой половины буфера АЦП.
  * Осуществляет вычисление среднего значения с АЦП и его загрузку в ЦАП.
  *
  * @param  Указатель на дескриптор канала ПДП
  * @retval Нет
  */
static void USER_DMAHalfConvCplt(DMA_HandleTypeDef *hdma)   
{
  if (hdma == &hdma_adc1)
	{
		uint16_t average = USER_ADC_ProcessData(USER_ADC_Buffer);
		LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, average);
	}
	else
	{
		fprintf(stderr, "USER_DMAHalfConvCplt() called unexpected !!!\r\n");
	}
}

/**
  * @brief  Функция обратного вызова ПДП (для второй половины буфера).
  *
  * Данная функция вызывается модулем HAL_DMA при заполнении второй половины буфера АЦП.
  * Осуществляет вычисление среднего значения с АЦП и его загрузку в ЦАП.
  *
  * @param  Указатель на дескриптор канала ПДП
  * @retval Нет
  */
static void USER_DMAConvCplt(DMA_HandleTypeDef *hdma)   
{
  if (hdma == &hdma_adc1)
	{
		uint16_t average = USER_ADC_ProcessData(&USER_ADC_Buffer[USER_ADC_BUFFER_SIZE]);
		LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, average);
	}
	else
	{
		fprintf(stderr, "USER_DMAConvCplt() called unexpected !!!\r\n");
	}
}

#else /* USE_LL_DRIVER */
/**
  * @brief  Функция обратного вызова АЦП (для первой половины буфера).
  *
  * Данная функция вызывается модулем HAL_ADC при заполнении первой половины буфера АЦП.
  * Осуществляет вычисление среднего значения с АЦП и его загрузку в ЦАП.
  *
  * @param  Указатель на дескриптор АЦП
  * @retval Нет
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc == &hadc1)
	{
		uint16_t average = USER_ADC_ProcessData(USER_ADC_Buffer);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, average);
	}
	else
	{
		fprintf(stderr, "HAL_ADC_ConvHalfCpltCallback() called unexpected !!!\r\n");
	}
}

/**
  * @brief  Функция обратного вызова АЦП (для второй половины буфера).
  *
  * Данная функция вызывается модулем HAL_ADC при заполнении второй половины буфера АЦП.
  * Осуществляет вычисление среднего значения с АЦП и его загрузку в ЦАП.
  *
  * @param  Указатель на дескриптор АЦП
  * @retval Нет
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc == &hadc1)
	{
		uint16_t average = USER_ADC_ProcessData(&USER_ADC_Buffer[USER_ADC_BUFFER_SIZE]);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, average);
	}
	else
	{
		fprintf(stderr, "HAL_ADC_ConvCpltCallback() called unexpected !!!\r\n");
	}
}

#endif /* USE_LL_DRIVER */

/**
  * @brief  Функция периодического таймера.
  *
  * Данная функция вызывается раз в секунду для вывода среднего значения.
  *
  * @param  Указатель на дескриптор сработавшего таймера
  * @retval Нет
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim7)
	{
		if (USER_ADC_Count > 0)
		{
			// Завершаем вторичное усреднение
		
			uint16_t average = USER_ADC_Summa / USER_ADC_Count;
			uint16_t average_mv = 3300u * average / 4096u;       // Считаем, что Vref+ = 3.3 V
	
			fprintf(stderr, "Average from %u values is %u (%u mV)\r\n", USER_ADC_Count, average, average_mv);
		
			// Печатаем результат
		
			printf("%4u mV\r", average_mv);
		}
		else
		{
			fprintf(stderr, "Internal error: no enought conversions made !!!\r\n");
		}
		
		// Начинаем новое вторичное усреднение
		
		USER_ADC_Summa = USER_ADC_Count = 0;
	}
  else 
	{
		fprintf(stderr, "HAL_TIM_PeriodElapsedCallback() called unexpected !!!\r\n");
	}
}

/**
  * @brief  Вывод символа из стандартного потока в последовательный порт.
  *
  * Данная функция вызывается из стандартной библиотеки ввода-вывода (stdio)
  * для каждого символа, передаваемого в поток stdout. Функция осуществляет
  * буферизацию передаваемых данных до 126 символов или до получения одного
  * из символов конца строки (CR or LF) и передачу буфера через виртуальный
  * последовательный порт.
  *
  * @param  Выводимый символ
  * @retval Выводимый символ (копия входного значения)
  */
int stdout_putchar(int ch) 
{
#if 1
	static int     buflen = 0;  // длина строки
	static uint8_t buffer[128]; // буфер строки
	
	if (isprint(ch))
	{
		// добавляем в буфер вывода
		buffer[buflen++] = (uint8_t) ch;
		
		if (buflen >= sizeof(buffer) - 2)
		{
			// сброс буфера по заполнению
			CDC_Transmit_FS(buffer, buflen);
			buflen = 0;
		}
	}
	else if (ch == '\n' || ch == '\r')
	{
		buffer[buflen++] = (uint8_t) ch;
	
		// сброс буфера по переводу строки
		CDC_Transmit_FS(buffer, buflen);
		buflen = 0;
	}
#else
  uint8_t bt = (uint8_t) ch;
  HAL_UART_Transmit(&huart2, &bt, 1, 1); // timeout = 1 ms
#endif
  return ch;
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
  fprintf(stderr, "Wrong parameter's value in file %s on line %u\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
