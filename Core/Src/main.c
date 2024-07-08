/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <max7219_Yncrea2.h>
#include "iks01a3_env_sensors.h"
#include "iks01a3_motion_sensors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef struct displayFloatToInt_s
{
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t out_int;
  uint32_t out_dec;
} displayFloatToInt_t;


typedef struct IKS01A3_MOTION_SENSOR_Axes_t
{
  int32_t x;
  int32_t y;
  int32_t z;
} MAGNETO_AXES;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUF_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
 char dataOut1[MAX_BUF_SIZE];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t analogValue;
uint32_t flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Temperature;

void TIM6_IRQ(void)		// Gestion et Affichage Température
{
	IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_TEMPERATURE, &Temperature);
    Affichage_TEMP();
    display_temp();
}



 void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)		// Formatage
{
  if(in >= 0.0f)
  {
    out_value->sign = 0;
  }else
  {
    out_value->sign = 1;
    in = -in;
  }

  in = in + (0.5 / pow(10, dec_prec));
  out_value->out_int = (int32_t)in;
  in = in - (float)(out_value->out_int);
  out_value->out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}


void Affichage_TEMP(void)		// Affichage Température console

{
	displayFloatToInt_t out_value;
	floatToInt(Temperature, &out_value, 2);

	snprintf(dataOut1, MAX_BUF_SIZE, "Temperature: %c%d.%02d degC ", ((out_value.sign > 0) ? '-' : '+'), (int)out_value.out_int, (int)out_value.out_dec);
	printf("%s \r\n", dataOut1);
}

void display_temp(void)		// Affichage Température LCD
{
	IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_TEMPERATURE, &Temperature);
	char buff[4];
	displayFloatToInt_t out_value;
	floatToInt(Temperature, &out_value, 2);
	sprintf(buff, "%d", (int)out_value.out_int);
	MAX7219_DisplayChar('1',buff[0],0);
	MAX7219_DisplayChar('2',buff[1],1);
	sprintf(buff, "%d", (int)out_value.out_dec);
	MAX7219_DisplayChar('3',buff[0],0);
	//MAX7219_DisplayChar('3','.');
	MAX7219_DisplayChar('4',buff[1],0);
}

void adcFunction(void)		// Releve des valeurs ADC
{
  HAL_ADC_Start(&hadc); // Start ADC with interrupt
  HAL_ADC_PollForConversion(&hadc,100);
  analogValue= HAL_ADC_GetValue(&hadc);
  // Borne supérieure à 2100 car bug au delà
  if (analogValue>2100)
  {
	  analogValue=2100;
  }
  HAL_ADC_Stop(&hadc);
  printf("ADC_Value = %lu \n\r",analogValue);//valeur max à gauche 4095

	  if(analogValue >= 0 && analogValue < 300)
  		{
	  	  	  printf("0-300");
	  	  	HAL_GPIO_WritePin(L0_GPIO_Port, L0_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin,GPIO_PIN_RESET);
  		}

	   if(analogValue >= 300 && analogValue < 600)
				{
		  	  printf("300-600");
		  	HAL_GPIO_WritePin(L0_GPIO_Port, L0_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin,GPIO_PIN_RESET);
				}

	   if(analogValue >= 600 && analogValue < 900)
				{
				printf("600-900");
				HAL_GPIO_WritePin(L0_GPIO_Port, L0_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin,GPIO_PIN_RESET);
				}

	   if(analogValue >= 900 && analogValue < 1200)
			{
				printf("900-1200");
				HAL_GPIO_WritePin(L0_GPIO_Port, L0_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin,GPIO_PIN_RESET);
				}

	   if(analogValue >= 1200 && analogValue < 1500)
				{
				printf("1200-1500");
				HAL_GPIO_WritePin(L0_GPIO_Port, L0_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin,GPIO_PIN_RESET);
				}

	   if(analogValue >= 1500)
				{
				printf("1500++");
				HAL_GPIO_WritePin(L0_GPIO_Port, L0_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin,GPIO_PIN_SET);
				}
}

void buzzer(float Delay)		// Buzzer avec délai fonction du champ
{
	Delay = 200000/Delay;
	printf("Delay = %.4f !!!! \r\n",Delay);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Démarre le buzzer
    HAL_Delay(100); // Durée du bip
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // Arrête le buzzer
    HAL_Delay(Delay); // Délai avant le prochain bip
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	IKS01A3_MOTION_SENSOR_Axes_t mag_axes;
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_ADC_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Test LCD
   MAX7219_Init();
   MAX7219_DisplayTestStart();
   HAL_Delay(2000);
   MAX7219_DisplayTestStop();

   // Test de comm' des capteurs
if(IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_TEMPERATURE) != BSP_ERROR_NONE)
{
	printf("Defaut init Temp !!! \r\n");
}

if(IKS01A3_MOTION_SENSOR_Init(IKS01A3_LIS2MDL_0,MOTION_MAGNETO) != BSP_ERROR_NONE)
{
	printf("Defaut init Magneto !!! \r\n");
}

// Init Buzzer
HAL_TIM_PWM_Init(&htim3);
HAL_TIM_Base_Start_IT(&htim3);

// Test Timer interruption Température
if(HAL_TIM_Base_Start_IT(&htim6)!= HAL_OK)
{
	Error_Handler();
}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

while (1)
{
	{
	    // Gestion et Recupération valeur magneto
	    IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LIS2MDL_0, MOTION_MAGNETO, &mag_axes);
	    printf("\n X = %ld Y = %ld Z = %ld \n ", mag_axes.x, mag_axes.y, mag_axes.z);
	    float moyenne_mag = sqrt(mag_axes.x * mag_axes.x + mag_axes.y * mag_axes.y + mag_axes.z * mag_axes.z);
	    printf("Magnitude moyenne: %ld\n", (long)moyenne_mag);
	    fflush(stdout);
	    // Buzzer fonction du champ
	    buzzer(moyenne_mag);

	    // ADC et LED
	    adcFunction();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_0;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 1000;
  AnalogWDGConfig.LowThreshold = 0;
  if (HAL_ADC_AnalogWDGConfig(&hadc, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2278;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1139;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L0_Pin|L1_Pin|L2_Pin|L3_Pin
                          |L4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : L0_Pin L1_Pin L2_Pin L3_Pin
                           L4_Pin */
  GPIO_InitStruct.Pin = L0_Pin|L1_Pin|L2_Pin|L3_Pin
                          |L4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
	//ITM_SendChar(ch);	// Printf sur la console, mais le debugger ne fonctionne pas
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
	return(ch);
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
