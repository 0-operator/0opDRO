/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "usbd_cdc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define SHOW_LOW_VOLTAGE
#define countof(a) (sizeof(a)/sizeof(a[0]))
#define UART_STRING_SIZE (20)
#define AT_DELAY (100)
#define TACH_TIMEOUT_SEC (5)
#define LOOP_PERIOD (200)
#define htim_X (htim2)
#define htim_Y (htim3)
#define htim_Z (htim5)
#define htim_W (htim1)
#define htim_T (htim9)
#define VOLTAGE_LIMIT (3.5)
#define VOLTAGE_DEVIDER (4)
#define VOLTAGE_BATTERY_NUMBER (1)
#define ADC_REF (3.3)
#define ADC_MAX (0xFFF)
#define VOLTAGE_LIMIT_ADC (VOLTAGE_LIMIT*VOLTAGE_BATTERY_NUMBER/VOLTAGE_DEVIDER/ADC_REF*ADC_MAX)
#define TO_VOLTAGE(adc) ((float)adc*VOLTAGE_DEVIDER*ADC_REF/ADC_MAX)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

char g_strRX[2] =
{ 0 };
char g_strUart[UART_STRING_SIZE] =
{ 0 };
char g_strUsb[UART_STRING_SIZE] =
{ 0 };
char g_strVersion[40] =
{ 0 };
volatile bool g_probeChanged = false;
volatile long g_overflowX = 0, g_overflowY = 0, g_overflowZ = 0,
		g_overflowW = 0;
volatile uint32_t g_adcVoltage = 0;
volatile bool g_tachStarted = false;
volatile uint64_t g_period = 0;
volatile uint32_t g_tachCaptured = 0;
volatile uint32_t g_tachCapturedPrev = 0;
volatile uint32_t g_tachCapturedOverflowCounter = 0;
volatile uint32_t g_tachOvercapture = 0;
volatile uint32_t g_tachOverflow = 0;
volatile uint16_t g_tachInputPsc = 0;
uint16_t g_tachTimeout = 0;
volatile bool g_receiveAllowed = false;
volatile bool g_showDiagnostic = false;
volatile bool g_unexpectedInterrurt = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

bool IsUartStringReady(char* p)
{
	return !!strchr(p, '\n') || !!strchr(p, '\r');
}

void ResetUartString(char* p)
{
	*p = 0;
}

void WaitForUartStringReady(int32_t time)
{
	int32_t counter = 0;
	while (!IsUartStringReady(g_strUart) && (time > counter++))
		HAL_Delay(1);
	printf("%s\n", g_strUart);
	if (IsUartStringReady(g_strUart))
	{
		const char *versionTag = "VERSION:";
		const char *begin = strstr(g_strUart, versionTag);
		if (begin)
		{
			const size_t end = strcspn(g_strUart, "\r\n");
			strncpy(g_strVersion, begin, end - (begin - g_strUart));
		}
	}
	ResetUartString(g_strUart);
	printf("wait %d\n", (int) counter);
}

void SendATCommand(char *command)
{
	ResetUartString(g_strUart);
	HAL_UART_Transmit(&huart1, (uint8_t*) command, strlen(command), 0x1000);
	WaitForUartStringReady(AT_DELAY);
}

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
  HAL_EnableDBGSleepMode();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, (uint8_t*) g_strRX, 1);

	char buf[100] =
	{ 0 };
	HAL_GPIO_WritePin(ATCMD_GPIO_Port, ATCMD_Pin, GPIO_PIN_SET);
	HAL_Delay(AT_DELAY);
	SendATCommand("AT\r\n");
	HAL_Delay(5 * AT_DELAY);
//	SendATCommand("AT+NAME?\r\n");
//	SendATCommand("AT+UART?\r\n");
//	SendATCommand("AT+PSWD?\r\n");
	SendATCommand("AT+VERSION?\r\n");
	SendATCommand("AT+NAME=0opDRO\r\n");
	SendATCommand("AT+UART=38400,0,0\r\n");
	SendATCommand("AT+RESET\r\n");
	HAL_GPIO_WritePin(ATCMD_GPIO_Port, ATCMD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ERROR_GPIO_Port, ERROR_Pin, GPIO_PIN_RESET);
	HAL_Delay(AT_DELAY);
	HAL_GPIO_WritePin(ERROR_GPIO_Port, ERROR_Pin, GPIO_PIN_SET);

	g_receiveAllowed = true;

	HAL_TIM_Base_Start_IT(&htim_X);
	HAL_TIM_Base_Start_IT(&htim_Y);
	HAL_TIM_Base_Start_IT(&htim_Z);
	HAL_TIM_Base_Start_IT(&htim_W);
	HAL_TIM_Base_Start_IT(&htim_T);
	HAL_TIM_IC_Start_IT(&htim_T, TIM_CHANNEL_1);

	const uint64_t overflowValueX = (uint64_t) htim_X.Instance->ARR + 1;
	const uint32_t shiftX = overflowValueX / 2;
	htim_X.Instance->CNT = shiftX;
	const uint64_t overflowValueY = (uint64_t) htim_Y.Instance->ARR + 1;
	const uint32_t shiftY = overflowValueY / 2;
	htim_Y.Instance->CNT = shiftY;
	const uint64_t overflowValueZ = (uint64_t) htim_Z.Instance->ARR + 1;
	const uint32_t shiftZ = overflowValueZ / 2;
	htim_Z.Instance->CNT = shiftZ;
	const uint64_t overflowValueW = (uint64_t) htim_W.Instance->ARR + 1;
	const uint32_t shiftW = overflowValueW / 2;
	htim_W.Instance->CNT = shiftW;
	g_probeChanged = !HAL_GPIO_ReadPin(PROBE_GPIO_Port, PROBE_Pin);
	bool probe = g_probeChanged;
	g_overflowX = 0;
	g_overflowY = 0;
	g_overflowZ = 0;
	g_overflowW = 0;
	g_unexpectedInterrurt = false;
	uint64_t clockPerMin = 60 * (uint64_t)SystemCoreClock;
	g_tachTimeout = (uint64_t) SystemCoreClock * TACH_TIMEOUT_SEC
			/ (htim_T.Instance->ARR + 1);
	uint32_t ledCounter = 0;
	uint32_t ledCounterOvercapture = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		bool connected = HAL_GPIO_ReadPin(LED_GPIO_Port, connected_Pin);
		++ledCounter;
		uint32_t ledCounterMod = ledCounter % 20;
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
				connected ^ !ledCounterMod ? GPIO_PIN_RESET : GPIO_PIN_SET);

		uint64_t period = g_period;
		if (g_tachOvercapture)
		{
			ledCounterOvercapture = 4;
			g_tachOvercapture = 0;
			period = 0;
		}
		bool error = (0 < ledCounterOvercapture)
				&& (9 == ledCounterMod || 11 == ledCounterMod
						|| 13 == ledCounterMod);
#ifdef SHOW_LOW_VOLTAGE
		error = error || ((g_adcVoltage < VOLTAGE_LIMIT_ADC) & !ledCounterMod);
#endif
		error ^= g_unexpectedInterrurt;
		HAL_GPIO_WritePin(ERROR_GPIO_Port, ERROR_Pin,
				error ? GPIO_PIN_RESET : GPIO_PIN_SET);
		if (!ledCounterMod && 0 < ledCounterOvercapture)
			--ledCounterOvercapture;

		HAL_ADC_Start_IT(&hadc1);

		if (g_probeChanged != probe)
		{
			sprintf(buf, "p%d;", g_probeChanged);
			HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), 0x1000);
			CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
			probe = g_probeChanged;
		}
		else if (g_showDiagnostic)
		{
			sprintf(buf, "Version 1.0\r\nBluetooth: %s\r\nBattery %.1fV\r\nSystemCoreClock %lu\r\n",
					g_strVersion, TO_VOLTAGE(g_adcVoltage), SystemCoreClock);
			HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), 0x1000);
			CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
			g_showDiagnostic = false;
			HAL_Delay(2000);
		}
		else
		{
			int32_t x = htim_X.Instance->CNT - shiftX
					+ g_overflowX * overflowValueX;
			int32_t y = htim_Y.Instance->CNT - shiftY
					+ g_overflowY * overflowValueY;
			int32_t z = htim_Z.Instance->CNT - shiftZ
					+ g_overflowZ * overflowValueZ;
			int32_t w = htim_W.Instance->CNT - shiftW
					+ g_overflowW * overflowValueW;
			uint32_t tach = !period ? 0 : clockPerMin / period;
			sprintf(buf, "x%ld;y%ld;z%ld;w%ld;t%lu;p%u;\r\n", -x, -y, -z, -w, tach,
					g_probeChanged);
			HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), 0x1000);
			CDC_Transmit_FS((uint8_t*) buf, strlen(buf));
		}

		// Restart tachometer
		if(!(htim_T.Instance->CCER & TIM_CCER_CC1E_Msk))
			htim_T.Instance->CCER |= TIM_CCER_CC1E;
		HAL_Delay(LOOP_PERIOD);
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TIMER_W_LIMIT;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIMER_X_Z_LIMIT;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIMER_Y_LIMIT;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = TIMER_X_Z_LIMIT;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 50000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ATCMD_GPIO_Port, ATCMD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ERROR_GPIO_Port, ERROR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : connected_Pin */
  GPIO_InitStruct.Pin = connected_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(connected_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ATCMD_Pin */
  GPIO_InitStruct.Pin = ATCMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ATCMD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ERROR_Pin */
  GPIO_InitStruct.Pin = ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ERROR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PROBE_Pin */
  GPIO_InitStruct.Pin = PROBE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PROBE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == PROBE_Pin)
	{
		g_probeChanged = !HAL_GPIO_ReadPin(PROBE_GPIO_Port, PROBE_Pin);
	}
}

void Receive(char* p)
{
	if (!g_receiveAllowed)
		return;
	if (!IsUartStringReady(p))
		return;
	if (strstr(p, "Diag"))
		g_showDiagnostic = true;
	ResetUartString(p);
}

void ConcatSafe(char *dest, const char *src, const int destSize)
{
	int destLength = strlen(dest);
	int srcLength = strlen(src);
	if (destLength + srcLength < (destSize - 1))
		strcat(dest, src);
	else
		memset(dest, 0, destSize);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		ConcatSafe(g_strUart, g_strRX, countof(g_strUart));
		memset(g_strRX, 0, countof(g_strRX));
		HAL_UART_Receive_IT(&huart1, (uint8_t*) g_strRX, 1);
		Receive(g_strUart);
	}
}

#define OVERFLOW_TIMER(htim, overflow)\
{\
	if (htim->Instance->CR1 & TIM_CR1_DIR)\
		--overflow;\
	else\
		++overflow;\
}

// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim_X)
	{
//		OVERFLOW_TIMER(htim, g_overflowX);
		g_unexpectedInterrurt = true;
	}
	if (htim == &htim_Y)
	{
		OVERFLOW_TIMER(htim, g_overflowY);
	}
	if (htim == &htim_Z)
	{
//		OVERFLOW_TIMER(htim, g_overflowZ);
		g_unexpectedInterrurt = true;
	}
	if (htim == &htim_W)
	{
		OVERFLOW_TIMER(htim, g_overflowW);
	}

	if (htim == &htim_T )
	{
		++g_tachCapturedOverflowCounter;
		if (g_tachTimeout < g_tachCapturedOverflowCounter)
		{
			g_period = 0;
			htim->Instance->CCMR1 &= ~TIM_CCMR1_IC1PSC_Msk;
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim_T)
	{
		if (!g_tachStarted)
		{
			g_tachCapturedOverflowCounter = 0;
			htim->Instance->SR &= ~TIM_SR_CC1OF_Msk;
			g_tachStarted = true;
			g_tachCapturedPrev = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		}
		else
		{
			g_tachCaptured = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			// Stop tachometer, it will be restarted from the main loop
			htim->Instance->CCER &= ~TIM_CCER_CC1E_Msk;
			g_tachOverflow = g_tachCapturedOverflowCounter;
			g_period = ((g_tachCaptured
					+ (g_tachOverflow * (htim->Instance->ARR + 1)))
					- g_tachCapturedPrev);
			g_tachInputPsc = (htim->Instance->CCMR1 & TIM_CCMR1_IC1PSC_Msk)
					>> TIM_CCMR1_IC1PSC_Pos;
			g_period >>= g_tachInputPsc;
			if (htim->Instance->SR & TIM_SR_CC1OF)
			{
				++g_tachOvercapture;
				htim->Instance->SR &= ~TIM_SR_CC1OF_Msk;
			}
			if (g_period < 5000)
				htim->Instance->CCMR1 |= (TIM_CCMR1_IC1PSC_0
						| TIM_CCMR1_IC1PSC_1);
			if (g_period > 50000)
				htim->Instance->CCMR1 &= ~TIM_CCMR1_IC1PSC_Msk;
			g_tachStarted = false;
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc1)
	{
		g_adcVoltage = HAL_ADC_GetValue(&hadc1);
	}
}

void USBCallback(PCD_HandleTypeDef *hpcd)
{
	USBD_HandleTypeDef *pData = (USBD_HandleTypeDef*) hpcd->pData;
	if (pData)
	{
		USBD_CDC_HandleTypeDef *pClassData =
				(USBD_CDC_HandleTypeDef*) pData->pClassData;
		if (pClassData)
		{
			char *p = (char*) (pClassData->RxBuffer);
			if (p)
			{
				ConcatSafe(g_strUsb, p, countof(g_strUsb));
				*p = 0;
				Receive(g_strUsb);
			}
		}
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
