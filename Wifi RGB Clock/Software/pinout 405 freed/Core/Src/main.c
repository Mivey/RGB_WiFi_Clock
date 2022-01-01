/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "time.h"
#include "cheapLibation.h"
#include "ColorDefinitions.h"
#include "mytime.h"
#include "usbd_cdc_if.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t flags;
	uint8_t mincnt;
	uint8_t hrcnt;
	uint8_t time;
}LEDDataShift_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMLED 60
#define GLOBAL 0
#define R_LED 3
#define B_LED 1
#define G_LED 2
#define MINMASK 0x00000001U
#define HRMASK	0x00000002U
#define LATMASK	0x00000008U
#define TIMMASK	0x00000004U
#define WIFISTATUS 0x000080U
#define WIFISTATUSD 0x000100U
#define ESPDATA 0x000040U
#define SETTIME 0x000020U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for TimerAndLatch */
osThreadId_t TimerAndLatchHandle;
const osThreadAttr_t TimerAndLatch_attributes = {
  .name = "TimerAndLatch",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for LEDShifter */
osThreadId_t LEDShifterHandle;
const osThreadAttr_t LEDShifter_attributes = {
  .name = "LEDShifter",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for RGBShift */
osThreadId_t RGBShiftHandle;
const osThreadAttr_t RGBShift_attributes = {
  .name = "RGBShift",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for RTCManage */
osThreadId_t RTCManageHandle;
const osThreadAttr_t RTCManage_attributes = {
  .name = "RTCManage",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ClockManage */
osThreadId_t ClockManageHandle;
const osThreadAttr_t ClockManage_attributes = {
  .name = "ClockManage",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for GlobalStatus */
osEventFlagsId_t GlobalStatusHandle;
const osEventFlagsAttr_t GlobalStatus_attributes = {
  .name = "GlobalStatus"
};
/* USER CODE BEGIN PV */
uint32_t data[] = {0x70007000, 0x00, 0x08080000};
uint16_t newarr[6];
//DMA_HandleTypeDef hdma_memtomem_dma1_channel1;

uint8_t rxbuff[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t txbuff[] = {0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 , 0x00, 0x00 };
uint8_t wififlag = 0;
uint8_t hrs = 0x00;
uint8_t min = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_RNG_Init(void);
void startTimerAndLatch(void *argument);
void StartLEDShift(void *argument);
void StartRGBShift(void *argument);
void StartRTCManage(void *argument);
void StartClockMngr(void *argument);

/* USER CODE BEGIN PFP */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);

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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  // reset wifi to programming mode
//  HAL_GPIO_WritePin(IO0_GPIO_Port, IO0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
//  HAL_Delay(500);
//  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);

  // enable all the slave timers
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
//  uint8_t sttest = beatsin8()



//  time_t now;
//  localtime_r(&now);
//  setenv("TZ")

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (2, sizeof(LEDDataShift_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TimerAndLatch */
  TimerAndLatchHandle = osThreadNew(startTimerAndLatch, NULL, &TimerAndLatch_attributes);

  /* creation of LEDShifter */
  LEDShifterHandle = osThreadNew(StartLEDShift, NULL, &LEDShifter_attributes);

  /* creation of RGBShift */
  RGBShiftHandle = osThreadNew(StartRGBShift, NULL, &RGBShift_attributes);

  /* creation of RTCManage */
  RTCManageHandle = osThreadNew(StartRTCManage, NULL, &RTCManage_attributes);

  /* creation of ClockManage */
  ClockManageHandle = osThreadNew(StartClockMngr, NULL, &ClockManage_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of GlobalStatus */
  GlobalStatusHandle = osEventFlagsNew(&GlobalStatus_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  sConfig.Channel = ADC_CHANNEL_15;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 12;
  sTime.Minutes = 10;
  sTime.Seconds = 45;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_MAY;
  sDate.Date = 11;
  sDate.Year = 21;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 1;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
                              |RTC_ALARMMASK_MINUTES;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_1LINE;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 20-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 168-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 16-1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 150;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 5-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 84-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 75;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 84-1;
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
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 75;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_4);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 84-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 75;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_2);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 10-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 84-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim8, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 70;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim8, TIM_CHANNEL_2);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  huart2.Init.BaudRate = 38400;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LAT2_Pin|SIN2_Pin|VOUT1_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BLK2_Pin|BLK12_Pin|BLK8_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_Pin|LAT12_Pin|SIN12_Pin|WC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LAT10_Pin|SIN10_Pin|LAT8_Pin|SIN8_Pin
                          |SIN6_Pin|LAT4_Pin|SIN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLK10_Pin|BLK6_Pin|BLK4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LAT6_GPIO_Port, LAT6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LAT2_Pin SIN2_Pin BLK2_Pin BLK12_Pin
                           BLK8_Pin VOUT1_EN_Pin */
  GPIO_InitStruct.Pin = LAT2_Pin|SIN2_Pin|BLK2_Pin|BLK12_Pin
                          |BLK8_Pin|VOUT1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin LAT12_Pin SIN12_Pin WC_Pin */
  GPIO_InitStruct.Pin = RST_Pin|LAT12_Pin|SIN12_Pin|WC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IO0_Pin */
  GPIO_InitStruct.Pin = IO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LAT10_Pin SIN10_Pin BLK10_Pin LAT8_Pin
                           SIN8_Pin SIN6_Pin BLK6_Pin LAT4_Pin
                           SIN4_Pin BLK4_Pin */
  GPIO_InitStruct.Pin = LAT10_Pin|SIN10_Pin|BLK10_Pin|LAT8_Pin
                          |SIN8_Pin|SIN6_Pin|BLK6_Pin|LAT4_Pin
                          |SIN4_Pin|BLK4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FAULT_Pin */
  GPIO_InitStruct.Pin = FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LAT6_Pin */
  GPIO_InitStruct.Pin = LAT6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LAT6_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{
		static uint16_t newarr_inc = 0x8000U;

		HAL_GPIO_WritePin(SIN12_GPIO_Port, SIN12_Pin,( ((newarr[0] & newarr_inc) != 0)  ? GPIO_PIN_SET : GPIO_PIN_RESET) );
		HAL_GPIO_WritePin(SIN2_GPIO_Port, SIN2_Pin,( ((newarr[1] & newarr_inc) != 0)  ? GPIO_PIN_SET : GPIO_PIN_RESET) );
		HAL_GPIO_WritePin(SIN4_GPIO_Port, SIN4_Pin,( ((newarr[2] & newarr_inc) != 0)  ? GPIO_PIN_SET : GPIO_PIN_RESET) );
		HAL_GPIO_WritePin(SIN6_GPIO_Port, SIN6_Pin,( ((newarr[3] & newarr_inc) != 0)  ? GPIO_PIN_SET : GPIO_PIN_RESET) );
		HAL_GPIO_WritePin(SIN8_GPIO_Port, SIN8_Pin,( ((newarr[4] & newarr_inc) != 0)  ? GPIO_PIN_SET : GPIO_PIN_RESET) );
		HAL_GPIO_WritePin(SIN10_GPIO_Port, SIN10_Pin,( ((newarr[5] & newarr_inc) != 0)  ? GPIO_PIN_SET : GPIO_PIN_RESET) );

		if (newarr_inc != 0x0001U)
			newarr_inc >>= 1;
		else
		{
			newarr_inc = 0x8000U;
			osThreadFlagsSet(TimerAndLatchHandle, LATMASK);
		}

	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	HAL_RNG_GenerateRandomNumber_IT(&hrng);
//	osEventFlagsSet(GlobalStatusHandle, 0x00010101U); // set minute flags for all threads
//	osThreadFlagsSet(RGBShiftHandle, MINMASK);
	osThreadFlagsSet(ClockManageHandle, MINMASK);
//	osThreadFlagsSet(RTCManageHandle, MINMASK);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == IO0_Pin)
	{
		if(HAL_GPIO_ReadPin(IO0_GPIO_Port, IO0_Pin) == 0)
		{
			txbuff[0] = 0x55;
//			HAL_UART_DMAResume(&huart2);
			wififlag = wififlag | WIFISTATUS;
			HAL_UART_Receive_DMA(&huart2, rxbuff, 6);
			HAL_UART_Transmit_DMA(&huart2, txbuff, 1);
			osThreadFlagsSet(ClockManageHandle, WIFISTATUS);
		}
		else
		{
			wififlag = wififlag & ~WIFISTATUS;
			HAL_UART_Abort(&huart2);
			osThreadFlagsSet(ClockManageHandle, WIFISTATUSD);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart2.Instance)
	{
		wififlag = wififlag | ESPDATA;
		osThreadFlagsSet(ClockManageHandle, ESPDATA);
//		osThreadFlagsSet(ClockManageHandle, ESPDATA);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startTimerAndLatch */
/**
  * @brief  Function implementing the TimerAndLatch thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startTimerAndLatch */
void startTimerAndLatch(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  HAL_GPIO_WritePin(BLK2_GPIO_Port, BLK2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLK4_GPIO_Port, BLK4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLK6_GPIO_Port, BLK6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLK8_GPIO_Port, BLK8_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLK10_GPIO_Port, BLK10_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLK12_GPIO_Port, BLK12_Pin, GPIO_PIN_RESET);

  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(TIMMASK, osFlagsWaitAny, osWaitForever);

	  TIM1->CR1 |= TIM_CR1_CEN;
	  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	  osThreadFlagsWait(LATMASK, osFlagsWaitAny, osWaitForever);

	  HAL_GPIO_WritePin(LAT2_GPIO_Port, LAT2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LAT4_GPIO_Port, LAT4_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LAT6_GPIO_Port, LAT6_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LAT8_GPIO_Port, LAT8_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LAT10_GPIO_Port, LAT10_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LAT12_GPIO_Port, LAT12_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LAT2_GPIO_Port, LAT2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LAT4_GPIO_Port, LAT4_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LAT6_GPIO_Port, LAT6_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LAT8_GPIO_Port, LAT8_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LAT10_GPIO_Port, LAT10_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LAT12_GPIO_Port, LAT12_Pin, GPIO_PIN_RESET);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLEDShift */
/**
* @brief Function implementing the LEDShifter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDShift */
void StartLEDShift(void *argument)
{
  /* USER CODE BEGIN StartLEDShift */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(10000);
  }

  /* USER CODE END StartLEDShift */
}

/* USER CODE BEGIN Header_StartRGBShift */
/**
* @brief Function implementing the RGBShift thread.
* @param argument: Not used
* @retval N one
*/
/* USER CODE END Header_StartRGBShift */
void StartRGBShift(void *argument)
{
  /* USER CODE BEGIN StartRGBShift */
//	uint8_t i = 0;
//	RTC_TimeTypeDef sTime = {0};
//	RTC_DateTypeDef sDate = {0};

	uint8_t parr[NUMPIXELS][PIXELARR];
	fillColor(parr, Blue);
	uint8_t farr[NUMPIXELS][PIXELARR];
	fillColor(farr, White);
	uint8_t carr[NUMPIXELS + 2][PIXELARR + 1];
	int index = 0;

	HAL_RNG_GenerateRandomNumber_IT(&hrng);

	HAL_GPIO_WritePin(VOUT1_EN_GPIO_Port, VOUT1_EN_Pin, GPIO_PIN_SET); // Vout1 en

	uint32_t tick = osKernelGetTickCount();
	uint32_t rng = 0;

  /* Infinite loop */
  for(;;)
  {
	  tick += 20;
	  rng = HAL_RNG_ReadLastRandomNumber(&hrng);
//	  filterPointReset(farr, index);
	  fadeToBlackBy(farr, (uint8_t *) index, FFSHORT, 10, rng, 0x07);
//	  filterPointReset(farr, index);
	  pointfill(parr, farr, Gray, index);
	  index = (index + 1 ) % 60;
	  composite(carr, parr, farr, 37, -23, 0xE7);

//	  HAL_SPI_Transmit(&hspi3, (uint8_t *) &carr, sizeof(carr), HAL_MAX_DELAY);
	  HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *) &carr, sizeof(carr));
	  HAL_RNG_GenerateRandomNumber_IT(&hrng);
	  osDelayUntil(tick);

  }
  /* USER CODE END StartRGBShift */
}

/* USER CODE BEGIN Header_StartRTCManage */
/**
* @brief Function implementing the RTCManage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRTCManage */
void StartRTCManage(void *argument)
{
  /* USER CODE BEGIN StartRTCManage */
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	uint8_t posm = 0;
	uint8_t posh = 0;
	uint16_t x[6];
	uint32_t temp[] = {0, 0, 0};
	uint32_t temp1[] = {0, 0, 0};
	uint32_t mflags[2] = {0x00};
	uint16_t hflags = 0x00;
  /* Infinite loop */
  for(;;)
  {
	  //values for time s hould be passed to this thread
//	  osThreadFlagsWait(0x01, osFlagsWaitAny, 0x03FFU);
	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	  if ((sTime.Minutes == 0 ) && (sTime.Seconds <= 5))
	  {
		  int mod = 0;
		  int div = 0;
		  int moda = 0;
		  int diva = 0;
		  mflags[0] = 0;
		  mflags[1] = 0;
		  for(int i = 0; i<30; i++)
		  {
			  mod = (2 * i + 0) % 30;
			  div = (2 * i + 0) / 30;
			  moda = (59 - (2 * i ) ) % 30;
			  diva = (59 - (2 * i ) ) / 30;

			  mflags[div] |= (0x01<<mod);
			  mflags[diva] |= (0x01 << moda);

			  if (i % 2 == 0)
			  {
				  hflags >>= 1;
				  hflags |= 0x01<<12;
			  }
			  else
			  {
				  hflags >>=1;
			  }
			  displayData(newarr, mflags, hflags);
			  osThreadFlagsSet(TimerAndLatchHandle, TIMMASK);
			  osDelay(100);f
		  }
		  for (int i = 0; i < 30; i++)
		  {
			  mflags[0] &= ~(1<<(29 - i));
			  mflags[1]  &= ~(1<<i);
			  if (i % 2 == 0)
			  {
				  hflags >>= 1;
				  hflags |= 0x01<<12;
			  }
			  else
				  hflags >>=1;

			  displayData(newarr, mflags, hflags);
			  osThreadFlagsSet(TimerAndLatchHandle, TIMMASK);
			  osDelay(100);
		  }

	  }
	  hflags = 0x00;
	  mflags[0] = 0;
	  mflags[1] = 0;
//	  for(int i = 0; i < 24; i++)
//	  {
//		   if((hflags & 0x1000)!= 0)
//			  hflags = (hflags & 0xFFF)|0x01;
//		  displayData(newarr, mflags, hflags);
//		  hflags <<=1;
//		  osThreadFlagsSet(TimerAndLatchHandle, TIMMASK);
//		  osDelay(200);
//	  }

	  int d =((min)% 60);
	  mflags[d / 30] = 0x01 << (d% 30);
	  hflags = 0x01 << ((hrs) % 12);
	  displayData(newarr, mflags, hflags);

	  osThreadFlagsSet(TimerAndLatchHandle, TIMMASK);
	  osThreadFlagsWait(MINMASK, osFlagsWaitAny, osWaitForever);
  }
  /* USER CODE END StartRTCManage */
}

/* USER CODE BEGIN Header_StartClockMngr */
/**
* @brief Function implementing the ClockManage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartClockMngr */
void StartClockMngr(void *argument)
{
  /* USER CODE BEGIN StartClockMngr */
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	uint32_t flags = 0;
	int updateflag = 0;
	uint32_t makeTime = 0x00;
	tm rawTime;
//	uint8_t rxBuff[] = {0x00, 0x00,0x00,0x00,0x00};


	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
  /* Infinite loop */
  for(;;)
  {


	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	  if (HAL_GPIO_ReadPin(IO0_GPIO_Port, IO0_Pin) == 0)
//		  flags = 1;
//	  else
//		  flags = 0;
//	  if ( (wififlag & (WIFISTATUS )) == (WIFISTATUS ))
	  {// Check if we got 0x5
//		  HAL_UART_Receive_DMA(&huart2, rxbuff, 6);
//		  HAL_UART_Transmit_DMA(&huart2, txbuff, 1);

		  osThreadFlagsWait(ESPDATA, osFlagsWaitAny, 100);


		  //Clear data callback flag
		  osEventFlagsClear(GlobalStatusHandle, ESPDATA);
		  if (rxbuff[0] == 0x55)
		  {
			  //Set rxbuff to zero
			  rxbuff[0] = 0x00;
			  makeTime = ((rxbuff[1]<<24) + (rxbuff[2]<<16) + (rxbuff[3]<<8) + (rxbuff[4]));
			  rawTime = breaktime(makeTime);
			  sTime.Hours = rawTime.hour;
			  sTime.Minutes = rawTime.minute;
			  sTime.Seconds = rawTime.second;
			  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
			  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
			  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
			  sDate.Date = rawTime.day;
			  sDate.Month = rawTime.month;
			  sDate.Year = rawTime.year;
			  sDate.WeekDay = (rawTime.weekday == 0x00 ? 0x07 : rawTime.weekday);
			  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			  osThreadFlagsSet(RTCManageHandle, MINMASK);
			  updateflag = 1;
			  rxbuff[6] = '\r';
			  rxbuff[7] = '\n';
			  CDC_Transmit_FS(rxbuff, 8);
//			  osThreadYield();
		  }
//		  if(rxbuff[0] != 0x00)
//		  {
//			  HAL_UART_Receive_DMA(&huart2, rxbuff, 6);
//			  HAL_UART_Transmit_DMA(&huart2, txbuff, 1);
//		  }

	  }
	  osThreadFlagsSet(RGBShiftHandle, MINMASK);
	  osThreadFlagsSet(RTCManageHandle, MINMASK);
	  osThreadFlagsWait(MINMASK, osFlagsWaitAny, 2000);
	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	  min = sTime.Minutes;
	  hrs = sTime.Hours;
	  CDC_Transmit_FS(rxbuff, 8);

	  if ((sTime.Minutes % 5 == 0) && (sTime.Seconds >= 50) && (rxbuff[0] == 0x00))
	  {
		  if (updateflag)
		  {
			  HAL_UART_Abort(&huart2);
			  HAL_UART_DMAResume(&huart2);
			  HAL_UART_Receive_DMA(&huart2, rxbuff, 6);
			  HAL_UART_Transmit_DMA(&huart2, txbuff, 1);
			  updateflag = 0;

		  }
	  }


  }
  /* USER CODE END StartClockMngr */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */


  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
