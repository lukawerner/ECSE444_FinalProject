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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#include "motor.h"

#include "stm32l4s5i_iot01_qspi.h"
#include "stm32l4s5i_iot01_tsensor.h"

#include "wifi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t angle;
    uint16_t distance;
    uint8_t intruder_detected;
} ScanPoint;

typedef struct {
	uint16_t angle;
	uint16_t distance;
	uint32_t timestamp;
} logSample;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SINEWAVE_LENGTH 32

#define FLASH_BLOCK_LIMIT 0x10000
#define FLASH_BLOCK_INDEX_LIMIT 128 // 128 64kB blocks in 64 Mb of flash
#define FLASH_START_ADDRESS 0x00

enum {
    SWEEP_DEGREE = 120,                         // Total sweep in degrees
    STEP_DEGREE  = 2,                           // Degrees per step
    STEPS        = SWEEP_DEGREE / STEP_DEGREE   // Number of points in a sweep
};

//#define QUEUE_SET_SIZE 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId tempTaskHandle;
osThreadId scanTaskHandle;
osThreadId transmitTaskHandle;
osThreadId alarmTaskHandle;
osThreadId logDataTaskHandle;
osMessageQId tempQueueHandle;
osMessageQId scanDataQueueHandle;
osMutexId logFlashMutexHandle;
osSemaphoreId alarmSemaphoreHandle;
/* USER CODE BEGIN PV */
volatile uint8_t start = 0;

// temp sensing variables
const float32_t TEMP_THRESHOLD = 40;
const uint32_t TEMP_READ_PERIOD = 3000; // milliseconds

// US sensor scanning variables
const uint32_t DIST_THRESHOLD = 5; // centimeters
const uint32_t WINDOW_SIZE = 3;
ScanPoint baselineData[STEPS];
volatile uint8_t measuring = 0;
volatile uint32_t micro_sec;

// alarm variables
static uint8_t alarmTriggered = 0;
uint16_t sinewave[SINEWAVE_LENGTH];

// Queue variables
//QueueSetHandle_t xQueueSet;

// Flash variables
uint32_t numLogSample = 0;
uint32_t curr_flash_address = FLASH_START_ADDRESS;
uint32_t last_read_flash_address = FLASH_START_ADDRESS;

// text buffer for printing
char output[64];
char UARTbuffer[64];

// WIFI variables
uint32_t lastWifiTick = 0;
WIFI_HandleTypeDef hwifi;
char ssid[] = "TestLan"; //"TestLan"; //VIRGIN184";
char passphrase[] = "12345678";//"12345678"; //"25235A211337";
char remoteIpAddress[] = "10.217.85.33"; // 192.168.1.100";//"192.168.1.100"; //"192.168.2.12"; //"192.168.1.102";
uint8_t WIFI_connection = 1;

const uint32_t TIMEOUT = 5000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void const * argument);
void StartTempTask(void const * argument);
void StartScanTask(void const * argument);
void StartTransmitTask(void const * argument);
void StartAlarmTask(void const * argument);
void StartLogDataTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LUT_sine_builder(uint16_t sinewave[], uint32_t length) {
  float32_t max = 2.0f/3.0f * 4095.0f; // 2730 is 2/3 of the max 12 bit unsigned integer, 4095
  for (int i = 0; i<length; i++) {
    float32_t angle = 2.0f * PI * i/(float32_t) length;
    float32_t sine = arm_sin_f32(angle); // varies from -1 to 1
    sine = (sine + 1.0f) / 2.0f; // varies from  0 to 1
    sine = max * sine; // varies from 0 to 2730
    sinewave[i] =  (uint16_t)sine;
  }
}

uint16_t getDistance(uint32_t time) {
  float speed_of_sound = 0.0343f; // in centimeters per microsecond
  uint16_t distance = ((float)time * speed_of_sound) / 2.0f + 0.5f; // divide by 2 due to round trip, +0.5f to round to nearest int
  return distance;
}

uint16_t take_samples() {
  uint32_t sum = 0;
  for (uint32_t i = 0; i < WINDOW_SIZE; i++) {
    measuring = 1;
    uint32_t start_time = HAL_GetTick();
    while (measuring && HAL_GetTick() - start_time < TIMEOUT) {
      taskYIELD();  // let other tasks run
    }
    uint16_t x_cm = getDistance(micro_sec);
    sum += x_cm;
  }
  uint16_t avg = (float)sum / (float)WINDOW_SIZE + 0.5f;
  return avg;
}

uint16_t take_samples_calibration() {
  uint32_t sum = 0;
  for (uint32_t i = 0; i < WINDOW_SIZE; i++) {
    measuring = 1;
    uint32_t start_time = HAL_GetTick();
    while (measuring && HAL_GetTick() - start_time < TIMEOUT) {
    }
    uint16_t x_cm = getDistance(micro_sec);
    sum += x_cm;
  }
  uint16_t avg = (float)sum / (float)WINDOW_SIZE + 0.5f;
  return avg;
}

static void WIFI_Init_main(){
  hwifi.handle = &hspi3;
  hwifi.ssid = ssid;
  hwifi.passphrase = passphrase;
  hwifi.securityType = WPA_MIXED;
  hwifi.DHCP = SET;
  hwifi.ipStatus = IP_V4;
  hwifi.transportProtocol = WIFI_TCP_PROTOCOL;
  hwifi.port = 8080;
  snprintf(hwifi.remoteIpAddress, sizeof(hwifi.remoteIpAddress), "%s", remoteIpAddress);
  hwifi.remotePort = 8080;

  WIFI_Init(&hwifi);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_OCTOSPI1_Init();
  MX_SPI3_Init();
  MX_DAC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_Base_Start(&htim4);

  BSP_TSENSOR_Init();
  BSP_QSPI_Init();

  sprintf(output, "Initializing\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*) output, strlen(output), HAL_MAX_DELAY);

  // WiFi
  WIFI_Init_main();
  if (WIFI_JoinNetwork(&hwifi) != WIFI_OK) {
    sprintf(output, "Wi-Fi connection FAILED!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*) output, strlen(output), HAL_MAX_DELAY);
    WIFI_connection = 0;
  }
  sprintf(output, "Wi-Fi connected successfully!\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*) output, strlen(output), HAL_MAX_DELAY);

  sprintf(output, "IP: %s\r\n", hwifi.ipAddress);
  HAL_UART_Transmit(&huart1, (uint8_t*)output, strlen(output), HAL_MAX_DELAY);

  if (WIFI_connection && WIFI_SetupSocket(&hwifi) != WIFI_OK) {
    sprintf(output, "Wi-Fi socket FAILED!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*) output, strlen(output), HAL_MAX_DELAY);
    WIFI_connection = 0;
  }

  // TESTING
//  WIFI_connection = 0;

  // speaker sound array initialization
  LUT_sine_builder(sinewave, SINEWAVE_LENGTH);

  while (!start); // wait for button press

  // gather baseline room data
  for (int i = 0; i < STEPS; i++) {
      HAL_Delay(10);
      baselineData[i].distance = take_samples_calibration();
      baselineData[i].angle = STEP_DEGREE * i;
      baselineData[i].intruder_detected = 0;

      if (WIFI_connection) {
        snprintf(output, sizeof(output), "B %d %d\n", (int)baselineData[i].angle, (int)baselineData[i].distance);
        WIFI_SendTCPData(&hwifi, output);
      }
      stepDeg(STEP_DEGREE);
  }
  stepDeg(-SWEEP_DEGREE); // return to starting point
  HAL_Delay(10);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of logFlashMutex */
  osMutexDef(logFlashMutex);
  logFlashMutexHandle = osMutexCreate(osMutex(logFlashMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of alarmSemaphore */
  osSemaphoreDef(alarmSemaphore);
  alarmSemaphoreHandle = osSemaphoreCreate(osSemaphore(alarmSemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of tempQueue */
  osMessageQDef(tempQueue, 16, uint16_t);
  tempQueueHandle = osMessageCreate(osMessageQ(tempQueue), NULL);

  /* definition and creation of scanDataQueue */
  osMessageQDef(scanDataQueue, 16, ScanPoint);
  scanDataQueueHandle = osMessageCreate(osMessageQ(scanDataQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of tempTask */
  osThreadDef(tempTask, StartTempTask, osPriorityAboveNormal, 0, 128);
  tempTaskHandle = osThreadCreate(osThread(tempTask), NULL);

  /* definition and creation of scanTask */
  osThreadDef(scanTask, StartScanTask, osPriorityNormal, 0, 128);
  scanTaskHandle = osThreadCreate(osThread(scanTask), NULL);

  /* definition and creation of transmitTask */
  osThreadDef(transmitTask, StartTransmitTask, osPriorityAboveNormal, 0, 128);
  transmitTaskHandle = osThreadCreate(osThread(transmitTask), NULL);

  /* definition and creation of alarmTask */
  osThreadDef(alarmTask, StartAlarmTask, osPriorityHigh, 0, 128);
  alarmTaskHandle = osThreadCreate(osThread(alarmTask), NULL);

  /* definition and creation of logDataTask */
  osThreadDef(logDataTask, StartLogDataTask, osPriorityHigh, 0, 160);
  logDataTaskHandle = osThreadCreate(osThread(logDataTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30A175AB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

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
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 119;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  htim3.Init.Prescaler = 239;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 6000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, WIFI_RESET_Pin|WIFI_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB_Pin */
  GPIO_InitStruct.Pin = PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : WIFI_RESET_Pin WIFI_NSS_Pin */
  GPIO_InitStruct.Pin = WIFI_RESET_Pin|WIFI_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE11 PE12 PE13
                           PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_CMD_DATA_READY_Pin */
  GPIO_InitStruct.Pin = WIFI_CMD_DATA_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WIFI_CMD_DATA_READY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == PB_Pin) {
    if (!start) {
      start = 1;
    }
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    alarmTriggered = 0;
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)) {
    micro_sec = TIM2->CCR2;
    measuring = 0;
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTempTask */
/**
* @brief Function implementing the tempTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTempTask */
void StartTempTask(void const * argument)
{
  /* USER CODE BEGIN StartTempTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(TEMP_READ_PERIOD);
    uint16_t temp = BSP_TSENSOR_ReadTemp();
    if (xQueueSend(tempQueueHandle, &temp, 0) != pdPASS) {
      // edge case if queue is full, trash oldest reading
      uint16_t oldest;
      xQueueReceive(tempQueueHandle, &oldest, 0);

      xQueueSend(tempQueueHandle, &temp, 0);
    }

    if (!alarmTriggered && temp > TEMP_THRESHOLD) {
      alarmTriggered = 1;
      xSemaphoreGive(alarmSemaphoreHandle);
    }
  }
  /* USER CODE END StartTempTask */
}

/* USER CODE BEGIN Header_StartScanTask */
/**
* @brief Function implementing the scanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartScanTask */
void StartScanTask(void const * argument)
{
  /* USER CODE BEGIN StartScanTask */
  int i = 0;
  int direction = 1;

  ScanPoint scanData;
  /* Infinite loop */
  for(;;)
  {
    osDelay(10); // allow motor to settle
    scanData.angle = i * STEP_DEGREE;
    scanData.distance = take_samples();
    scanData.intruder_detected = 0;

    if (abs(scanData.distance - baselineData[i].distance) > DIST_THRESHOLD) {
      alarmTriggered = 1;
      xSemaphoreGive(alarmSemaphoreHandle);
      scanData.intruder_detected = 1;
    }

    if (xQueueSend(scanDataQueueHandle, &scanData, 0) != pdPASS) {
      // edge case if queue is full, trash oldest reading
      uint16_t oldest;
      xQueueReceive(scanDataQueueHandle, &oldest, 0);

      xQueueSend(scanDataQueueHandle, &scanData, 0);
    }

    stepDeg(direction * STEP_DEGREE);

    if (i == STEPS - 1) {
      direction = -1;
    } else if (i == 0) {
      direction = 1;
    }
    i += direction;
  }
  /* USER CODE END StartScanTask */
}

/* USER CODE BEGIN Header_StartTransmitTask */
/**
* @brief Function implementing the transmitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTransmitTask */
void StartTransmitTask(void const * argument)
{
  /* USER CODE BEGIN StartTransmitTask */
  uint16_t tempData;
  ScanPoint scanData;
  /* Infinite loop */
  for(;;)
  {
    osDelay(5);
    if (xQueueReceive(tempQueueHandle, &tempData, 0) == pdPASS) {
      sprintf(output, "TEMP: %u\r\n", tempData);
      HAL_UART_Transmit(&huart1, (uint8_t*)output, strlen(output), HAL_MAX_DELAY);

      if (WIFI_connection) {
      	snprintf(output, sizeof(output), "T %d\n", (int)tempData);
      	WIFI_SendTCPData(&hwifi, output);
      }
    }

    if (xQueueReceive(scanDataQueueHandle, &scanData, 0)== pdPASS) {
      sprintf(output, "angle: %u, distance: %u\r\n", scanData.angle, scanData.distance);
      HAL_UART_Transmit(&huart1, (uint8_t*)output, strlen(output), HAL_MAX_DELAY);
      if (WIFI_connection) {

        if (scanData.intruder_detected) {
          snprintf(output, sizeof(output), "I %d %d\n", (int)scanData.angle, (int)scanData.distance);
          WIFI_SendTCPData(&hwifi, output);

          logSample intruder_sample;
          intruder_sample.distance = scanData.distance;
          intruder_sample.angle = scanData.angle;
          intruder_sample.timestamp = HAL_GetTick();

          osMutexWait(logFlashMutexHandle, osWaitForever);

          if (curr_flash_address >= (FLASH_BLOCK_INDEX_LIMIT * FLASH_BLOCK_LIMIT)) // flash is full
          {
            curr_flash_address = FLASH_START_ADDRESS;
            numLogSample = 0;
          }

          if (curr_flash_address % FLASH_BLOCK_LIMIT == 0) // we are at the start of a block
          {
            if (BSP_QSPI_Erase_Block(curr_flash_address) != QSPI_OK)
            {
              osMutexRelease(logFlashMutexHandle);
              Error_Handler();
            }
          }
          if (BSP_QSPI_Write((uint8_t*)&intruder_sample, curr_flash_address, sizeof(intruder_sample)) != QSPI_OK)
          {
            osMutexRelease(logFlashMutexHandle);
            Error_Handler();
          }
          curr_flash_address += sizeof(intruder_sample);
          numLogSample++;

          osMutexRelease(logFlashMutexHandle);
        } else {
          snprintf(output, sizeof(output), "N %d\n", (int)scanData.angle);
          WIFI_SendTCPData(&hwifi, output);
        }
      }
    }
  }
  /* USER CODE END StartTransmitTask */
}

/* USER CODE BEGIN Header_StartAlarmTask */
/**
* @brief Function implementing the alarmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlarmTask */
void StartAlarmTask(void const * argument)
{
  /* USER CODE BEGIN StartAlarmTask */
  /* Infinite loop */
  for(;;)
  {
    if(xSemaphoreTake(alarmSemaphoreHandle, portMAX_DELAY) == pdPASS){
      HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)sinewave, SINEWAVE_LENGTH, DAC_ALIGN_12B_R);
    }
  }
  /* USER CODE END StartAlarmTask */
}

/* USER CODE BEGIN Header_StartLogDataTask */
/**
* @brief Function implementing the logDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLogDataTask */
void StartLogDataTask(void const * argument)
{
  /* USER CODE BEGIN StartLogDataTask */
	#define BUFFER_SIZE 16
	logSample buffer[BUFFER_SIZE];


	const uint32_t max_flash_size = FLASH_BLOCK_LIMIT * FLASH_BLOCK_INDEX_LIMIT;

  /* Infinite loop */
  for(;;)
  {
    osDelay(10000); // every 10 seconds

    osMutexWait(logFlashMutexHandle, osWaitForever);

    uint32_t head = curr_flash_address;
    uint32_t tail = last_read_flash_address;
    uint32_t numRead = 0;
    uint32_t sizeOfSample = sizeof(logSample);

    osMutexRelease(logFlashMutexHandle);

    while (head != tail) {
      if (head < tail) { // we circled back the flash
        if (tail + BUFFER_SIZE*sizeof(logSample) >= max_flash_size) {// our buffer reached the end of flash
            numRead = (max_flash_size - tail)/sizeOfSample;
        } else { // we aren't at the flash limit yet
            numRead = BUFFER_SIZE;
        }
      } else { // we didn't circle back
        if ((head - tail)/sizeOfSample < BUFFER_SIZE)// we don't have enough slots for a full buffer
        {
            numRead = (head - tail)/sizeOfSample;
        }
        else { // if we have enough slots for one full buffer
            numRead = BUFFER_SIZE;
        }
      }

      if (BSP_QSPI_Read((uint8_t*) buffer, tail, numRead*sizeOfSample) != QSPI_OK) {
        Error_Handler();
      }
      tail = (tail + numRead*sizeOfSample) % max_flash_size; // increment tail, circling back if needed

      for (int i = 0; i<numRead; i++) {
        snprintf(UARTbuffer, sizeof(UARTbuffer), "I (%u, %u, %lu)\r\nc", buffer[i].distance, buffer[i].angle, buffer[i].timestamp);
        HAL_UART_Transmit(&huart1, (uint8_t*)UARTbuffer, strlen(UARTbuffer), HAL_MAX_DELAY);
      }
    }
    osMutexWait(logFlashMutexHandle, osWaitForever);
    last_read_flash_address = tail;
    osMutexRelease(logFlashMutexHandle);
  }
  /* USER CODE END StartLogDataTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2) {

  }
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
