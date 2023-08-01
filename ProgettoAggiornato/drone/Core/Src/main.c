/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "accel.h"
#include "attitude.h"
#include "baro.h"
#include "control_motor.h"
#include "gyro.h"
#include "mag.h"
#include "iks01a2_env_sensors.h"
#include "iks01a2_motion_sensors.h"
#include <stdint.h>
#include <string.h>
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

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for tControlMotor */
osThreadId_t tControlMotorHandle;
const osThreadAttr_t tControlMotor_attributes = {
  .name = "tControlMotor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime3,
};
/* Definitions for tAttitude */
osThreadId_t tAttitudeHandle;
const osThreadAttr_t tAttitude_attributes = {
  .name = "tAttitude",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for tAltitude */
osThreadId_t tAltitudeHandle;
const osThreadAttr_t tAltitude_attributes = {
  .name = "tAltitude",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for tRecoverAcc */
osThreadId_t tRecoverAccHandle;
const osThreadAttr_t tRecoverAcc_attributes = {
  .name = "tRecoverAcc",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for tRecoverPress */
osThreadId_t tRecoverPressHandle;
const osThreadAttr_t tRecoverPress_attributes = {
  .name = "tRecoverPress",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime5,
};
/* Definitions for tRecoverMag */
osThreadId_t tRecoverMagHandle;
const osThreadAttr_t tRecoverMag_attributes = {
  .name = "tRecoverMag",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for PrintData */
osThreadId_t PrintDataHandle;
const osThreadAttr_t PrintData_attributes = {
  .name = "PrintData",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for tRecoverGyro */
osThreadId_t tRecoverGyroHandle;
const osThreadAttr_t tRecoverGyro_attributes = {
  .name = "tRecoverGyro",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for Sem_I2C */
osSemaphoreId_t Sem_I2CHandle;
const osSemaphoreAttr_t Sem_I2C_attributes = {
  .name = "Sem_I2C"
};
/* Definitions for Sem_LSM6DSL_Acc */
osSemaphoreId_t Sem_LSM6DSL_AccHandle;
const osSemaphoreAttr_t Sem_LSM6DSL_Acc_attributes = {
  .name = "Sem_LSM6DSL_Acc"
};
/* Definitions for Sem_LSM303AGR_Acc */
osSemaphoreId_t Sem_LSM303AGR_AccHandle;
const osSemaphoreAttr_t Sem_LSM303AGR_Acc_attributes = {
  .name = "Sem_LSM303AGR_Acc"
};
/* Definitions for Sem_Mag */
osSemaphoreId_t Sem_MagHandle;
const osSemaphoreAttr_t Sem_Mag_attributes = {
  .name = "Sem_Mag"
};
/* Definitions for Sem_Gyro */
osSemaphoreId_t Sem_GyroHandle;
const osSemaphoreAttr_t Sem_Gyro_attributes = {
  .name = "Sem_Gyro"
};
/* Definitions for Sem_Press */
osSemaphoreId_t Sem_PressHandle;
const osSemaphoreAttr_t Sem_Press_attributes = {
  .name = "Sem_Press"
};
/* USER CODE BEGIN PV */
float VectAcc_LSM303AGR[3];
float VectAcc_LSM6DSL[3];
float VectPress;
float VectMag[3];
float VectGyro[3];
IKS01A2_MOTION_SENSOR_Axes_t LSM6DSL_Acc;
IKS01A2_MOTION_SENSOR_Axes_t LSM303AGR_Acc;
IKS01A2_MOTION_SENSOR_Axes_t LSM303AGR_Mag;
IKS01A2_MOTION_SENSOR_AxesRaw_t LSM6DSL_Gyro;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void startTaskControlMotor(void *argument);
void startTaskAttitude(void *argument);
void startTaskAltitude(void *argument);
void StartRecoverAcc(void *argument);
void StartRecoverPress(void *argument);
void StartRecoverMag(void *argument);
void StartPrintData(void *argument);
void StartRecoverGyro(void *argument);

/* USER CODE BEGIN PFP */
static void sensorGyroInit(struct gyroDev_s *gyro);
static bool sensorGyroRead(struct gyroDev_s *gyro);
static void sensorAccInit(struct accDev_s *acc);
static bool sensorAccRead(struct accDev_s *acc);
static void sensorMagInit(struct magDev_s *mag);
static bool sensorMagRead(struct magDev_s *mag);
static void sensorBaroInit(struct baroDev_s *baro);
static bool sensorBaroRead(struct baroDev_s *baro);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  /* initialization of control task */
  gyroInit(sensorGyroInit, sensorGyroRead);
  controlMotorInit();

  /* initialization of attitude task */
  accInit(sensorAccInit, sensorAccRead);
  magInit(sensorMagInit, sensorMagRead);
  attitudeInit();

  /* initialization of altitude task */
  baroInit(sensorBaroInit, sensorBaroRead);

  /*Init dei sensori*/
  IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0,ENV_PRESSURE);
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0,(MOTION_GYRO | MOTION_ACCELERO) );
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_ACC_0,MOTION_ACCELERO);
  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0,MOTION_MAGNETO);

  /*enable dei sensori*/
  IKS01A2_ENV_SENSOR_Enable(IKS01A2_LPS22HB_0,ENV_PRESSURE);
  IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0,(MOTION_GYRO | MOTION_ACCELERO));
  IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_ACC_0,MOTION_ACCELERO);
  IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_MAG_0,MOTION_MAGNETO);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Sem_I2C */
  Sem_I2CHandle = osSemaphoreNew(1, 1, &Sem_I2C_attributes);

  /* creation of Sem_LSM6DSL_Acc */
  Sem_LSM6DSL_AccHandle = osSemaphoreNew(1, 1, &Sem_LSM6DSL_Acc_attributes);

  /* creation of Sem_LSM303AGR_Acc */
  Sem_LSM303AGR_AccHandle = osSemaphoreNew(1, 1, &Sem_LSM303AGR_Acc_attributes);

  /* creation of Sem_Mag */
  Sem_MagHandle = osSemaphoreNew(1, 1, &Sem_Mag_attributes);

  /* creation of Sem_Gyro */
  Sem_GyroHandle = osSemaphoreNew(1, 1, &Sem_Gyro_attributes);

  /* creation of Sem_Press */
  Sem_PressHandle = osSemaphoreNew(1, 1, &Sem_Press_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of tControlMotor */
  tControlMotorHandle = osThreadNew(startTaskControlMotor, NULL, &tControlMotor_attributes);

  /* creation of tAttitude */
  tAttitudeHandle = osThreadNew(startTaskAttitude, NULL, &tAttitude_attributes);

  /* creation of tAltitude */
  tAltitudeHandle = osThreadNew(startTaskAltitude, NULL, &tAltitude_attributes);

  /* creation of tRecoverAcc */
  tRecoverAccHandle = osThreadNew(StartRecoverAcc, NULL, &tRecoverAcc_attributes);

  /* creation of tRecoverPress */
  tRecoverPressHandle = osThreadNew(StartRecoverPress, NULL, &tRecoverPress_attributes);

  /* creation of tRecoverMag */
  tRecoverMagHandle = osThreadNew(StartRecoverMag, NULL, &tRecoverMag_attributes);

  /* creation of PrintData */
  PrintDataHandle = osThreadNew(StartPrintData, NULL, &PrintData_attributes);

  /* creation of tRecoverGyro */
  tRecoverGyroHandle = osThreadNew(StartRecoverGyro, NULL, &tRecoverGyro_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
	/*uint32_t start = HAL_GetTick();
	for (int i = 0; i < 100000; ++i) {*/
		/* first task: control */
	    /*gyroUpdate();
		controlMotorUpdate();*/

		/* second task: attitude */
		/*accUpdate();
		magUpdate();
		attitudeUpdate();*/

		/* third task: altitude */
		/*baroUpdate();
	}*/
	/*volatile uint32_t last = HAL_GetTick() - start;
	Error_Handler();*/
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
int _write(int file, char *ptr, int len) {
	for (int i = 0; i < len; ++i) {
		ITM_SendChar(*ptr++);
	}
	return len;
}
*/

static void sensorGyroInit(struct gyroDev_s *gyro) { }

static bool sensorGyroRead(struct gyroDev_s *gyro) {
	if(osSemaphoreAcquire(Sem_GyroHandle, 0) == osOK){
		gyro->gyroADC[0] = VectGyro[0];
		gyro->gyroADC[1] = VectGyro[1];
		gyro->gyroADC[2] = VectGyro[2];
		osSemaphoreRelease(Sem_GyroHandle);
	}
	return true;
}

static void sensorAccInit(struct accDev_s *acc) { }

static bool sensorAccRead(struct accDev_s *acc) {
	if(osSemaphoreAcquire(Sem_LSM303AGR_AccHandle, 0) == osOK){
		if(osSemaphoreAcquire(Sem_LSM6DSL_AccHandle, 0)==osOK){
			acc->accADC[0] = (VectAcc_LSM303AGR[0]+VectAcc_LSM6DSL[0])/2;
			acc->accADC[1] = (VectAcc_LSM303AGR[1]+VectAcc_LSM6DSL[1])/2;
			acc->accADC[2] = (VectAcc_LSM303AGR[2]+VectAcc_LSM6DSL[2])/2;
			osSemaphoreRelease(Sem_LSM6DSL_AccHandle);
		}
		osSemaphoreRelease(Sem_LSM303AGR_AccHandle);
	}
	return true;
}

static void sensorMagInit(struct magDev_s *mag) { }

static bool sensorMagRead(struct magDev_s *mag) {
	if(osSemaphoreAcquire(Sem_MagHandle, 0)==osOK){
		mag->magADC[0] = VectMag[0];
		mag->magADC[1] = VectMag[1];
		mag->magADC[2] = VectMag[2];
		osSemaphoreRelease(Sem_MagHandle);
	}
	return true;
}

static void sensorBaroInit(struct baroDev_s *baro) {
	baro->baroADC = 0;
}

static bool sensorBaroRead(struct baroDev_s *baro) {
	if(osSemaphoreAcquire(Sem_PressHandle, 0)== osOK){
		baro->baroADC = VectPress;
		osSemaphoreRelease(Sem_PressHandle);
	}
	return true;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startTaskControlMotor */
/**
  * @brief  Function implementing the tControlMotor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startTaskControlMotor */
void startTaskControlMotor(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	gyroUpdate();
	controlMotorUpdate();
    osDelay(2);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startTaskAttitude */
/**
* @brief Function implementing the tAttitude thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTaskAttitude */
void startTaskAttitude(void *argument)
{
  /* USER CODE BEGIN startTaskAttitude */
  /* Infinite loop */
  for(;;)
  {
    accUpdate();
    magUpdate();
    attitudeUpdate();
    osDelay(10);
  }
  /* USER CODE END startTaskAttitude */
}

/* USER CODE BEGIN Header_startTaskAltitude */
/**
* @brief Function implementing the tAltitude thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTaskAltitude */
void startTaskAltitude(void *argument)
{
  /* USER CODE BEGIN startTaskAltitude */
  /* Infinite loop */
  for(;;)
  {
    baroUpdate();
    osDelay(25);
  }
  /* USER CODE END startTaskAltitude */
}

/* USER CODE BEGIN Header_StartRecoverAcc */
/**
* @brief Function implementing the tRecoverAcc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecoverAcc */
void StartRecoverAcc(void *argument)
{
  /* USER CODE BEGIN StartRecoverAcc */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreAcquire(Sem_I2CHandle, 0) == osOK){
		  IKS01A2_MOTION_SENSOR_GetAxes(IKS01A2_LSM6DSL_0,MOTION_ACCELERO,&LSM6DSL_Acc);
		  osSemaphoreRelease(Sem_I2CHandle);
	  }
	  if(osSemaphoreAcquire(Sem_LSM6DSL_AccHandle, 0) == osOK){
		  VectAcc_LSM6DSL[0] = (((float)LSM6DSL_Acc.x) / 1000) * 9.81;
		  VectAcc_LSM6DSL[1] = (((float)LSM6DSL_Acc.y) / 1000) * 9.81;
		  VectAcc_LSM6DSL[2] = (((float)LSM6DSL_Acc.z) / 1000) * 9.81;
		  osSemaphoreRelease(Sem_LSM6DSL_AccHandle);
	  }

	  if(osSemaphoreAcquire(Sem_I2CHandle, 0) == osOK){
		  IKS01A2_MOTION_SENSOR_GetAxes(IKS01A2_LSM303AGR_ACC_0,MOTION_ACCELERO,&LSM303AGR_Acc);
		  osSemaphoreRelease(Sem_I2CHandle);
	  }
	  if(osSemaphoreAcquire(Sem_LSM303AGR_AccHandle, 0) == osOK){
		  VectAcc_LSM303AGR[0] = (((float)LSM303AGR_Acc.x )/ 1000) * 9.81;
		  VectAcc_LSM303AGR[1] = (((float)LSM303AGR_Acc.y ) / 1000) * 9.81;
		  VectAcc_LSM303AGR[2] = (((float)LSM303AGR_Acc.z ) / 1000) * 9.81;
		  osSemaphoreRelease(Sem_LSM303AGR_AccHandle);
	  }

    osDelay(10);
  }
  /* USER CODE END StartRecoverAcc */
}

/* USER CODE BEGIN Header_StartRecoverPress */
/**
* @brief Function implementing the tRecoverPress thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecoverPress */
void StartRecoverPress(void *argument)
{
  /* USER CODE BEGIN StartRecoverPress */
  /* Infinite loop */
  for(;;)
  {
	if(osSemaphoreAcquire(Sem_I2CHandle, 0) == osOK){
		if(osSemaphoreAcquire(Sem_PressHandle, 0)== osOK){
			IKS01A2_ENV_SENSOR_GetValue(IKS01A2_LPS22HB_0,ENV_PRESSURE,&VectPress);
			osSemaphoreRelease(Sem_PressHandle);
		}
		osSemaphoreRelease(Sem_I2CHandle);
	}
    osDelay(50);
  }
  /* USER CODE END StartRecoverPress */
}

/* USER CODE BEGIN Header_StartRecoverMag */
/**
* @brief Function implementing the tRecoverMag thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecoverMag */
void StartRecoverMag(void *argument)
{
  /* USER CODE BEGIN StartRecoverMag */
  /* Infinite loop */
  for(;;)
  {
	if(osSemaphoreAcquire(Sem_I2CHandle, 0)== osOK){
		IKS01A2_MOTION_SENSOR_GetAxes(IKS01A2_LSM303AGR_MAG_0,MOTION_MAGNETO,&LSM303AGR_Mag);
		osSemaphoreRelease(Sem_I2CHandle);
	}
	if(osSemaphoreAcquire(Sem_MagHandle, 0)== osOK){
		VectMag[0] = LSM303AGR_Mag.x * 100;
		VectMag[1] = LSM303AGR_Mag.y * 100;
		VectMag[2] = LSM303AGR_Mag.z * 100;
		osSemaphoreRelease(Sem_MagHandle);
	}
    osDelay(100);
  }
  /* USER CODE END StartRecoverMag */
}

/* USER CODE BEGIN Header_StartPrintData */
/**
* @brief Function implementing the PrintData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPrintData */
void StartPrintData(void *argument)
{
  /* USER CODE BEGIN StartPrintData */
  /* Infinite loop */
  for(;;)
  {
	 uint8_t MSG_Gyro[150] = {'\0'};
	 uint8_t MSG_LSM6DSL_Acc[150] = {'\0'};
	 uint8_t MSG_LSM303AGR_Acc[150] = {'\0'};
	 uint8_t MSG_Mean_Acc[150] = {'\0'};
	 uint8_t MSG_Mag[150] = {'\0'};
	 uint8_t MSG_Press[150] = {'\0'};
	/*
	if(osSemaphoreAcquire(Sem_GyroHandle, 0)== osOK){
		printf("Gyroscope tridimensional data  x = %.2f,   y = %.2f,  z= %.2f in rad*sec^-1\r\n", VectGyro[0],VectGyro[1],VectGyro[2]);
		osSemaphoreRelease(Sem_GyroHandle);
	}
	if(osSemaphoreAcquire(Sem_LSM6DSL_AccHandle, 0)== osOK){
		printf("LSM6DSL Accelerometer tridimensional data x = %.2f,   y = %.2f,  z= %.2f  in m*sec^-2\r\n", VectAcc_LSM6DSL[0],VectAcc_LSM6DSL[1],VectAcc_LSM6DSL[2]);
		osSemaphoreRelease(Sem_LSM6DSL_AccHandle);
	}
	if(osSemaphoreAcquire(Sem_LSM303AGR_AccHandle, 0)== osOK){
		printf("LSM303AGR Accelerometer tridimensional data x = %.2f,    y = %.2f,     z = %.2f  in m*sec^-2\r\n", VectAcc_LSM303AGR[0],VectAcc_LSM303AGR[1],VectAcc_LSM303AGR[2]);
		osSemaphoreRelease(Sem_LSM303AGR_AccHandle);
	}
	if(osSemaphoreAcquire(Sem_LSM303AGR_AccHandle, 0)== osOK){
		if(osSemaphoreAcquire(Sem_LSM6DSL_AccHandle, 0)== osOK){
			float variable_append0 = (VectAcc_LSM303AGR[0]+VectAcc_LSM6DSL[0])/2;
			float variable_append1 = (VectAcc_LSM303AGR[1]+VectAcc_LSM6DSL[1])/2;
			float variable_append2 = (VectAcc_LSM303AGR[2]+VectAcc_LSM6DSL[2])/2;
			osSemaphoreRelease(Sem_LSM6DSL_AccHandle);
			printf("Mean Accelerometer tridimensional data x = %.2f,   y = %.2f,     z = %.2f  in m*sec^-2\r\n", variable_append0, variable_append1, variable_append2);
		}
		osSemaphoreRelease(Sem_LSM303AGR_AccHandle);
	}

	if(osSemaphoreAcquire(Sem_MagHandle, 0)== osOK){
		printf("Magnetometer tridimensional data x = %.2f,   y = %.2f,  z= %.2f in MicroT\r\n", VectMag[0],VectMag[1],VectMag[2]);
		osSemaphoreRelease(Sem_MagHandle);
	}
	if(osSemaphoreAcquire(Sem_PressHandle, 0)== osOK){
		printf("Barometer data  = %.1f in hPa\r\n\n", VectPress);
		osSemaphoreRelease(Sem_PressHandle);
	}
	*/
	  if(osSemaphoreAcquire(Sem_GyroHandle, 0)== osOK){
		  sprintf(MSG_Gyro,"Gyroscope tridimensional data  x = %.2f,   y = %.2f,  z= %.2f in rad*sec^-1\r\n", VectGyro[0],VectGyro[1],VectGyro[2]);
		  HAL_UART_Transmit(&huart3, MSG_Gyro, sizeof(MSG_Gyro), 100);
		  osSemaphoreRelease(Sem_GyroHandle);
	  }
	  if(osSemaphoreAcquire(Sem_LSM6DSL_AccHandle, 0)== osOK){
		  sprintf(MSG_LSM6DSL_Acc,"LSM6DSL Accelerometer tridimensional data x = %.2f,   y = %.2f,  z= %.2f  in m*sec^-2\r\n", VectAcc_LSM6DSL[0],VectAcc_LSM6DSL[1],VectAcc_LSM6DSL[2]);
		  HAL_UART_Transmit(&huart3, MSG_LSM6DSL_Acc, sizeof(MSG_LSM6DSL_Acc), 100);
		  osSemaphoreRelease(Sem_LSM6DSL_AccHandle);
	  }
	  if(osSemaphoreAcquire(Sem_LSM303AGR_AccHandle, 0)== osOK){
		  sprintf(MSG_LSM303AGR_Acc,"LSM303AGR Accelerometer tridimensional data x = %.2f,    y = %.2f,     z = %.2f  in m*sec^-2\r\n", VectAcc_LSM303AGR[0],VectAcc_LSM303AGR[1],VectAcc_LSM303AGR[2]);
		  HAL_UART_Transmit(&huart3, MSG_LSM303AGR_Acc, sizeof(MSG_LSM303AGR_Acc), 100);
		  osSemaphoreRelease(Sem_LSM303AGR_AccHandle);
	  }
	  if(osSemaphoreAcquire(Sem_LSM303AGR_AccHandle, 0)== osOK){
	  		if(osSemaphoreAcquire(Sem_LSM6DSL_AccHandle, 0)== osOK){
	  			float variable_append0 = (VectAcc_LSM303AGR[0]+VectAcc_LSM6DSL[0])/2;
	  			float variable_append1 = (VectAcc_LSM303AGR[1]+VectAcc_LSM6DSL[1])/2;
	  			float variable_append2 = (VectAcc_LSM303AGR[2]+VectAcc_LSM6DSL[2])/2;
	  			sprintf(MSG_Mean_Acc,"Mean Accelerometer tridimensional data x = %.2f,   y = %.2f,     z = %.2f  in m*sec^-2\r\n", variable_append0, variable_append1, variable_append2);
	  			HAL_UART_Transmit(&huart3, MSG_Mean_Acc, sizeof(MSG_Mean_Acc), 100);
	  			osSemaphoreRelease(Sem_LSM6DSL_AccHandle);
	  		}
	  		osSemaphoreRelease(Sem_LSM303AGR_AccHandle);
	  }
	  if(osSemaphoreAcquire(Sem_MagHandle, 0)== osOK){
		  sprintf(MSG_Mag,"Magnetometer tridimensional data x = %.2f,   y = %.2f,  z= %.2f in MicroT\r\n", VectMag[0],VectMag[1],VectMag[2]);
		  HAL_UART_Transmit(&huart3, MSG_Mag, sizeof(MSG_Mag), 100);
		  osSemaphoreRelease(Sem_MagHandle);
	  }
	  if(osSemaphoreAcquire(Sem_PressHandle, 0)== osOK){
		  sprintf(MSG_Press,"Barometer data  = %.1f in hPa\r\n\n", VectPress);
		  HAL_UART_Transmit(&huart3, MSG_Press, sizeof(MSG_Press), 100);
		  osSemaphoreRelease(Sem_PressHandle);
	  }
    osDelay(2000);
  }
  /* USER CODE END StartPrintData */
}

/* USER CODE BEGIN Header_StartRecoverGyro */
/**
* @brief Function implementing the tRecoverGyro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecoverGyro */
void StartRecoverGyro(void *argument)
{
  /* USER CODE BEGIN StartRecoverGyro */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreAcquire(Sem_I2CHandle, 0)== osOK){
		  IKS01A2_MOTION_SENSOR_GetAxesRaw(IKS01A2_LSM6DSL_0,MOTION_GYRO,&LSM6DSL_Gyro);
		  osSemaphoreRelease(Sem_I2CHandle);
	  }
	  if(osSemaphoreAcquire(Sem_GyroHandle, 0)== osOK){
		  VectGyro[0] = (((float)LSM6DSL_Gyro.x) * 100) * 0.0175;
		  VectGyro[1] = (((float)LSM6DSL_Gyro.y) * 100) * 0.0175;
		  VectGyro[2] = (((float)LSM6DSL_Gyro.z) * 100) * 0.0175;
		  osSemaphoreRelease(Sem_GyroHandle);
	  }
    osDelay(2);
  }
  /* USER CODE END StartRecoverGyro */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
