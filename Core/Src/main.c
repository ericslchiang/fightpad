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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	// digital buttons, 0 = off, 1 = on, definitions below
	uint16_t buttons;

	// Direction value define below
	uint8_t direction : 4;
	// pad the byte out
	uint8_t : 4;

	// analogs --- left stick x/y, right stick x/y
	uint8_t l_x_axis;
	uint8_t l_y_axis;
	uint8_t r_x_axis;
	uint8_t r_y_axis;
} FightpadReport;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// HAT report (4 bits)
#define HID_HAT_UP        0x00
#define HID_HAT_UPRIGHT   0x01
#define HID_HAT_RIGHT     0x02
#define HID_HAT_DOWNRIGHT 0x03
#define HID_HAT_DOWN      0x04
#define HID_HAT_DOWNLEFT  0x05
#define HID_HAT_LEFT      0x06
#define HID_HAT_UPLEFT    0x07
#define HID_HAT_NEUTRAL   0x08

// Button Defines
#define HID_BTN_B1 (1 << 0)
#define HID_BTN_B2 (1 << 1)
#define HID_BTN_B3 (1 << 2)
#define HID_BTN_B4 (1 << 3)
#define HID_BTN_L1 (1 << 4)
#define HID_BTN_R1 (1 << 5)
#define HID_BTN_L2 (1 << 6)
#define HID_BTN_R2 (1 << 7)
#define HID_BTN_S1 (1 << 8)
#define HID_BTN_S2 (1 << 9)
#define HID_BTN_L3 (1 << 10)
#define HID_BTN_R3 (1 << 11)

//Control stick axis values
#define HID_AXIS_MIN 0x00
#define HID_AXIS_MID 0x80
#define HID_AXIS_MAX 0xFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile FightpadReport fightpadReport = {
	.buttons = 0,
	.direction = HID_HAT_NEUTRAL,
	.l_x_axis = HID_AXIS_MID,
	.l_y_axis = HID_AXIS_MID,
	.r_x_axis = HID_AXIS_MID,
	.r_y_axis = HID_AXIS_MID
};
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void dpad(void);
void updateButtons(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void dpad(void) {
	uint8_t up = HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, BTN_UP_Pin);
	uint8_t down = HAL_GPIO_ReadPin(BTN_DOWN_GPIO_Port, BTN_DOWN_Pin);
	uint8_t left = HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin);
	uint8_t right = HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin);

	// SOCD Neutral
	if (up && down) { up = down = GPIO_PIN_RESET; }
	if (left && right) { left = right = GPIO_PIN_RESET; }

	if (up && right) { fightpadReport.direction = HID_HAT_UPRIGHT; fightpadReport.l_x_axis = HID_AXIS_MAX; fightpadReport.l_y_axis = HID_AXIS_MAX; }
	else if (up && left) { fightpadReport.direction = HID_HAT_UPLEFT; fightpadReport.l_x_axis = HID_AXIS_MIN; fightpadReport.l_y_axis = HID_AXIS_MAX; }
	else if (down && right) { fightpadReport.direction = HID_HAT_DOWNRIGHT; fightpadReport.l_x_axis = HID_AXIS_MAX; fightpadReport.l_y_axis = HID_AXIS_MIN; }
	else if (down && left) { fightpadReport.direction = HID_HAT_DOWNLEFT; fightpadReport.l_x_axis = HID_AXIS_MIN; fightpadReport.l_y_axis = HID_AXIS_MIN; }
	else if (up) { fightpadReport.direction = HID_HAT_UP; fightpadReport.l_x_axis = HID_AXIS_MID; fightpadReport.l_y_axis = HID_AXIS_MAX; }
	else if (down) { fightpadReport.direction = HID_HAT_DOWN; fightpadReport.l_x_axis = HID_AXIS_MID; fightpadReport.l_y_axis = HID_AXIS_MIN; }
	else if (right) { fightpadReport.direction = HID_HAT_RIGHT; fightpadReport.l_x_axis = HID_AXIS_MAX; fightpadReport.l_y_axis = HID_AXIS_MID; }
	else if (left) { fightpadReport.direction = HID_HAT_LEFT; fightpadReport.l_x_axis = HID_AXIS_MIN; fightpadReport.l_y_axis = HID_AXIS_MID; }
	else { fightpadReport.direction = HID_HAT_NEUTRAL; fightpadReport.l_x_axis = HID_AXIS_MID; fightpadReport.l_y_axis = HID_AXIS_MID; }
}

void updateButtons(void) {
	fightpadReport.buttons = 0
		| (HAL_GPIO_ReadPin(BTN_B1_GPIO_Port, BTN_B1_Pin) ? HID_BTN_B2 : 0)
		| (HAL_GPIO_ReadPin(BTN_B2_GPIO_Port, BTN_B2_Pin) ? HID_BTN_B3 : 0)
		| (HAL_GPIO_ReadPin(BTN_B3_GPIO_Port, BTN_B3_Pin) ? HID_BTN_B1 : 0)
		| (HAL_GPIO_ReadPin(BTN_B4_GPIO_Port, BTN_B4_Pin) ? HID_BTN_B4 : 0)
		| (HAL_GPIO_ReadPin(BTN_L1_GPIO_Port, BTN_L1_Pin) ? HID_BTN_L1 : 0)
		| (HAL_GPIO_ReadPin(BTN_R1_GPIO_Port, BTN_R1_Pin) ? HID_BTN_R1 : 0)
		| (HAL_GPIO_ReadPin(BTN_L2_GPIO_Port, BTN_L2_Pin) ? HID_BTN_L2 : 0)
		| (HAL_GPIO_ReadPin(BTN_R2_GPIO_Port, BTN_R2_Pin) ? HID_BTN_R2 : 0)
		| (HAL_GPIO_ReadPin(BTN_S1_GPIO_Port, BTN_S1_Pin) ? HID_BTN_S1 : 0)
		| (HAL_GPIO_ReadPin(BTN_S2_GPIO_Port, BTN_S2_Pin) ? HID_BTN_S2 : 0)
		| (HAL_GPIO_ReadPin(BTN_L3_GPIO_Port, BTN_L3_Pin) ? HID_BTN_L3 : 0)
		| (HAL_GPIO_ReadPin(BTN_R3_GPIO_Port, BTN_R3_Pin) ? HID_BTN_R3 : 0)
	;
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
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  dpad();
	  updateButtons();

	  // Send report
	  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *) &fightpadReport, 7);
	  HAL_Delay(10);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : BTN_L2_Pin BTN_UP_Pin BTN_DOWN_Pin BTN_RIGHT_Pin
                           BTN_LEFT_Pin BTN_B1_Pin BTN_B4_Pin BTN_R1_Pin
                           BTN_L1_Pin */
  GPIO_InitStruct.Pin = BTN_L2_Pin|BTN_UP_Pin|BTN_DOWN_Pin|BTN_RIGHT_Pin
                          |BTN_LEFT_Pin|BTN_B1_Pin|BTN_B4_Pin|BTN_R1_Pin
                          |BTN_L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_B2_Pin BTN_B3_Pin BTN_R2_Pin BTN_S2_Pin
                           BTN_S1_Pin BTN_R3_Pin BTN_L3_Pin */
  GPIO_InitStruct.Pin = BTN_B2_Pin|BTN_B3_Pin|BTN_R2_Pin|BTN_S2_Pin
                          |BTN_S1_Pin|BTN_R3_Pin|BTN_L3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
