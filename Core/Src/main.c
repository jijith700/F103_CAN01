/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx.h"
#include <stdio.h>
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void updateLightState(uint8_t);
static void sendLightState(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_TxHeaderTypeDef TxIVIHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;

bool lightsStatus = false;

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	char TagCanCallback[30] = { "Master Callback fifo1....\n\r" };
	HAL_UART_Transmit(&huart3, (uint8_t*) &TagCanCallback, 30, 30);
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	char TagCanResult[35];
	sprintf(TagCanResult, "Callback data %ld, %ld, %d\n\r", RxHeader.DLC,
			RxHeader.StdId, RxData[0]);
	HAL_UART_Transmit(&huart3, (uint8_t*) &TagCanResult, 35, 35);
	if (RxHeader.DLC == 2 && RxHeader.StdId == 259) {
		lightsStatus = true;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_CAN_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan);

	// Activate the notification
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

	TxHeader.DLC = 2;  // data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x446;  // ID

	TxData[0] = 200;   // ms Delay
	TxData[1] = 40;    // loop rep

	TxIVIHeader.DLC = 2;
	TxIVIHeader.IDE = CAN_ID_STD;
	TxIVIHeader.RTR = CAN_RTR_DATA;
	TxIVIHeader.StdId = 0x242;
	TxIVIHeader.TransmitGlobalTime = 1;

	uint8_t sensorAddress = 0x23 << 1;
	uint8_t buffer = 0x01;

	// Power On
	char *TagLightSensor = { '\0' };
	if (HAL_I2C_Master_Transmit(&hi2c1, sensorAddress, &buffer, 1, 100)
			!= HAL_OK) {
		TagLightSensor = "Light sensor power ON failed\n\r";
		HAL_UART_Transmit(&huart3, (uint8_t*) TagLightSensor, 33, 33);
	}

	HAL_Delay(5);

	if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
		TagLightSensor = "Light sensor get state failed\n\r";
		HAL_UART_Transmit(&huart3, (uint8_t*) TagLightSensor, 33, 33);
	}

	// Reset Illuminance Register
	buffer = 0x03;
	if (HAL_I2C_Master_Transmit(&hi2c1, sensorAddress, &buffer, 1, 100)
			!= HAL_OK) {
		TagLightSensor = "Light sensor reset failed\n\r";
		HAL_UART_Transmit(&huart3, (uint8_t*) TagLightSensor, 29, 29);
	}

	// Set measurement mode
	buffer = 0x10;
	if (HAL_I2C_Master_Transmit(&hi2c1, sensorAddress, &buffer, 1, 100)
			!= HAL_OK) {
		TagLightSensor = "Light sensor read failed\n\r";
		HAL_UART_Transmit(&huart3, (uint8_t*) TagLightSensor, 28, 28);
	}

	HAL_Delay(180); // Wait for measurement (or come back after 180 ms instead of idling)

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		/* USER CODE END WHILE */
		if (HAL_GPIO_ReadPin(GPIOA, BTN_CAN_Pin) == GPIO_PIN_SET) { //Check if CAN button pressed
			char TagCan[35] = { "Master trigger from switch....\n\r" };
			TxData[0] = 200;   // ms Delay
			TxData[1] = 40;    // loop rep
			HAL_UART_Transmit(&huart3, (uint8_t*) &TagCan, 35, 35);
			HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
		}

		if (lightsStatus) {
			if (RxData[0] == 4)
				sendLightState();
			else
				updateLightState(RxData[0]);

			lightsStatus = false;
		}

		if (HAL_GPIO_ReadPin(GPIOB, BTN_HEAD_LIGHT_Pin) == GPIO_PIN_SET) { //Check if head light button pressed
			char *TagBtnHeadLight = { "Head light Switch pressed\n\r" };
			HAL_UART_Transmit(&huart3, (uint8_t*) TagBtnHeadLight, 29, 29);
			TxData[0] = 1;
			if (HAL_GPIO_ReadPin(GPIOA, HEAD_LIGHT_Pin) == GPIO_PIN_SET) {
				TxData[1] = 0;
				HAL_GPIO_WritePin(GPIOA, HEAD_LIGHT_Pin, GPIO_PIN_RESET); //HEAD LIGHT OFF
				HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
			} else {
				TxData[1] = 1;
				HAL_GPIO_WritePin(GPIOA, HEAD_LIGHT_Pin, GPIO_PIN_SET); // HEAD LIGHT ON
				HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
			}
		}

		if (HAL_GPIO_ReadPin(GPIOB, BTN_LEFT_INDICATOR_Pin) == GPIO_PIN_SET) { //Check if left indicator button pressed
			char *TagBtnLeftIndicator = { "Left Indicator Switch pressed\n\r" };
			HAL_UART_Transmit(&huart3, (uint8_t*) TagBtnLeftIndicator, 35, 35);
			TxData[0] = 2;
			if (HAL_GPIO_ReadPin(GPIOA, LEFT_INDICATOR_Pin) == GPIO_PIN_SET) {
				TxData[1] = 0;
				HAL_GPIO_WritePin(GPIOA, LEFT_INDICATOR_Pin, GPIO_PIN_RESET); //LEFT INDICATOR OFF
				HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
			} else {
				TxData[1] = 1;
				HAL_GPIO_WritePin(GPIOA, LEFT_INDICATOR_Pin, GPIO_PIN_SET); // LEFT INDICATOR ON
				HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
			}
		}

		if (HAL_GPIO_ReadPin(BTN_RIGHT_INDICATOR_GPIO_Port,
		BTN_RIGHT_INDICATOR_Pin) == GPIO_PIN_SET) { //Check if right indicator button pressed
			char *TagBtnRighttIndic = { "Right Indicator Switch pressed\n\r" };
			HAL_UART_Transmit(&huart3, (uint8_t*) TagBtnRighttIndic, 35, 35);
			TxData[0] = 3;
			if (HAL_GPIO_ReadPin(GPIOA, RIGHT_INDICATOR_Pin) == GPIO_PIN_SET) {
				TxData[1] = 0;
				HAL_GPIO_WritePin(GPIOA, RIGHT_INDICATOR_Pin, GPIO_PIN_RESET); //RIGHT INDICATOR OFF
				HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
			} else {
				TxData[1] = 1;
				HAL_GPIO_WritePin(GPIOA, RIGHT_INDICATOR_Pin, GPIO_PIN_SET); // RIGHT INDICATOR ON
				HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
			}
		}

		uint8_t data[3];
		if (HAL_I2C_Master_Receive(&hi2c1, sensorAddress, data, 3, 100)
				!= HAL_OK) {
			//char TagLightSensor[35];
			//sprintf(TagLightSensor, "Data failed %d, %d, %d\n\r", data[0], data[1], data[2]);
			//HAL_UART_Transmit(&huart3, (uint8_t*) &TagLightSensor, 35, 35);
		} else {
			//char TagLightSensor[35];
			//sprintf(TagLightSensor, "Sensor value %d, %d, %d\n\r", data[0], data[1], data[2]);
			//HAL_UART_Transmit(&huart3, (uint8_t*) &TagLightSensor, 35, 35);
			TxData[0] = 1;
			if (HAL_GPIO_ReadPin(GPIOA, HEAD_LIGHT_Pin) == GPIO_PIN_RESET
					&& data[1] < 50) {
				TxData[1] = 1;
				HAL_GPIO_WritePin(GPIOA, HEAD_LIGHT_Pin, GPIO_PIN_SET); // HEAD LIGHT ON
				HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
				char *hLightTag = { "Low light, turning ON head light\n\r" };
				HAL_UART_Transmit(&huart3, (uint8_t*) hLightTag, 36, 36);
			} else if (HAL_GPIO_ReadPin(GPIOA, HEAD_LIGHT_Pin) == GPIO_PIN_SET
					&& data[1] > 50) {
				TxData[1] = 0;
				HAL_GPIO_WritePin(GPIOA, HEAD_LIGHT_Pin, GPIO_PIN_RESET); // HEAD LIGHT OFF
				HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
				char *hLightTag = { "Normal light, turning OFF head light\n\r" };
				HAL_UART_Transmit(&huart3, (uint8_t*) hLightTag, 40, 40);
			}
		}

		HAL_Delay(500);
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 18;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 8; // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	canfilterconfig.FilterIdHigh = 0x103 << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0x103 << 5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 10; // how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 70;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	RIGHT_INDICATOR_Pin | LEFT_INDICATOR_Pin | HEAD_LIGHT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN_CAN_Pin BTN_RIGHT_INDICATOR_Pin */
	GPIO_InitStruct.Pin = BTN_CAN_Pin | BTN_RIGHT_INDICATOR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : RIGHT_INDICATOR_Pin LEFT_INDICATOR_Pin HEAD_LIGHT_Pin */
	GPIO_InitStruct.Pin = RIGHT_INDICATOR_Pin | LEFT_INDICATOR_Pin
			| HEAD_LIGHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN_LEFT_INDICATOR_Pin BTN_HEAD_LIGHT_Pin */
	GPIO_InitStruct.Pin = BTN_LEFT_INDICATOR_Pin | BTN_HEAD_LIGHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void updateLightState(uint8_t light) {
	GPIO_PinState state = RxData[1] == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET;
	if (light == 1)
		HAL_GPIO_WritePin(GPIOA, HEAD_LIGHT_Pin, state); //HEAD LIGHT
	else if (light == 2)
		HAL_GPIO_WritePin(GPIOA, LEFT_INDICATOR_Pin, state); //LEFT INDICATOR
	else if (light == 3)
		HAL_GPIO_WritePin(GPIOA, RIGHT_INDICATOR_Pin, state); //RIGHT INDICATOR

}

static void sendLightState(void) {
	TxData[0] = 1;
	if (HAL_GPIO_ReadPin(GPIOA, HEAD_LIGHT_Pin) == GPIO_PIN_RESET) {
		TxData[1] = 0;
		HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
	} else {
		TxData[1] = 1;
		HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
	}
	HAL_Delay(50);
	TxData[0] = 2;
	if (HAL_GPIO_ReadPin(GPIOA, LEFT_INDICATOR_Pin) == GPIO_PIN_RESET) {
		TxData[1] = 0;
		HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
	} else {
		TxData[1] = 1;
		HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
	}
	HAL_Delay(50);
	TxData[0] = 3;
	if (HAL_GPIO_ReadPin(GPIOA, RIGHT_INDICATOR_Pin) == GPIO_PIN_RESET) {
		TxData[1] = 0;
		HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
	} else {
		TxData[1] = 1;
		HAL_CAN_AddTxMessage(&hcan, &TxIVIHeader, TxData, &TxMailbox);
	}
	HAL_Delay(50);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
