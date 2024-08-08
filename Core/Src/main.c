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
#include "stdio.h"
#include "string.h"
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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef rxHeader;
CAN_TxHeaderTypeDef txHeader;
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};
CAN_FilterTypeDef canfil;
uint32_t canMailbox;

uint8_t direction_speed_mode;
uint8_t direction;
uint16_t dc_voltage;
uint16_t motor_current;
uint16_t motor_temp;
uint16_t controller_temp;
uint16_t rotating_speed;
uint8_t error_code;
uint8_t low_power_mode;
uint16_t mileage_subtotal;
uint8_t speed_mode;
uint8_t speed;
uint8_t soc;
char gear;
char lwp;
uint8_t myMessage[50];
uint8_t myfanmessage[20];
uint8_t coming_message[1];
uint8_t cmdEnd[3] = {0xFF,0xFF,0xFF};
uint8_t buffer[20];
uint16_t buffer_index = 0;
volatile uint8_t message_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  canfil.FilterBank = 0;
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfil.FilterIdHigh = 0;
  canfil.FilterIdLow = 0;
  canfil.FilterMaskIdHigh = 0;
  canfil.FilterMaskIdLow = 0;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterActivation = ENABLE;
  canfil.SlaveStartFilterBank = 14;

  txHeader.DLC = 8;
  txHeader.IDE = CAN_ID_EXT;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = 0x030;
  txHeader.ExtId = 0x02;
  txHeader.TransmitGlobalTime = DISABLE;
  HAL_CAN_ConfigFilter(&hcan1,&canfil);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_UART_Receive_IT(&huart2, coming_message, sizeof(coming_message));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (message_ready)
	  {
	  // Process the message
	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)buffer, strlen(buffer));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
// Reset buffer and flag
	  buffer_index = 0;
	  message_ready = 0;
	  }
	  if(controller_temp >=26)
	  {
		  sprintf((char*)myfanmessage,"ON\n");
		  HAL_UART_Transmit_IT(&huart2, myfanmessage, strlen((char*)myfanmessage));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  }
	  else
	  {
		  sprintf((char*)myfanmessage,"OFF\n");
		  HAL_UART_Transmit_IT(&huart2, myfanmessage, strlen((char*)myfanmessage));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  }


	  speed = ((rotating_speed/100)*1.23);
	  sprintf((char*)myMessage,"speed.val=%d",speed);
	  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

	  sprintf((char*)myMessage,"vol.val=%d",dc_voltage);
	  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

	  sprintf((char*)myMessage,"rpm.val=%d",rotating_speed/100);
	  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

	  sprintf((char*)myMessage,"amper.val=%d",motor_current);
	  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

	  sprintf((char*)myMessage,"motor.val=%d",motor_temp);
	  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

	  sprintf((char*)myMessage,"sistem.val=%d",controller_temp);
	  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

	  sprintf((char*)myMessage,"gear.txt=\"%c\"",gear);
	  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

	  soc = ((dc_voltage - 48.0) / (60.0 - 48.0)) * 100.0;
	  sprintf((char*)myMessage,"soc.val=%d",soc);
	  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
	  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

	  if(error_code == 0)
	  {
		  sprintf((char*)myMessage,"t2.txt=\"\"");
		  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
		  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

		  sprintf((char*)myMessage,"t2.bco=65535");
		  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
		  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  }
	  else{
		  sprintf((char*)myMessage,"t2.txt=\"%d\"",error_code);
		  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
		  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

		  sprintf((char*)myMessage,"t2.bco=63488");
		  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
		  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

	  }
	  //sprintf((char*)myMessage,"t2.txt=\"%c\"",gear);
	  //HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
	  //while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  //HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
	  //while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

	  if(speed_mode == 1)
	  {
		  sprintf((char*)myMessage,"eco.pic=4");
		  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
		  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
	  }

	  else
	  {
		  sprintf((char*)myMessage,"eco.pic=5");
		  HAL_UART_Transmit_IT(&huart1, myMessage, strlen((char*)myMessage));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
		  HAL_UART_Transmit_IT(&huart1, cmdEnd, sizeof(cmdEnd));
		  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

	  }
	  HAL_Delay(30);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  huart2.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX);
	 if (rxHeader.IDE == CAN_ID_EXT && rxHeader.ExtId == 0x10F8108D)
	    {
	        dc_voltage = (uint16_t)(canRX[1] << 8 | canRX[0]) * 0.1f;
	        motor_current = (uint16_t)(canRX[3] << 8 | canRX[2]) * 0.1f;
	        motor_temp = (uint16_t)(canRX[5] << 8 | canRX[4]) * 0.1f;
	        controller_temp = (uint16_t)(canRX[7] << 8 | canRX[6]) * 0.1f;

	    }
	 else if (rxHeader.IDE == CAN_ID_EXT && rxHeader.ExtId == 0x10F8109A)
	 {
	        direction_speed_mode = canRX[0];
	        rotating_speed = (uint16_t)(canRX[2] << 8 | canRX[1]);
	        error_code = canRX[3];
	        low_power_mode = canRX[4];
	        mileage_subtotal = (uint16_t)(canRX[6] << 8 | canRX[5]) * 0.1f;

	        direction = direction_speed_mode & 0x03;
	        speed_mode = (direction_speed_mode & 0x08) >> 3;

	        switch(direction) {
	            case 1:
	                gear = 'D';
	                break;
	            case 2:
	            	gear = 'R';
	                break;
	            default:
	            	gear = 'N';
	                break;
	        }


	 }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (buffer_index < sizeof(buffer) - 3) // Reserve space for 3 0xFF
        {
            if (coming_message[0] == '\n')
            {
                // Replace '\n' with three 0xFF bytes
                buffer[buffer_index++] = 0xFF;
                buffer[buffer_index++] = 0xFF;
                buffer[buffer_index++] = 0xFF;
                buffer[buffer_index++] = 0xFF;

                // Null-terminate the string
                buffer[buffer_index] = '\0';

                // Set flag to indicate message is ready
                message_ready = 1;
            }
            else
            {
                // Add the character to the buffer
                buffer[buffer_index++] = coming_message[0];
            }
        }
        else
        {
            // Buffer overflow, reset the buffer
            buffer_index = 0;
        }

        // Re-enable UART receive interrupt
        HAL_UART_Receive_IT(&huart2, coming_message, sizeof(coming_message));
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
