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
#include "bootloader.h"
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

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

#define TRUE 1
#define FALSE 0
#define G_MAX_MS_COUNTER 4294967296


uint32_t G_mSCounter=0;

uint8_t G_loader_mode=0;
uint8_t G_flash_erase_CMD=0;
uint8_t G_run_app_CMD=0;
uint8_t G_return_CRC_CMD=0;
uint8_t G_return_BOOT_CRC_CMD=0;

uint8_t G_start_flash_CMD=0;
uint8_t G_write_next_CMD=0;
uint32_t G_uint32_to_write=0;
uint8_t G_end_flash_CMD=0;
uint8_t G_FlashInProgress=0;

uint8_t G_LedUpdate=0;

uint32_t G_index=0;

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void config_can_filter(void);
void CAN_TX_Cplt(CAN_HandleTypeDef* phcan);
void Run_Application();

void send_confirm_msg(uint8_t status);
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
  MX_CAN1_Init();
  MX_CRC_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  /* Configura CAN RX FILTER */
  config_can_filter();
  /* Start CAN recieve Interrupt */
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

  HAL_TIM_Base_Start_IT(&htim17); // 1mS
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  if(btld_CheckForApplication()==BL_NO_APP){
	  G_loader_mode=1;
  }

  while (1)
  {
    /* USER CODE END WHILE */
	  /* Loader Mode */
	  if(G_loader_mode){

		  /* Erase Flash */
		  if(G_flash_erase_CMD){
			  if(btld_EraseFlash()==HAL_OK){
				  /* Erase Success */
					//Send CAN message to know we are in loader mode
					TxHeader.ExtId=TX_FEEDBACK_CANID + CANTX_SA;
					TxHeader.IDE=CAN_ID_EXT;
					TxHeader.RTR=CAN_RTR_DATA;
					TxHeader.DLC=1;
					TxData[0]=0xF0;

					HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
			  }else{
				  /* Erase failed */
					//Send CAN message to know we are in loader mode
					TxHeader.ExtId=TX_FEEDBACK_CANID + CANTX_SA;
					TxHeader.IDE=CAN_ID_EXT;
					TxHeader.RTR=CAN_RTR_DATA;
					TxHeader.DLC=1;
					TxData[0]=0xFF;

					HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
			  }
			  G_flash_erase_CMD=0;
		  }
		  /* Run Application command */
		  if(G_run_app_CMD){
			  btld_JumpToApp();
		  }
		  /* Calculate and send CRC */
		  if(G_return_CRC_CMD){
			  uint32_t checksum;
			  checksum=btld_GetChecksum();

				/* Send Available flash size for application */
				TxHeader.ExtId=TX_FEEDBACK_CANID + CANTX_SA;
				TxHeader.IDE=CAN_ID_EXT;
				TxHeader.RTR=CAN_RTR_DATA;
				TxHeader.DLC=5;
				TxData[0]=0xF4;

				TxData[1]=(checksum & 0xFF000000)>>24;
				TxData[2]=(checksum & 0x00FF0000)>>16;
				TxData[3]=(checksum & 0x0000FF00)>>8;
				TxData[4]=(checksum & 0x000000FF);

				/*TxData[1]=(~checksum & 0xFF000000)>>24;
				TxData[2]=(~checksum & 0x00FF0000)>>16;
				TxData[3]=(~checksum & 0x0000FF00)>>8;
				TxData[4]=(~checksum & 0x000000FF);*/

				HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);

				G_return_CRC_CMD=0;
		  }
			/* Calculate and send CRC */
			if (G_return_BOOT_CRC_CMD) {
				uint32_t checksum;
				checksum = btld_GetBootChecksum();

				/* Send Achecksu, */
				TxHeader.ExtId = TX_FEEDBACK_CANID + CANTX_SA;
				TxHeader.IDE = CAN_ID_EXT;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.DLC = 5;
				TxData[0] = 0x0F;

				TxData[1] = (checksum & 0xFF000000) >> 24;
				TxData[2] = (checksum & 0x00FF0000) >> 16;
				TxData[3] = (checksum & 0x0000FF00) >> 8;
				TxData[4] = (checksum & 0x000000FF);

				HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

				G_return_BOOT_CRC_CMD = 0;
			}
			if (G_start_flash_CMD) {
				btld_FlashBegin();
				G_FlashInProgress = 1;
			  // request next WORD
			  /* Send success */
			  TxHeader.ExtId=TX_FEEDBACK_CANID + CANTX_SA;
			  TxHeader.IDE=CAN_ID_EXT;
			  TxHeader.RTR=CAN_RTR_DATA;
			  TxHeader.DLC=2;
			  TxData[0]=0xF5;
			  TxData[1]=0xFF;	//Success

			  HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);

			  G_start_flash_CMD=0;
		  	}
		  if (G_end_flash_CMD && G_FlashInProgress){

			  btld_FlashEnd();
			  G_FlashInProgress=0;
			  G_end_flash_CMD=0;

			  // request next WORD
			  /* Send success */
			  TxHeader.ExtId=TX_FEEDBACK_CANID + CANTX_SA;
			  TxHeader.IDE=CAN_ID_EXT;
			  TxHeader.RTR=CAN_RTR_DATA;
			  TxHeader.DLC=2;
			  TxData[0]=0xF7;
			  TxData[1]=0xFF;	//Success

			  HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
		  }


			// LED CONTROL
			if (G_LedUpdate) {
				// every 500mS update the LED Status
				static uint8_t led_state = 1;

				switch (led_state) {
				case 1:
					HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin,1);
					led_state++;
					break;
				case 2:
					HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin,0);
					led_state++;
					break;
				case 3:
					HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin,0);
					led_state = 1;
					break;
				default:
					break;
				}
				G_LedUpdate=0;
			}


	  }else{
		  /* Jump to app if timeout*/
		  if(G_mSCounter>50){
			  btld_JumpToApp();
		  }
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
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
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 32000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DEBUG_LED_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Interrupt callback functions-----------------------------------------------*/

/* HAL_TIM_PeriodElapsedCallback ---------------------------------------------*/
/* Interrupt callback to manage timer interrupts							  */
/* htim19 1ms																  */
/*----------------------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	/* htim17 1ms */
	if (&htim17 == htim) {

		static uint16_t cpt2=0;

		G_mSCounter++;

		if(G_mSCounter % 500 == 0){
			G_LedUpdate = 1;
		}

		if(G_mSCounter>=G_MAX_MS_COUNTER){
			G_mSCounter=0;
		}

		cpt2++;

		/* 1 S, reinitiate cpt2 */
		if(cpt2>=1000){
			cpt2=0;
		}

		/* 100 mS */
		if(cpt2%100==0){
			/* Main loop -----------------------------------------------------*/

			/* Input Read ------------------------------------------------*/

			/*------------------------------------------------ Input Read */

			/* Control loop ----------------------------------------------*/


			/*---------------------------------------------- Control loop */

			/* Output Write ----------------------------------------------*/
			if(G_loader_mode){
				//Send CAN message to know we are in loader mode
				TxHeader.ExtId=TX_HEARTBEAT_CANID + CANTX_SA;
				TxHeader.IDE=CAN_ID_EXT;
				TxHeader.RTR=CAN_RTR_DATA;
				TxHeader.DLC=1;
				TxData[0]=0xFF;

				HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
			}
			/*---------------------------------------------- Output Write */

			/*----------------------------------------------------- Main loop */
		}

	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *phcan){
	//uint32_t transmitmailbox;
	int retry=0;

	if((phcan->ErrorCode & HAL_CAN_ERROR_TX_ALST0) || (phcan->ErrorCode & HAL_CAN_ERROR_TX_TERR0)){
		HAL_CAN_AbortTxRequest(phcan,CAN_TX_MAILBOX0);
		retry=1;
	}
	if((phcan->ErrorCode & HAL_CAN_ERROR_TX_ALST1) || (phcan->ErrorCode & HAL_CAN_ERROR_TX_TERR1)){
		HAL_CAN_AbortTxRequest(phcan,CAN_TX_MAILBOX1);
		retry=1;
	}
	if((phcan->ErrorCode & HAL_CAN_ERROR_TX_ALST2) || (phcan->ErrorCode & HAL_CAN_ERROR_TX_TERR2)){
		HAL_CAN_AbortTxRequest(phcan,CAN_TX_MAILBOX2);
		retry=1;
	}

	HAL_CAN_ResetError(phcan);

	if(retry==1){
		HAL_CAN_DeactivateNotification(phcan,CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_ActivateNotification(phcan,CAN_IT_TX_MAILBOX_EMPTY);
		//HAL_CAN_AddTxMessage(phcan,&(CanTxList.SendMsgBuff.header),CanTxList.SendMsgBuff.Data,&transmitmailbox);
	}
}

/* HAL_CAN_TxCpltCallback ----------------------------------------------------*/
/* Interrupt callback to manage Can Tx Ready                                  */
/*----------------------------------------------------------------------------*/
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* phcan){
	CAN_TX_Cplt(phcan);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* phcan){
	CAN_TX_Cplt(phcan);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* phcan){
	CAN_TX_Cplt(phcan);
}

void CAN_TX_Cplt(CAN_HandleTypeDef* phcan){

}



/* HAL_CAN_RxCpltCallback ----------------------------------------------------*/
/* Interrupt callback to manage Can Rx message ready to read                  */
/*----------------------------------------------------------------------------*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* phcan){

	CAN_RxHeaderTypeDef rxheader;
	uint8_t data[8];

	HAL_CAN_GetRxMessage(phcan,CAN_RX_FIFO0,&rxheader,data);

	/* Low level Can management ----------------------------------------------*/
	if(rxheader.ExtId==(RX_CMD_CANID + CANRX_SA) && rxheader.IDE==CAN_ID_EXT){
		if (data[0]==0xF0){
			G_loader_mode=1;
		}
		if (data[0]==0xF1 && G_loader_mode==1){
			G_flash_erase_CMD=1;
		}
		if (data[0]==0xF2 && G_loader_mode==1){
			if(btld_CheckForApplication()==BL_OK){
				G_run_app_CMD=1;
			}
		}
		if (data[0]==0xF3 && G_loader_mode==1){
			/* Send the available App size in bytes */
			uint32_t app_size;

			/* APP_SIZE is in DWORD, we multiply by 4 to have the size in bytes */
			app_size=APP_SIZE;//((END_ADDRESS - APP_ADDRESS) + 1);

			/* Send Available flash size for application */
			TxHeader.ExtId=TX_FEEDBACK_CANID + CANTX_SA;
			TxHeader.IDE=CAN_ID_EXT;
			TxHeader.RTR=CAN_RTR_DATA;
			TxHeader.DLC=5;
			TxData[0]=0xF3;

			TxData[1]=(app_size & 0xFF000000)>>24;
			TxData[2]=(app_size & 0x00FF0000)>>16;
			TxData[3]=(app_size & 0x0000FF00)>>8;
			TxData[4]=(app_size & 0x000000FF);

			HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
		}
		if (data[0]==0xF4 && G_loader_mode==1){
			G_return_CRC_CMD=1;
		}
		if (data[0]==0xF5 && G_loader_mode==1){
			G_index=0;
			G_start_flash_CMD=1;
		}
		if (data[0] == 0xF6 && G_loader_mode == 1 && rxheader.DLC >= 5) {


			G_write_next_CMD = 1;
			G_uint32_to_write = (((uint32_t) data[1]))
					| (((uint32_t) data[2]) << 8) | (((uint32_t) data[3]) << 16)
					| (((uint32_t) data[4]) << 24);

			HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
			HAL_TIM_Base_Stop_IT(&htim17);

			if (G_index==((((uint32_t) data[5]) | ((uint32_t) data[6]) << 8))*4) {

				if (btld_FlashNext_32(G_uint32_to_write,&G_index) == BL_OK) {
					G_write_next_CMD = 0;

					/* Send success // request next WORD */
					send_confirm_msg(0xFF);

				} else {
					// Send failed msg

					/* Send failure */
					send_confirm_msg(0x00);

					G_write_next_CMD = 0;
					btld_FlashEnd();
					G_FlashInProgress = 0;
				}
			} else {
				G_write_next_CMD = 0;

				/* Send success // request next WORD */
				send_confirm_msg(0xFF);
			}
			HAL_TIM_Base_Start_IT(&htim17);
			HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
		}
		if (data[0]==0xF7 && G_loader_mode==1){
			G_end_flash_CMD=1;
		}
		if (data[0]==0x0F && G_loader_mode==1){
			G_return_BOOT_CRC_CMD=1;
		}

#if READ_PROTECT
		if (data[0] == 0xF8 && data[1] == 0xEF && data[2] == 0xFE
				&& data[3] == 0x40 && data[4] == 0x20) {
			/*Disable READ/WRITE protection*/
			btld_ConfigProtection(BL_PROTECTION_NONE);

		}
		if (data[0] == 0xF9 && data[1] == 0xEF && data[2] == 0xFE
				&& data[3] == 0x40 && data[4] == 0x20) {
			/*Enable READ/WRITE protection*/
			btld_ConfigProtection(BL_PROTECTION_RDP);

		}
#endif

	}

}


void config_can_filter(void){
   CAN_FilterTypeDef sFilterConfig;

   	   	   	   	   	   	   	  /* Leave mask bits for different messages commands
   	   	   	   	   	   	   	  |  	  	  	  	  	  	  	  	  	  	  	  	 */
   uint32_t filterMask=	RXFILTERMASK;
   uint32_t filterID=	RXFILTERID; // Only accept bootloader CAN message ID

  /*##-2- Configure the CAN Filter ###########################################*/
  //sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = filterID >> 13;
  sFilterConfig.FilterIdLow = (0x00FF & (filterID << 3)) | (1 << 2);
  sFilterConfig.FilterMaskIdHigh = filterMask >> 13;
  sFilterConfig.FilterMaskIdLow = (0x00FF & (filterMask << 3)) | (1 << 2);
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterBank = 0;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void Run_Application(void){
	  HAL_TIM_Base_Stop_IT(&htim17);
	  HAL_TIM_Base_DeInit(&htim17);
	  HAL_CAN_DeactivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	  HAL_CAN_Stop(&hcan);
	  HAL_CAN_DeInit(&hcan);
	  btld_JumpToApp();
}


void send_confirm_msg(uint8_t status){
	uint16_t index;

	index = G_index/4;

	TxHeader.ExtId = 0x00FE00 + CANTX_SA;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 4;
	TxData[0] = 0xF6;
	TxData[1] = status;	//0xFF = Success, 0x00 = failed
	TxData[2] = index & 0x00FF;
	TxData[3] = (index & 0xFF00)>>8;

	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
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
