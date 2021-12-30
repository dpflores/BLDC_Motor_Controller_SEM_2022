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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "PID.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* Controller parameters */
#define PID_KP  0.04f
#define PID_KI  0.1f
#define PID_KD  0.0f

#define PID_TAU 0.02f

#define PID_LIM_MIN  0.0f
#define PID_LIM_MAX  255.0f

#define SAMPLE_TIME_S 0.01f


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//FUNCTIONS
void show_velocity(uint16_t vel); //Shows integer Velocity using three digits

void Hall_Decoder(void); // Makes the hall encoding for commutation



//VARIABLES

uint8_t power = 0; // Power state

int Sensors[3] = {0,0,0}; //Hall sensors

float duty_cycle = 0; //Duty cycle para el PWM

uint16_t adc_sample = 0;

uint8_t counts = 0;

float adc_sum = 0;

uint16_t adc_av = 0; // adc_average

float vel_d = 0; //desired velocity


float cruise_factor1 = 7.29; // factor to obtain the first cruise speed (35 km/h)

float cruise_factor2 = 8.5; // factor to obtain the second cruise speed (30 km/h)

float cruise_factor3 = 10.2; // factor to obtain the second cruise speed (25 km/h)

float cruise_factor = 7.29; // General cruise factor, initial value with 1

uint16_t steps = 0; // Number of steps by the motor

float vel_rpm = 0; // Number of steps by the motor

float velocity = 0; // Any of the other velocities

//CONTROL VALUES


float error = 0; //difference between desired and measured speed

float cumError = 0; //error*elapsedTime


float proportional = 0; // Proportional part

float integral = 0; //Integral part

float u = 0; //control law

//CAN VARIABLES

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint32_t TxMailbox;

uint8_t TxData[8];
uint8_t RxData[8];


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

	if (RxData[0] == 96 && RxData[1] == 234){ // HMI Page 60000

		if (RxData[4] == 1){ //HMI button 1
			power = 1;
		}
		if (RxData[4] == 2){ //HMI button 2
			power = 0;
		}


		if (RxData[5] == 16){ //HMI down button
			TxData[0] = 50;

		}
		if (RxData[5] == 8){ //HMI left button
			TxData[0] = 100;

		}
		if (RxData[5] == 32){ //HMI right button
			TxData[0] = 150;

		}
		if (RxData[5] == 2){ //HMI up button
			TxData[0] = 200;

		}

	}


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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,  GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim2);


  //CAN
  HAL_CAN_Start(&hcan);

  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  //Configurando la transmision

  TxHeader.DLC = 8;  // Son 8 bytes de data
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD; //Identificador del mensaje
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x103;  // Este es el ID que mandaremos al periferico
  TxHeader.TransmitGlobalTime = DISABLE;

  TxData[0] = 0; 	//Speed component
  TxData[1] = 0;	//Speed component
  TxData[2] = 0;	//Zero flag
  TxData[3] = 0;	//Power flag
  TxData[4] = 0;
  TxData[5] = 0;
  TxData[6] = 0;
  TxData[7] = 255;	//General flag


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  /* Initialize PID controller */

  PIDController pid = { PID_KP, PID_KI, PID_KD, PID_TAU,
						PID_LIM_MIN, PID_LIM_MAX, SAMPLE_TIME_S };

  PIDController_Init(&pid);

  HAL_ADC_Start_IT(&hadc1);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_cycle);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_cycle);

  //INITIAL POSITION

      //HALL A
      if (HAL_GPIO_ReadPin(HALL_A_GPIO_Port,HALL_A_Pin)) Sensors[0] = 1;
      else Sensors[0] = 0;
      //HALL B
      if (HAL_GPIO_ReadPin(HALL_B_GPIO_Port,HALL_B_Pin)) Sensors[1] = 1;
      else Sensors[1] = 0;
      //HALL C
      if (HAL_GPIO_ReadPin(HALL_C_GPIO_Port,HALL_C_Pin)) Sensors[2] = 1;
      else Sensors[2] = 0;



  while (1)
  {


	  if (power == 1){

		  HAL_ADC_Start_IT(&hadc1);

		  //HAL_CAN_AddTxMessage(&hcan, &TxHeader, &TxData, &TxMailbox);


		  //CONTROL

		  PIDController_Update(&pid, vel_d, vel_rpm);


		  integral = pid.integrator;

		  u = pid.out;

		  duty_cycle = u;


		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_cycle);
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_cycle);


		  //HALL DECODER

		  Hall_Decoder(); //Makes the hall encoding


	  }

	  //Poner modo sleep

	  else{
		  //Turn off the steering wheel leds
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,  GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,  GPIO_PIN_RESET);



		  //Turn off the low gates
		  HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);



		  //TURN OFF PID

		  PIDController_Reset(&pid);

		  integral = pid.integrator;

		  u = pid.out;

		  duty_cycle = u;


		  //Turn off the high gates
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, u);
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, u);
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, u);

	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
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
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x01<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x01<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* USER CODE END CAN_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 800 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(A_LOW_GPIO_Port, A_LOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, B_LOW_Pin|C_LOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : A_LOW_Pin */
  GPIO_InitStruct.Pin = A_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A_LOW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_A_Pin HALL_B_Pin HALL_C_Pin */
  GPIO_InitStruct.Pin = HALL_A_Pin|HALL_B_Pin|HALL_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B_LOW_Pin C_LOW_Pin */
  GPIO_InitStruct.Pin = B_LOW_Pin|C_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

//FUNCIONES



void Hall_Decoder(void){
	if (Sensors[2] == 1 && Sensors[1] == 0 && Sensors[0] == 1) {
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);


		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);


		//delay
		//HAL_Delay(0.001);

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_SET);



	}
	else if(Sensors[2] == 1 && Sensors[1] == 0 && Sensors[0] == 0) {

		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);

		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);


		//delay
		//HAL_Delay(0.001);

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_SET);

	}

	else if(Sensors[2] == 1 && Sensors[1] == 1 && Sensors[0] == 0) {

		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);

		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,   GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);


		//delay
		//HAL_Delay(0.001);

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_SET);

	}

	else if(Sensors[2] == 0 && Sensors[1] == 1 && Sensors[0] == 0) {

		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);


		//delay
		//HAL_Delay(0.001);

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_SET);

	}

	else if(Sensors[2] == 0 && Sensors[1] == 1 && Sensors[0] == 1) {

		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);


		//delay
		//HAL_Delay(0.001);

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_SET);

	}

	else if(Sensors[2] == 0 && Sensors[1] == 0 && Sensors[0] == 1) {

		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

		HAL_GPIO_WritePin(B_LOW_GPIO_Port , B_LOW_Pin,  GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A_LOW_GPIO_Port , A_LOW_Pin,  GPIO_PIN_RESET);


		//delay
		//HAL_Delay(0.001);

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(C_LOW_GPIO_Port , C_LOW_Pin,  GPIO_PIN_SET);

	}
}

//INTERRUPCIONES

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

  adc_sample = HAL_ADC_GetValue(&hadc1);

  /*
  adc_sum+= adc_sample/16.06;  // 16.06 para convertir de 12 bits a 8 bits
  counts += 1;

  if (counts == 150){
	  adc_av = adc_sum/counts;
	  adc_sum = 0;
	  counts = 0;

  }

  vel_d = adc_av/cruise_factor;
  duty_cycle = adc_av;
  */

  //PI control (reference)
  	adc_sum+= adc_sample/8.192;  // 13.65 para convertir de 12 bits a un intervalo de (0-500 rpm)
    counts += 1;

    if (counts == 1){
  	  adc_av = adc_sum/counts;
  	  adc_sum = 0;
  	  counts = 0;

    }

    vel_d = adc_av;

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){


	//HALL A
	if (GPIO_Pin == GPIO_PIN_0) {

	if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)) Sensors[0] = 1;
	else Sensors[0] = 0;

	steps+=1;
	}

	//HALL B
	else if (GPIO_Pin == GPIO_PIN_1) {

	if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)) Sensors[1] = 1;
	else Sensors[1] = 0;

	steps+=1;
	}

	//HALL C
	else if (GPIO_Pin == GPIO_PIN_10) {

	if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)) Sensors[2] = 1;
	else Sensors[2] = 0;

	steps+=1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//Formula para el motor de prueba
	//vel_rpm = 2*60*steps/90;  //cada MEDIO SEGUNDO se mide la cantidad de revoluciones por minuto

	//Formula para el motor de MK III
	vel_rpm = 2*60*steps/138;
	steps = 0;


	//Enviamos el valor de la velocidad

	if (vel_rpm > 255){
		TxData[0] = 255;
		TxData[1] = vel_rpm - 255;
	}
	else{
		TxData[0] = vel_rpm;
		TxData[1] = 0;
	}

	//Enviamos el estado ON OFF del sistema

	TxData[3] = power;  // Sending the ON (1) or OFF (0)

	//Send by CAN
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);


};


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

