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
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "font.h"
#include "ssd1306.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
float Time_sec =0.01;
float ADC_Reading=0;
float f_value=0;
uint8_t Vref=3.3;

float current=0.0;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*not the clock in simulation no equal the clock in the real time
 * so the timer2 period is 99 instead 9999
 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define sleep   0
#define Ready   1
#define ON 		2
#define mode_0  0
#define mode_1  1
#define mode_2  2
#define mode_3  3

#define speed_0	0
#define speed_1	64
#define speed_2	128
#define speed_3	170
#define speed_4	255

#define stop 	2
#define forward 1
#define reverse 0

#define NUM_BUTTONS 3

#define DEBOUNCE_DELAY 300




uint8_t PWM_value=speed_1;
uint8_t botton_flag[4]={0};
uint8_t select_flag[2]={mode_0,mode_0};
uint8_t Power_flag =sleep ;				//	main mode , sup mode
uint8_t timer_counter=0;
uint8_t pushed_flag[4]={0};
uint8_t released_flag[4]={0};
uint8_t buttonPins[4]={GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6};
uint8_t lastDebounceTime[4]={0};
uint32_t currentTime =0;
uint8_t speed_m[]="speed    ";
uint8_t tourq_stop_m[]="torq stop";
uint8_t tourq_rev_m[]="tourq rev";
uint8_t angle_m[]="angle    ";

uint8_t * modes[4]={tourq_stop_m,speed_m,tourq_rev_m,angle_m};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* remember clear botton flag after end */
/* remember clear botton flag after end */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void excute (uint8_t P_flag,uint8_t *S_flag);
void Select (uint8_t *B_flag);
void SetMotorSpeed(uint32_t pulse);
void check_botton (void);
uint8_t check_power_button (void);
void motor_direction (uint8_t dirc,uint8_t speed);
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */
 /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE END 2 */
  uint8_t string[4]={0};
SSD1306_Init();
SSD1306_GotoXY(42,0);
SSD1306_Puts("MODE",&Font_11x18,1);
SSD1306_UpdateScreen();
while (1)
{
	switch (check_power_button())
	{
	case Ready:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_Base_Stop_IT(&htim3);
        check_botton();
        Select(botton_flag);
        SSD1306_GotoXY(0, 23);
        SSD1306_Puts(modes[select_flag[0]],&Font_11x18,1);
        itoa(select_flag[1],string,10);
        SSD1306_GotoXY(58, 42);
        SSD1306_Puts(string,&Font_11x18,1);
        SSD1306_UpdateScreen();
        break;
	case ON:
		excute(ON,select_flag);
		break;
	default :break;
	}
	}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = PWM_value;
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
// timer 2 the clock source is
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000-1;
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
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */


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
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : button_1_Pin button_2_Pin button_3_Pin button_4_Pin */
  GPIO_InitStruct.Pin = button_1_Pin|button_2_Pin|button_3_Pin|button_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
//
//  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
//
//  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 20);
	  uint16_t value= HAL_ADC_GetValue(&hadc1);
	  f_value=(value/4096.0)*Vref;
  }
  if (htim->Instance == TIM3)
  {
	  timer_counter++;
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

//    for (int i = 0; i < NUM_BUTTONS; i++) {
//        if (GPIO_Pin == buttonPins[i]) {
//// Set the button pressed flag
//            }
//            if (HAL_GPIO_ReadPin(GPIOA,buttonPins[i])==GPIO_PIN_RESET)
//            {
//                uint32_t currentTime = HAL_GetTick();
//            	pushed_flag[i]=1;
//            }
//            else
//            {
//            	if (pushed_flag[i]==1)
//            	{
//                botton_flag[i] = 1;
//            	pushed_flag[i]=0;
//            	}
//            }
//            break;
//        }
//    }
}


/* USER CODE END 4 */
void excute (uint8_t P_flag,uint8_t *S_flag)
{
static uint8_t Speed =speed_3;
static uint8_t direction =forward;
static uint8_t cont =0;
// forward ---> PB4 +     PB5-
// reverse ---> PB4 -     PB5+

    if (P_flag == ON)
    {
    		// ***************************forward direction ***********************
    	motor_direction(direction,Speed);
    		switch (S_flag[0])
    		{
    		case mode_0:
    			HAL_TIM_Base_Start_IT(&htim2);
    			switch (S_flag[1])
    			    	{
				case mode_0:
					if (f_value >= 0.5 || cont==1)
					{
						if (timer_counter<= 10)
						{
							HAL_TIM_Base_Start_IT(&htim3);
	        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
	        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
	        				cont=1;
	        				direction=stop;
						}
						else
						{
							HAL_TIM_Base_Stop_IT(&htim3);
		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		        			cont=0;
		        			direction=forward;
		        			timer_counter=0;
						}
					}
    		    	else
    		    	{
        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
	        			direction=forward;
    		    	}

					break;
				case mode_1:
					if (f_value >= 1 || cont==1)
					{
						if (timer_counter<= 10)
						{
							HAL_TIM_Base_Start_IT(&htim3);
	        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
	        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		        			direction=stop;
		        			cont=1;
						}
						else
						{
							HAL_TIM_Base_Stop_IT(&htim3);
		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		        			direction=forward;
		        			timer_counter=0;
		        			cont=0;
						}
					}
    		    	else
    		    	{
        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
	        			direction=forward;
    		    	}

					break;
				case mode_2:
					if (f_value >= 1.59 || cont==1)
					{
						if (timer_counter<= 10)
						{
							HAL_TIM_Base_Start_IT(&htim3);
	        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
	        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		        			direction=stop;
		        			cont=1;
						}
						else
						{
							HAL_TIM_Base_Stop_IT(&htim3);
		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		        			direction=forward;
		        			timer_counter=0;
		        			cont=0;
						}
					}
    		    	else
    		    	{
        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
	        			direction=forward;
    		    	}

					break;
				case mode_3:
					if (f_value >= 2 || cont==1)
					{
						if (timer_counter<= 10)
						{
							HAL_TIM_Base_Start_IT(&htim3);
	        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
	        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		        			direction=stop;
		        			cont=1;
						}
						else
						{
							HAL_TIM_Base_Stop_IT(&htim3);
		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		        			direction=forward;
		        			timer_counter=0;
		        			cont=0;
						}
					}
    		    	else
    		    	{
        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
	        			direction=forward;
    		    	}

					break;
    			    	default :break;
    			    	}
    			HAL_Delay(200);
    			break;
    			case mode_1:// *************************** speed mode *****************************************
    				switch (S_flag[1])
    				    			    	{
    					case mode_0:
							SetMotorSpeed(speed_1);
							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
							Speed = speed_1;
    				    	break;
						case mode_1:
							SetMotorSpeed(speed_2);
							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
							Speed = speed_2;
							break;
						case mode_2:
							SetMotorSpeed(speed_3);
							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
							Speed = speed_3;
							break;
						case mode_3:
							SetMotorSpeed(speed_4);
							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
							Speed = speed_4;
							break;

						default :break;
    				    			    	}
    				break;
    				case mode_2: //****************************** torque reverse mode **********************************
    					HAL_TIM_Base_Start_IT(&htim2);
    					switch (S_flag[1])
    					{
    					case mode_0:
    						if (f_value >= 0.5 || cont==1)
    						{
    							if (timer_counter<= 10)
    							{
    								HAL_TIM_Base_Start_IT(&htim3);
    		        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
    		        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
    			        			direction=reverse;
    			        			cont=1;
    							}
    							else
    							{
    								HAL_TIM_Base_Stop_IT(&htim3);
        		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
        		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
        		        			direction=forward;
        		        			timer_counter=0;
        		        			cont=0;
    							}
    						}
    	    		    	else
    	    		    	{
    	        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
    	        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
    		        			direction=forward;
    	    		    	}

    						break;
						case mode_1:
    						if (f_value >= 1 || cont==1)
    						{
    							if (timer_counter<= 10)
    							{
    								HAL_TIM_Base_Start_IT(&htim3);
    		        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
    		        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
    			        			direction=reverse;
    			        			cont=1;
    							}
    							else
    							{
    								HAL_TIM_Base_Stop_IT(&htim3);
        		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
        		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
        		        			direction=forward;
        		        			timer_counter=0;
        		        			cont=0;
    							}
    						}
    	    		    	else
    	    		    	{
    	        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
    	        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
    		        			direction=forward;
    	    		    	}

							break;
						case mode_2:
    						if (f_value >= 1.59 || cont==1)
    						{
    							if (timer_counter<= 10)
    							{
    								HAL_TIM_Base_Start_IT(&htim3);
    		        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
    		        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
    			        			direction=reverse;
    			        			cont=1;
    							}
    							else
    							{
    								HAL_TIM_Base_Stop_IT(&htim3);
        		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
        		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
        		        			direction=forward;
        		        			timer_counter=0;
        		        			cont=0;
    							}
    						}
    	    		    	else
    	    		    	{
    	        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
    	        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
    		        			direction=forward;
    	    		    	}

							break;
						case mode_3:
    						if (f_value >= 2 || cont==1)
    						{
    							if (timer_counter<= 10)
    							{
    								HAL_TIM_Base_Start_IT(&htim3);
    		        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
    		        				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
    			        			direction=reverse;
    			        			cont=1;
    							}
    							else
    							{
    								HAL_TIM_Base_Stop_IT(&htim3);
        		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
        		        			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
        		        			direction=forward;
        		        			timer_counter=0;
        		        			cont=0;
    							}
    						}
    	    		    	else
    	    		    	{
    	        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
    	        		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
    		        			direction=forward;
    	    		    	}

							break;

						default :break;
    					}
    					break;
    					case mode_3://***************************** additional mode **************************
    						switch (S_flag[1])
    						{
    						case mode_0:
    			//    		excution mode 0
    			//    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
    			//    		HAL_Delay(50);
    							break;
    						case mode_1:
    			//    		excution mode 1
    			//    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
    			//			HAL_Delay(50);
    			//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
    			//			HAL_Delay(50);
    							break;
    						case mode_2:
    			//    		excution mode 2
    			//    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
    			//			HAL_Delay(50);
    			//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
    			//			HAL_Delay(50);
    			//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
    			//			HAL_Delay(50);
    			//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
    			//			HAL_Delay(50);
    							break;
    						case mode_3:
    			//    		excution mode 3
    			//    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
    			//			HAL_Delay(50);
    			//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
    			//			HAL_Delay(50);
    			//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
    			//			HAL_Delay(50);
    			//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
    			//			HAL_Delay(50);
    			//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
    			//			HAL_Delay(50);
    			//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
    			//			HAL_Delay(50);
    							break;

    						default :break;
    						}
    						break;
    						default :break;


    		}


//    	HAL_Delay(200);
    }
}
void Select (uint8_t *B_flag)
{

    if (B_flag[1]==1)
    {
//		up selection
    	select_flag[1]++;
        if (select_flag[1]==4)
        {
        	select_flag[1]=mode_0;
        }
        B_flag[1]=0;
    }
    else if (B_flag[2]==1)
    {
//    	down selection
        if (select_flag[1]==0)
        {
        	select_flag[1]=4;
        }
        select_flag[1]--;
        B_flag[2]=0;
    }
//    selection button
		if (B_flag[0]==1)
		    {
		//		up selection
			select_flag[0]++;
		        if (select_flag[0]==4)
		        {
		        	select_flag[0]=mode_0;
		        }
		        select_flag[1]=mode_0;
		        B_flag[0]=0;
		    }


	/*
	 *
	 * 		section of displaying oled selection
	 *
	 *
	 */
}
void SetMotorSpeed(uint32_t pulse)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;   // Inverted polarity
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; // Inverted polarity
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}
void check_botton (void)
{
	currentTime = HAL_GetTick();
	for (uint8_t i =0;i<NUM_BUTTONS;i++)
	{
	    if ((currentTime - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
	        lastDebounceTime[i] = currentTime;

		if (HAL_GPIO_ReadPin(GPIOA,buttonPins[i])==GPIO_PIN_RESET)
		{
			botton_flag[i]=1;
			while (1)
			{
				if (HAL_GPIO_ReadPin(GPIOA,buttonPins[i])==GPIO_PIN_SET)
					break;
			}
			break;
		}
	}
    }
}
uint8_t check_power_button (void)
{
	static uint8_t state=0;
	currentTime = HAL_GetTick();
	    if ((currentTime - lastDebounceTime[3]) > DEBOUNCE_DELAY) {
	        lastDebounceTime[3] = currentTime;

		if (HAL_GPIO_ReadPin(GPIOA,buttonPins[3])==GPIO_PIN_RESET)
		{
			botton_flag[3]=1;
			while (1)
			{
				if (HAL_GPIO_ReadPin(GPIOA,buttonPins[3])==GPIO_PIN_SET)
				{
					state++;
					if (state==3)
						state=1;
					break;
				}
			}
		}
	}

return state;
}
void motor_direction (uint8_t dirc,uint8_t speed)
{
	switch (dirc)
	{
	case stop :
    	SetMotorSpeed(speed);
    	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		break;
	case forward:
    	SetMotorSpeed(speed);
    	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		break;
	case reverse:
    	SetMotorSpeed(speed);
    	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
		break;
	default :break;
	}
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
