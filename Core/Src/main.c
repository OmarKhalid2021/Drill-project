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
#include "ST7735.h"
#include "GFX_FUNCTIONS.h"
#include "figure.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
float Time_sec =0.01;
float ADC_Reading=0;
float last_f_value=0;
float f_value=0;
uint8_t Vref=3.3;
float current=0.0;
float shunt_voltage =0;
float last_shunt_voltage =0;


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
#define MODE_S	3
#define ON 		2
#define mode_0  0
#define mode_1  1
#define mode_2  2
#define mode_3  3
#define mode_4  4

#define stop 	2
#define forward 1
#define reverse 3

#define NUM_BUTTONS 3

#define DEBOUNCE_DELAY 300

#define INA219_ADDRESS (0x40 << 1) // Default I2C address for INA219
#define INA219_REG_CONFIG 0x00     // INA219 configuration register
#define INA219_REG_SHUNTVOLTAGE 0x01 // Shunt voltage register
#define INA219_REG_BUSVOLTAGE 0x02 // Bus voltage register

#define ST7735_ON 	1
#define ST7735_OFF 	0




uint8_t botton_flag[4]={0};
uint8_t Power_flag =sleep ;				//	main mode , sup mode
uint32_t timer_counter=0;
uint8_t buttonPins[4]={GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6};
uint8_t lastDebounceTime[4]={0};
uint32_t currentTime =0;
uint16_t A_Speed[20]={50,100,150,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000};
uint8_t A_Direction[2]={forward,reverse};
float A_Torque[]={0.6,1.2,1.8,2.4,3.0,3.6,4.2,4.8,5.4,6.0,6.6,7.2,7.8,7.4,8.0,8.6,9.2,9.8};
uint8_t A_Right_angle[]={30,60,90,120,150,180};
uint8_t A_Left_angle[]={180,150,120,90,60,30};
uint8_t array_state[5]={0};
uint8_t size_array[5]={20,2,18,6,6};
uint8_t Flag=0;
uint16_t Speed ;
uint16_t Last_Speed=0 ;
uint8_t Direction ;
uint16_t Last_Direction=0;
uint8_t Right_A;
uint16_t Last_Right_A=0;
uint8_t Left_A;
uint16_t Last_Left_A=0;
/* USER CODE END PD */
float Torque;
float Last_Torque=0;
uint8_t last_D=forward;
uint8_t power_state=sleep;
uint8_t IN_MODE=0;
uint8_t Last_IN_MODE=0;
uint8_t Mode=mode_0;


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* remember clear botton flag after end */
/* remember clear botton flag after end */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

WWDG_HandleTypeDef hwwdg;

SPI_HandleTypeDef hspi2;


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
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
void ST7735_state(uint8_t state);

/* USER CODE BEGIN PFP */
void excute ();
void Select (uint8_t *B_flag);
void SetMotorSpeed(uint32_t pulse);
void check_botton (void);
uint8_t check_power_button (void);
void motor_Direction (uint8_t dirc,uint16_t speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */

  /* USER CODE BEGIN 1 */
	void ADC_init(uint32_t channel)
	{
		  ADC_ChannelConfTypeDef sConfig = {0};
		  sConfig.Channel = channel;
		  sConfig.Rank = 1;
		  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
		  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		  {
		    Error_Handler();
		  }

	}
	int main(void)
	{
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
  MX_I2C2_Init();
  MX_SPI2_Init();
  ST7735_Init(3);
  /* USER CODE BE
   * GIN 2 */
//  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */
 /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE END 2 */
  uint8_t ch[2]={0};
  uint8_t string[3]={0};
  uint8_t buffer[3]={0};
  uint8_t flag=0;
  uint8_t flag_1=0;
  uint8_t flag_2=0;
  uint8_t flag_3=0;
  uint8_t flag_4=0;
  uint8_t e_flag=0;

  uint16_t config_data = 0x399F; // Example: Configuring for continuous bus and shunt voltage measurement
uint8_t config_data_bytes[2];
config_data_bytes[0] = (config_data >> 8) & 0xFF; // MSB
config_data_bytes[1] = config_data & 0xFF; // LSB


// Write configuration to INA219
HAL_I2C_Mem_Write(&hi2c2, INA219_ADDRESS, INA219_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, config_data_bytes, 2, HAL_MAX_DELAY);
ST7735_SetRotation(3);
while (1)
{
	switch (check_power_button())
	{
	case sleep:
	ST7735_state(ST7735_OFF);
	break;
	case Ready:
	case MODE_S:
		ST7735_state(ST7735_ON);
	if (flag==0)
	{
		ST7735_DrawImage(0,0, 160, 128, image_data);
		HAL_Delay(4000);
		flag=1;
		flag_1=0;
		flag_2=0;
		flag_3=0;
		flag_4=0;
		e_flag=0;
		Last_Direction=0;
		Last_IN_MODE=0;
		Last_Left_A=0;
		Last_Right_A=0;
		Last_Speed=0;
		Last_Torque=0;
	}
	if (flag_1==0)
	{
		fillScreen(BLACK);
		HAL_TIM_Base_Start_IT(&htim3);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_Base_Start_IT(&htim2);
		ST7735_FillRectangle(125,24, 34, 16, BLUE);
		ST7735_FillRectangle(127,26, 30, 12, BLACK);//
        flag_1=1;
        e_flag=0;
	}
	check_botton();
    Select(botton_flag);

        if (Mode==mode_0 || Mode==mode_1)
{
        	if (flag_4==0)
{

	if (Mode==mode_0)
		ST7735_WriteString(5, 26, "M1", Font_11x18, WHITE, BLACK);
	flag_4=1;
	flag_3=0;
	flag_2=0;
	Last_Left_A=0;
	Last_Right_A=0;
}
	if (flag_2==0)
	{

	if (Mode ==mode_1)
		ST7735_WriteString(5, 26, "M2", Font_11x18, WHITE, BLACK);
	ST7735_WriteString(5, 65, "(RMB)", Font_7x10, WHITE, BLACK);
	ST7735_WriteString(95, 65, "(Ncm)", Font_7x10, WHITE, BLACK);
	flag_2=1;
	flag_3=0;
	flag_4=0;

        	}

        Speed=A_Speed[array_state[0]];
        Torque=A_Torque[array_state[2]];
        Direction=A_Direction[array_state[1]];
if (Direction!=Last_Direction)
{
        if (Direction==forward)
        	  ST7735_DrawImage(63,27, 33, 33, Forward_d);
        else if (Direction==reverse)
        	  ST7735_DrawImage(63,27, 33, 33, reverse_d);
        Last_Direction=Direction;
}
if (Speed!=Last_Speed)
{
        itoa(Speed,string,10);
        ST7735_WriteString(5, 85,"    ", Font_11x18, BLACK, BLACK);
        ST7735_WriteString(5, 85, string, Font_11x18, WHITE, BLACK);
        Last_Speed=Speed;
}
if (Torque!=Last_Torque)
{
        sprintf(buffer, "%f", Torque);
        ST7735_WriteString(95, 85, buffer, Font_11x18, WHITE, BLACK);
        ST7735_WriteString(129, 85, "  ", Font_11x18, BLACK, BLACK);

        Last_Torque=Torque;
}

}
else if (Mode==mode_2)
{
	if (flag_3==0)
	{
	ST7735_WriteString(5, 26, "M3", Font_11x18, WHITE, BLACK);
	ST7735_WriteString(5, 65, "(Right A)", Font_7x10, WHITE, BLACK);
	ST7735_WriteString(95, 65,"(Left A) ", Font_7x10, WHITE, BLACK);
	ST7735_DrawImage(63, 27, 33, 33, Forward_reverse);
	Speed=800;
	Direction=forward;
	flag_3=1;
	flag_2=0;
	flag_4=0;
	Last_Direction=0;
	Last_IN_MODE=0;
	Last_Speed=0;
	Last_Torque=0;
	}
	Right_A=A_Right_angle[array_state[3]];
	Left_A= A_Left_angle[array_state[4]];
if (Right_A!=Last_Right_A)
{
	itoa(Right_A,string,10);
    ST7735_WriteString(5, 85,"     ", Font_11x18, BLACK, BLACK);
    ST7735_WriteString(5, 85, string, Font_11x18, WHITE, BLACK);
    Last_Right_A=Right_A;
}
if (Left_A!=Last_Left_A)
{
    itoa(Left_A,buffer,10);
    ST7735_WriteString(95, 85, "     ", Font_11x18, BLACK, BLACK);
    ST7735_WriteString(95, 85, buffer, Font_11x18, WHITE, BLACK);
    Last_Left_A=Left_A;
}

}
        if (IN_MODE!=Last_IN_MODE)
        {
        	switch (IN_MODE)
        	{
        	case 1:
            	ST7735_DrawImage(60, 80, 20, 20, left_arrow);
            	break;
        	case 2:
            	ST7735_DrawImage(60, 80, 20, 20, up_arrow);
            	break;
        	case 3:
            	ST7735_DrawImage(60, 80, 20, 20, right_arrow);
            	break;
        	case 0:
            	ST7735_DrawImage(60, 80, 20, 20, right);
            	break;
        	default :break;
        	}
        	Last_IN_MODE=IN_MODE;
        }
        if (timer_counter>=60000)
        {
        	power_state=sleep;
        	HAL_TIM_Base_Stop_IT(&htim3);
        	timer_counter=0;
        	flag=0;
        	ST7735_DrawImage(0, 0, 160, 128, good_bye);
        	HAL_Delay(1500);
        }
        break;
	case ON:
		if(e_flag==0)
		{
			uint8_t counter=11;
        	ST7735_DrawImage(60, 80, 20, 20, right);
			timer_counter=0;
        	HAL_TIM_Base_Stop_IT(&htim3);
        	HAL_TIM_Base_Stop_IT(&htim2);
			last_D=Direction;
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
			fillScreen(BLACK);
			for(uint8_t i=4;i<160;i+=16)
			{
				counter--;
				itoa(counter,ch,10);
				ST7735_WriteString(i, 87, ch, Font_7x10, WHITE, BLACK);
				if (counter==1)
					break;
			}
				for (uint8_t i =0;i<160;i++)
			ST7735_DrawPixel(i,84, BLUE);
			e_flag=1;
			flag_1=0;
			flag_2=0;
			flag_3=0;
			Last_Direction=0;
			Last_IN_MODE=0;
			Last_Left_A=0;
			Last_Right_A=0;
			Last_Speed=0;
			Last_Torque=0;

		}

		excute();
	    uint8_t shunt_voltage_bytes[2]={0};
	    HAL_I2C_Mem_Read(&hi2c2, INA219_ADDRESS, INA219_REG_SHUNTVOLTAGE, I2C_MEMADD_SIZE_8BIT, shunt_voltage_bytes, 2, HAL_MAX_DELAY);
	    int16_t raw_shunt_voltage = (shunt_voltage_bytes[0] << 8) | shunt_voltage_bytes[1];
	    shunt_voltage = raw_shunt_voltage * 0.001; // 10 µV per LSB
	    break;

	default :break;
	}
	 // Read shunt voltage




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



  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
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
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}


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
  htim1.Init.Prescaler = 45;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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
  sConfigOC.Pulse = 1000;
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
  htim3.Init.Prescaler = 83999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10-1;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin|ST7735_CS_Pin|ST7735_BLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7735_RES_Pin|ST7735DC_Pin|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : button_1_Pin button_2_Pin button_3_Pin button_4_Pin */
  GPIO_InitStruct.Pin = button_1_Pin|button_2_Pin|button_3_Pin|button_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin ST7735_CS_Pin ST7735_BLK_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|ST7735_CS_Pin|ST7735_BLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : charging_2_Pin */
  GPIO_InitStruct.Pin = charging_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(charging_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7735_RES_Pin ST7735DC_Pin PB4 PB5 */
  GPIO_InitStruct.Pin = ST7735_RES_Pin|ST7735DC_Pin|GPIO_PIN_4|GPIO_PIN_5;
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
	  ADC_init(ADC_CHANNEL_1);
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 20);
	  uint16_t value= HAL_ADC_GetValue(&hadc1);
	  f_value=(value/4096.0)*Vref;
	  f_value-=1.7;
	  if ((((uint8_t)last_f_value*10)*3)>(((uint8_t)f_value*10)*3))
	  ST7735_FillRectangle(127,26, ((uint8_t)last_f_value*10)*3, 12, BLACK);
	  ST7735_FillRectangle(127,26, ((uint8_t)f_value*10)*3, 12, BLUE);
	  last_f_value=f_value;

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
void excute ()
{
  motor_Direction(Direction,Speed);
HAL_Delay(80);
if (shunt_voltage<0)
	shunt_voltage=shunt_voltage*(-1);
ST7735_FillRectangle(159-(16*((uint8_t)last_shunt_voltage)), 44, 16, 40, BLACK);
ST7735_FillRectangle(159-(16*((uint8_t)shunt_voltage)), 44, 16, 40, BLUE);
last_shunt_voltage=shunt_voltage;
switch (Mode){
	case mode_0:
		if (shunt_voltage >= Torque )
			{
			if (last_D==forward)
				Direction=reverse;
			else if (last_D==reverse)
				Direction=forward;
			}
		else
			Direction=last_D;
		break;
	case mode_1:
		if (shunt_voltage >= Torque )
			{
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,GPIO_PIN_SET);
			HAL_Delay(80);
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,GPIO_PIN_RESET);
			HAL_Delay(80);
			}
		break;
	case mode_2:
		HAL_TIM_Base_Start_IT(&htim3);
		if (timer_counter<Right_A)
			Direction=forward;
		else if (timer_counter<(Right_A+Left_A))
			Direction=reverse;
		else
		{
			HAL_TIM_Base_Stop_IT(&htim3);
			timer_counter=0;
		}
		break;
	default :break;
}

}

void Select (uint8_t *B_flag)
{

	if (B_flag[0]==1 )
	{
		power_state=MODE_S;
		IN_MODE++;
		if(IN_MODE==4)
			IN_MODE=1;
		B_flag[0]=0;
	}
	if (power_state==Ready)
	{
		if (B_flag[1]==1)
		{
			Mode++;
			if (Mode==4)
				Mode=0;
			B_flag[1]=0;
		}
		if (B_flag[2]==1)
		{
			if (Mode==0)
				Mode=4;
			Mode--;
			B_flag[2]=0;
		}

	}
	if (power_state==MODE_S)
	{
		switch (Mode)
		{
		case mode_0:
		case mode_1:
		if (B_flag[1]==1)
		{
			//up
			array_state[IN_MODE-1]++;
			if (array_state[IN_MODE-1]==size_array[IN_MODE-1])
				array_state[IN_MODE-1]=0;
			B_flag[1]=0;
		}
		if (B_flag[2]==1)
		{
			//down
			if (array_state[IN_MODE-1]==0)
				array_state[IN_MODE-1]=size_array[IN_MODE-1];
			array_state[IN_MODE-1]--;
			B_flag[2]=0;
		}
		break;
		case mode_2:
			if (B_flag[1]==1)
			{
				//up
				array_state[3]++;
				array_state[4]++;
				if (array_state[3]==size_array[3])
					array_state[3]=0;
				if (array_state[4]==size_array[4])
					array_state[4]=0;
				B_flag[1]=0;
			}
			if (B_flag[2]==1)
			{
				//down
				if (array_state[3]==0)
					array_state[3]=size_array[3];
				if (array_state[4]==0)
					array_state[4]=size_array[4];
				array_state[3]--;
				array_state[4]--;
				B_flag[2]=0;
			}
			break;
		default :break;
		}


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
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
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
			timer_counter=0;
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
			timer_counter=0;
			while (1)
			{
				if (HAL_GPIO_ReadPin(GPIOA,buttonPins[3])==GPIO_PIN_SET)
				{
					if (power_state==2 )
						power_state=0;
					else if (power_state==3)
					{
						power_state=0;
						IN_MODE=0;
					}
					power_state++;
					break;
				}
			}
		}
	}

return power_state;
}
void motor_Direction (uint8_t dirc,uint16_t speed)
{
	SetMotorSpeed(speed);
	switch (dirc)
	{
	case stop :
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		break;
	case forward:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
		break;
	case reverse:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
		break;
	default :break;
	}
}
void ST7735_state(uint8_t state)
{
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, state);
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
