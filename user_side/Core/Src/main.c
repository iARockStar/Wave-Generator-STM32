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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD.h"
#include "stdio.h"
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
 ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
	LCD_Puts(0, 0,"99243042 - Rey");
	LCD_Puts(0, 1,"99243011 - ili");
	HAL_Delay(1200);
	LCD_Clear();
	
	uint8_t data = 2;
	uint8_t selected = 0;
	
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(&hspi1, &data, sizeof(data), HAL_MAX_DELAY);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	
	//HAL_SPI_Transmit(&hspi1, &data, sizeof(data), HAL_MAX_DELAY);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(&hspi1, &data, sizeof(data), HAL_MAX_DELAY);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	
	float voltage_value  = 0;
	float voltage_to_ms = 0;
	float voltage_to_Hz = 0;
	char show_ms[100];
	char show_Hz[100];
	int is_prompt_flag = 0;
	int is_prompt_flag2 = 0;
	int is_prompt_flag3 = 0;
	int row = -1;
	int col = -1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LCD_Clear();
		is_prompt_flag3 = 0;
		while(is_prompt_flag == 0){
		
		// show the available waves to the user 
		LCD_Puts(0, 0, "1Sin 2Squ 3Tri 4SinA 5Stp 6Saw");
		
		row = -1;
		col = -1;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			is_prompt_flag = 1;
			row = 0;
			col = 0;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
			is_prompt_flag = 1;
			row = 0;
			col = 1;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0){
			is_prompt_flag = 1;
			row = 0;
			col = 2;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);
		}
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			is_prompt_flag = 1;
			row = 1;
			col = 0;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
			is_prompt_flag = 1;
			row = 1;
			col = 1;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0){
			is_prompt_flag = 1;
			row = 1;
			col = 2;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);
		}
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			is_prompt_flag = 1;
			row = 2;
			col = 0;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
			is_prompt_flag = 1;
			row = 2;
			col = 1;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0){
			is_prompt_flag = 1;
			row = 2;
			col = 2;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);
		}
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			is_prompt_flag = 1;
			row = 3;
			col = 0;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
			is_prompt_flag = 1;
			row = 3;
			col = 1;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0){
			is_prompt_flag = 1;
			row = 3;
			col = 2;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);
		}
		
		if(is_prompt_flag == 1){
				LCD_Clear();
				
			uint8_t data = -1;
				//TODO
			if(row == 0 && col == 0){
				data = 1;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_SPI_Transmit(&hspi1, &data, sizeof(data), HAL_MAX_DELAY);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			
			}else if(row == 0 && col == 1){
				data = 2;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_SPI_Transmit(&hspi1, &data, sizeof(data), HAL_MAX_DELAY);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
				
			}else if(row == 0 && col == 2){
				data = 3;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_SPI_Transmit(&hspi1, &data, sizeof(data), HAL_MAX_DELAY);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			}else if(row == 1 && col == 0){
				data = 4;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_SPI_Transmit(&hspi1, &data, sizeof(data), HAL_MAX_DELAY);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			}else if(row == 1 && col == 1){
				data = 5;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_SPI_Transmit(&hspi1, &data, sizeof(data), HAL_MAX_DELAY);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			}else if(row == 1 && col == 2){
				data = 6;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_SPI_Transmit(&hspi1, &data, sizeof(data), HAL_MAX_DELAY);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		}
		}
	}
		LCD_Clear();
	
		while(is_prompt_flag2 == 0){
		
		// start ADC
		HAL_ADC_Start(&hadc1);
		
		// wait for converting from analog to digital value
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		
		// create voltage value, maximum VDD is 5 v and resolution is 12 bits 
		// so 2 ^ 12 = 4096 - 1 = 4095
    voltage_value = (float)(HAL_ADC_GetValue(&hadc1) * 5) / (4095);
		
		// convert to ms, which is 500 <= ms <= 10000
		voltage_to_ms = (voltage_value * (10000 - 500) / 5) + 500;
		
		
		sprintf(show_ms,"%.1fms",voltage_to_ms);	
			
		LCD_Puts(0, 0, show_ms);
				
		row = -1;
		col = -1;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			
			row = 0;
			col = 0;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
			
			row = 0;
			col = 1;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0){
			
			row = 0;
			col = 2;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);
		}
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			
			row = 1;
			col = 0;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
			
			row = 1;
			col = 1;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0){
			
			row = 1;
			col = 2;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);
		}
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			
			row = 2;
			col = 0;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
			
			row = 2;
			col = 1;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0){
			
			row = 2;
			col = 2;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);
		}
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			
			row = 3;
			col = 0;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
			
			row = 3;
			col = 1;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0){
			
			row = 3;
			col = 2;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);
		}
		
		if(row == 3 && col == 2){
			LCD_Clear();
			// TODO 
			LCD_Puts(0, 0,show_ms);
			is_prompt_flag2 = 1;
			uint8_t tmp = (uint8_t) (voltage_value * 10);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,&tmp,sizeof(tmp),HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		
		}	
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
  }
		LCD_Clear();
		LCD_Puts(0, 0, show_ms);
		while(is_prompt_flag3 == 0){
		
		// start ADC
		HAL_ADC_Start(&hadc1);
		
		// wait for converting from analog to digital value
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		
		// create voltage value, maximum VDD is 5 v and resolution is 12 bits 
		// so 2 ^ 12 = 4096 - 1 = 4095
    voltage_value = (float)(HAL_ADC_GetValue(&hadc1) * 5) / (4095);
			
		// convert to Hz, which is 1 <= hz <= 1000
		voltage_to_Hz = (voltage_value * (1000 - 1) / 5) + 1;
		
		sprintf(show_Hz,"%.1fHz",voltage_to_Hz);
			
		LCD_Puts(0, 1, show_Hz);
		
		row = -1;
		col = -1;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			is_prompt_flag = 1;
			row = 0;
			col = 0;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
			is_prompt_flag = 1;
			row = 0;
			col = 1;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0){
			is_prompt_flag = 1;
			row = 0;
			col = 2;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);
		}
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			is_prompt_flag = 1;
			row = 1;
			col = 0;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
			is_prompt_flag = 1;
			row = 1;
			col = 1;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0){
			is_prompt_flag = 1;
			row = 1;
			col = 2;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);
		}
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			is_prompt_flag = 1;
			row = 2;
			col = 0;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
			is_prompt_flag = 1;
			row = 2;
			col = 1;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0){
			is_prompt_flag = 1;
			row = 2;
			col = 2;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);
		}
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0){
			is_prompt_flag = 1;
			row = 3;
			col = 0;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
			is_prompt_flag = 1;
			row = 3;
			col = 1;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		}else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0){
			is_prompt_flag = 1;
			row = 3;
			col = 2;
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0);
		}
		
			if(row == 3 && col == 2){
			
			LCD_Clear();
			// TODO 
			LCD_Puts(0, 0, show_ms);
			LCD_Puts(0, 1,show_Hz);
			uint8_t tmp = (uint8_t) (voltage_value * 10);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,&tmp,sizeof(tmp),HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			tmp = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			while(tmp != 1){
			
			HAL_SPI_Receive(&hspi1,&tmp,sizeof(tmp),HAL_MAX_DELAY);
			
			}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			char sel[200];
			
			is_prompt_flag3 = 1;
			is_prompt_flag = 0;
			is_prompt_flag2 = 0;
			
			LCD_Clear();
				sprintf(sel,"%d",tmp);
			LCD_Puts(0, 0, sel);
			
			HAL_Delay(1000);
		
		}
	}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
