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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct {
	GPIO_TypeDef* PORT;
	uint16_t PIN;
}PortPin;

PortPin R[4] =
{
		{GPIOA,GPIO_PIN_10},
		{GPIOB,GPIO_PIN_3},
		{GPIOB,GPIO_PIN_5},
		{GPIOB,GPIO_PIN_4}
};

PortPin L[4] =
{
		{GPIOA,GPIO_PIN_9},
		{GPIOC,GPIO_PIN_7},
		{GPIOB,GPIO_PIN_6},
		{GPIOA,GPIO_PIN_7}
};

uint16_t ButtonMatrix = 0;
uint16_t state = 1;
uint16_t NumberInput = 0;
uint16_t NumberDelay = 0;
uint8_t NowState = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int ReadMatrixButton_1Row();
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static uint32_t timestamp = 0;
	  // 100 time per 1 second || 1 time per second = 1/100 = 0.01 s = 10 ms
	  if(HAL_GetTick() > timestamp){
		  timestamp = HAL_GetTick() + 10; // +10 ms
		  NumberInput = ReadMatrixButton_1Row();
	  }
	  switch (state) {
		case 1: //initial
			NowState = 1;

			if(NumberInput == 64 && NumberDelay == 0){ //Correct Number
				state = 3;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2;//Wrong
			break;
		case 2://wrong
			NowState = 2;

			if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			break;
		case 3://6
			NowState = 3;

			if(NumberInput == 16 && NumberDelay == 0){ //Correct Number
				state = 4;
			}
			else if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2;//Wrong

			break;
		case 4://4
			NowState = 4;

			if(NumberInput == 1024 && NumberDelay == 0){ //Correct Number
				state = 5;
			}
			else if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2;//Wrong

			break;
		case 5://3
			NowState = 5;

			if(NumberInput == 16 && NumberDelay == 0){ //Correct Number
				state = 6;
			}
			else if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2;//Wrong

			break;
		case 6://4
			NowState = 6;

			if(NumberInput == 4096 && NumberDelay == 0){ //Correct Number
				state = 7;
			}
			else if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2;//Wrong

			break;
		case 7://0
			NowState = 7;

			if(NumberInput == 32 && NumberDelay == 0){ //Correct Number
				state = 8;
			}
			else if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2;//Wrong

			break;
		case 8://5
			NowState = 8;

			if(NumberInput == 4096 && NumberDelay == 0){ //Correct Number
				state = 9;
			}
			else if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2;//Wrong

			break;
		case 9://0
			NowState = 9;

			if(NumberInput == 4096 && NumberDelay == 0){ //Correct Number
				state = 10;
			}
			else if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2;//Wrong

			break;
		case 10://0
			NowState = 10;

			if(NumberInput == 4096 && NumberDelay == 0){ //Correct Number
				state = 11;
			}
			else if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2;//Wrong

			break;
		case 11://0
			NowState = 11;

			if(NumberInput == 16 && NumberDelay == 0){ //Correct Number
				state = 12;
			}
			else if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2;//Wrong

			break;
		case 12://4
			NowState = 12;

			if(NumberInput == 16 && NumberDelay == 0){ //Correct Number
				state = 13;
			}
			else if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2;//Wrong

			break;
		case 13://4
			NowState = 13;

			if(NumberInput == 32768 && NumberDelay == 0){ //Click OK
				state = 14;
			}
			else if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2;//Wrong

			break;
		case 14://LED on
			NowState = 14;

			if(NumberInput == 8 && NumberDelay == 0){ //clear
				state = 1;
			}
			else if(NumberInput != 0 && NumberDelay == 0) state = 2; //Wrong

			break;
	}
	  NumberDelay = NumberInput;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int ReadMatrixButton_1Row() {
    static uint8_t X = 0;
    register int i;
    //check row (first time anything will pass because all R[X] set as high. it can start row check at the second time by set R[1] as low )
    for (i = 0; i < 4; i++) {
        if (HAL_GPIO_ReadPin(L[i].PORT, L[i].PIN)) {
            ButtonMatrix &= ~(1 << (X * 4 + i));
            //i = 0 x = 0
            //~(1 << (0*4 + 0))
            //~0b0000000000000001
            // 0b1111111111111110 & ButtonMatrix
            //=0b***************0
            //if noting change (doesn't pass button) ButtonMatrix will always 0b0000000000000000 by this step
            //it do right to left repeat
        }
        else {
            ButtonMatrix |= 1 << (X * 4 + i);
            //i = 0 x = 0
            //(1 << (0*4 + 0))
            // 0b0000000000000001
            // 0b0000000000000001 | ButtonMatrix
            //=0b***************1
            //if button is passed ButtonMatrix will get 1 to some bit by this step.
            //it can have double 1 in some bit like 0b0001000000001000 but for human response it hard to be like that (it check and shift bit every 10 ms.)
        }
    }
    //change column
    HAL_GPIO_WritePin(R[X].PORT, R[X].PIN, 1); //reset R[x] to 1 Floating
    HAL_GPIO_WritePin(R[(X + 1) % 4].PORT, R[(X + 1) % 4].PIN, 0); //Let R[X] to 0 by column (x start at 0 if it check all L[i] then do next column)
    X++; // because 1 set of i plus 1 to x
    X %= 4; // if X = 4 then x = 0
    return ButtonMatrix;
    /*---------------------------------STEP of Work---------------------------------------
     1. check row of column x (form i = 0 to i = 3)
     2. reset column x (R[x]) to high
     3. set next column R[x+1] to low *if x = 4 then x = 0
     repeat...
    */
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
