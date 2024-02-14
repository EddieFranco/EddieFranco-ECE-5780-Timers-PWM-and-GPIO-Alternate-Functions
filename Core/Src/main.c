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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
void TIM2_IRQHandler()
		{
			// Toggle between the green (PC8) and orange (PC9) LEDs in the interrupt handler
			GPIOC->ODR ^= (GPIO_ODR_9 | GPIO_ODR_8);
			
			// clear the pending flag for the update interrupt in the status register
			 TIM2->SR = 0;
		}

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
  __HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	// Set up a configuration struct to pass to the initialization function
	GPIO_InitTypeDef initStr = {GPIO_PIN_6|GPIO_PIN_7| GPIO_PIN_8 | GPIO_PIN_9,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	
	// Initialize all of the LED pins in the main function.
	HAL_GPIO_Init(GPIOC, &initStr); 
	// Set the green LED (PC9) high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  /* USER CODE END SysInit */

	
	
  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	//Enable the timer 2 peripheral (TIM2) in the RCC.
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	// Configure the timer to trigger an update event (UEV) at 4 Hz.
	TIM2 -> PSC = 999;
	TIM2 -> ARR = 1999; 
	// Configure the timer to generate an interrupt on the UEV event
	TIM2->DIER |= TIM_DIER_UIE;
	// Configure and enable/start the timer
	TIM2->CR1 |= TIM_CR1_CEN;
	// Set up the timer’s interrupt handler, and enable in the NVIC
	NVIC_EnableIRQ(TIM2_IRQn);
	
	
	
  /* USER CODE END 2 */
	//Enable the timer 3 peripheral (TIM3) in the RCC.
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	// configure the timer to a UEV period related to 800 Hz (T = 1/f).
	TIM2 -> PSC = 999;
	TIM2 -> ARR = 1999; 
	// Use the Capture/Compare Mode Register 1 (CCMR1) register to configure the output channels
  //to PWM mode
	// The CCMR1 register configures channels 1 & 2, and the CCMR2 register for channels
  //3 & 4.
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC1S_0;
	// Examine the bit definitions for the OC1M[2:0] bit field; set output channel 1 to PWM
	// Mode 2.
	TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;  //  Clear OC1M bits
	TIM3->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); 
	// Use the OC2M[2:0] bit field to set channel 2 to PWM Mode 1.
	TIM3->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
	// Enable the output compare preload for both channels
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE_Msk ;
	TIM3->CCMR1 |=  TIM_CCMR1_OC2PE_Msk ;
	// Set the output enable bits for channels 1 & 2 in the CCER register.
	TIM3->CCMR1 |=  TIM_CCER_CC1E_Msk;
	TIM3->CCMR1 |=  TIM_CCER_CC2E_Msk;
	// Set the capture/compare registers (CCRx) for both channels to 20% of your ARR value
	TIM3->CCR1|=1000;
	TIM3->CCR2|=1000;
	
	/* USER CODE END 3 */
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN_Msk;
	GPIOC->MODER |= GPIO_MODER_MODER6_1 ;
	GPIOC->MODER |= GPIO_MODER_MODER7_1 ;
	//Alternate functions that connect to the capture/compare channels of timers have the
	//form: “TIMx_CHy”.
	GPIOC->AFR[0] |= GPIO_AFRL_AFSEL6_Msk;
	GPIOC->AFR[0] |= GPIO_AFRL_AFSEL7_Msk;
	
	
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//HAL_Delay(200);
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
