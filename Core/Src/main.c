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
#include <stdio.h>
//#include <string.h>

#include "eeprom_emul.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEFAULT_APP1_START_ADDRESS 0x08005000
#define DEFAULT_APP2_START_ADDRESS 0x08012000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define LED_ON 	(LED_GPIO_Port->ODR |= LED_Pin)
#define LED_OFF (LED_GPIO_Port->ODR &= ~LED_Pin)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t application_write_idx = 0;


/*##################################*/
/* EEPROM EMULATION VARIABLES */
__IO uint32_t ErasingOnGoing = 0;
EE_Status EE_status = EE_OK;
//uint32_t eeprom_conf_app_addr;
/*##################################*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

static void eeprom_read_conf(uint32_t *value);
static void eeprom_write_conf(uint32_t value);

static void goto_application(void);
static HAL_StatusTypeDef copy_App2_to_App1(uint8_t *data, uint16_t data_len);

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
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  /*############################*/
  /*      EERPOM Emulation      */

  /* Enable and set FLASH Interrupt priority */
  /* FLASH interrupt is used for the purpose of pages clean up under interrupt */
  HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FLASH_IRQn);

  /* Activate NMI generation when two errors are detected */
  __HAL_FLASH_ENABLE_IT(FLASH_IT_ECCC);

  /* Set EEPROM emulation firmware to erase all potentially incompletely erased
     pages if the system came from an asynchronous reset. Conditional erase is
     safe to use if all Flash operations where completed before the system reset */


    /* System reset comes from a power-on reset: Forced Erase */
    /* Initialize EEPROM emulation driver (mandatory) */
	HAL_FLASH_Unlock();
    EE_status = EE_Init(EE_FORCED_ERASE);
    HAL_FLASH_Lock();
    if(EE_status != EE_OK) {Error_Handler();}

  /*    End EERPOM Emulation    */
  /*############################*/


  uint8_t uart_data;
  HAL_UART_Receive_IT(&huart2, &uart_data, 1);


  uint32_t value;
  eeprom_read_conf(&value);

//  printf("Valor lido da EEPROM: %08lX\n", value);

  uint16_t fw_len = value;

  if(fw_len)
  {
	  printf("New firmware size: %d\n", fw_len);

	  application_write_idx = 0;
	  fw_len = (fw_len % 8) ? fw_len + 8 - (fw_len % 8) : fw_len;

	  uint32_t APP2_Address = DEFAULT_APP2_START_ADDRESS;
	  copy_App2_to_App1( (uint8_t *)(APP2_Address), fw_len);

	  eeprom_write_conf(0x00000000);

  }

  for(uint8_t i = 0; i < 3; i++)
  {
//	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  LED_ON;
	  HAL_Delay(80);
//	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  LED_OFF;
	  HAL_Delay(80);
  }

    // Jump to application
    printf("Going to APP1\n");
    goto_application();









  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */





#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  return ch;
}




static void eeprom_read_conf(uint32_t *value)
{
	HAL_FLASH_Unlock();

	while(ErasingOnGoing){}

	EE_status = EE_ReadVariable32bits(1, value);

	HAL_FLASH_Lock();

	return;
}



static void eeprom_write_conf(uint32_t value)
{

	HAL_FLASH_Unlock();

	while(ErasingOnGoing){}

	EE_status  = EE_WriteVariable32bits(1, value);

	if ((EE_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {ErasingOnGoing = 1;EE_status|= EE_CleanUp_IT();}
	if ((EE_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {Error_Handler();}

	while(ErasingOnGoing){}

	HAL_FLASH_Lock();

}









static void goto_application( void )
{

	void (*app_reset_handler)(void) = (void*)( *( (volatile uint32_t*)(DEFAULT_APP1_START_ADDRESS + 4U) ) );

//	__set_MSP(*(volatile uint32_t*) ETX_APP_START_ADDRESS);

	// Turn OFF the Led to tell the user that Bootloader is not running
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET );

	app_reset_handler();    //call the app reset handler
}









static HAL_StatusTypeDef copy_App2_to_App1(uint8_t *data, uint16_t data_len)
{
	HAL_StatusTypeDef ret;

	do
	{
		ret = HAL_FLASH_Unlock();
//		printf("Erasing APP1...\r\n");
		//Erase the Flash
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t SectorError;

		EraseInitStruct.TypeErase	= FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Page		= 10; // Start page of APP1
		EraseInitStruct.NbPages		= 26; // Quant pages of APP1

		ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
		FLASH_WaitForLastOperation(HAL_MAX_DELAY);

		if(SectorError != 0xFFFFFFFF)
		{
		  printf("Flash Erase ERROR\r\n");
		  break;
		}
		if( ret != HAL_OK )
		{
		break;
		}
		printf("APP1 Erased!\r\n");

		application_write_idx = 0;

//		printf("data_len: %d\n", data_len);

	    for(int i = 0; i < data_len/8 ; i++)
	    {
	    	FLASH_WaitForLastOperation(HAL_MAX_DELAY);

	      uint64_t doubleword_data = 	data[i * 8] |
	    		  	  	  	  	   ((uint64_t)(data[i * 8 + 1]) << 8)	|
								   ((uint64_t)(data[i * 8 + 2]) << 16)	|
								   ((uint64_t)(data[i * 8 + 3]) << 24)	|
								   ((uint64_t)(data[i * 8 + 4]) << 32)	|
								   ((uint64_t)(data[i * 8 + 5]) << 40)	|
								   ((uint64_t)(data[i * 8 + 6]) << 48)	|
								   ((uint64_t)(data[i * 8 + 7]) << 56);

	      ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD,
	                               (DEFAULT_APP1_START_ADDRESS + application_write_idx ),
	                               doubleword_data
	                             );
	      FLASH_WaitForLastOperation(HAL_MAX_DELAY);

	      if( ret == HAL_OK )
	      {
	        //update the data count
	        application_write_idx += 8;
	      }
	      else
	      {
	        printf("Flash Write Error...HALT!!!\r\n");
	        break;
	      }
	    }// end for

	    ret = HAL_FLASH_Lock();
	    if( ret != HAL_OK )
	    {
	      break;
	    }

	}while(0);


	printf("Written %d bytes\n", application_write_idx);


	return ret;

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

	if(EE_status) // if error
	{
		EE_Format(EE_FORCED_ERASE);
		HAL_Delay(100);
		HAL_NVIC_SystemReset();
	}

  __disable_irq();
  while (1)
  {
	  LED_ON;
	  HAL_Delay(1000);
	  LED_OFF;
	  HAL_Delay(1000);
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
