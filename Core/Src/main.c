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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "sipf_client.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SW_POLL_TIMEOUT	(100)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UART_HANDLE_STLINK	(&hlpuart1)
#define UART_HANDLE_NRF9160	(&huart1)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t buff[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SipfClientUartInit(UART_HandleTypeDef *puart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int print_msg(const char *fmt, ...)
{
    static char msg[128];
    int  len;

    va_list list;
    va_start(list, fmt);
    len = vsprintf(msg, fmt, list);
    va_end(list);

    HAL_UART_Transmit(UART_HANDLE_STLINK, (uint8_t*)msg, len, 100);

    return len;
}

static void requestResetModule(void)
{
	// Reset request.
	HAL_GPIO_WritePin(OUTPUT_WAKE_IN_GPIO_Port, OUTPUT_WAKE_IN_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(OUTPUT_WAKE_IN_GPIO_Port, OUTPUT_WAKE_IN_Pin, GPIO_PIN_RESET);

	HAL_Delay(200);
}

static int waitBootModule(void)
{
	int len, is_echo = 0;

	// Wait READY message.
	for (;;) {
		len = SipfUtilReadLine(buff, sizeof(buff), 65000);
		if (len < 0) {
			// ERROR or BUSY or TIMEOUT
			return len;
		}
		if (len == 0) {
			continue;
		}
		if (len >= 13) {
			if (memcmp(buff, "*** SIPF Client", 15) == 0) {
				is_echo = 1;
			}
			//Detect READY message.
			if (memcmp(buff, "+++ Ready +++", 13) == 0) {
				break;
			}
			if (memcmp(buff, "ERR:Faild", 9) == 0) {
				print_msg("%s\r\n", buff);
				return -1;
			}
		}
		if (is_echo) {
			print_msg("%s\r\n", buff);
		}
	}
	return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int ret;
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
  MX_LPUART1_UART_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  print_msg("*** SIPF Client for Nucleo ***\r\n");

  SipfClientUartInit(UART_HANDLE_NRF9160);

  print_msg("Request module reset.\r\n");
  requestResetModule();

  print_msg("Waiting module boot\r\n");
  print_msg("### MODULE OUTPUT ###\r\n");
  ret = waitBootModule();
  if (ret != 0) {
	  print_msg("FAILED(%d)\r\n", ret);
	  return -1;
  }
  print_msg("#####################\r\n");
  print_msg("OK\r\n");

  HAL_Delay(100);

  print_msg("Set Auth mode... ");
  ret = SipfSetAuthMode(0x01);
  if (ret != 0) {
	print_msg((char *)buff, "FAILED(%d)\r\n", ret);
	return -1;
  }
  print_msg("OK\r\n");

  SipfClientFlushReadBuff();

  print_msg("+++ Ready +++\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t count_tx = 1;
  uint32_t poll_timeout = uwTick + SW_POLL_TIMEOUT;
  GPIO_PinState prev_ps = GPIO_PIN_SET;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	uint8_t b;
	for (;;) {
		if (SipfClientUartReadByte(&b) != -1) {
			HAL_UART_Transmit(UART_HANDLE_STLINK, &b, 1, 0);
		} else {
			break;
		}
	}
	for (;;) {
		if (HAL_UART_Receive(UART_HANDLE_STLINK, &b, 1, 0) == HAL_OK) {
			SipfClientUartWriteByte(b);
		} else {
			break;
		}
	}

	// B2 Push
	if (((int)poll_timeout - (int)uwTick) <= 0) {
		poll_timeout = uwTick + SW_POLL_TIMEOUT;
		GPIO_PinState ps;
		ps = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
		if ((prev_ps == GPIO_PIN_SET) && (ps == GPIO_PIN_RESET)) {
			print_msg("B1 PUSHED\r\nTX(tag_id: 0x01, type: 0x04, value: %d)\r\n", count_tx);
			memset(buff, 0, sizeof(buff));
			ret = SipfCmdTx(0x01, 0x04, (uint8_t*)&count_tx, 4, buff);
			if (ret == 0) {
				print_msg("OK(OTID: %s)\r\n", buff);
				count_tx++;
			} else {
				print_msg("NG\r\n");
			}
		}
		prev_ps = ps;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
