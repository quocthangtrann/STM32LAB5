/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t rx_temp = 0;

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;
volatile uint8_t rx_flag = 0;
volatile uint8_t at_line_start = 1;


/* Command parser */
char cmd_buffer[CMD_BUFFER_SIZE];
uint8_t cmd_len = 0;
volatile uint8_t command_available = 0;

/* ADC and packet storage */
uint32_t last_adc_value = 0;
char last_packet[PACKET_MAXLEN] = {0};

/* UART communication FSM */
typedef enum {
  COMM_IDLE = 0,
  COMM_SENT_WAIT_ACK,
} comm_state_t;

comm_state_t comm_state = COMM_IDLE;
uint32_t comm_timestamp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void uart_send_crlf(void)
{
  const char crlf[] = "\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)crlf, 2, 50);
}

/* Forward declarations of user functions */
void command_parser_fsm(void);
void uart_communication_fsm(void);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
   if (HAL_ADC_Start(&hadc1) != HAL_OK) {
     Error_Handler();
   }

   if (HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_temp, 1) != HAL_OK) {
     Error_Handler();
   }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	    /* blink LED to show main loop alive (pa5) */
	    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
	    HAL_Delay(500);

	    if (rx_flag) {
	      rx_flag = 0;
	      command_parser_fsm();
	    }

	    uart_communication_fsm();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint16_t next_head = rx_head + 1;
    if (next_head >= RX_BUFFER_SIZE) next_head = 0;

    if (next_head != rx_tail) {
      rx_buffer[rx_head] = rx_temp;
      rx_head = next_head;
      rx_flag = 1;
    } else {
    }

    if (rx_temp == '\r' || rx_temp == '\n') {
      const char crlf[] = "\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)crlf, 2, 10);
      at_line_start = 1;

    } else if (rx_temp == 0x08 || rx_temp == 127) {
      const char bs_seq[] = "\b \b";
      HAL_UART_Transmit(&huart2, (uint8_t*)bs_seq, 3, 10);
    } else {
      HAL_UART_Transmit(&huart2, (uint8_t *)&rx_temp, 1, 10);
      at_line_start = 0;
    }

    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_temp, 1);
  }
}

void command_parser_fsm(void)
{
  while (rx_tail != rx_head)
  {
    uint8_t b = rx_buffer[rx_tail];
    rx_tail++;
    if (rx_tail >= RX_BUFFER_SIZE) rx_tail = 0;

    if (b == '!') {
      cmd_len = 0;
      if (cmd_len < (CMD_BUFFER_SIZE - 1)) {
        cmd_buffer[cmd_len++] = '!';
      }
    } else if (cmd_len > 0) {
      if (cmd_len < (CMD_BUFFER_SIZE - 1)) {
        cmd_buffer[cmd_len++] = (char)b;
      } else {
        cmd_len = 0;
      }

      if (b == '#') {
        cmd_buffer[cmd_len] = '\0';

        {
          const char crlf[] = "\r\n";
          HAL_UART_Transmit(&huart2, (uint8_t*)crlf, 2, 50);
          at_line_start = 1;
        }

        {
          char dbg[48];
          int n = snprintf(dbg, sizeof(dbg), "PARSER: got='%s'\r\n", cmd_buffer);
          HAL_UART_Transmit(&huart2, (uint8_t*)dbg, n, 100);
        }

        command_available = 1;
        cmd_len = 0;
      }
    } else {
    }
  }
}

/* Helper: send last_packet stored (non-blocking within reason) */
static void transmit_last_packet(void)
{
  int len = (int)strlen(last_packet);
  if (len > 0) {
    HAL_UART_Transmit(&huart2, (uint8_t *)last_packet, len, 200);
    uart_send_crlf();
    at_line_start = 1; /* now we are at start of next line */

  }
}

/* Build and send packet from last_adc_value */
static void send_adc_packet(void)
{
  /* format: !ADC=1234# (enough space: PACKET_MAXLEN >= 12) */
  int len = snprintf(last_packet, PACKET_MAXLEN, "!ADC=%lu#", (unsigned long)last_adc_value);
  if (len > 0 && len < PACKET_MAXLEN) {
    HAL_UART_Transmit(&huart2, (uint8_t *)last_packet, len, 200);
    uart_send_crlf();
    at_line_start = 1;
    comm_state = COMM_SENT_WAIT_ACK;
    comm_timestamp = HAL_GetTick();
  }
}

/* ------------------ UART Communication FSM ------------------
   Handles commands:
     - !RST# : read ADC and send !ADC=xxxx#, then wait for !OK#
     - !OK#  : acknowledge receipt (stop waiting/resend)
   If ACK not received within ACK_TIMEOUT_MS, resend the packet (store last_packet).
*/
void uart_communication_fsm(void)
{
  if (command_available) {
    /* copy command to local buffer to avoid races */
    char local_cmd[CMD_BUFFER_SIZE];
    strncpy(local_cmd, cmd_buffer, CMD_BUFFER_SIZE);
    local_cmd[CMD_BUFFER_SIZE - 1] = '\0';
    command_available = 0;

    if (strcmp(local_cmd, "!RST#") == 0) {
      /* read latest ADC value (continuous conversion mode) */
      /* Optionally check conversion ready with HAL_ADC_PollForConversion if desired */
      last_adc_value = HAL_ADC_GetValue(&hadc1);
      send_adc_packet();
    } else if (strcmp(local_cmd, "!OK#") == 0) {
      /* ACK received: clear state */
      comm_state = COMM_IDLE;
    } else {
      /* Unknown command: optionally echo or ignore */
    }
  }

  /* handle ACK timeout and resend */
  if (comm_state == COMM_SENT_WAIT_ACK) {
    uint32_t now = HAL_GetTick();
    if ((now - comm_timestamp) >= ACK_TIMEOUT_MS) {
      /* resend stored packet */
      transmit_last_packet();
      comm_timestamp = now; /* restart timer */
    }
  }
}

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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

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
