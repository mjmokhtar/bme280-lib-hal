/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "BME280_STM32.h"
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
BME280_HandleTypeDef bme280;
char uart_buffer[200];

// SWV ITM printf support (alternative method)
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        ITM_SendChar((*ptr++));
    }
    return len;
}

void print_float_value(float value, char* label, char* unit) {
    int integer_part = (int)value;
    int fractional_part = (int)((value - integer_part) * 100);

    if (value < 0 && integer_part == 0) {
        sprintf(uart_buffer, "%s: -%d.%02d%s\r\n", label, integer_part, fractional_part, unit);
    } else {
        sprintf(uart_buffer, "%s: %d.%02d%s\r\n", label, integer_part, abs(fractional_part), unit);
    }

    // Send via ST-Link VCP
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
}

void print_pressure_value(float pressure) {
    int pa_integer = (int)pressure;
    int pa_fractional = (int)((pressure - pa_integer) * 100);

    int hpa_integer = (int)(pressure / 100.0f);
    int hpa_fractional = (int)(((pressure / 100.0f) - hpa_integer) * 100);

    sprintf(uart_buffer, "Pressure: %d.%02d Pa (%d.%02d hPa)\r\n",
            pa_integer, pa_fractional, hpa_integer, hpa_fractional);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  sprintf(uart_buffer, "\r\n=== ST-Link VCP Test ===\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

      sprintf(uart_buffer, "System Clock: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

      sprintf(uart_buffer, "HAL Tick: %lu ms\r\n", HAL_GetTick());
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

      // Also send to SWV ITM (for debug console)
      printf("\n=== ST-Link VCP Test ===\n");
      printf("System Clock: %lu Hz\n", HAL_RCC_GetSysClockFreq());
      printf("HAL Tick: %lu ms\n", HAL_GetTick());

      HAL_Delay(1000);

      // Initialize BME280
      sprintf(uart_buffer, "Initializing BME280...\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
      printf("Initializing BME280...\n");

      if (BME280_Init(&bme280, &hi2c1, BME280_ADDRESS_PRIMARY)) {
          sprintf(uart_buffer, "BME280 initialization successful!\r\n");
          HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
          printf("BME280 initialization successful!\n");

          // Read and display chip ID
          uint8_t chipID = BME280_GetChipID(&bme280);
          sprintf(uart_buffer, "BME280 Chip ID: 0x%02X\r\n", chipID);
          HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
          printf("BME280 Chip ID: 0x%02X\n", chipID);

      } else {
          sprintf(uart_buffer, "BME280 initialization failed!\r\n");
          HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
          printf("BME280 initialization failed!\n");

          // Try I2C scan
          sprintf(uart_buffer, "Scanning I2C devices...\r\n");
          HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

          bool found = false;
          for (uint8_t addr = 1; addr < 128; addr++) {
              if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 3, 100) == HAL_OK) {
                  sprintf(uart_buffer, "Found I2C device at: 0x%02X\r\n", addr);
                  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
                  printf("Found I2C device at: 0x%02X\n", addr);
                  found = true;
              }
          }

          if (!found) {
              sprintf(uart_buffer, "No I2C devices found!\r\n");
              HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
              printf("No I2C devices found!\n");
          }

          // Continue with test pattern even if BME280 failed
      }

      uint32_t counter = 0;

      while (1) {
          // Send heartbeat message
          sprintf(uart_buffer, "\r\n=== Loop #%lu ===\r\n", counter);
          HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
          printf("\n=== Loop #%lu ===\n", counter);

          if (BME280_IsConnected(&bme280)) {
              // Read sensor data
              float temperature = BME280_ReadTemperature(&bme280);
              float pressure = BME280_ReadPressure(&bme280);
              float humidity = BME280_ReadHumidity(&bme280);
              float altitude = BME280_ReadAltitude(&bme280, 1013.25f);

              // Print to ST-Link VCP
              print_float_value(temperature, "Temperature", " C");
              print_pressure_value(pressure);
              print_float_value(humidity, "Humidity", "%");
              print_float_value(altitude, "Altitude", " m");

              // Print to SWV ITM
              int temp_int = (int)temperature;
              int temp_frac = (int)((temperature - temp_int) * 100);
              printf("Temperature: %d.%02d C\n", temp_int, abs(temp_frac));

              int press_int = (int)(pressure / 100.0f);
              int press_frac = (int)(((pressure / 100.0f) - press_int) * 100);
              printf("Pressure: %d.%02d hPa\n", press_int, press_frac);

              int hum_int = (int)humidity;
              int hum_frac = (int)((humidity - hum_int) * 100);
              printf("Humidity: %d.%02d %%\n", hum_int, hum_frac);

          } else {
              sprintf(uart_buffer, "BME280 not connected!\r\n");
              HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
              printf("BME280 not connected!\n");
          }

          sprintf(uart_buffer, "Uptime: %lu ms\r\n", HAL_GetTick());
          HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
          printf("Uptime: %lu ms\n", HAL_GetTick());

          counter++;
          HAL_Delay(2000);
      }
  /* USER CODE END 3 */
}

// Test function untuk manual call
void Test_VCP_Output(void) {
    char test_msg[] = "Manual VCP Test - Hello World!\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  hi2c1.Init.ClockSpeed = 100000;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
