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
#include "usb_host.h"

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
#define AHT20_ADDR (0x38 << 1)  // Địa chỉ I2C của AHT20
uint8_t data_buffer[6];
char uart_buffer[50];
float sensor_data[4] = {0, 0, 0, 0};
char hc05_response[50];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void Debug_UART(char *msg);
void AHT20_Init();
float Read_AT20_Temperature();
void Read_AT20(float *temperature, float *humidity);
void Send_BLE(char *data);
void HC05_Receive();
void HC05_SendCommand(char *cmd);
void HC05_Config();
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
  MX_I2C1_Init();
  MX_USB_HOST_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Debug_UART("\r\n[DEBUG] System Initialized");
  /* USER CODE END 2 */

  AHT20_Init();
  HC05_Config();
    for (uint8_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 3, 100) == HAL_OK) {
            sprintf(uart_buffer, "\r\n[DEBUG] Found I2C Device at 0x%02X", addr);
            Debug_UART(uart_buffer);
        }
    }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		MX_USB_HOST_Process();

		Debug_UART("\r\n[DEBUG] Reading temperature");
//		float temp = Read_AT20_Temperature();

		float temp, humidity;
		Read_AT20(&temp, &humidity);

		sprintf(uart_buffer, "\r\nTemp: %.2f C | Humidity: %.2f%%", temp, humidity);
		Send_BLE(uart_buffer);

		sensor_data[0] = temp;
		sensor_data[2] = humidity;
		//    sprintf(uart_buffer, "Temp: %.2f C\n", temp);
//		if(temp > 0)
		char buffer[10];
	    int test_value = 40;  // Giá trị cần gửi
	    sprintf(buffer, "\r\n%d", test_value);  // Định dạng dữ liệu, thêm \n
		Send_BLE(buffer);
		Debug_UART(buffer);
		HC05_Receive();
//		Debug_UART("\r\n[DEBUG] Cycle complete, waiting...");
		HAL_Delay(2000);  // Gửi mỗi 2 giây

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
	  RCC_ClkInitTypeDef RCC_ClkInitStruct;
	  RCC_OscInitTypeDef RCC_OscInitStruct;

	  /* Enable Power Control clock */
	  __HAL_RCC_PWR_CLK_ENABLE();

	  /* The voltage scaling allows optimizing the power consumption when the device is
	     clocked below the maximum system frequency, to update the voltage scaling value
	     regarding system frequency refer to product datasheet.  */
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	  /* Enable HSI Oscillator and activate PLL with HSI as source */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = 0x10;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	  RCC_OscInitStruct.PLL.PLLM = 16;
	  RCC_OscInitStruct.PLL.PLLN = 400;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	  RCC_OscInitStruct.PLL.PLLQ = 7;
	  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	     clocks dividers */
	  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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
	while (1) {
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

void Debug_UART(char *msg) {
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
}

void AHT20_Init() {
    uint8_t command[3] = {0xBA, 0x00, 0x00};  // Lệnh reset cảm biến
    HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, command, 3, 100);
    HAL_Delay(10);  // Chờ reset xong

    Debug_UART("\r\n[DEBUG] AHT20 Reset Done");
}


float Read_AT20_Temperature() {

    if (HAL_I2C_IsDeviceReady(&hi2c1, AHT20_ADDR, 3, 100) != HAL_OK) {
        Debug_UART("[ERROR] AHT20 Not Found!\n");
        return -100.0;
    }

    uint8_t command[3] = {0xAC, 0x33, 0x00};

	HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, command, 3, 100);
	HAL_Delay(80);  // Đợi cảm biến xử lý (80ms)

	HAL_I2C_Master_Receive(&hi2c1, AHT20_ADDR, data_buffer, 6, 100);  // Đọc 6 byte dữ liệu

	if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
		Debug_UART("\r\n[ERROR] I2C Read Failed!");
		return -100.0;
	}

	// Kiểm tra bit trạng thái (bit 7 của byte đầu tiên)
	if ((data_buffer[0] & 0x80) != 0) {
		Debug_UART("\r\n[ERROR] Sensor Not Ready!");
		return -100.0;
	}

	// Ghép 20-bit dữ liệu nhiệt độ
	uint32_t raw_temp = ((data_buffer[3] & 0x0F) << 16) | (data_buffer[4] << 8) | data_buffer[5];
	float temperature = ((raw_temp * 200.0) / 1048576.0) - 50.0;  // Công thức từ datasheet

	sprintf(uart_buffer, "\r\n[DEBUG] Temp: %.2foC | Raw: 0x%02X 0x%02X 0x%02X",
			temperature, data_buffer[3], data_buffer[4], data_buffer[5]);
	Debug_UART(uart_buffer);

	return temperature;
}

void Read_AT20(float *temperature, float *humidity) {

    if (HAL_I2C_IsDeviceReady(&hi2c1, AHT20_ADDR, 3, 100) != HAL_OK) {
        Debug_UART("[ERROR] AHT20 Not Found!\n");
        return -100.0;
    }

    uint8_t command[3] = {0xAC, 0x33, 0x00};

	HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, command, 3, 100);
	HAL_Delay(80);  // Đợi cảm biến xử lý (80ms)

	HAL_I2C_Master_Receive(&hi2c1, AHT20_ADDR, data_buffer, 6, 100);  // Đọc 6 byte dữ liệu

	if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
		Debug_UART("\r\n[ERROR] I2C Read Failed!");
		return -100.0;
	}

	// Kiểm tra bit trạng thái (bit 7 của byte đầu tiên)
	if ((data_buffer[0] & 0x80) != 0) {
		Debug_UART("\r\n[ERROR] Sensor Not Ready!");
		return -100.0;
	}

	uint32_t raw_humidity = ((data_buffer[1] << 16) | (data_buffer[2] << 8) | data_buffer[3]) >> 4;
	uint32_t raw_temperature = ((data_buffer[3] & 0x0F) << 16) | (data_buffer[4] << 8) | data_buffer[5];

	*humidity = ((float)raw_humidity / 1048576) * 100.0; // Chuyển đổi độ ẩm
	*temperature = ((float)raw_temperature / 1048576) * 200.0 - 50.0; // Chuyển đổi nhiệt độ

	sprintf(uart_buffer, "\r\n[DEBUG] Temp: %.2f C | Humidity: %.2f%%", *temperature, *humidity);
	Debug_UART(uart_buffer);
}

void Send_BLE(char *data) {
    Debug_UART("\r\n[DEBUG] Sending data to BLE");
    HAL_UART_Transmit(&huart1, (uint8_t *)data, strlen(data), 100);
//    Debug_UART("\r\n[DEBUG] Data sent to BLE");
}

void HC05_Receive() {
    uint8_t rx_data[50];
    if (HAL_UART_Receive(&huart1, rx_data, sizeof(rx_data), 1000) == HAL_OK) {
        sprintf(uart_buffer, "[BLE] Received: %s\n", rx_data);
        Debug_UART(uart_buffer);
    }
}

void HC05_SendCommand(char *cmd) {
    char buffer[50];
    sprintf(buffer, "%s\r\n", cmd);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 100);
    HAL_Delay(200); // Chờ HC-05 xử lý lệnh

    // Nhận phản hồi từ HC-05
    memset(hc05_response, 0, sizeof(hc05_response));
    HAL_UART_Receive(&huart1, (uint8_t *)hc05_response, sizeof(hc05_response), 100);
    sprintf(uart_buffer,"\r\nHC-05 Response: %s", hc05_response); // In ra UART Debug
    Debug_UART(uart_buffer);
}

// Hàm cấu hình HC-05
void HC05_Config() {
	Debug_UART("\r\n[INFO] Configuring HC-05...");

    // Kiểm tra module có phản hồi không
    HC05_SendCommand("AT");

    // Đặt baudrate về 9600
    HC05_SendCommand("AT+UART=9600,0,0");

    // Đặt HC-05 vào chế độ Slave
    HC05_SendCommand("AT+ROLE=0");

//    // Đặt tên thiết bị là "STM32_BT"
//    HC05_SendCommand("AT+NAME=STM32_BT");
//
//    // Đặt mật khẩu kết nối Bluetooth
//    HC05_SendCommand("AT+PSWD=1234");

    Debug_UART("\r\n[INFO] HC-05 Configuration Complete!");
}
