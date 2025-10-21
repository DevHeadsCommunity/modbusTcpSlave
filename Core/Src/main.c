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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "modbus/config.h"
#include "modbus/modbus.h"
#include "modbus/modbusMaster.h"
#include "modbus/db.h"

#include <stdarg.h>


#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "wizchip_conf.h"
#include "socket.h"


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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

wiz_NetInfo net_info = {
		.mac  = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED},
		.ip   = {192, 168, 2, 99},
		.sn   = {255, 255, 255, 0},
		.gw   = {192, 168, 2, 1},
		.dns  = {8, 8, 8, 8},
		.dhcp = NETINFO_STATIC // Use static IP
		//    .dhcp = NETINFO_DHCP
};

#define MODBUS_TCP_PORT 502

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Buffer for received data
#define RX_BUFFER_SIZE 256 // Buffer size for receiving data
typedef struct uartStream
{
	uint8_t rxBuffer[RX_BUFFER_SIZE];
	uint8_t txBuffer[RX_BUFFER_SIZE];
	uint8_t rxByte;
	uint8_t rxFillIndex;
	uint8_t txFillIndex;
	uint8_t rxReadIndex;
	uint8_t txReadIndex;
	uint32_t lastByteTimestamp;
} uartStream;

uartStream uart2Stream;


void myPrintf(/*uint8_t debugLevel,*/ const char *format, ...)
{
	//	if (debugLevel <= DEBUG_LEVEL)
	{
		char buffer[128];
		va_list args;
		va_start(args, format);
		vsnprintf(buffer, sizeof(buffer), format, args);
		va_end(args);

		// Ensure the buffer does not overflow
		size_t length = strlen(buffer);
		size_t index = 0;
		while (length)
		{
			uart2Stream.txBuffer[uart2Stream.txFillIndex++] = buffer[index++];
			if (uart2Stream.txFillIndex >= RX_BUFFER_SIZE)
			{
				uart2Stream.txFillIndex = 0;  // Reset index on overflow
			}
			length--;
		}
	}

	while (uart2Stream.txFillIndex != uart2Stream.txReadIndex)
	{
		// Calculate the number of bytes to send

		// Send all available data from txBuffer
		HAL_UART_Transmit(&huart2, &uart2Stream.txBuffer[uart2Stream.txReadIndex], 1, HAL_MAX_DELAY);
		uart2Stream.txReadIndex++;

		// Prevent buffer overflow
		if (uart2Stream.txReadIndex >= RX_BUFFER_SIZE)
		{
			uart2Stream.txReadIndex = 0;  // Reset index on overflow
		}
	}
}


void wizchipSelect(void) {
	//	myPrintf("wizchipSelect\n");
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void wizchipUnselect(void) {
	//	myPrintf("wizchipUnselect\n");
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void wizchipReadBurst(uint8_t* buff, uint16_t len) {
	//	myPrintf("wizchipReadBurst\n");
	HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
}

void wizchipWriteBurst(uint8_t* buff, uint16_t len) {
	//	myPrintf("wizchipWriteBurst\n");
	HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
}

// single byte read
uint8_t wizchipReadByte(void) {
	//	myPrintf("wizchipReadByte\n");
	uint8_t byte;
	wizchipReadBurst(&byte, sizeof(byte));
	return byte;
}

// single byte write
void wizchipWriteByte(uint8_t byte) {
	//	myPrintf("wizchipWriteByte\n");
	wizchipWriteBurst(&byte, sizeof(byte));
}

//volatile bool ip_assigned = false;
//
//void Callback_IPAssigned(void) {
//	//	myPrintf("ipAddress assigned\n");
//	ip_assigned = true;
//}
//
//void Callback_IPConflict(void) {
//	ip_assigned = false;
//}

void wizChipInit()
{
	// registering function pointers
	reg_wizchip_cs_cbfunc(wizchipSelect, wizchipUnselect);
	reg_wizchip_spi_cbfunc(wizchipReadByte, wizchipWriteByte);
	reg_wizchip_spiburst_cbfunc(wizchipReadBurst, wizchipWriteBurst);

	myPrintf("wiznet functions registered\n");

	/*The W5500 has its own internal 32KB RAM for network data buffering
	 *(16 KB for RX, 16 KB for TX, or You can split each 16 KB block 
	 * among the 8 sockets as needed).
	 */
	uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
	wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
}


uint8_t checkLinkStatus(void)
{
	uint8_t phycfgr = getPHYCFGR();
	if (phycfgr & PHYCFGR_LNK_ON) {
		return 1; // Link is up
	} else {
		return 0; // Link is down
	}
}


/*
 * if master write something on registers, this callback gets triggered
 */
void slaveCallback(uint16_t regAddress, uint16_t numOfRegisters)
{
	myPrintf("slave data modified, regAddress=%d  no of registers=%d\n", regAddress, numOfRegisters);

	if(regAddress == 00001) //coil
	{
		bool tempVal;
		getCoilState(&modbusSlave, 1, &tempVal);
		myPrintf("coil-1 status changed to %d\n", tempVal);

		if (tempVal == 0)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Turn off LED
		}
		else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Turn on LED
		}
	}
}

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  wizChipInit();

  while (checkLinkStatus() == 0)
  {
	  myPrintf("Waiting for link to be established...\n");
	  HAL_Delay(1000); // Wait for 1 second before checking again
  }

  myPrintf("Link established, initializing W5500...\n");

  wizchip_setnetinfo(&net_info);   // configure net information

  const char TestString[] = "Hello World!";
  initModbusSlaveData(&modbusSlave, slaveID, numOfHoldingRegs, numOfInputRegs, numOfCoils, numOfDisInput);
  writeDataToHoldingRegisters(&modbusSlave, 40001, TestString, strlen(TestString));//writing TestString to holding register

	/**
	 * Creates a new socket for Modbus TCP communication.
	 * @param 0 : The socket number or channel. Typically, this is an integer value (0-7) 
	 * 	representing the hardware socket/channel to use.
	 * @param Sn_MR_TCP : The protocol mode for the socket. This constant specifies the socket will operate in TCP mode.
	 * @param 502 :  The port number to bind the socket to. For Modbus TCP, the standard port is 502.
	 * @param SF_TCP_NODELAY :  Socket flag options. This controls socket behavior.
	 * @return :    Returns the socket descriptor (integer >= 0) on success, or a negative value on error.
	 */

	int modbus_sock = socket(0, Sn_MR_TCP, MODBUS_TCP_PORT, SF_TCP_NODELAY);

	if (modbus_sock != 0)
	{
		myPrintf("Failed to open Modbus TCP socket\r\n");
	}
	else
	{
		int8_t ret = listen(modbus_sock);
		if (ret != SOCK_OK)
		{
			myPrintf("Failed to listen on Modbus TCP socket: %d\r\n", ret);
			close(modbus_sock);
		}
		else
		{
			myPrintf("Modbus TCP server listening on port 502\r\n");
		}
	}

	uint8_t connectionStatus = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		if (checkLinkStatus() == 0)
		{
			if (connectionStatus != 0)
			{
				myPrintf("Ethernet link lost! Closing Modbus TCP connection.\r\n");
				connectionStatus = 0;
				disconnect(0);
			}
			while (checkLinkStatus() == 0)
			{
				myPrintf("Waiting for link to be established...\n");
				HAL_Delay(1000);
			}
			myPrintf("Ethernet link re-established.\n");
			// Re-listen after link is up
			int modbus_sock = socket(0, Sn_MR_TCP, 502, SF_TCP_NODELAY);
			if (modbus_sock == 0)
			{
				int8_t ret = listen(modbus_sock);
				if (ret == SOCK_OK)
				{
					myPrintf("Modbus TCP server re-listening on port 502\r\n");
				}
			}
			continue; // Skip rest of loop until link is up
		}


		// Check Modbus TCP socket connection status
		uint8_t sock_status = getSn_SR(0); //non blocking call
		if (sock_status == SOCK_ESTABLISHED)
		{
			if(connectionStatus == 0)
			{
				connectionStatus = 1; // Set connection status to connected
				myPrintf("Modbus TCP client connected\r\n");
			}

			if(connectionStatus == 1)
			{
				uint8_t modbus_rx_buf[260];
				int32_t recv_len = recv(0, modbus_rx_buf, sizeof(modbus_rx_buf)); //blocking call
				if (recv_len > 0)
				{
					myPrintf("Received %ld bytes from Modbus TCP client\r\n", recv_len);
					myPrintf("data received=");

					for(uint8_t index = 0; index < recv_len; index++)
					{
						myPrintf("%d,", modbus_rx_buf[index]);
					}
					myPrintf("\n");

					// Handle MBAP header (Modbus TCP)
					if (recv_len < 7)
					{
						myPrintf("Received packet too short for MBAP header\r\n");
						continue;
					}
					uint16_t transaction_id = (modbus_rx_buf[0] << 8) | modbus_rx_buf[1];//transaction id increments at each request send
					uint16_t protocol_id = (modbus_rx_buf[2] << 8) | modbus_rx_buf[3];//Which is always 0, in case of modbus tcp
					uint16_t length = (modbus_rx_buf[4] << 8) | modbus_rx_buf[5]; // length in bytes, includes unit id and onward
					uint8_t unit_id = modbus_rx_buf[6]; //slave ID

					myPrintf("MBAP: Transition ID=%u, Protocol ID=%u, LEN=%u, UID/Slave Id=%u\r\n",
							transaction_id, protocol_id, length, unit_id);

					// Optionally, check protocol_id == 0 and length is valid
					if (protocol_id != 0)
					{
						myPrintf("Invalid protocol ID in MBAP header\r\n");
						continue;
					}
					if (length + 6 > recv_len)
					{
						myPrintf("MBAP length field mismatch\r\n");
						continue;
					}


					uint8_t modbus_tx_buf[256] ={0};
					uint16_t respLen=0;
					handleModbusRequest(modbus_rx_buf+6, length, modbus_tx_buf, &respLen, &modbusSlave);


					// Add MBAP header before transmitting response
					uint8_t mbap_header[7];
					mbap_header[0] = (transaction_id >> 8) & 0xFF;
					mbap_header[1] = transaction_id & 0xFF;
					mbap_header[2] = (protocol_id >> 8) & 0xFF;
					mbap_header[3] = protocol_id & 0xFF;
					mbap_header[4] = ((respLen) >> 8) & 0xFF; // Length MSB
					mbap_header[5] = (respLen) & 0xFF; //length LSB
					mbap_header[6] = unit_id;

					// Shift response buffer to make space for MBAP header
					memmove(modbus_tx_buf + 6, modbus_tx_buf+0, respLen);
					memcpy(modbus_tx_buf, mbap_header, 7);
					respLen += 6;


					myPrintf("Sending response with MBAP header, total length: %d\r\n", respLen);

					// Echo back the received data (for testing)
					int32_t sent_len = send(0, modbus_tx_buf, respLen);
					myPrintf("Sent bytes: %d\r\n", sent_len);

					if (sent_len != respLen) {
						myPrintf("Failed to send all data back to client\r\n");
					}

				}
			}
			// You can add code here to handle Modbus TCP requests
		}
		else if (sock_status == SOCK_CLOSE_WAIT)
		{
			connectionStatus = 0;
			myPrintf("Modbus TCP client disconnected\r\n");
			disconnect(0);

			while (checkLinkStatus() == 0)
			{
				myPrintf("Waiting for link to be established...\n");
				HAL_Delay(1000); // Wait for 1 second before checking again
			}

		}
		else if (sock_status == SOCK_CLOSED)
		{
			// Re-listen if socket closed
			connectionStatus = 0;
			int modbus_sock = socket(0, Sn_MR_TCP, 502, SF_TCP_NODELAY);
			if (modbus_sock == 0)
			{
				int8_t ret = listen(modbus_sock);
				if (ret == SOCK_OK)
				{
					myPrintf("Modbus TCP server re-listening on port 502\r\n");
				}
			}
		}

//		myPrintf(".");
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
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_COIL_1_GPIO_Port, LED_COIL_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_COIL_1_Pin */
  GPIO_InitStruct.Pin = LED_COIL_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_COIL_1_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
