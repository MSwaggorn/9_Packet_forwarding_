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

// MMCP Protokoll Version 5
// Abgabe ULP 1
// Moritz Prenzlow, 1152710
// 18.10.2023
// Team 03
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef uint8_t crc;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* DEFINES */
# define MMCP_MASTER_ADDRESS 0
# define MMCP_VERSION 5

# define L7_PDU_size 9
# define L7_SDU_size 8
# define L7_PCI_size 1

# define L3_PDU_size 13
# define L3_SDU_size 9
# define L3_PCI_size 4

# define L2_PDU_size 14
# define L2_SDU_size 13
# define L2_PCI_size 1

# define L1_PDU_size 16
# define L1_SDU_size 14
# define L1_PCI_size 2

#define DEBOUNCE_INTERVAL 10 // button debounce time in milliseconds

// CRC defines
#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))
#define POLYNOMIAL 0x9b
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* GLOBALS */
uint8_t myAddress = 0x01; // Board address

bool rx_complete = 0; // TRUE, when serial receive is complete and HAL_UART_RxCpltCallback is called
bool tx_complete = 0; // TRUE, when serial transmit is complete and HAL_UART_TxCpltCallback is called

uint8_t rx_buf[L1_PDU_size] = {0}; // receive buffer
uint8_t L1_PDU[L1_PDU_size] = {0};

uint8_t cnt = 0; // button press counter
unsigned long millis = 0; // elapsed milliseconds for button debouncing
unsigned long lastPress = 0; // time of last rising edge (button press)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void AL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin );

void L1_receive(uint8_t L1_PDU[]);
void L2_receive(uint8_t L2_PDU[]);
void L3_receive(uint8_t L3_PDU[]);
void L7_receive(uint8_t L7_PDU[]);

void L7_send(uint8_t ApNr, uint8_t L7_SDU[]);
void L3_send(uint8_t L3_SDU[]);
void L2_send(uint8_t L2_SDU[]);
void L1_send(uint8_t L1_SDU[]);

void ApNr_100(uint8_t L7_SDU[], uint8_t L7_SDU_send[]);
void ApNr_101(uint8_t L7_SDU_send[]);
void ApNr_102(uint8_t L7_SDU_send[]);
void ApNr_103(uint8_t L7_SDU_send[]);

crc crcSlow(uint8_t const message[], int nBytes);
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
  HAL_UART_Receive_IT(&huart2, rx_buf, L1_PDU_size);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
	  HAL_UART_Receive_IT(&huart2, rx_buf, L1_PDU_size); // Receive L1_PDU from USART
		  while(!tx_complete){ // wait for response packet to ensure "one packet in, one packet out" rule; when a packet is discarded, tx_complete is also set
		  }
		  tx_complete = 0;
	  } */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Rx Transfer completed callbacks.
// Gets called when HAL_UART_Receive_IT receive is completed
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	for(int i = 0; i < L1_PDU_size; i++){ // copy received packet from buffer
		L1_PDU[i] = rx_buf[i];
	}

  L1_receive(L1_PDU); // Pass L1_PDU to protocol stack
  HAL_UART_Receive_IT(&huart2, rx_buf, L1_PDU_size); // Attach interrupt to receive L1_PDU from USART
}

// Tx Transfer completed callbacks.
// Gets called when HAL_UART_Transmit_IT transmission is completed
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	tx_complete = 1; // packet received
}

// GPIO interrupt callback
// increments button press counter
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	millis = HAL_GetTick(); // get current elapsed time in milliseconds

	// rising edge at button pin was detected and DEBOUNCE_INTERVAL has elapsed since last rising edge
	if (GPIO_Pin == B1_Pin && (millis - lastPress) > DEBOUNCE_INTERVAL){
		cnt++;
		lastPress = millis;
	}
}

/* protocol stack functions begin */
void L1_receive(uint8_t L1_PDU[]){
	// remove first (SOF) and last (EOF) byte from packet to get L1_SDU
	uint8_t L1_SDU[L1_SDU_size] = {0};

	for(int i = 0; i < L1_SDU_size; i++){
		L1_SDU[i] = L1_PDU[i+1];
	}
	L2_receive(L1_SDU); // L1_SDU = L2_PDU
}

void L2_receive(uint8_t L2_PDU[]){
	uint8_t L2_SDU[L2_SDU_size] = {0};
	uint8_t checksum = L2_PDU[13]; // last byte is checksum

	for(int i = 0; i < L2_SDU_size; i++){ // remove last byte (checksum) to get L2_SDU
		L2_SDU[i] = L2_PDU[i];
	}

	if(crcSlow(L2_SDU, L2_SDU_size) == checksum){ // checksum is valid -> pass packet to next Layer
		L3_receive(L2_SDU); // L2_SDU = L3_PDU
	} else { // checksum is invalid -> discard packet
		tx_complete = 1;
	}
}

void L3_receive(uint8_t L3_PDU[]){
	uint8_t L3_SDU[L3_SDU_size] = {0};

	for(int i = 0; i < L3_SDU_size; i++){ // remove first 4 bytes (To, From, Vers, Hops) tp get L3_SDU
		L3_SDU[i] = L3_PDU[i+4];
	}
	if(L3_PDU[0] != MMCP_MASTER_ADDRESS && L3_PDU[1] == MMCP_MASTER_ADDRESS && L3_PDU[2] == MMCP_VERSION){ // packet is not addressed to master, is from master and version is correct -> packet is valid
		if(L3_PDU[0] == myAddress){ // packet is addressed to this device -> pass packet to next Layer
			L7_receive(L3_SDU); // L3_SDU = L7_PDU)
		} else { // packet is addressed to different device -> forward packet
			L3_PDU[3]++; // increment hop-counter
			L2_send(L3_PDU);
		}
	} else { // packet is addressed from master to master (invalid) -> discard packet
		tx_complete = 1;
	}
}

void L7_receive(uint8_t L7_PDU[]){
	uint8_t L7_SDU[L7_SDU_size] = {0};
	uint8_t L7_SDU_send[L7_SDU_size] = {0}; // information to send back

	for(int i = 0; i < L7_SDU_size; i++){ // remove first byte (ApNr) to get L7_SDU
		L7_SDU[i] = L7_PDU[i+1];
	}

	// ApNr 100
	// turn on onboard LED if last byte of L7_SDU is not 0
	// send back received L7_SDU
	if(L7_PDU[0] == 100){
		ApNr_100(L7_SDU, L7_SDU_send);
		L7_send(100, L7_SDU_send);
	}

	// ApNr 101
	// ignore information in L7_SDU
	// send back button presses counter in L7_SDU[7], reset counter
	if(L7_PDU[0] == 101){
		ApNr_101(L7_SDU_send);
		L7_send(101, L7_SDU_send);
	}

	// ApNr 102
	// ignore information in L7_SDU
	// send back lower 64 bits of device UID
	if(L7_PDU[0] == 102){
		ApNr_102(L7_SDU_send);
		L7_send(102, L7_SDU_send);
	}

	// ApNr 103
	// ignore information in L7_SDU
	// send back upper bits 64 to 95 of device UID
	if(L7_PDU[0] == 103){
		ApNr_103(L7_SDU_send);
		L7_send(103, L7_SDU_send);
	}

	tx_complete = 1;  // ApNr invalid (unknown) -> discard packet
}

void L7_send(uint8_t ApNr, uint8_t L7_SDU[]){
	uint8_t L7_PDU[L7_PDU_size] = {0};

	// copy ApNr and L7_SDU to get L7_PDU
	L7_PDU[0] = ApNr;
	for(int i = 0; i < L7_SDU_size; i++){
		L7_PDU[i+1] = L7_SDU[i];
	}

	L3_send(L7_PDU); // L7_PDU = l3_SDU
}

void L3_send(uint8_t L3_SDU[]){
	uint8_t L3_PDU[L3_PDU_size] = {0};

	L3_PDU[0] = MMCP_MASTER_ADDRESS; // To: Master
	L3_PDU[1] = myAddress; // From: device
	L3_PDU[2] = MMCP_VERSION; // protocol version
	L3_PDU[3] = 0; // Hops: 0
	for(int i = 0; i < L3_SDU_size; i++){
		L3_PDU[i+L3_PCI_size] = L3_SDU[i];
	}

	L2_send(L3_PDU); // L3_PDU = L2_SDU
}

void L2_send(uint8_t L2_SDU[]){
	uint8_t L2_PDU[L2_PDU_size] = {0};
	uint8_t checksum = 42;

	// copy L2_SDU to first 13 bytes of L2_PDU
	for(int i = 0; i < L2_SDU_size; i++){
		L2_PDU[i] = L2_SDU[i];
	}

	checksum = crcSlow(L2_SDU, L2_SDU_size); // calculate checksum
	L2_PDU[13] = checksum; // last bit is checksum

	L1_send(L2_PDU);
}

void L1_send(uint8_t L1_SDU[]){
	L1_PDU[0] = 0; // SOF: 0
	// copy L1_SDU to bytes 1...15 of L1_PDU
	for(int i = 0; i < L1_SDU_size; i++){
		L1_PDU[i+1] = L1_SDU[i];
	}
	L1_PDU[15] = 0; // EOF: 0

	HAL_UART_Transmit_IT(&huart2, L1_PDU, L1_PDU_size); // send L1_PDU over USART2 (non-blocking)
}
/* protocol stack functions end */

// ApNr 100
// turn on onboard LED, if last byte of L7_SDU is not 0
// send back received L7_SDU
void ApNr_100(uint8_t L7_SDU[], uint8_t L7_SDU_send[]){
	if(L7_SDU[7] != 0){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}

		for(int i = 0; i < L7_SDU_size; i++){ // copy L7_SDU to L7_SDU_send
			L7_SDU_send[i] = L7_SDU[i];
		}
}

// ApNr 101
// ignore information in L7_SDU
// send back button presses counter in L7_SDU[7], reset counter
void ApNr_101(uint8_t L7_SDU_send[]){
	L7_SDU_send[7] = cnt; // store button presses
	cnt = 0; // reset counter
}

// ignore information in L7_SDU
// send back lower 64 bits of device UID
void ApNr_102(uint8_t L7_SDU_send[]){
	uint32_t UIDw0 = HAL_GetUIDw0(); // get bits 0 to 31
	uint32_t UIDw1 = HAL_GetUIDw1(); // get bits 32 to 63

	memcpy(L7_SDU_send, &UIDw0, 4); // copy bits 0 to 31
	memcpy(L7_SDU_send+4, &UIDw1, 4); // copy bits 32 to 63
}

// ignore information in L7_SDU
// send back upper bits 64 to 95 of device UID
void ApNr_103(uint8_t L7_SDU_send[]){
	uint32_t UIDw2 = HAL_GetUIDw2(); // get bits 64 to 95

	memcpy(L7_SDU_send, &UIDw2, 4); // copy bits 64 to 95
}

// crc algorithm
// source: https://barrgroup.com/embedded-systems/how-to/crc-calculation-c-code
crc crcSlow(uint8_t const message[], int nBytes) {
    crc  remainder = 0;

    // perform modulo-2 division, a byte at a time.
    for (int byte = 0; byte < nBytes; ++byte){

        // bring the next byte into the remainder.
        remainder ^= (message[byte] << (WIDTH - 8));

        // perform modulo-2 division, a bit at a time.
        for (uint8_t bit = 8; bit > 0; --bit){
            // try to divide the current data bit.
            if (remainder & TOPBIT){
                remainder = (remainder << 1) ^ POLYNOMIAL;
            } else {
                remainder = (remainder << 1);
            }
        }
    }

    // The final remainder is the CRC result.
    return (remainder);
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
