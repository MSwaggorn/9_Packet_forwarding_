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

// Packetverwaltung als Application auf MMCP Protokoll Version 5
// Abgabe ULP 3
// Moritz Prenzlow, 1152710
// 12.12.2023
// Team 03
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//TODO: what if busy in state and receives ap nr other than 50 (poll)??? Dürfen prozesse kontrolllflüsse zurücksetzen??
typedef uint8_t crc;
typedef enum {FALSE=0, TRUE=1} BOOL; //make sure the FALSE and TRUE get the value 0 and 1

/* LED stuff */
typedef union
{
  struct
  {
    uint8_t b;
    uint8_t r;
    uint8_t g;
  } color;
  uint32_t data;
} PixelRGB_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* SETUP */
// set the neighbor_id, 0 for no neighbor connected to the send and receive pins.
# define N1_id 0
# define N2_id 0
# define N3_id 0
# define N4_id 0


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

/* LED stuff */
#define NEOPIXEL_ZERO 34 // appr. 1/3 of 105 
#define NEOPIXEL_ONE 67 // appr. 2/3 of 105
#define NUM_PIXELS 8
#define DMA_BUFF_SIZE (NUM_PIXELS * 24) + 50 // time of 50 pules needed to comfirm the setting for NeoPixel

//* Packet forwarding defines *//
#define NUM_NEIGHBOURS 4
#define LAGER_SIZE 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch3_up;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* GLOBALS */
uint8_t myAddress = 12; // Board address

bool rx_complete = 0; // TRUE, when serial receive is complete and HAL_UART_RxCpltCallback is called
bool tx_complete = 0; // TRUE, when serial transmit is complete and HAL_UART_TxCpltCallback is called

uint8_t rx_buf[L1_PDU_size] = {0}; // receive buffer
uint8_t L1_PDU[L1_PDU_size] = {0};

uint8_t cnt = 0; // button press counter
unsigned long millis = 0; // elapsed milliseconds for button debouncing
unsigned long lastPress = 0; // time of last rising edge (button press)

/* LED stuff */
uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0};
uint32_t *pBuff;
uint8_t LEDColors[17][3] = { {0, 0, 0}, {255, 255, 255},  {255, 0, 0},
		 {0, 255, 0},  {0, 0, 255},  {0, 255, 255},
		 {255, 0, 255},  {255, 255, 0},  {191, 128, 64},
		 {191, 255, 0},  {128, 128, 0},  {255, 128, 0},
		 {255, 191, 191},  {191, 0, 64},  {0, 128, 128},
		 {128, 0, 128},  {224, 176, 255} };
PixelRGB_t pixels[NUM_PIXELS] = {0}; // TODO: SA/RT
BOOL timer_irq = FALSE; // gets set HIGH every 750ms

//BOOL count = FALSE;

// Utility for ISR (not specified in SA/RT)
uint16_t neighbourSendPins[NUM_NEIGHBOURS] = {S_N1_Pin, S_N2_Pin, S_N3_Pin, S_N4_Pin};
const uint8_t neighbourIDs[NUM_NEIGHBOURS] = {N1_id, N2_id, N3_id, N4_id}; // 0, if no neighbour at Pin // Input Pins are: R_N1_Pin, R_N2_Pin, R_N3_Pin, R_N4_Pin

//* Packet forwarding begin *//

// Speicher
uint8_t state;
uint8_t errorId;
uint8_t partnerId;
uint8_t packageId;
uint8_t Lager[6] = {0};
uint8_t tempLager[6] = {0};
uint8_t ApNr;

// Kontrollfluesse
BOOL receive = FALSE;
BOOL passOn = FALSE;
BOOL create = FALSE;
BOOL deliver = FALSE;
BOOL poll = FALSE;
BOOL await = FALSE;
BOOL finishedSend = FALSE;
BOOL finishedStore = FALSE;
BOOL failure = FALSE;
BOOL receivedSDU = FALSE;
BOOL GPIO_neighbour_in = FALSE;

// Zustandstyp + Zustandsvariable
typedef enum {Z_idle, Z_processing, Z_failure, Z_deliver, Z_passOn, Z_sent, Z_awaiting, Z_received} zustand_t;
zustand_t zustand;

// Aktionentyp + Aktionsvariable
typedef enum {A_idle, A_setup, A_deliver, A_passOn, A_failure, A_pulse, A_updateLager, A_await, A_create, A_handleStore, A_storeAwait, A_storeCreate, A_checkFailure} aktion_t;
aktion_t aktion;
//* Packet forwarding end *//

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void AL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin);

void L1_receive(uint8_t L1_PDU[]);
void L2_receive(uint8_t L2_PDU[]);
void L3_receive(uint8_t L3_PDU[]);
void L7_receive(uint8_t L7_PDU[]);

void L7_send(uint8_t ApNr, uint8_t L7_SDU[]);
void L3_send(uint8_t L3_SDU[]);
void L2_send(uint8_t L2_SDU[]);
void L1_send(uint8_t L1_SDU[]);

void ApNr_42(uint8_t L7_SDU[], uint8_t L7_SDU_send[]);
void ApNr_43(uint8_t L7_SDU[], uint8_t L7_SDU_send[]);
void ApNr_50(uint8_t L7_SDU[], uint8_t L7_SDU_send[]);
void ApNr_100(uint8_t L7_SDU[], uint8_t L7_SDU_send[]);
void ApNr_101(uint8_t L7_SDU_send[]);
void ApNr_102(uint8_t L7_SDU_send[]);
void ApNr_103(uint8_t L7_SDU_send[]);

crc crcSlow(uint8_t const message[], int nBytes);

//* Packet forwarding begin *//
void stateProcessing(void);
void stateAwait(void);
void stateReceived(void);
void stateSent(void);
void stateFailure(void);
void handleStore(void);
void handleSend(void);
void updateLager(void);
void animateSend(void);
void animateReceive(void);
void animateCreate(void);
void animateDeliver(void);
void pulse(void);
void resetData(void);
//* Packet forwarding end *//

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* LED stuff */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
}

void writeLEDs(PixelRGB_t* pixel){
	uint8_t i,j; //use uint8_t to safe RAM

	pBuff = dmaBuffer;
	  for (i = 0; i < NUM_PIXELS; i++)
	  {
		 for (j = 23; j >= 0; j--)
		 {
		   if ((pixel[i].data >> j) & 0x01)
		   {
			 *pBuff = NEOPIXEL_ONE;
		   }
		   else
		   {
			 *pBuff = NEOPIXEL_ZERO;
		   }
		   pBuff++;
	   }
	  } 
	  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, dmaBuffer, DMA_BUFF_SIZE);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		timer_irq = TRUE;
	}
}
//* Packet forwarding begin *//
// DFD processes //
void stateProcessing(void){
	state = 0;
}
void stateAwait(void){
	state = 1;
}
void stateReceived(void){
	state = 2;
}
void stateSent(void){
	state = 3;
}
void stateFailure(void){
	state = 4;
}

void handleStore(void){
	uint8_t i;

	// copy Lager to tempLager
	for(i = 0; i < LAGER_SIZE; i++){
		tempLager[i] = Lager[i];
	}


	// put package in first free spot (0) in tempLager
	for(i = 0; i < LAGER_SIZE; i++){
		if(tempLager[i] == 0){
			tempLager[i] = packageId;
			break;
		}
	}

	finishedStore = TRUE;
}
void handleSend(void){
	uint_8 i; //use uint8_t to safe RAM

	// copy Lager to tempLager
	for(i = 0; i < LAGER_SIZE; i++){
		tempLager[i] = Lager[i];
	}


	// delete package out of tempLager
	for(i = 0; i < LAGER_SIZE; i++){
		if(tempLager[i] == packageId){
			tempLager[i] = 0;
			break;
		}
	}

	finishedSend = TRUE;
}

void updateLager(void){
	int i;

	// copy tempLager to Lager
	for(i = 0; i < LAGER_SIZE; i++){
		Lager[i] = tempLager[i];
	}
}

void animateSend(void){
	BOOL on = FALSE;
	BOOL wait = FALSE;
	uint8_t i = 0;

	// find index of Lager, were packageId was stored //TODO: SA/RT Lager noch rein!!
	for(i = 0; i < LAGER_SIZE; i++){
		if(Lager[i] == packageId){
			break;
		}
	}

	// turn off corresponding LED
	pixels[i+1].color.g = 0; // TODO: SA/RT
	pixels[i+1].color.r = 0;
	pixels[i+1].color.b = 0;
	writeLEDs(pixels);


	// blink last (outgoing) LED once
	timer_irq = FALSE;
	while(1){
		if(timer_irq && !on && !wait){
			timer_irq = FALSE;
			on = TRUE;
			pixels[7].color.g = (uint8_t)LEDColors[packageId][1]*0.1; // TODO: SA/RT
			pixels[7].color.r = (uint8_t)LEDColors[packageId][0]*0.1;
			pixels[7].color.b = (uint8_t)LEDColors[packageId][2]*0.1;
			writeLEDs(pixels);
		}
		if(timer_irq && on && !wait){
			timer_irq = FALSE;
			wait = TRUE;
			pixels[7].color.g = 0; // TODO: SA/RT
			pixels[7].color.r = 0;
			pixels[7].color.b = 0;
			writeLEDs(pixels);
		}
		if(timer_irq && wait){
			break;
		}
	}

	// display current Lager
	for(i = 0; i < LAGER_SIZE; i++){
		pixels[i+1].color.g = (uint8_t)LEDColors[tempLager[i]][1]*0.1;
		pixels[i+1].color.r = (uint8_t)LEDColors[tempLager[i]][0]*0.1;
		pixels[i+1].color.b = (uint8_t)LEDColors[tempLager[i]][2]*0.1;

	}
	writeLEDs(pixels);
}

void animateReceive(void){
	BOOL on = FALSE;
	BOOL wait = FALSE;

	// blink first (incoming) LED once
	timer_irq = FALSE;
	while(1){
		if(timer_irq && !on && !wait){
			timer_irq = FALSE;
			on = TRUE;
			pixels[0].color.g = (uint8_t)LEDColors[packageId][1]*0.1; // TODO: SA/RT
			pixels[0].color.r = (uint8_t)LEDColors[packageId][0]*0.1;
			pixels[0].color.b = (uint8_t)LEDColors[packageId][2]*0.1;
			writeLEDs(pixels);
		}
		if(timer_irq && on && !wait){
			timer_irq = FALSE;
			wait = TRUE;
			pixels[0].color.g = 0; // TODO: SA/RT
			pixels[0].color.r = 0;
			pixels[0].color.b = 0;
			writeLEDs(pixels);
		}
		if(timer_irq && wait){
			break;
		}
	}

	// display current Lager
	for(int i = 0; i < LAGER_SIZE; i++){
		pixels[i+1].color.g = (uint8_t)LEDColors[tempLager[i]][1]*0.1;
		pixels[i+1].color.r = (uint8_t)LEDColors[tempLager[i]][0]*0.1;
		pixels[i+1].color.b = (uint8_t)LEDColors[tempLager[i]][2]*0.1;

	}
	writeLEDs(pixels);
}
void animateCreate(void){

	BOOL on = FALSE;
	BOOL wait = FALSE;
	uint8_t i = 0;

	// find index of tempLager, were packageId is stored
	for(i = 0; i < LAGER_SIZE; i++){
		if(tempLager[i] == packageId){
			break;
		}
	}

	// blink corresponding LED once
	timer_irq = FALSE;
	while(1){
		if(timer_irq && !on && !wait){
			timer_irq = FALSE;
			on = TRUE;
			pixels[i+1].color.g = (uint8_t)LEDColors[packageId][1]*0.1; // TODO: SA/RT
			pixels[i+1].color.r = (uint8_t)LEDColors[packageId][0]*0.1;
			pixels[i+1].color.b = (uint8_t)LEDColors[packageId][2]*0.1;
			writeLEDs(pixels);
		}
		if(timer_irq && on && !wait){
			timer_irq = FALSE;
			wait = TRUE;
			pixels[i+1].color.g = 0; // TODO: SA/RT
			pixels[i+1].color.r = 0;
			pixels[i+1].color.b = 0;
			writeLEDs(pixels);
		}
		if(timer_irq && wait){
			break;
		}

	}

	// display current Lager
	for(i = 0; i < LAGER_SIZE; i++){
		pixels[i+1].color.g = (uint8_t)LEDColors[tempLager[i]][1]*0.1; //TODO: greift auch auf temp
		pixels[i+1].color.r = (uint8_t)LEDColors[tempLager[i]][0]*0.1;
		pixels[i+1].color.b = (uint8_t)LEDColors[tempLager[i]][2]*0.1;

	}
	writeLEDs(pixels);
}
void animateDeliver(void){
	BOOL on = FALSE;
	BOOL wait = FALSE;
	uint8_t i = 0;

	// find index of Lager, were packageId was stored //TODO: SA/RT Lager noch rein!!
	for(i = 0; i < LAGER_SIZE; i++){
		if(Lager[i] == packageId){
			break;
		}
	}

	// blink corresponding LED once
	timer_irq = FALSE;
	while(1){
		if(timer_irq && !on && !wait){
			timer_irq = FALSE;
			on = TRUE;
			pixels[i+1].color.g = (uint8_t)LEDColors[packageId][1]*0.1; // TODO: SA/RT
			pixels[i+1].color.r = (uint8_t)LEDColors[packageId][0]*0.1;
			pixels[i+1].color.b = (uint8_t)LEDColors[packageId][2]*0.1;
			writeLEDs(pixels);
		}
		if(timer_irq && on && !wait){
			timer_irq = FALSE;
			wait = TRUE;
			pixels[i+1].color.g = 0; // TODO: SA/RT
			pixels[i+1].color.r = 0;
			pixels[i+1].color.b = 0;
			writeLEDs(pixels);
		}
		if(timer_irq && wait){
			break;
		}

	}

	// display current Lager
	for(int i = 0; i < LAGER_SIZE; i++){
		pixels[i+1].color.g = (uint8_t)LEDColors[tempLager[i]][1]*0.1;
		pixels[i+1].color.r = (uint8_t)LEDColors[tempLager[i]][0]*0.1;
		pixels[i+1].color.b = (uint8_t)LEDColors[tempLager[i]][2]*0.1;

	}
	writeLEDs(pixels);
}
void pulse(void){
	uint8_t partnerNumber = 0;
	uint8_t i = 0;

	// find out number of partner
	for(i = 0; i < NUM_NEIGHBOURS; i++){
		if(neighbourIDs[i] == partnerId){
			partnerNumber = i;
			break;
		}
	}

	// toggle corresponding pin for 1ms // TODO: actually implement pulse with a state between passOn and sent (passOnPulse or something) and a timer
	HAL_GPIO_WritePin (GPIOC, neighbourSendPins[partnerNumber], GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin (GPIOC, neighbourSendPins[partnerNumber], GPIO_PIN_RESET);
}
void checkFailure(void){
	BOOL lagerFull = TRUE;
	BOOL packetInLager = FALSE;
	BOOL packetNoExist = TRUE;
	BOOL unknownNeighbour = TRUE;
	BOOL unknownPacket = FALSE;

	int i;

	// check if neighbour is known
	if(partnerId == 0){ //TODO: fixed with idle in processing, but that should not be there
		unknownNeighbour = FALSE; // neighbourId is valid
	}
	for(i = 0; i < NUM_NEIGHBOURS; i++){
		if(partnerId == neighbourIDs[i]){
			unknownNeighbour = FALSE; // neighbourId is valid
			break;
		}
	}

	if(ApNr == 42){ // create or await is set
		// check if Lager is already completely filled
		for(i = 0; i < LAGER_SIZE; i++){
			if(Lager[i] == 0){
				lagerFull = FALSE; // Lager is not completely filled
				break;
			}
		}

		packetNoExist = FALSE;
		// check if packageId is already stored in Lager
		for(i = 0; i < LAGER_SIZE; i++){
			if(Lager[i] == packageId){
				packetInLager = TRUE; // packageId already exists in Lager
				break;
			}
		}
	}

	if(ApNr == 43){ // deliver or passOn is set
		lagerFull = FALSE;

		// Check if packageId exists in Lager
		for(i = 0; i < LAGER_SIZE; i++){
			if(Lager[i] == packageId){
				packetNoExist = FALSE; // packageId does exist in Lager
			}
		}
	}

	// check if package has a valid number
	if((packageId < 0) || (packageId > 16)){
		unknownPacket = TRUE;
	}


	// set errorId according to failure
	if(packetInLager){
		failure = TRUE;
		errorId = 1;
	}
	else if(lagerFull){
		failure = TRUE;
		errorId = 2;
	}
	else if(packetNoExist){
		failure = TRUE;
		errorId = 3;
	}
	else if(unknownNeighbour){
		failure = TRUE;
		errorId = 4;
	}
	else if(unknownPacket){
		failure = TRUE;
		errorId = 5; //TODO: Minispezifikation
	}
}
void resetData(){
	packageId = 0;
	partnerId = 0;
	errorId = 0;
	receive = FALSE;
	passOn = FALSE;
	create = FALSE;
	deliver = FALSE;
	poll = FALSE;
	await = FALSE;
	ApNr = 0;
	failure = FALSE;
	finishedStore = FALSE;
	finishedSend = FALSE;
	receivedSDU = FALSE;
}

// STD
void std(void){
	switch(zustand){
	case Z_idle:
		if (receivedSDU){
			aktion = A_checkFailure;
			zustand = Z_processing;
			receivedSDU = FALSE; //*
		}
		else if(poll || (!receivedSDU)){ //TODO: SA/RT
			aktion = A_idle;
			poll = FALSE; //*
		}
		/*
		else if((!poll) && (!receivedSDU)){
			aktion = A_idle; // so it is not stuck in A_reset
		}
		else if(!receivedSDU){
			aktion = A_idle; // so it is not stuck in A_reset
		}*/
		break;


	case Z_processing:
		if (create && (!failure)){ // no poll (transient state), because processing + packageId should only be reported in modes passOn and deliver
			aktion = A_create;
			zustand = Z_awaiting;
			poll = FALSE; //*
		}
		else if (await && (!failure)){ // no poll (transient state), because processing + packageId should only be reported in modes passOn and deliver
			aktion = A_await;
			zustand = Z_awaiting;
			poll = FALSE; //*
		}
		else if (deliver && (!failure)){
			aktion = A_deliver;
			zustand = Z_deliver;
		}
		else if (passOn && (!failure)){
			aktion = A_passOn;
			zustand = Z_passOn;
		}
		else if (failure){
			aktion = A_failure;
			zustand = Z_failure;
			poll = FALSE; //*
		}
		/*
		else{
			aktion = A_idle; // TODO: why do we need this so failure for unknown neighbour works??
		}*/
		break;

	case Z_failure:
		if(poll){
			aktion = A_setup;
			zustand = Z_idle;
			poll = FALSE; //*
		}
		break;

	case Z_deliver:
		if (poll && finishedSend){
			aktion = A_updateLager;
			zustand = Z_sent;
			poll = FALSE; //*
		}
		break;

	case Z_passOn:
		if (poll && finishedSend){
			aktion = A_pulse;
			zustand = Z_sent;
			poll = FALSE; //*
		}
		break;

	case Z_sent:
		if (poll){
			aktion = A_setup;
			zustand = Z_idle;
			poll = FALSE; //*
		}
		break;

	case Z_awaiting: // TODO: no GPIO needed for going from awating to send?? maybe something to do with pull up, but it used to work?
		if (GPIO_neighbour_in && await){
			aktion = A_handleStore;
			GPIO_neighbour_in = FALSE; //*
		}
		else if (poll && finishedStore && await){
			aktion = A_storeAwait;
			zustand = Z_received;
			poll = FALSE; //*
		}
		else if (poll && finishedStore && create){ //else if (poll && finishedStore && create && count	){
			aktion = A_storeCreate;
			zustand = Z_received;
			poll = FALSE; //*
			//count = FALSE;
		}
		break;

	case Z_received:
		if (poll){
			aktion = A_setup;
			zustand = Z_idle;
			poll = FALSE; //*
		}
		else if(!poll){
			aktion = A_idle; //*
		}
		break;
	}
}

//* Kontrollflüsse werden im Interrupt gesetzt und müssen deshalb hier zurückgesetzt werden

// PAT
void pat(void){
	switch(aktion){
	case A_setup:
		resetData(); // TODO: SA/RT
		stateProcessing();
		break;
	case A_deliver: // TODO: A_deliver and A_passOn are identical
		stateProcessing();
		handleSend();
		break;
	case A_passOn:
		stateProcessing();
		handleSend();
		break;
	case A_failure:
		stateFailure();
		break;
	case A_pulse:
		animateSend();
		pulse();
		updateLager();
		stateSent();
		break;
	case A_updateLager:
		animateDeliver();
		updateLager();
		stateSent();
		break;
	case A_await:
		stateAwait();
		break;
	case A_create:
		stateAwait();
		handleStore();
		break;
	case A_handleStore:
		handleStore();
		break;
	case A_storeAwait:
		animateReceive();
		updateLager();
		stateReceived();
		break;
	case A_storeCreate:
		animateCreate();
		updateLager();
		stateReceived();
		break;
	case A_checkFailure:
		checkFailure();
		stateProcessing();
		break;

	//TODO: SA/RT
	case A_idle:
		break;
	}
}
//* Packet forwarding end *//
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/*
	// Debug variables
	char buf[200];
	long lastMillis = 0;
	long millis = 0;
	*/
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, rx_buf, L1_PDU_size);
    HAL_TIM_Base_Start_IT(&htim3);
	// Zustandsuebergangsdiagramm reset
	zustand = Z_idle;
	aktion = A_idle;
	pat();

	// Reset LEDs
	for(int i = 0; i < NUM_PIXELS; i++){
		pixels[i].color.g = 0;
		pixels[i].color.r = 0;
		pixels[i].color.b = 0;
	}
	writeLEDs(pixels);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* DEBUG Output on USART2
		millis = HAL_GetTick(); // get current elapsed time in milliseconds

		// rising edge at button pin was detected and DEBOUNCE_INTERVAL has elapsed since last rising edge
		if ((millis - lastMillis) > 500){
			sprintf (buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", state, errorId, partnerId, packageId, Lager[0], Lager[1], Lager[2], Lager[3], Lager[4], Lager[5],
							tempLager[0], tempLager[1], tempLager[2], tempLager[3], tempLager[4], tempLager[5], flag_send, flag_await, receive,
							passOn, create, deliver, poll, await, finishedSend, finishedStore, failure, receivedSDU, GPIO_neighbour_in);
			HAL_UART_Transmit_IT(&huart2, buf, strlen(buf));
			lastMillis = millis;
		}
		*/

	  std();
	  pat();
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4200;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, S_N1_Pin|S_N2_Pin|S_N3_Pin|S_N4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_N1_Pin R_N2_Pin R_N3_Pin R_N4_Pin */
  GPIO_InitStruct.Pin = R_N1_Pin|R_N2_Pin|R_N3_Pin|R_N4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S_N1_Pin S_N2_Pin S_N3_Pin S_N4_Pin */
  GPIO_InitStruct.Pin = S_N1_Pin|S_N2_Pin|S_N3_Pin|S_N4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Rx Transfer completed callbacks.
// Gets called when HAL_UART_Receive_IT receive is completed
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){ // check if something was received on UART1
		for(int i = 0; i < L1_PDU_size; i++){ // copy received packet from buffer
			L1_PDU[i] = rx_buf[i];
		}
	}
  HAL_UART_Receive_IT(&huart1, rx_buf, L1_PDU_size); // Attach interrupt to receive L1_PDU from USART
  L1_receive(L1_PDU); // Pass L1_PDU to protocol stack
}

// Tx Transfer completed callbacks.
// Gets called when HAL_UART_Transmit_IT transmission is completed
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	tx_complete = 1; // packet received
}

// GPIO interrupt callback
// increments button press counter
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	// rising edge at button pin was detected and DEBOUNCE_INTERVAL has elapsed since last rising edge
	if (GPIO_Pin == B1_Pin){
		millis = HAL_GetTick(); // get current elapsed time in milliseconds
		if((millis - lastPress) > DEBOUNCE_INTERVAL){
			cnt++;
			lastPress = millis;
		}
	}

	// rising edge at neighbour receive pin 1 was detected and packet should be received from neighbour 1
	if((GPIO_Pin == R_N1_Pin) && (neighbourIDs[0] == partnerId)){
		GPIO_neighbour_in = TRUE;
	}

	// rising edge at neighbour receive pin 1 was detected and packet should be received from neighbour 1
	if((GPIO_Pin == R_N2_Pin) && (neighbourIDs[1] == partnerId)){
		GPIO_neighbour_in = TRUE;
	}

	// rising edge at neighbour receive pin 1 was detected and packet should be received from neighbour 1
	if((GPIO_Pin == R_N3_Pin) && (neighbourIDs[2] == partnerId)){
		GPIO_neighbour_in = TRUE;
	}

	// rising edge at neighbour receive pin 1 was detected and packet should be received from neighbour 1
	if((GPIO_Pin == R_N4_Pin) && (neighbourIDs[3] == partnerId)){
		GPIO_neighbour_in = TRUE;
	}

}

/* protocol stack functions begin */
void L1_receive(uint8_t L1_PDU[]){
	uint8_t i;
	// remove first (SOF) and last (EOF) byte from packet to get L1_SDU
	uint8_t L1_SDU[L1_SDU_size]; // no need to set to 0, if you overwrite all the elemets on the next line

	for(i = 0; i < L1_SDU_size; i++){
		L1_SDU[i] = L1_PDU[i+1];
	}
	L2_receive(L1_SDU); // L1_SDU = L2_PDU
}

void L2_receive(uint8_t L2_PDU[]){
	uint8_t L2_SDU[L2_SDU_size];
	//uint8_t checksum = L2_PDU[13]; // last byte is checksum

	for(int i = 0; i < L2_SDU_size; i++){ // remove last byte (checksum) to get L2_SDU
		L2_SDU[i] = L2_PDU[i];
	}

	if(crcSlow(L2_SDU, L2_SDU_size) == L2_PDU[13]){ // checksum is valid -> pass packet to next Layer
		L3_receive(L2_SDU); // L2_SDU = L3_PDU
	} else { // checksum is invalid -> discard packet
		tx_complete = 1;
	}
}

void L3_receive(uint8_t L3_PDU[]){
	uint8_t L3_SDU[L3_SDU_size];

	for(int i = 0; i < L3_SDU_size; i++){ // remove first 4 bytes (To, From, Vers, Hops) tp get L3_SDU
		L3_SDU[i] = L3_PDU[i+4];
	}
	if(L3_PDU[0] != MMCP_MASTER_ADDRESS && L3_PDU[1] == MMCP_MASTER_ADDRESS && L3_PDU[2] == MMCP_VERSION){ // packet is not addressed to master, is from master and version is correct -> packet is valid
		if(L3_PDU[0] == myAddress){ // packet is addressed to this device -> pass packet to next Layer
			L7_receive(L3_SDU); // L3_SDU = L7_PDU)
		}
	}
	if(L3_PDU[0] != myAddress && L3_PDU[2] == MMCP_VERSION){ // packet is addressed to different device -> forward packet
		L3_PDU[3]++; // increment hop-counter
		L2_send(L3_PDU);
	}
	else if(L3_PDU[0] == MMCP_MASTER_ADDRESS && L3_PDU[1] == MMCP_MASTER_ADDRESS && L3_PDU[2] == MMCP_VERSION){ // packet is addressed from master to master (invalid) -> discard packet
		tx_complete = 1;
	}
}

void L7_receive(uint8_t L7_PDU[]){
	uint8_t L7_SDU[L7_SDU_size];
	uint8_t L7_SDU_send[L7_SDU_size]; // information to send back

	for(int i = 0; i < L7_SDU_size; i++){ // remove first byte (ApNr) to get L7_SDU
		L7_SDU[i] = L7_PDU[i+1];
	}

	// ApNr 42
	// await / create package
	// send back received L7_SDU
	if(L7_PDU[0] == 42){
		ApNr_42(L7_SDU, L7_SDU_send);
		L7_send(42, L7_SDU_send);
	}

	// ApNr 43
	// pass on / deliver package
	// send back received L7_SDU
	if(L7_PDU[0] == 43){
		ApNr_43(L7_SDU, L7_SDU_send);
		L7_send(43, L7_SDU_send);
	}

	// ApNr 50
	// poll status
	if(L7_PDU[0] == 50){
		ApNr_50(L7_SDU, L7_SDU_send);
		L7_send(50, L7_SDU_send);
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
//	HAL_UART_Receive_IT(&huart1, rx_buf, L1_PDU_size); // Attach interrupt to receive L1_PDU from USART
}

void L7_send(uint8_t ApNr, uint8_t L7_SDU[]){
	uint8_t L7_PDU[L7_PDU_size];

	// copy ApNr and L7_SDU to get L7_PDU
	L7_PDU[0] = ApNr;
	for(int i = 0; i < L7_SDU_size; i++){
		L7_PDU[i+1] = L7_SDU[i];
	}

	L3_send(L7_PDU); // L7_PDU = l3_SDU
}

void L3_send(uint8_t L3_SDU[]){
	uint8_t L3_PDU[L3_PDU_size];

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
	uint8_t L2_PDU[L2_PDU_size];
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

	HAL_UART_Transmit_IT(&huart1, L1_PDU, L1_PDU_size); // send L1_PDU over USART2 (non-blocking)
}
/* protocol stack functions end */
// ApNr 42
// await / create package
// send back received L7_SDU
void ApNr_42(uint8_t L7_SDU[], uint8_t L7_SDU_send[]){
	packageId = L7_SDU[0];
	partnerId = L7_SDU[1];

	receivedSDU = TRUE;
	ApNr = 42;

	if(partnerId != 0){ // partnerId is not 0 -> await new package
		await = TRUE;
	} else { // partnerId is 0 -> create new package
		create = TRUE;
	}

	for(int i = 0; i < L7_SDU_size; i++){ // copy L7_SDU to L7_SDU_send
		L7_SDU_send[i] = L7_SDU[i];
	}
}

// ApNr 43
// pass on / deliver package
// send back received L7_SDU
void ApNr_43(uint8_t L7_SDU[], uint8_t L7_SDU_send[]){
	packageId = L7_SDU[0];
	partnerId = L7_SDU[1];

	receivedSDU = TRUE;
	ApNr = 43;

	if(partnerId != 0){ // partnerId is not 0 -> passOn package
		passOn = TRUE;
	} else { // partnerId is 0 -> deliver package
		deliver = TRUE;
	}

	for(int i = 0; i < L7_SDU_size; i++){ // copy L7_SDU to L7_SDU_send
		L7_SDU_send[i] = L7_SDU[i];
	}
}

// ApNr 50
// poll status
void ApNr_50(uint8_t L7_SDU[], uint8_t L7_SDU_send[]){
	poll = TRUE;

	L7_SDU_send[0] = state;
	if(state == 4){ // state is failure, send errorId instead of packageId
		L7_SDU_send[1] = errorId;
	} else {
		L7_SDU_send[1] = packageId;
	}
	for(int i = 2; i < L7_SDU_size; i++){ // copy Lager to L7_SDU_send, index 2 to 7 //TODO: adapt to Lager size constant
		L7_SDU_send[i] = Lager[i-2];
	}
}

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
