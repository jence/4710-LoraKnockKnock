/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "utilities_def.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "main.h"
#include "stdio.h"
#include "stm32wlxx_hal_subghz.h"
#include "radio_driver.h"
#include "stm32_lpm_if.h"
#include "timer_if.h"
#include "fatfs.h"
#include "user_diskio_spi.h"
#include "stm32wlxx_hal_tim.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "diskio.h"
#include "gpio.h"
#include "core_cm4.h"

#define BATCH_SIZE 256

static uint8_t  batch_raw[BATCH_SIZE * 2];   // raw PCM
//static uint16_t batch_dac[BATCH_SIZE];       // DAC buffer

uint16_t batch_dac_A[BATCH_SIZE];
uint16_t batch_dac_B[BATCH_SIZE];
uint16_t circular_buffer[BATCH_SIZE * 2];  // Combined buffer for circular DMA
uint8_t current_half = 0;          // 0 = first half, 1 = second half
uint8_t next_buffer_ready = 0;
uint32_t next_samples_count = 0;   // Number of samples in the next buffer
uint8_t file_ended = 0;             // Flag to track if file has ended
uint8_t currently_filling = 0;      // NEW: Flag to prevent race conditions

uint32_t callback_timestamp = 0;
uint32_t last_callback_time = 0;

int newIndex = -1;

// Pushbutton and Seek Variables
#define LONG_PRESS_TIME 2000  // 2 seconds in ms
#define DEBOUNCE_TIME 50      // 50ms debounce

static uint32_t button_fwd_press_time = 0;
static uint32_t button_bwd_press_time = 0;
static uint8_t button_fwd_pressed = 0;
static uint8_t button_bwd_pressed = 0;
static uint8_t button_fwd_handled = 0;
static uint8_t button_bwd_handled = 0;

static int default_audio_index = -1;  // Default audio for transmitter
static uint8_t authentication_passed = 0;  // Authentication flag

// Device ID structure for parsing
typedef struct {
    char company;           // 1 byte: Company name (e.g., 'J' for Jence)
    char testing_code[3];   // 2 bytes: Testing code (e.g., "10")
    char product_type[3];   // 2 bytes: Product type (e.g., "LR" for LoRa)
    char version[3];        // 2 bytes: Version (e.g., "01")
    char year[3];           // 2 bytes: Year (e.g., "25")
    char serial[9];         // 8 bytes: Serial number (e.g., "00000001")
} DeviceID_Parsed;

// Device Name to Serial Number mapping
typedef struct {
    const char* device_name;
    const char* serial_number;
} DeviceMapping;

// Define device mappings here
static const DeviceMapping device_mappings[] = {
    {"Office 1", "00000001"},
    {"Office 2", "00000002"},
    {"Office 3", "00000003"},
    // Add more mappings as needed
};
#define DEVICE_MAPPING_COUNT (sizeof(device_mappings) / sizeof(DeviceMapping))

static uint8_t IsFileOpen(FIL* fp) {
    // Check if file is open by testing if file system pointer is valid
    return (fp->fs != NULL);
}

extern FIL pcmFiles[AUDIO_FILE_COUNT];
extern void SD_ResetInterface(void);
UINT bytes_read;

volatile uint8_t playing = 0;



#define USE_MODEM_LORA 1
#define USE_MODEM_FSK  0
#define BUFFER_SIZE 	256



/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static UTIL_TIMER_Object_t timerTransmit;   
static UTIL_TIMER_Object_t timerButtonCheck;  // Timer for button polling
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRANSMIT_PERIOD_MS 2000  /* 1000 ticks is equal to 1 second */
#define RX_CONTINUOUS_ON 1
#define EnableLog 1
#define TRANSMITTER 0
#define RECEIVER  1






/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

///* USER CODE BEGIN PFP */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;
TxConfigGeneric_t TxConfig;
RxConfigGeneric_t RxConfig = {0};

/* USER CODE BEGIN PV */


uint8_t TxBuffer[PAYLOAD_LEN];
uint8_t RxBuffer[PAYLOAD_LEN];
uint16_t BufferSize = BUFFER_SIZE;
uint8_t i=0;
uint16_t TxBufferLength = PAYLOAD_LEN;
uint32_t rnd;
uint32_t TxPeriod = 0;
uint32_t rndMask;
uint8_t PacketCount = 0;

static uint8_t syncword[3] = { 0x55, 0x90, 0x4E};  /* for FSK  */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */

static void TransmitPacket(void *context);
static void Tx_Process(void);
static void Rx_Process(void);
void InitLoRaPHY(void);
void InitGFSKPHY(void);
void SD_Process(void);
void DAC_Process(void);
void Start_Playback(void);
void Test_44kHz_Feasibility(void);
int GetFileIndexForDevice(const char* deviceName);

// Button and Authentication Functions
void Button_Check_Process(void *context);
void Handle_Button_Forward(void);
void Handle_Button_Backward(void);
void Handle_Button_Long_Press(int button_id);
void Seek_Forward(void);
void Seek_Backward(void);
void Parse_DeviceID(const char* deviceID_str, DeviceID_Parsed* parsed);
uint8_t Authenticate_Device(const char* deviceID_str, const char* deviceName);
const char* Get_Serial_For_Device(const char* deviceName);

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */

#if(USE_MODEM_LORA)
#if(TRANSMITTER)
  /* update both the Tx and Rx configurations for LoRa here  */
  	APP_LOG(TS_ON, VLEVEL_L, "******LORA TRANSMITTER Init ******\n\r");
     Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
  	LORA_SPREADING_FACTOR, LORA_CODINGRATE,LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
     true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
#endif
#if(RECEIVER)
  	APP_LOG(TS_ON, VLEVEL_L, "******LORA RECEIVER Init  ******\n\r");
  	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
  	LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
  	LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
  	0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
      Radio.SetMaxPayloadLength(MODEM_LORA, PAYLOAD_LEN);
#endif
#endif    /* endif for USE_MODEM_LORA */

#if (USE_MODEM_FSK)
  TxConfigGeneric_t TxConfig;
  RxConfigGeneric_t RxConfig = {0};
#if(TRANSMITTER)


      	APP_LOG(TS_ON, VLEVEL_L, "******TRANSMITTER FSK GENERIC INIT******\n\r");

      	  /* fsk modulation*/
      	  TxConfig.fsk.ModulationShaping = RADIO_FSK_MOD_SHAPING_G_BT_05;
      	  TxConfig.fsk.FrequencyDeviation = FSK_FDEV;
      	  TxConfig.fsk.BitRate = FSK_DATARATE;
      	  TxConfig.fsk.PreambleLen = FSK_PREAMBLE_LENGTH;
      	  TxConfig.fsk.SyncWordLength = sizeof(syncword);
      	  TxConfig.fsk.SyncWord = syncword;
      	  TxConfig.fsk.whiteSeed =  0x0000;
      	  TxConfig.fsk.HeaderType  = RADIO_FSK_PACKET_VARIABLE_LENGTH;
      	  TxConfig.fsk.CrcLength = RADIO_FSK_CRC_OFF;
      	  TxConfig.fsk.Whitening = RADIO_FSK_DC_FREE_OFF;
      	  if (0UL != Radio.RadioSetTxGenericConfig(GENERIC_FSK, &TxConfig, TX_OUTPUT_POWER, TX_TIMEOUT_VALUE))
      	  {
      	    while (1);
      	  }
#endif
#if(RECEIVER)
      	  APP_LOG(TS_ON, VLEVEL_L, "******RECEIVER FSK GENERIC INIT******\n\r");
      	 /*  RX Continuous */
      	  RxConfig.fsk.ModulationShaping = RADIO_FSK_MOD_SHAPING_G_BT_05;
      	  RxConfig.fsk.Bandwidth = FSK_BANDWIDTH;
      	  RxConfig.fsk.BitRate = FSK_DATARATE;
      	  RxConfig.fsk.PreambleLen = FSK_PREAMBLE_LENGTH;
      	  RxConfig.fsk.SyncWordLength = sizeof(syncword);
      	  RxConfig.fsk.PreambleMinDetect = RADIO_FSK_PREAMBLE_DETECTOR_08_BITS;
      	  RxConfig.fsk.SyncWord = syncword;
      	  RxConfig.fsk.whiteSeed = 0x0000;
      	  RxConfig.fsk.LengthMode = RADIO_FSK_PACKET_VARIABLE_LENGTH;
      	  RxConfig.fsk.CrcLength = RADIO_FSK_CRC_OFF;
      	  RxConfig.fsk.Whitening = RADIO_FSK_DC_FREE_OFF;
      	  RxConfig.fsk.MaxPayloadLength = PAYLOAD_LEN;
      	  RxConfig.fsk.StopTimerOnPreambleDetect = 0;
      	  RxConfig.fsk.AddrComp = RADIO_FSK_ADDRESSCOMP_FILT_OFF;
      	  if (0UL != Radio.RadioSetRxGenericConfig(GENERIC_FSK, &RxConfig, RX_CONTINUOUS_ON, 0))
      	  {
      	    while (1);
      	  }
#endif
#endif


  /* init the GPIO pins to static value to ensure logic level */

     HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);  /* turn on GPIO when in Tx state */


  Radio.SetChannel(RF_FREQUENCY);

#if(TRANSMITTER)
  /* generate a 32 bit random number and from this number self assign a Node_ID */
  /* this section below is only used when sending a random Tx transmission and Tx_Process is Enabled */

/*
    rnd = SUBGRF_GetRandom();   generate a 32 bit random number
  	rndMask = rnd & (0x0000000000000FFF);   extension of up to 4 seconds
  	TxPeriod =  rndMask + TRANSMIT_PERIOD_MS;
  */


  UTIL_TIMER_Create(&timerTransmit, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, TransmitPacket, NULL);
  UTIL_TIMER_SetPeriod(&timerTransmit, TxPeriod);
   /* provides enough time to take into account path delays when distance increases */
   UTIL_TIMER_Start(&timerTransmit);   /* how to start the timer */

//  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_Tx_Process), UTIL_SEQ_RFU, Tx_Process);
//  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_Tx_Process), CFG_SEQ_Prio_0);
#endif

#ifdef RECEIVER

  /* ST added sequencer code for Receiver */

  // Create button check timer (check every 50ms)
  UTIL_TIMER_Create(&timerButtonCheck, 0xFFFFFFFFU, UTIL_TIMER_PERIODIC, Button_Check_Process, NULL);
  UTIL_TIMER_SetPeriod(&timerButtonCheck, 50);  // 50ms polling
  UTIL_TIMER_Start(&timerButtonCheck);

  UTIL_SEQ_RegTask(1 << CFG_SEQ_Task_DAC_Process, UTIL_SEQ_RFU, DAC_Process);
  UTIL_SEQ_RegTask(1 << CFG_SEQ_Task_Start_Playback, UTIL_SEQ_RFU, Start_Playback);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_Rx_Process), UTIL_SEQ_RFU, Rx_Process);
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_Rx_Process), CFG_SEQ_Prio_0);


#endif

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

static void Rx_Process(void)
{
	Radio.SetChannel(RF_FREQUENCY);
	APP_LOG(TS_OFF, VLEVEL_L, "\n\r");
    APP_LOG(TS_ON, VLEVEL_L, "Radio Rx\n\r");
    Radio.Rx(0);
}

/*
static void Tx_Process(void)
{
	UTIL_TIMER_Stop(&timerTransmit);
	HAL_Delay(500);
	rnd = SUBGRF_GetRandom();   generate a 32 bit random number
	rndMask = rnd & (0x0000000000000FF);   filter only a small part to add variance
	if(PacketCount % 2 == 0)   simple flip of rndMask sign
	{
		TxPeriod =   TRANSMIT_PERIOD_MS+rndMask;
	}
	else
	{
		TxPeriod =   TRANSMIT_PERIOD_MS-rndMask;
	}

	UTIL_TIMER_SetPeriod(&timerTransmit, TxPeriod);
	Radio.Sleep();   place radio into sleep mode, sequencer also goes into idle mode and therefore stop2 mode
	UTIL_TIMER_Start(&timerTransmit);   restart the Transmit timer with specified period
}
*/

static uint8_t payload_sent = 0;

#if 0
static void TransmitPacket(void *context)
	{
	/* send the payload buffer and display on the terminal output */
	if (payload_sent) {
	        // Do nothing — stay silent
	        return;
	    }
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);  /* turn on GPIO when in Tx state */

		// Clear buffer
	    memset(TxBuffer, 0, PAYLOAD_LEN);

	    /* data will be transmitted every TxPeriod */

	    // Define device data (ASCII strings)
	    char modelNumber[9]   = "BanBel1O";          // 8 chars (64 bits)
	    char deviceID[17]     = "ABCD1234EFGH5678";  // 16 chars (128 bits)
	    char deviceName[33]   = "Office 2"; // up to 32 chars (256 bits)

	    // Fill TxBuffer sequentially
	    memset(TxBuffer, 0, 56);
	    memcpy(TxBuffer, modelNumber, strlen(modelNumber));
	    memcpy(TxBuffer + 8, deviceID, strlen(deviceID));
	    memcpy(TxBuffer + 24, deviceName, strlen(deviceName));

	        APP_LOG(TS_ON, VLEVEL_L, "Transmitting 448-bit packet:\r\n");
	        for (int i = 0; i < PAYLOAD_LEN; i++)
	        {
	            APP_LOG(TS_OFF, VLEVEL_L, "%02X ", TxBuffer[i]);
	        }
	        APP_LOG(TS_OFF, VLEVEL_L, "\n");

	        HAL_Delay(20);
	        Radio.Send(TxBuffer, PAYLOAD_LEN);

	        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	        payload_sent = 1;  // mark as sent

}
#endif

static void TransmitPacket(void *context)
{
    if (payload_sent) {
        return;
    }

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    memset(TxBuffer, 0, PAYLOAD_LEN);

    // Use default audio index if set by long press
    char deviceName[33] = {0};
    if (default_audio_index >= 0 && default_audio_index < AUDIO_FILE_COUNT) {
        // Map file index back to device name
        if (default_audio_index == 0) {
            strcpy(deviceName, "Office 1");
        } else if (default_audio_index == 1) {
            strcpy(deviceName, "Office 2");
        }
        // Add more mappings as needed
    } else {
        strcpy(deviceName, "Office 2");  // Default
    }

    char modelNumber[9]   = "BanBel1O";
    char deviceID[17]     = "J10LR01250000002";  // Example: Jence, test 10, LoRa, v01, 2025, serial 00000002

    memset(TxBuffer, 0, 56);
    memcpy(TxBuffer, modelNumber, strlen(modelNumber));
    memcpy(TxBuffer + 8, deviceID, strlen(deviceID));
    memcpy(TxBuffer + 24, deviceName, strlen(deviceName));

    APP_LOG(TS_ON, VLEVEL_L, "Transmitting 448-bit packet:\r\n");
    for (int i = 0; i < PAYLOAD_LEN; i++)
    {
        APP_LOG(TS_OFF, VLEVEL_L, "%02X ", TxBuffer[i]);
    }
    APP_LOG(TS_OFF, VLEVEL_L, "\n");

    HAL_Delay(20);
    Radio.Send(TxBuffer, PAYLOAD_LEN);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    payload_sent = 1;
}

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  APP_LOG(TS_ON, VLEVEL_L, "OnTxDone\n\r");
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    /* uncomment this section when using the sequencer and Tx_Process restart the sequencer for next Tx interval */
 /* UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_Tx_Process), CFG_SEQ_Prio_0); */
  /* restart the transmit timer */
  UTIL_TIMER_Start(&timerTransmit);   /* how to start the timer */

  /* USER CODE END OnTxDone */
}
//void SD_Process(void);   // <-- add this near the top of the file

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
	/*  ST  added code  */
	  APP_LOG(TS_ON, VLEVEL_L, "OnRxDone\n\r");
	  APP_LOG(TS_ON, VLEVEL_L,  "RssiValue=%d dBm, SnrValue=%d\n\r", rssi, LoraSnr_FskCfo);

	  Radio.Sleep();
	  BufferSize = size;
	  if (BufferSize > sizeof(RxBuffer)) {
	      APP_LOG(TS_ON, VLEVEL_L, "OnRxDone: BufferSize too large: %d\r\n", BufferSize);
	      BufferSize = sizeof(RxBuffer);
	  }
	  memcpy(RxBuffer, payload, BufferSize);

	  APP_LOG(TS_OFF, VLEVEL_L,  "Received RxBuffer");
	  APP_LOG(TS_OFF, VLEVEL_L,  "Tx Sending ");  /* print out data */

	  for (i=0;i<BufferSize;i++)
	  	  	   {
	  	  	  		APP_LOG(TS_OFF, VLEVEL_L,  "%02X ", RxBuffer[i]);
	  	  	   }
	  	 // Decode if expected size
	  	     if (BufferSize >= 56)
	  	     {
	  	         char modelNumber[9] = {0};
	  	         char deviceID[17]    = {0};
	  	         char deviceName[33] = {0};
	  	         memcpy(modelNumber, RxBuffer, 8);
	  	         memcpy(deviceID,   RxBuffer + 8, 16);
	  	         memcpy(deviceName, RxBuffer + 24, 32);
	  	         modelNumber[8] = '\0';
	  	     	 deviceID[16]   = '\0';
	  	     	 deviceName[32] = '\0';

	  	         APP_LOG(TS_ON, VLEVEL_L, "\r\nDecoded Data:\r\n");
	  	         APP_LOG(TS_ON, VLEVEL_L, "Model Number: %s\r\n", modelNumber);
	  	         APP_LOG(TS_ON, VLEVEL_L, "Device ID: %s\r\n", deviceID);
	  	         APP_LOG(TS_ON, VLEVEL_L, "Device Name: %s\r\n", deviceName);
	  	         newIndex = GetFileIndexForDevice(deviceName);
	  	         APP_LOG(TS_ON, VLEVEL_L, "Mapped to file index: %d\r\n", newIndex);

	  	       // Parse Device ID
				 DeviceID_Parsed parsed_id;
				 Parse_DeviceID(deviceID, &parsed_id);

				 APP_LOG(TS_ON, VLEVEL_L, "Parsed Device ID:\r\n");
				 APP_LOG(TS_ON, VLEVEL_L, "  Company: %c\r\n", parsed_id.company);
				 APP_LOG(TS_ON, VLEVEL_L, "  Testing Code: %s\r\n", parsed_id.testing_code);
				 APP_LOG(TS_ON, VLEVEL_L, "  Product Type: %s\r\n", parsed_id.product_type);
				 APP_LOG(TS_ON, VLEVEL_L, "  Version: %s\r\n", parsed_id.version);
				 APP_LOG(TS_ON, VLEVEL_L, "  Year: %s\r\n", parsed_id.year);
				 APP_LOG(TS_ON, VLEVEL_L, "  Serial: %s\r\n", parsed_id.serial);

				 // Authenticate Device
				 authentication_passed = Authenticate_Device(deviceID, deviceName);

				 if (authentication_passed) {
					 APP_LOG(TS_ON, VLEVEL_L, "Authentication PASSED!\r\n");

					 newIndex = GetFileIndexForDevice(deviceName);
					 APP_LOG(TS_ON, VLEVEL_L, "Mapped to file index: %d\r\n", newIndex);
				 } else {
					 APP_LOG(TS_ON, VLEVEL_L, "Authentication FAILED! Device Name and Serial Number mismatch.\r\n");
					 newIndex = -1;  // Don't play anything
				 }
	  	     }
	  	     else
	  	     {
	  	         APP_LOG(TS_ON, VLEVEL_L, "Invalid packet size: %d\r\n", BufferSize);
	  	         authentication_passed = 0;
				 newIndex = -1;
	  	     }






	  	 if (playing) {
	  	         HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
	  	         HAL_TIM_Base_Stop(&htim2);
	  	         playing = 0;

	  	         // Reset all flags for clean restart
	  	         file_ended = 0;
	  	         next_buffer_ready = 0;
	  	         currently_filling = 0;
	  	         current_half = 0;
	  	     }



#if 0
	  	// Update current audio index if valid
	  	    if (newIndex >= 0 && newIndex < AUDIO_FILE_COUNT) {
	  	        currentAudioIndex = newIndex;
	  	        APP_LOG(TS_ON, VLEVEL_L, "Switched to file index: %d\r\n", currentAudioIndex);

	  	        // Rewind the file (assumes it's already open from main)
	  	        if (IsFileOpen(&pcmFiles[currentAudioIndex])) {
	  	            f_lseek(&pcmFiles[currentAudioIndex], 0);
	  	            APP_LOG(TS_ON, VLEVEL_L, "File rewound to beginning\r\n");
	  	        } else {
	  	            APP_LOG(TS_ON, VLEVEL_L, "WARNING: File not open! Check main init\r\n");
	  	        }
	  	    }


	  UTIL_SEQ_SetTask(1 << CFG_SEQ_Task_Start_Playback, CFG_SEQ_Prio_0);

#endif
	  // Update current audio index if valid and authenticated
	    if (authentication_passed && newIndex >= 0 && newIndex < AUDIO_FILE_COUNT) {
	        currentAudioIndex = newIndex;
	        APP_LOG(TS_ON, VLEVEL_L, "Switched to file index: %d\r\n", currentAudioIndex);

	        if (IsFileOpen(&pcmFiles[currentAudioIndex])) {
	            f_lseek(&pcmFiles[currentAudioIndex], 0);
	            APP_LOG(TS_ON, VLEVEL_L, "File rewound to beginning\r\n");
	        } else {
	            APP_LOG(TS_ON, VLEVEL_L, "WARNING: File not open! Check main init\r\n");
	        }

//	        UTIL_SEQ_SetTask(1 << CFG_SEQ_Task_Start_Playback, CFG_SEQ_Prio_0);
	    } else {
	        APP_LOG(TS_ON, VLEVEL_L, "Not starting playback - authentication failed or invalid index\r\n");
//	        UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_Rx_Process), CFG_SEQ_Prio_0);
	    }

	    UTIL_SEQ_SetTask(1 << CFG_SEQ_Task_Start_Playback, CFG_SEQ_Prio_0);

  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */

// Button Check Process (called by timer every 50ms)
void Button_Check_Process(void *context)
{
    uint32_t current_time = HAL_GetTick();

    // Read button states (assuming active LOW buttons with pull-up)
    GPIO_PinState fwd_state = HAL_GPIO_ReadPin(BUTTON_FWD_GPIO_Port, BUTTON_FWD_Pin);
    GPIO_PinState bwd_state = HAL_GPIO_ReadPin(BUTTON_BWD_GPIO_Port, BUTTON_BWD_Pin);

    // Forward Button Logic
    if (fwd_state == GPIO_PIN_RESET) {  // Button pressed (active LOW)
        if (!button_fwd_pressed) {
            // Button just pressed
            button_fwd_pressed = 1;
            button_fwd_press_time = current_time;
            button_fwd_handled = 0;
        } else {
            // Button held down
            if (!button_fwd_handled && (current_time - button_fwd_press_time >= LONG_PRESS_TIME)) {
                // Long press detected
                Handle_Button_Long_Press(0);  // 0 = forward button
                button_fwd_handled = 1;
            }
        }
    } else {
        // Button released
        if (button_fwd_pressed && !button_fwd_handled) {
            // Short press detected
            if (current_time - button_fwd_press_time >= DEBOUNCE_TIME) {
                Handle_Button_Forward();
            }
        }
        button_fwd_pressed = 0;
        button_fwd_handled = 0;
    }

    // Backward Button Logic
    if (bwd_state == GPIO_PIN_RESET) {  // Button pressed (active LOW)
        if (!button_bwd_pressed) {
            button_bwd_pressed = 1;
            button_bwd_press_time = current_time;
            button_bwd_handled = 0;
        } else {
            if (!button_bwd_handled && (current_time - button_bwd_press_time >= LONG_PRESS_TIME)) {
                Handle_Button_Long_Press(1);  // 1 = backward button
                button_bwd_handled = 1;
            }
        }
    } else {
        if (button_bwd_pressed && !button_bwd_handled) {
            if (current_time - button_bwd_press_time >= DEBOUNCE_TIME) {
                Handle_Button_Backward();
            }
        }
        button_bwd_pressed = 0;
        button_bwd_handled = 0;
    }
}

// Handle Forward Button Short Press
void Handle_Button_Forward(void)
{
    APP_LOG(TS_ON, VLEVEL_L, "Button Forward Pressed - Seeking Forward\r\n");
    Seek_Forward();
}

// Handle Backward Button Short Press
void Handle_Button_Backward(void)
{
    APP_LOG(TS_ON, VLEVEL_L, "Button Backward Pressed - Seeking Backward\r\n");
    Seek_Backward();
}

// Handle Long Press (set as default for transmitter)
void Handle_Button_Long_Press(int button_id)
{
    APP_LOG(TS_ON, VLEVEL_L, "Long Press Detected on Button %d\r\n", button_id);
    APP_LOG(TS_ON, VLEVEL_L, "Setting current audio as default for transmitter\r\n");

    default_audio_index = currentAudioIndex;
    APP_LOG(TS_ON, VLEVEL_L, "Default audio index set to: %d\r\n", default_audio_index);

    // Optional: Provide feedback (e.g., flash LED)
//    for (int i = 0; i < 3; i++) {
//        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//        HAL_Delay(100);
//        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//        HAL_Delay(100);
//    }
}

// Seek Forward Function
void Seek_Forward(void)
{
    // Stop current playback
    if (playing) {
        HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
        HAL_TIM_Base_Stop(&htim2);
        playing = 0;
    }

    // Move to next file
    currentAudioIndex++;
    if (currentAudioIndex >= AUDIO_FILE_COUNT) {
        currentAudioIndex = 0;  // Wrap around
    }

    APP_LOG(TS_ON, VLEVEL_L, "Seeking forward to file index: %d\r\n", currentAudioIndex);

    // Reset playback state
    file_ended = 0;
    next_buffer_ready = 0;
    currently_filling = 0;
    current_half = 0;

    // Rewind file
    if (IsFileOpen(&pcmFiles[currentAudioIndex])) {
        f_lseek(&pcmFiles[currentAudioIndex], 0);
        UTIL_SEQ_SetTask(1 << CFG_SEQ_Task_Start_Playback, CFG_SEQ_Prio_0);
    } else {
        APP_LOG(TS_ON, VLEVEL_L, "ERROR: File %d not open!\r\n", currentAudioIndex);
    }
}

// Seek Backward Function
void Seek_Backward(void)
{
    // Stop current playback
    if (playing) {
        HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
        HAL_TIM_Base_Stop(&htim2);
        playing = 0;
    }

    // Move to previous file
//    currentAudioIndex--;
//    if (currentAudioIndex <= 0) {
//        currentAudioIndex = AUDIO_FILE_COUNT - 1;  // Wrap around
//    }
    // Store old index for debugging
        int old_index = currentAudioIndex;

        // Move to previous file
        currentAudioIndex--;

        APP_LOG(TS_ON, VLEVEL_L, "After decrement: old=%d, new=%d\r\n", old_index, currentAudioIndex);

        // Wrap around if needed
        if (currentAudioIndex < 0) {
            currentAudioIndex = AUDIO_FILE_COUNT - 1;
            APP_LOG(TS_ON, VLEVEL_L, "Wrapped around to: %d\r\n", currentAudioIndex);
        }

        APP_LOG(TS_ON, VLEVEL_L, "Final index before playback: %d\r\n", currentAudioIndex);




//    APP_LOG(TS_ON, VLEVEL_L, "Seeking backward to file index: %d\r\n", currentAudioIndex);

    // Reset playback state
    file_ended = 0;
    next_buffer_ready = 0;
    currently_filling = 0;
    current_half = 0;

    // Rewind file
    if (IsFileOpen(&pcmFiles[currentAudioIndex])) {
        f_lseek(&pcmFiles[currentAudioIndex], 0);
        UTIL_SEQ_SetTask(1 << CFG_SEQ_Task_Start_Playback, CFG_SEQ_Prio_0);
    } else {
        APP_LOG(TS_ON, VLEVEL_L, "ERROR: File %d not open!\r\n", currentAudioIndex);
    }
}

// Parse Device ID into structured format
void Parse_DeviceID(const char* deviceID_str, DeviceID_Parsed* parsed)
{
    if (strlen(deviceID_str) < 16) {
        memset(parsed, 0, sizeof(DeviceID_Parsed));
        return;
    }

    // Parse: J 10 LR 01 25 00000001
    parsed->company = deviceID_str[0];

    memcpy(parsed->testing_code, &deviceID_str[1], 2);
    parsed->testing_code[2] = '\0';

    memcpy(parsed->product_type, &deviceID_str[3], 2);
    parsed->product_type[2] = '\0';

    memcpy(parsed->version, &deviceID_str[5], 2);
    parsed->version[2] = '\0';

    memcpy(parsed->year, &deviceID_str[7], 2);
    parsed->year[2] = '\0';

    memcpy(parsed->serial, &deviceID_str[9], 8);
    parsed->serial[8] = '\0';
}

// Get expected serial number for a device name
const char* Get_Serial_For_Device(const char* deviceName)
{
    for (int i = 0; i < DEVICE_MAPPING_COUNT; i++) {
        if (strcmp(deviceName, device_mappings[i].device_name) == 0) {
            return device_mappings[i].serial_number;
        }
    }
    return NULL;  // No mapping found
}

// Authenticate device using Device ID and Device Name
uint8_t Authenticate_Device(const char* deviceID_str, const char* deviceName)
{
    DeviceID_Parsed parsed_id;
    Parse_DeviceID(deviceID_str, &parsed_id);

    // Get expected serial number for this device name
    const char* expected_serial = Get_Serial_For_Device(deviceName);

    if (expected_serial == NULL) {
        APP_LOG(TS_ON, VLEVEL_L, "Authentication: No mapping found for device '%s'\r\n", deviceName);
        return 0;  // Failed - no mapping exists
    }

    // Compare serial numbers (AND logic)
    if (strcmp(parsed_id.serial, expected_serial) == 0) {
        APP_LOG(TS_ON, VLEVEL_L, "Authentication: Serial match! Expected=%s, Got=%s\r\n",
                expected_serial, parsed_id.serial);
        return 1;  // Passed
    } else {
        APP_LOG(TS_ON, VLEVEL_L, "Authentication: Serial mismatch! Expected=%s, Got=%s\r\n",
                expected_serial, parsed_id.serial);
        return 0;  // Failed
    }
}


void SD_Process(void)
{

	UINT bytes_read_local = 0;
	// Determine which half of circular buffer to fill
	if (!next_buffer_ready) {
	        return;
	    }
	// Prevent re-entry while filling
	    if (currently_filling) {
	        return;
	    }
	    currently_filling = 1;  // Lock

	    	// buffer selection
	        uint16_t* target_buffer;
	        const char* target_name;
	        // current_half indicates which half is EMPTY (needs filling)
	        if (current_half == 0) {
	        // First half is empty, fill it
	            target_buffer = &circular_buffer[0];
	            target_name = "first";
	        } else {
	        // Second half is empty, fill it
	            target_buffer = &circular_buffer[BATCH_SIZE];
	            target_name = "second";
	        }




		FRESULT	res = f_read(&pcmFiles[currentAudioIndex], batch_raw, BATCH_SIZE * 2, &bytes_read_local);




		//error diagnostics
		    if (res != FR_OK) {
		        APP_LOG(TS_ON, VLEVEL_L, "f_read failed: res=%d, bytes_read=%u\r\n",
		                res, bytes_read_local);
		        APP_LOG(TS_ON, VLEVEL_L, "File position: %u / %u\r\n",
		                (unsigned int)(f_tell(&pcmFiles[currentAudioIndex])),
		                (unsigned int)(f_size(&pcmFiles[currentAudioIndex])));

		        //Try to recover from read error
		        if (res == FR_DISK_ERR) {
		            APP_LOG(TS_ON, VLEVEL_L, "Disk error - attempting recovery\r\n");

		            // Give SD card a moment to recover
		            HAL_Delay(1);

		            // Try reading again
		            res = f_read(&pcmFiles[currentAudioIndex], batch_raw,
		                         BATCH_SIZE * 2, &bytes_read_local);

		            if (res == FR_OK && bytes_read_local > 0) {
		                APP_LOG(TS_ON, VLEVEL_L, "Recovery successful, read %u bytes\r\n",
		                        bytes_read_local);
		            // Continue processing below
		            } else {
		                APP_LOG(TS_ON, VLEVEL_L, "Recovery failed\r\n");
		                file_ended = 1;
		                next_buffer_ready = 0;
		                currently_filling = 0;
		                return;
		            }
		        } else {
		            file_ended = 1;
		            next_buffer_ready = 0;
		            currently_filling = 0;
		            return;
		        }
		    }

		    // Check if we actually got data
		    if (bytes_read_local == 0) {
		        APP_LOG(TS_ON, VLEVEL_L, "EOF reached - no more data\r\n");

		        // Fill with silence for smooth ending
		        for (uint32_t i = 0; i < BATCH_SIZE; i++) {
		            target_buffer[i] = 2048;
		        }

		        file_ended = 1;
		        next_buffer_ready = 0;
		        currently_filling = 0;
		        return;
		    }

	    next_samples_count = bytes_read_local / 2;
	    for (uint32_t i = 0; i < next_samples_count; i++) {
	        uint16_t s16 = (uint16_t)((batch_raw[2*i+1] << 8) | batch_raw[2*i]);
	        target_buffer[i] = (uint16_t)(s16 >> 4); // signed PCM → 12-bit DAC
	    }

	    // Fill remaining samples with silence if partial buffer
	       if (next_samples_count < BATCH_SIZE) {
	           APP_LOG(TS_ON, VLEVEL_L,"Padding %u samples with silence\r\n",
	                   BATCH_SIZE - next_samples_count);
	           for (uint32_t i = next_samples_count; i < BATCH_SIZE; i++) {
	               target_buffer[i] = 2048; // Mid-point for 12-bit DAC (silence)
	           }


	           // Only mark file ended if truly reached EOF
	               FIL *f = &pcmFiles[currentAudioIndex];
	               if (f_tell(f) >= f_size(f)) {
	                   file_ended = 1;
	                   APP_LOG(TS_ON, VLEVEL_L, "File completely read, marking EOF\r\n");
	               }
	       }

	    next_buffer_ready = 0;
	    currently_filling = 0;  // Unlock

//	    APP_LOG(TS_ON, VLEVEL_L,"SD_Process: %s half filled, waiting for next empty signal\r\n",
//	                target_name);

}


void DAC_Process(void) {
    if (!playing) return;
    static uint32_t call_count = 0;
        call_count++;
    if (!file_ended) {
            next_buffer_ready = 1;  // Signal that we can fill the empty buffer
            SD_Process();              // Fill it immediately
        } else {
            APP_LOG(TS_ON, VLEVEL_L,"File ended, not filling more buffers\r\n");
        }
    // Check if we should stop (file ended and no more data to play)
        if (file_ended && next_buffer_ready == 0) {
            // File ended and we've filled the last buffer
            // Let it play out, will stop after this buffer completes
            APP_LOG(TS_ON, VLEVEL_L,"File ended, letting final buffers play out\r\n");
        }


}


void Start_Playback(void) {


		HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
		HAL_TIM_Base_Stop(&htim2);
		__HAL_DMA_DISABLE(hdac.DMA_Handle1);


		uint32_t sysclk = HAL_RCC_GetSysClockFreq();
	    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
	    APP_LOG(TS_ON, VLEVEL_L,"System Clock: %u Hz\r\n", sysclk);
	    APP_LOG(TS_ON, VLEVEL_L,"PCLK1 (TIM2 clock): %u Hz\r\n", pclk1);
	    APP_LOG(TS_ON, VLEVEL_L,"TIM2 Prescaler: %u\r\n", htim2.Init.Prescaler);
	    APP_LOG(TS_ON, VLEVEL_L,"TIM2 Period: %u\r\n", htim2.Init.Period);
	    uint32_t calculated_freq = pclk1 / (htim2.Init.Prescaler + 1) / (htim2.Init.Period + 1);
	    APP_LOG(TS_ON, VLEVEL_L,"Calculated DAC frequency: %u Hz\r\n", calculated_freq);



	    //Ensure file is open and rewound

	    // Check if file is open (should be opened in main)
	        if (!IsFileOpen(&pcmFiles[currentAudioIndex])) {
	            APP_LOG(TS_ON, VLEVEL_L, "ERROR: File not open! Index: %d\r\n", currentAudioIndex);
	            APP_LOG(TS_ON, VLEVEL_L, "Make sure files are opened in main.c\r\n");
	            return;
	        }


	        //Log file size for debugging
	        DWORD file_size = f_size(&pcmFiles[currentAudioIndex]);
	            APP_LOG(TS_ON, VLEVEL_L, "File size: %u bytes (%u samples)\r\n",
	                    (unsigned int)(file_size),(unsigned int) (file_size / 2));
	            APP_LOG(TS_ON, VLEVEL_L, "Playback duration: ~%u ms\r\n",
	                    (unsigned int)((file_size / 2) * 1000 / 32000));


	    f_lseek(&pcmFiles[currentAudioIndex], 0);  // rewind file
	    // Initialize buffer state
	    current_half = 0;  // Will start with first half
	    file_ended = 0;
	    next_buffer_ready = 1;
	    next_samples_count = 0;

	    // Fill first half
        SD_Process();


        if (file_ended) {
                APP_LOG(TS_ON, VLEVEL_L, "ERROR: Immediate EOF\r\n");
                UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_Rx_Process), CFG_SEQ_Prio_0);
                return;
            }

        // Prepare to fill second half
        current_half = 1;       // Second half is empty
        next_buffer_ready = 1;    // Ready to fill second half
        SD_Process();
        current_half = 0;       // When first half finishes, it will be empty
        next_buffer_ready = 0;    // Wait for callback to signal empty

        playing = 1;

        // Start circular DMA with entire buffer
        HAL_StatusTypeDef dma_status = HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,
                                  (uint32_t*)circular_buffer,
                                  BATCH_SIZE * 2,  // Total size of circular buffer
                                  DAC_ALIGN_12B_R);

        if (dma_status == HAL_OK) {
                        HAL_TIM_Base_Start(&htim2);
                    } else {
                        playing = 0;
                        APP_LOG(TS_ON, VLEVEL_L,"Failed to start circular DMA\r\n");
                    }
        //Return to Rx mode after starting playback
            UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_Rx_Process), CFG_SEQ_Prio_0);

}


// DMA Half Complete Callback - first half finished (now empty)
  void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
  	uint32_t current_time = HAL_GetTick();
  	uint32_t elapsed = current_time - last_callback_time;
  	last_callback_time = current_time;

      if (!playing) return;

      // First half just finished and is now empty
      current_half = 0;  // First half is empty

      // Trigger task to fill the empty first half
      if (!file_ended) {
              next_buffer_ready = 1;
              SD_Process();
  }
  }

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
	uint32_t current_time = HAL_GetTick();
	uint32_t elapsed = current_time - last_callback_time;
	last_callback_time = current_time;


    if (!playing) return;

    // Second half just finished and is now empty
    current_half = 1;  // Second half is empty

    // Check if we should stop (both file ended and this might be final playback)
    if (file_ended && next_buffer_ready == 0) {
        // This was the final buffer, stop playback
        APP_LOG(TS_ON, VLEVEL_L,"Final buffer completed - stopping playback\r\n");
        HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_1);
        HAL_TIM_Base_Stop(&htim2);
        playing = 0;
        file_ended = 0; // Reset for next playback
        return;
    }


            next_buffer_ready = 1;
            SD_Process();
}


int GetFileIndexForDevice(const char* deviceName)
{
    if (strcmp(deviceName, "Office 1") == 0)
        return 0;  // Salaam32.pcm
    if (strcmp(deviceName, "Office 2") == 0)
        return 1;  // music1.pcm
    return -1;
}

/*void OnPreambleDetect(void)
{

//	UTIL_TIMER_Stop(&timerReceive);
	 Radio.SetChannel((RF_FREQUENCY) + NextChannel*freq_step);  set the next channel
     APP_LOG(TS_ON, VLEVEL_L, "OnPDDone\n\r");
    SUBGRF_ClearIrqStatus(IRQ_PREAMBLE_DETECTED);

 this is the preamble detected call back according to the setting
 RxConfig.fsk.PreambleMinDetect = RADIO_FSK_PREAMBLE_DETECTOR_08_BITS;
 this is set in radio.c as part of the IRQ detection
 no need to set here as we are not interrupting the Rx process

  SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE);

}*/

/*void InitLoRaPHY(void)
{
	 update both the Tx and Rx configurations for LoRa here
	APP_LOG(TS_ON, VLEVEL_L, "******LORA TRANSMITTER Init ******\n\r");
   Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
	LORA_SPREADING_FACTOR, LORA_CODINGRATE,LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
   true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

	APP_LOG(TS_ON, VLEVEL_L, "******LORA RECEIVER Init  ******\n\r");
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
	LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
	LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
	0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    Radio.SetMaxPayloadLength(MODEM_LORA, PAYLOAD_LEN);
}

void InitGFSKPHY(void)
{
	TxConfigGeneric_t TxConfig;
	RxConfigGeneric_t RxConfig = {0};

	APP_LOG(TS_ON, VLEVEL_L, "******TRANSMITTER FSK GENERIC INIT******\n\r");

	   fsk modulation
	  TxConfig.fsk.ModulationShaping = RADIO_FSK_MOD_SHAPING_G_BT_05;
	  TxConfig.fsk.FrequencyDeviation = FSK_FDEV;
	  TxConfig.fsk.BitRate = FSK_DATARATE;
	  TxConfig.fsk.PreambleLen = FSK_PREAMBLE_LENGTH;
	  TxConfig.fsk.SyncWordLength = sizeof(syncword);
	  TxConfig.fsk.SyncWord = syncword;
	  TxConfig.fsk.whiteSeed =  0x0000;
	  TxConfig.fsk.HeaderType  = RADIO_FSK_PACKET_VARIABLE_LENGTH;
	  TxConfig.fsk.CrcLength = RADIO_FSK_CRC_OFF;
	  TxConfig.fsk.Whitening = RADIO_FSK_DC_FREE_OFF;
	  if (0UL != Radio.RadioSetTxGenericConfig(GENERIC_FSK, &TxConfig, TX_OUTPUT_POWER, TX_TIMEOUT_VALUE))
	  {
	    while (1);
	  }

	  APP_LOG(TS_ON, VLEVEL_L, "******RECEIVER FSK GENERIC INIT******\n\r");
	   RX Continuous
	  RxConfig.fsk.ModulationShaping = RADIO_FSK_MOD_SHAPING_G_BT_05;
	  RxConfig.fsk.Bandwidth = FSK_BANDWIDTH;
	  RxConfig.fsk.BitRate = FSK_DATARATE;
	  RxConfig.fsk.PreambleLen = FSK_PREAMBLE_LENGTH;
	  RxConfig.fsk.SyncWordLength = sizeof(syncword);
	  RxConfig.fsk.PreambleMinDetect = RADIO_FSK_PREAMBLE_DETECTOR_08_BITS;
	  RxConfig.fsk.SyncWord = syncword;
	  RxConfig.fsk.whiteSeed = 0x0000;
	  RxConfig.fsk.LengthMode = RADIO_FSK_PACKET_VARIABLE_LENGTH;
	  RxConfig.fsk.CrcLength = RADIO_FSK_CRC_OFF;
	  RxConfig.fsk.Whitening = RADIO_FSK_DC_FREE_OFF;
	  RxConfig.fsk.MaxPayloadLength = PAYLOAD_LEN;
	  RxConfig.fsk.StopTimerOnPreambleDetect = 0;
	  RxConfig.fsk.AddrComp = RADIO_FSK_ADDRESSCOMP_FILT_OFF;
	  if (0UL != Radio.RadioSetRxGenericConfig(GENERIC_FSK, &RxConfig, RX_CONTINUOUS_ON, 0))
	  {
	    while (1);
	  }


}*/
/* USER CODE END PrFD */
