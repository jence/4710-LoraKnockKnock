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
#include "fatfs.h"
#include "app_subghz_phy.h"
#include "subghz_phy_app.h"
#include "gpio.h"
#include "stm32wlxx_hal_tim.h"
#include "sys_app.h"
#include "stm32_seq.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */
//Added code segment for microseconds delay
#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
const char *audioFiles[] = {
    "music1.pcm",
    "Salaam32.pcm",
};

FIL pcmFiles[AUDIO_FILE_COUNT];
uint8_t currentAudioIndex = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DAC_Init(void); //DAC
void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
void MX_USART1_UART_Init(void);


extern void SD_Process(void);
extern void DAC_Process(void);
extern void Start_Playback(void);
/* USER CODE BEGIN PFP */
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_out1;
TIM_HandleTypeDef htim2;
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
//  uint32_t startTime = HAL_GetTick(); // record start time
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_SubGHz_Phy_Init();
//  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  /* USER CODE END 3 */
    APP_LOG(TS_ON, VLEVEL_L,"\r\n~ SD card demo by kiwih ~\r\n\r\n");

        HAL_Delay(1000); //a short delay is important to let the SD card settle

        //some variables for FatFs
        FATFS FatFs; 	//Fatfs handle
        FRESULT fres; //Result after operations
        HAL_SPI_Init(&hspi1);
        APP_LOG(TS_ON, VLEVEL_L,"Digital probe SPI test starting...\r\n");

        for(int i = 0; i < 3; i++) {
            // Long CS setup
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // CS HIGH
            HAL_Delay(500);  // 500ms idle time - easy to see on scope

            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // CS LOW
            HAL_Delay(100);   // Setup time

            // Send clear test patterns
            uint8_t test_byte = (i == 0) ? 0xAA : (i == 1) ? 0x55 : 0xFF;
            uint8_t rx_data = 0x00;


            APP_LOG(TS_ON, VLEVEL_L,"Sending: 0x%02X\r\n", test_byte);
            HAL_SPI_TransmitReceive(&hspi1, &test_byte, &rx_data, 1, 1000);
            APP_LOG(TS_ON, VLEVEL_L,"Received: 0x%02X\r\n", rx_data);

            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // CS HIGH
            HAL_Delay(500);   // Long gap between tests
        }

        // Unmount first (important)
        fres = f_mount(&FatFs, "", 0);
        APP_LOG(TS_ON, VLEVEL_L, "Implementing fres = f_mount(&FatFs, "", 0)\r\n");
        HAL_SPI_DeInit(&hspi1);	//De-Initializing the SPI
        APP_LOG(TS_ON, VLEVEL_L, "Implementing HAL_SPI_DeInit(&hspi1)\r\n");
        HAL_SPI_Init(&hspi1);	//Re-Initializing the SPI
        APP_LOG(TS_ON, VLEVEL_L, "Implementing HAL_SPI_Init(&hspi1)\r\n");
        fres = f_mount(&FatFs, "", 1); //1=mount now
        APP_LOG(TS_ON, VLEVEL_L, "Implementing fres = f_mount(&FatFs, "", 1)\r\n");
        if (fres != FR_OK) {
      	  APP_LOG(TS_ON, VLEVEL_L,"f_mount error (%i)\r\n", fres);
      	while(1);
        }
        if (fres == FR_NO_FILESYSTEM) {
                APP_LOG(TS_ON, VLEVEL_L,"No filesystem, formatting...\r\n");
                fres = f_mkfs("", 0, 0);   // format with default cluster size
                if (fres == FR_OK) {
                    fres = f_mount(&FatFs, "", 1); // retry mount
                }
            }

        //Let's get some statistics from the SD card
        DWORD free_clusters, free_sectors, total_sectors;

        FATFS* getFreeFs;

        fres = f_getfree("", &free_clusters, &getFreeFs);
        if (fres != FR_OK) {
      	  APP_LOG(TS_ON, VLEVEL_L,"f_getfree error (%i)\r\n", fres);
      	while(1);
        }

        //Formula comes from ChaN's documentation
        total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
        free_sectors = free_clusters * getFreeFs->csize;
        APP_LOG(TS_ON, VLEVEL_L,"SD card stats:\r\n%10u KiB total drive space.\r\n%10u KiB available.\r\n", (unsigned int)(total_sectors / 2), (unsigned int)(free_sectors / 2));

        for (int i = 0; i < AUDIO_FILE_COUNT; i++) {
            APP_LOG(TS_ON, VLEVEL_L, "Opening file: %s\r\n", audioFiles[i]);
            FRESULT res = f_open(&pcmFiles[i], audioFiles[i], FA_READ);
            if (res != FR_OK) {
                APP_LOG(TS_ON, VLEVEL_L, "Failed to open %s, code=%d\r\n", audioFiles[i], res);
            }
        }
    // Register tasks
                                    UTIL_SEQ_RegTask(1 << CFG_SEQ_Task_SD_Process, CFG_SEQ_Prio_0, SD_Process);
                                    UTIL_SEQ_RegTask(1 << CFG_SEQ_Task_DAC_Process, CFG_SEQ_Prio_0, DAC_Process);
                                    UTIL_SEQ_RegTask(1 << CFG_SEQ_Task_Start_Playback, CFG_SEQ_Prio_0, Start_Playback);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  MX_SubGHz_Phy_Process();


	  UTIL_SEQ_Run(UTIL_SEQ_DEFAULT);

    /* USER CODE BEGIN 3 */
  }


}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void MX_DAC_Init(void)
{
	DAC_ChannelConfTypeDef sConfig = {0};

	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
}


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
	  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
/* USER CODE END 4 */
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 14;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}
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
