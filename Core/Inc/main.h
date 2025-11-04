/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#define AUDIO_FILE_COUNT 2

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern DAC_HandleTypeDef hdac;
extern DMA_HandleTypeDef hdma_dac_out1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

//extern uint8_t currentAudioIndex;

extern const char *audioFiles[];
//extern const uint8_t AUDIO_FILE_COUNT;
extern uint8_t currentAudioIndex;
//extern void myprintf(const char *fmt, ...);
//extern uint16_t convertedData[6000];
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART_BAUDRATE 115200
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define USART_Tx_Pin GPIO_PIN_6
#define USART_Tx_GPIO_Port GPIOB
#define USART__Rx_Pin GPIO_PIN_7
#define USART__Rx_GPIO_Port GPIOB
#define FE_CTRL2_Pin GPIO_PIN_8
#define FE_CTRL2_GPIO_Port GPIOB
//#define LED1_Pin GPIO_PIN_5
//#define LED1_GPIO_Port GPIOA
//#define LED2_Pin GPIO_PIN_6
//#define LED2_GPIO_Port GPIOA
#define FE_CTRL3_Pin GPIO_PIN_9
#define FE_CTRL3_GPIO_Port GPIOA
#define FE_CTRL1_Pin GPIO_PIN_13
#define FE_CTRL1_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_1
#define SPI1_CS_GPIO_Port GPIOA

//macros for seekbuttons
#define BUTTON_FWD_GPIO_Port GPIOA
#define BUTTON_FWD_Pin GPIO_PIN_9
#define BUTTON_BWD_GPIO_Port GPIOA
#define BUTTON_BWD_Pin GPIO_PIN_8

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
