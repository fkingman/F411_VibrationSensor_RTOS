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

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "arm_math.h"
#include "KX134.h"
#include "FreeRTOS.h"
#include "semphr.h"
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
#define KX134_TRIG_Pin GPIO_PIN_1
#define KX134_TRIG_GPIO_Port GPIOA
#define KX134_INT2_Pin GPIO_PIN_2
#define KX134_INT2_GPIO_Port GPIOA
#define KX134_INT2_EXTI_IRQn EXTI2_IRQn
#define KX134_INT1_Pin GPIO_PIN_3
#define KX134_INT1_GPIO_Port GPIOA
#define KX134_INT1_EXTI_IRQn EXTI3_IRQn
#define KX134_CS_Pin GPIO_PIN_4
#define KX134_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define UART_RX_BUF_SIZE  1024
#define FFT_POINTS  4096
#define AXIS_COUNT  3
extern int16_t g_SensorRawBuffer[2][FFT_POINTS * AXIS_COUNT];
extern float g_FftCalcBuffer[FFT_POINTS];
typedef struct {
    uint8_t write_index; // 当前 DMA 正在写哪
    uint8_t read_index;  // 当前 算法 正在读哪
    volatile uint8_t data_ready_flag; // 标志位：1表示有一半数据准备好
} PingPong_Mgr_t;//乒乓状�??
extern PingPong_Mgr_t g_PingPongMgr;

extern uint8_t rx_dma_buf[UART_RX_BUF_SIZE];  
extern uint8_t g_UartRxBuffer[UART_RX_BUF_SIZE];
extern volatile uint16_t g_UartRxLen;//实际接收字节
void Uart1_RxStart(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
