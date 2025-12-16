/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>


//#include <SEGGER_RTT.h>
//#include <SEGGER_SYSVIEW.h>
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
#define M4_CLK_Pin GPIO_PIN_1
#define M4_CLK_GPIO_Port GPIOA
#define M1_EN_Pin GPIO_PIN_2
#define M1_EN_GPIO_Port GPIOA
#define M2_EN_Pin GPIO_PIN_3
#define M2_EN_GPIO_Port GPIOA
#define M3_EN_Pin GPIO_PIN_4
#define M3_EN_GPIO_Port GPIOA
#define M4_EN_Pin GPIO_PIN_5
#define M4_EN_GPIO_Port GPIOA
#define TRAN1_EN_Pin GPIO_PIN_6
#define TRAN1_EN_GPIO_Port GPIOA
#define TRAN2_EN_Pin GPIO_PIN_7
#define TRAN2_EN_GPIO_Port GPIOA
#define TRAN3_EN_Pin GPIO_PIN_4
#define TRAN3_EN_GPIO_Port GPIOC
#define TRAN4_EN_Pin GPIO_PIN_5
#define TRAN4_EN_GPIO_Port GPIOC
#define M2_CLK_Pin GPIO_PIN_0
#define M2_CLK_GPIO_Port GPIOB
#define TRAN5_EN_Pin GPIO_PIN_1
#define TRAN5_EN_GPIO_Port GPIOB
#define STATE_Pin GPIO_PIN_2
#define STATE_GPIO_Port GPIOB
#define COM_TX_Pin GPIO_PIN_9
#define COM_TX_GPIO_Port GPIOA
#define COM_RX_Pin GPIO_PIN_10
#define COM_RX_GPIO_Port GPIOA
#define COM_CTS_Pin GPIO_PIN_11
#define COM_CTS_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define CTL_TX_Pin GPIO_PIN_10
#define CTL_TX_GPIO_Port GPIOC
#define CTL_RX_Pin GPIO_PIN_11
#define CTL_RX_GPIO_Port GPIOC
#define M1_CLK_Pin GPIO_PIN_3
#define M1_CLK_GPIO_Port GPIOB
#define M3_CLK_Pin GPIO_PIN_6
#define M3_CLK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//#define log_debug(format, ...)       \
//        SEGGER_RTT_printf(0, "[D]: " format "\n", ## __VA_ARGS__)
//#define log_info(format, ...)       \
//        SEGGER_RTT_printf(0, "[I]: " format "\n", ## __VA_ARGS__)
//#define log_error(format, ...)       \
//        SEGGER_RTT_printf(0, "[E]: " format "\n", ## __VA_ARGS__)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
