/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* SPI2 软件片选（PB12）的收发宏，给 ota_spi.c 用。
 * ★ 必须放在 USER CODE 块里：CubeMX 重新生成时只保留 USER CODE 之间的内容，
 *   写在块外面会被悄悄抹掉 —— 而症状是"编译报 OTA_SPI_CS_LOW 未定义"，
 *   排查方向很容易被带偏到 SPI 代码上去。
 * ★ CubeMX 只为普通 GPIO 标签生成上面那两行 _Pin/_GPIO_Port，
 *   _LOW/_HIGH 得自己补 —— 它不知道这根线是当片选用的。
 * ★ 名字用 OTA_SPI_ 前缀（而不是这里的 SPI2_ 前缀）：它是**OTA 通道**的片选，
 *   SPI2 只是实现手段；ota_spi.c 就是这么调的，两边必须一致。 */
#define OTA_SPI_CS_LOW()  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define OTA_SPI_CS_HIGH() HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
