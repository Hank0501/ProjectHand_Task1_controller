/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.h
 * @brief   This file contains all the function prototypes for
 *          the can.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
  void CAN1_FilterInit(uint32_t FilterBank, uint32_t FilterMode, uint32_t FilterScale,
                       uint32_t FilterIdHigh, uint32_t FilterIdLow, uint32_t MaskIdHigh, uint32_t MaskIdLow,
                       uint32_t FilterFIFOAssignment, uint32_t FilterActivation);
  void CAN_TxheaderSetup(CAN_TxHeaderTypeDef *Txheader, uint32_t StdId, uint32_t ExtId, uint32_t IDE, uint32_t RTR, uint32_t DLC, FunctionalState TransmitGlobalTime);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

