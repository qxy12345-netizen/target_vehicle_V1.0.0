/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* CAN1 上挂 ID=1 的电调，CAN2 上挂 ID=1、ID=2 两个电调。
   两条总线各用一套反馈数组，互不干扰。 */
motor_measure_t motor_chassis[4];           /* CAN1 反馈 */
motor_measure_t motor_chassis_can2[4];      /* CAN2 反馈 */

static uint8_t  chassis_can_send_data[8];   /* CAN1 发送缓冲区 */
static uint8_t  can2_send_data[8];          /* CAN2 发送缓冲区 */
static uint8_t  rx_data[8];                 /* 接收缓冲区 */

/* 最后一次收到反馈的时刻，掉线保护用（两条总线各一个） */
volatile uint32_t can_last_fdb_tick  = 0;
volatile uint32_t can2_last_fdb_tick = 0;

/* 接收诊断：进中断的帧数 + 最后一帧的 ID（排障用，稳定后可删） */
volatile uint32_t can_rx_count     = 0;
volatile uint32_t can2_rx_count    = 0;
volatile uint32_t can_last_std_id  = 0;
volatile uint32_t can2_last_std_id = 0;

/* 帧格式诊断：ID 一直是 0 时用它判断到底收到了什么
   can2_last_ide  0=标准帧  1=扩展帧（扩展帧时 StdId 不会被赋值，会显示为 0）
   can2_last_dlc  数据长度，C620 的反馈帧固定是 8
   can2_id_log[]  最近 8 帧的 ID，环形缓冲，看总线上都有哪些 ID */
volatile uint32_t can2_last_ide = 0xFF;
volatile uint32_t can2_last_dlc = 0;
volatile uint32_t can2_last_ext_id = 0;   /* 扩展帧的 ID，标准帧时这里是 0 */
volatile uint32_t can2_id_log[8] = {0};
volatile uint8_t  can2_id_log_idx = 0;

/**
  * @brief  装填 0x200 电流帧（四个电调的电流，高八位在前）
  */
static void can_pack_current(uint8_t buf[8], int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    buf[0] = (uint8_t)(m1 >> 8);
    buf[1] = (uint8_t)(m1);
    buf[2] = (uint8_t)(m2 >> 8);
    buf[3] = (uint8_t)(m2);
    buf[4] = (uint8_t)(m3 >> 8);
    buf[5] = (uint8_t)(m3);
    buf[6] = (uint8_t)(m4 >> 8);
    buf[7] = (uint8_t)(m4);
}

/**
  * @brief  经 CAN1 向电调下发转矩电流（教程文档 14.4.2 小节）
  * @param  motor1~motor4 电流控制值 [-16384, 16384]，对应 [-20A, 20A]
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box = 0;
    CAN_TxHeaderTypeDef tx_message;

    tx_message.StdId  = CAN_CHASSIS_ALL_ID;   /* 0x200 -> 电调 ID 1~4 */
    tx_message.ExtId  = 0;
    tx_message.IDE    = CAN_ID_STD;
    tx_message.RTR    = CAN_RTR_DATA;
    tx_message.DLC    = 0x08;
    tx_message.TransmitGlobalTime = DISABLE;

    can_pack_current(chassis_can_send_data, motor1, motor2, motor3, motor4);

    HAL_CAN_AddTxMessage(&hcan1, &tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief  经 CAN2 向电调下发转矩电流，用法同 CAN_cmd_chassis
  * @note   只用到了 ID=1、ID=2，所以 motor3 / motor4 传 0
  */
void CAN_cmd_chassis_can2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box = 0;
    CAN_TxHeaderTypeDef tx_message;

    tx_message.StdId  = CAN_CHASSIS_ALL_ID;
    tx_message.ExtId  = 0;
    tx_message.IDE    = CAN_ID_STD;
    tx_message.RTR    = CAN_RTR_DATA;
    tx_message.DLC    = 0x08;
    tx_message.TransmitGlobalTime = DISABLE;

    can_pack_current(can2_send_data, motor1, motor2, motor3, motor4);

    HAL_CAN_AddTxMessage(&hcan2, &tx_message, can2_send_data, &send_mail_box);
}
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief  CAN 接收中断回调（教程文档 14.4.3 小节）
  * @note   CAN1/CAN2 共用这一个弱回调，先判断是哪个外设进来的
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header = {0};   /* 必须清零：扩展帧时 StdId 不会被赋值 */
    motor_measure_t *dst;
    volatile uint32_t *tick;

    if (hcan == NULL)
    {
        return;
    }

    /* CAN1 的反馈存进 motor_chassis，CAN2 的存进 motor_chassis_can2 */
    if (hcan->Instance == CAN1)
    {
        dst  = motor_chassis;
        tick = &can_last_fdb_tick;
    }
    else if (hcan->Instance == CAN2)
    {
        dst  = motor_chassis_can2;
        tick = &can2_last_fdb_tick;
    }
    else
    {
        return;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }

    /* ---- 接收诊断：不管 ID 是什么先进来一帧就计数 + 记下 ID ----
       排障时看 can2_rx_count：
         一直 0      -> 中断压根没进，查物理层 / 滤波器 / 引脚
         在涨但数组仍全 0 -> 收到了帧，但 ID 不在 0x201~0x204，看 can2_last_std_id 是多少 */
    if (hcan->Instance == CAN1)
    {
        can_rx_count++;
        can_last_std_id = rx_header.StdId;
    }
    else
    {
        can2_rx_count++;
        can2_last_std_id = rx_header.StdId;
        can2_last_ide    = (rx_header.IDE == CAN_ID_STD) ? 0U : 1U;
        can2_last_ext_id = rx_header.ExtId;
        can2_last_dlc    = rx_header.DLC;
        can2_id_log[can2_id_log_idx] = rx_header.StdId;
        can2_id_log_idx = (uint8_t)((can2_id_log_idx + 1U) & 7U);
    }

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:    /* 0x201 -> 下标 0，以此类推 */
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        {
            uint8_t i = (uint8_t)(rx_header.StdId - CAN_3508_M1_ID);
            get_motor_measure(&dst[i], rx_data);

            /* 记录收到反馈的时刻，控制环据此判断电调是否掉线 */
            *tick = HAL_GetTick();
            break;
        }
        default:
        {
            break;
        }
    }
}

/* USER CODE END 1 */

