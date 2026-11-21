/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

/* 电调 CAN 标识符（C620 数据手册）：
   发送 0x200 一帧同时控制 1~4 号电调，接收 0x201~0x204 是各电调的反馈 */
#define CAN_CHASSIS_ALL_ID  0x200
#define CAN_3508_M1_ID      0x201
#define CAN_3508_M2_ID      0x202
#define CAN_3508_M3_ID      0x203
#define CAN_3508_M4_ID      0x204

/* 电流设定值范围 [-16384, 16384]，线性对应 [-20A, 20A] */
#define MAX_MOTOR_CAN_CURRENT 16384

/* 电机反馈数据结构体（教程文档 14.4.3 小节） */
typedef struct
{
    uint16_t ecd;            /* 转子机械角度 0~8191 */
    int16_t  speed_rpm;      /* 转子转速 rpm */
    int16_t  given_current;  /* 实际转矩电流 */
    uint8_t  temperate;      /* 电机温度 */
    int16_t  last_ecd;       /* 上一次的转子机械角度 */
} motor_measure_t;

/* 反馈解码：高低八位拼接。带符号，反转时速度才是负值 */
#define get_motor_measure(ptr, data)                                    \
{                                                                       \
    (ptr)->last_ecd = (ptr)->ecd;                                       \
    (ptr)->ecd = (int16_t)(((uint16_t)(data)[0] << 8) | (uint16_t)(data)[1]);  \
    (ptr)->speed_rpm = (int16_t)(((uint16_t)(data)[2] << 8) | (uint16_t)(data)[3]);  \
    (ptr)->given_current = (int16_t)(((uint16_t)(data)[4] << 8) | (uint16_t)(data)[5]);  \
    (ptr)->temperate = (data)[6];                                       \
}

extern motor_measure_t motor_chassis[4];        /* CAN1 上电调的反馈数据 */
extern motor_measure_t motor_chassis_can2[4];   /* CAN2 上电调的反馈数据 */

/* 最后一次收到反馈的时刻，用于判断电调是否掉线（两条总线各一个） */
extern volatile uint32_t can_last_fdb_tick;
extern volatile uint32_t can2_last_fdb_tick;

/* 接收诊断：进中断的帧数 + 最后一帧的 ID（排障用） */
extern volatile uint32_t can_rx_count;
extern volatile uint32_t can2_rx_count;
extern volatile uint32_t can_last_std_id;
extern volatile uint32_t can2_last_std_id;

/* 帧格式诊断：IDE 0=标准帧 1=扩展帧，DLC 数据长度，id_log 最近 8 帧的 ID */
extern volatile uint32_t can2_last_ide;
extern volatile uint32_t can2_last_dlc;
extern volatile uint32_t can2_last_ext_id;
extern volatile uint32_t can2_id_log[8];
extern volatile uint8_t  can2_id_log_idx;

/**
  * @brief  经 CAN1 向 1~4 号电调下发转矩电流
  * @param  motor1~motor4 电流控制值 [-16384, 16384]
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief  经 CAN2 向 1~4 号电调下发转矩电流，用法同 CAN_cmd_chassis
  */
void CAN_cmd_chassis_can2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

