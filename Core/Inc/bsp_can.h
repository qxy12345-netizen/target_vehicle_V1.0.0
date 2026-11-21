#ifndef BSP_CAN_H
#define BSP_CAN_H

/**
  ******************************************************************************
  * @file    bsp_can.h
  * @brief   CAN 启动：配置滤波器、启动 CAN1 / CAN2，打开接收中断
  ******************************************************************************
  */

#include "stdint.h"

void can_filter_init(void);

/* 启动状态：1 = 已进入 Normal 模式并开了接收中断，0 = 没起来。
   调试时在 Watch 里看这两个就能知道哪条总线通了 */
extern volatile uint8_t can1_ready;
extern volatile uint8_t can2_ready;

/* CAN2 硬件状态快照（排障用）：MSR 看是否还卡在初始化模式，ESR 看错误码和错误计数 */
extern volatile uint32_t can2_start_ret;
extern volatile uint32_t can2_msr;
extern volatile uint32_t can2_esr;

/* 从 ESR 拆好的两个数：can2_lec 3=ACK错误(没人应答)，can2_tec 一直在涨同理 */
extern volatile uint8_t  can2_lec;
extern volatile uint8_t  can2_tec;
extern volatile uint8_t  can2_rec;

#endif /* BSP_CAN_H */
