/**
  ******************************************************************************
  * @file    bsp_can.c
  * @brief   CAN 启动：滤波器 + Start + 接收中断
  ******************************************************************************
  */

#include "bsp_can.h"
#include "main.h"
#include "can.h"

/**
  * @brief  配置滤波器并启动 CAN1 / CAN2，打开接收中断
  * @note   MX_CANx_Init() 只做初始化，不 Start 的话 CAN 停在
  *         Initialization 模式，AddTxMessage 会一直失败。
  *         CAN1 挂 ID=1 的电调，CAN2 挂 ID=1、ID=2 两个电调。
  */
/* CAN 启动状态：1 = 已进入 Normal 模式并开了接收中断，0 = 没起来。
   调试时在 Watch 里看这两个，就知道哪条总线通了 */
volatile uint8_t can1_ready = 0;
volatile uint8_t can2_ready = 0;

/* CAN2 硬件状态快照（排障用，通了以后可删）：
   can2_start_ret  HAL_CAN_Start 的返回值，1=超时失败（总线上没节点）
   can2_msr        bit0 INAK=1 说明还卡在初始化模式，根本不发不收
   can2_esr        bit6:4 LEC 最后错误码：3=ACK 错误(无人应答)
                   bit23:16 TEC 发送错误计数：一直涨说明帧没人接
                   bit15:8  REC 接收错误计数 */
volatile uint32_t can2_start_ret = 0;
volatile uint32_t can2_msr       = 0;
volatile uint32_t can2_esr       = 0;

/* 从 ESR 里拆出来的两个数，Watch 里直接看值不用自己算：
   can2_lec  最后错误码：0=无错 1=填充 2=格式 3=ACK错误(发出去没人应答)
             5=位显性 6=位隐性 7=CRC
   can2_tec  发送错误计数：只要在涨，就说明帧发出去没人 ACK */
volatile uint8_t  can2_lec = 0xFF;   /* 0xFF = 还没采过 */
volatile uint8_t  can2_tec = 0;

/* can2_rec 接收错误计数，判断 RX 引脚上到底有没有信号：
   一直 0        -> RX 引脚上没有有效电平活动（线没接 / 收发器没工作 / 电调没上电）
   大于 0 且在涨 -> 线上确实有信号但解不出来（波特率对不上 / 电平异常） */
volatile uint8_t  can2_rec = 0;

/* ============================================================================
 * 临时自检开关：CAN2 回环模式(Loopback)
 * 设为 1 时，CAN2 发出的帧不经过外部引脚，直接被自己接收。用来判定
 * "MCU 的 CAN2 外设 + 接收中断 + 滤波器"这一整套是不是好的：
 *     can2_rx_count 开始涨 -> MCU 侧完全正常，问题 100% 在外部
 *                             (供电 / 接线 / 终端电阻 / 电调模式)
 *     can2_rx_count 还是 0 -> MCU 侧配置有问题
 * 注意：回环模式下电调收不到任何东西，测完必须改回 0！
 *
 * 已改回 0（正常模式）。判断 CAN2 到底通没通，别只看 can2_ready：
 * 退出初始化模式只要求总线出现 11 位隐性电平，线悬空也算空闲，
 * 所以没接电调时 HAL_CAN_Start 一样会成功。
 * 可靠判据是 can2_lec / can2_tec / can2_rx_count：
 *   can2_lec == 3 且 can2_tec 持续上涨 -> 帧发出去了但没人应答，物理层不通
 *   can2_rx_count 一直 0               -> 电调的帧没进到 RX 引脚
 * ==========================================================================*/
#define CAN2_LOOPBACK_TEST   0

void can_filter_init(void)
{
    /* 结构体必须整体清零，否则 SlaveStartFilterBank 等成员是脏数据，
       会被写进 CAN1->FMR 导致配置失败 */
    CAN_FilterTypeDef can_filter_st = {0};

    /* 掩码全 0 = 放过所有报文 */
    can_filter_st.FilterActivation     = ENABLE;
    can_filter_st.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh         = 0x0000;
    can_filter_st.FilterIdLow          = 0x0000;
    can_filter_st.FilterMaskIdHigh     = 0x0000;
    can_filter_st.FilterMaskIdLow      = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_st.SlaveStartFilterBank = 14;    /* 0~13 给 CAN1，14~27 给 CAN2 */

    /* ---------- CAN1：滤波器 bank 0 ----------
       重要：CAN1 上没接电调时，HAL_CAN_Start 会超时失败（总线上没节点，
       等不到退出初始化模式的确认）。这里有两个坑：
         1. 不能进 Error_Handler —— 那是 __disable_irq() + while(1)，
            一旦进去中断全关，CAN2 也彻底没戏；
         2. 不能 return —— CAN2 的配置写在这段后面，必须继续往下走。
       所以只记录状态，失败就跳过。 */
    can_filter_st.FilterBank = 0;
    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) == HAL_OK &&
        HAL_CAN_Start(&hcan1) == HAL_OK &&
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
    {
        can1_ready = 1;
    }

    /* ---------- CAN2：滤波器必须从 bank 14 起，否则会被 CAN1 抢走 ----------
       同理，CAN2 没接电调时也不能进 Error_Handler。 */
    can_filter_st.FilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan2, &can_filter_st) != HAL_OK)
    {
        return;
    }

    /* 开自动离线恢复(Bus-Off 自恢复)：CAN2 上没接电调时发送拿不到 ACK，
       错误计数累满会进 Bus-Off 并永久停发。开了 ABOM 后，等电调接上
       硬件能自己退出 Bus-Off，不用重启单片机。 */
    SET_BIT(hcan2.Instance->MCR, CAN_MCR_ABOM);

#if CAN2_LOOPBACK_TEST
    /* 回环自检：此时 CAN2 还在初始化模式(INRQ=1)，BTR 可写。
       置 LBKM 后发出的帧会被自己收回，不驱动外部引脚。 */
    SET_BIT(hcan2.Instance->BTR, CAN_BTR_LBKM);
#endif

    can2_start_ret = (uint32_t)HAL_CAN_Start(&hcan2);
    if (can2_start_ret != (uint32_t)HAL_OK)
    {
        can2_msr = hcan2.Instance->MSR;   /* 失败时抓一次，看是不是还卡在 INAK */
        return;
    }
    can2_msr = hcan2.Instance->MSR;
    can2_esr = hcan2.Instance->ESR;
    can2_lec = (uint8_t)((can2_esr >> 4) & 0x07U);
    can2_tec = (uint8_t)((can2_esr >> 16) & 0xFFU);
    can2_rec = (uint8_t)((can2_esr >> 8) & 0xFFU);

    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
    {
        can2_ready = 1;
    }
}
