#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  ******************************************************************************
  * @file    pid.h
  * @brief   PID 控制器，移植自《RoboMaster开发板C型嵌入式软件教程文档》
  *          第 16 章 16.4.3 / 16.4.4 小节的 pid_type_def + PID_init + PID_calc
  ******************************************************************************
  */

#include "struct_typedef.h"

/* PID 工作模式 */
typedef enum
{
    PID_POSITION = 0,   /* 位置式 PID：u(k) = Kp*e(k) + Ki*Σe(i) + Kd*[e(k)-e(k-1)] */
    PID_DELTA,          /* 增量式 PID：Δu(k) = Kp*Δe + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)] */
} pid_mode_e;

/* PID 控制器结构体 */
typedef struct _pid
{
    uint8_t mode;       /* PID 模式，取值见 pid_mode_e */

    /* PID 三参数 */
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;       /* 最大输出限幅 */
    fp32 max_iout;      /* 积分项最大限幅 */

    fp32 set;           /* 目标值 */
    fp32 fdb;           /* 反馈值 */
    fp32 out;           /* 本次输出值 */

    fp32 Pout;          /* 比例项输出 */
    fp32 Iout;          /* 积分项输出 */
    fp32 Dout;          /* 微分项输出 */

    fp32 Dbuf[3];       /* 微分项误差缓存：0最新 1上一次 2上上次 */
    fp32 error[3];      /* 误差缓存       ：0最新 1上一次 2上上次 */
} pid_type_def;

/**
  * @brief  初始化 PID 控制器，把全部状态量清零
  * @param  pid      PID 结构体指针
  * @param  mode     PID_POSITION 或 PID_DELTA
  * @param  PID      长度为 3 的数组，依次为 Kp Ki Kd
  * @param  max_out  最大输出
  * @param  max_iout 积分上限
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief  PID 计算
  * @param  pid  PID 结构体指针
  * @param  ref  反馈值 feedback
  * @param  set  目标值 setpoint
  * @retval 本次 PID 输出
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief  清空 PID 历史状态（积分累积、误差与微分缓存）
  * @param  pid PID 结构体指针
  */
void PID_clear(pid_type_def *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
