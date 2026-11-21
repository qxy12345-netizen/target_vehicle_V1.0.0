/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID 控制器实现
  * @note    实现严格对齐《RoboMaster开发板C型嵌入式软件教程文档》16.4.3/16.4.4
  ******************************************************************************
  */

#include "pid.h"
#ifndef NULL
#define NULL    ((void *)0)
#endif

/**
  * @brief  初始化 PID 控制器：设置模式、系数、输出限幅与积分上限，并清零所有状态
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }

    pid->mode     = mode;
    pid->Kp       = PID[0];
    pid->Ki       = PID[1];
    pid->Kd       = PID[2];
    pid->max_out  = max_out;
    pid->max_iout = max_iout;

    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;

    pid->set = 0.0f;
    pid->fdb = 0.0f;
}

/**
  * @brief  PID 计算，按当前模式选用位置式或增量式公式
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    /* 误差历史递推：error[0] 最新，error[1] 上一次，error[2] 上上次 */
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set      = set;
    pid->fdb      = ref;
    pid->error[0] = set - ref;

    if (pid->mode == PID_POSITION)
    {
        /* u(k) = Kp*e(k) + Ki*Σe(i) + Kd*[e(k) - e(k-1)] */
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];

        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        /* Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k) - 2e(k-1) + e(k-2)] */
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];

        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }

    return pid->out;
}

/**
  * @brief  清空 PID 历史状态，用于切换控制目标或反馈掉线后的保护
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->out = 0.0f;
    pid->set = pid->fdb = 0.0f;
}
