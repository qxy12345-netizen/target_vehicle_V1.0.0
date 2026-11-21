#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

/**
  ******************************************************************************
  * @file    struct_typedef.h
  * @brief   RoboMaster 开发板C型例程使用的公共类型与工具宏
  *          对应官方例程中的 struct_typedef.h + user_lib.h 的精简合集
  ******************************************************************************
  */

#include "stdint.h"

/* ---------------------------------------------------------------------------
 * RM 例程统一使用的浮点类型
 * ------------------------------------------------------------------------- */
typedef float   fp32;
typedef double  fp64;

/* ---------------------------------------------------------------------------
 * 限幅宏：把 x 限制在 [-max, +max] 内
 * 注意：宏不带分号，调用时需要写 LimitMax(in, max);
 * ------------------------------------------------------------------------- */
#define LimitMax(input, max)        \
    {                               \
        if ((input) > (max))        \
        {                           \
            (input) = (max);        \
        }                           \
        else if ((input) < -(max))  \
        {                           \
            (input) = -(max);       \
        }                           \
    }

/* 把 x 限制在 [min, max] 内，等价于 RM 例程的 fp32_constrain */
#define LimitMinMax(input, min, max) \
    {                                \
        if ((input) > (max))         \
        {                            \
            (input) = (max);         \
        }                            \
        else if ((input) < (min))    \
        {                            \
            (input) = (min);         \
        }                            \
    }

#endif /* STRUCT_TYPEDEF_H */
