/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern CAN_HandleTypeDef hcan1;

/* 电调 ID 与反馈数组下标的对应关系（反馈帧 0x201 -> 下标 0，0x202 -> 下标 1） */
#define MOTOR_INDEX        0U      /* CAN1 上的 ID=1 -> motor_chassis[0]，按键调速 */
#define CAN2_M1_INDEX      0U      /* CAN2 上的 ID=1 -> motor_chassis_can2[0]，反转 */
#define CAN2_M2_INDEX      1U      /* CAN2 上的 ID=2 -> motor_chassis_can2[1]，正转 */

/* CAN2 上两个电机的目标转速 rpm，绝对值都是 900：
   ID=1 反转取负、ID=2 正转取正，这是各自的初始方向；
   跑起来之后每 MOTOR_FLIP_PERIOD_MS 由 can2_m1_dir / can2_m2_dir 翻符号。
   （C620 反馈的是电机轴转速，没过减速箱） */
#define CAN2_M1_TARGET_SPEED_RPM  -900.0f
#define CAN2_M2_TARGET_SPEED_RPM   900.0f

/* 速度环 PID 参数。控制周期 1ms，Ki 每周期累加一次 */
#define SPEED_PID_KP       5.0f
#define SPEED_PID_KI       0.01f
#define SPEED_PID_KD       0.0f
#define SPEED_PID_MAX_OUT  8000.0f    /* 输出限幅，约 9.8A */
#define SPEED_PID_MAX_IOUT 3000.0f    /* 积分限幅 */

/* 超过 100ms 没收到电调反馈就停机 */
#define FDB_TIMEOUT_MS     100U

/* CAN2 上 ID=2 的电机：每隔这么久自动翻转一次运行方向 */
#define MOTOR_FLIP_PERIOD_MS  10000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static pid_type_def speed_pid;                          /* 速度环 PID */
static const fp32 speed_pid_param[3] = {
    SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD
};

volatile fp32 speed_fdb    = 0.0f;   /* 反馈转速，Watch 里看这个 */
volatile fp32 give_current = 0.0f;   /* PID 算出的电流值 */
volatile fp32 TARGET_SPEED_RPM = 0.0f;   /* 类型改对，中断里要读，加 volatile */
static GPIO_PinState key_last = GPIO_PIN_SET;   /* 上一次按键电平，放 PV 区 */

/* ===== CAN2 上的两个电机：固定 ±900rpm，各自独立闭环 ===== */
static pid_type_def speed_pid_c2m1;      /* CAN2 的 ID=1，反转 */
static pid_type_def speed_pid_c2m2;      /* CAN2 的 ID=2，正转 */

volatile fp32 speed_fdb_c2m1    = 0.0f;
volatile fp32 speed_fdb_c2m2    = 0.0f;
volatile fp32 give_current_c2m1 = 0.0f;
volatile fp32 give_current_c2m2 = 0.0f;

/* ID=2 定时换向：当前方向(+1/-1)、上次翻转时刻、已翻转次数，Watch 里都能直接看 */
volatile int8_t   can2_m2_dir = 1;
volatile uint32_t can2_m2_last_flip_tick = 0;
volatile uint32_t can2_m2_flip_count = 0;

/* ID=1 定时换向，结构同上 */
volatile int8_t   can2_m1_dir = 1;
volatile uint32_t can2_m1_last_flip_tick = 0;
volatile uint32_t can2_m1_flip_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  can_filter_init();          /* 启动 CAN，不开滤波器/不 Start 电调收不到 */

  PID_init(&speed_pid, PID_POSITION, speed_pid_param,
           SPEED_PID_MAX_OUT, SPEED_PID_MAX_IOUT);

  /* CAN2 上两个电机，固定 ±900rpm，沿用同一组 PID 参数 */
  PID_init(&speed_pid_c2m1, PID_POSITION, speed_pid_param,
           SPEED_PID_MAX_OUT, SPEED_PID_MAX_IOUT);
  PID_init(&speed_pid_c2m2, PID_POSITION, speed_pid_param,
           SPEED_PID_MAX_OUT, SPEED_PID_MAX_IOUT);

  HAL_TIM_Base_Start_IT(&htim6);   /* 启动 1kHz 控制节拍，闭环在中断里跑 */

 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    GPIO_PinState key_now = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    /* 下降沿：上次是高(松开)，这次是低(按下) */
    if (key_last == GPIO_PIN_SET && key_now == GPIO_PIN_RESET)
    {
        HAL_Delay(20);                                  /* 消抖 */
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
            TARGET_SPEED_RPM += 1000.0f;
        }
    }
    key_last = key_now;

    if (TARGET_SPEED_RPM > 5000.0f)
    {
        TARGET_SPEED_RPM = 0.0f;
    }

    HAL_Delay(10);

    /* CAN2 两个电机的闭环全在 TIM6 中断里跑，这里不再发指令：
       主循环 10ms 一拍跟 1kHz 中断抢发会让电流值打架。 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  TIM6 每 1ms 进一次这个回调，速度环闭环就在这里跑
  * @note   PID 要求控制周期固定，所以不能用 HAL_Delay 拼周期
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == NULL || htim->Instance != TIM6)
    {
        return;
    }

    /* 掉线保护：反馈断了还继续算 PID，积分会一路饱和，总线恢复瞬间电机飞车。
       所以超时没收到反馈就清 PID、发 0 电流。 */
    /* ---- CAN1 掉线：ID=1 停机 ---- */
#if  1	/* CAN1 暂未接电机，整段禁用。接上时把 0 改成 1 */
	if (can_last_fdb_tick == 0U ||
        (HAL_GetTick() - can_last_fdb_tick) > FDB_TIMEOUT_MS)
    {
        PID_clear(&speed_pid);
        speed_fdb    = 0.0f;
        give_current = 0.0f;
        CAN_cmd_chassis(0, 0, 0, 0);
    }
    else
    {
        /* 取反馈转速 -> PID 算电流 -> 发给电调，闭环完成 */
        speed_fdb    = (fp32)motor_chassis[MOTOR_INDEX].speed_rpm;
        give_current = PID_calc(&speed_pid, speed_fdb, TARGET_SPEED_RPM);
        if(give_current <= 0)
        {
            give_current = 0 ;
        }
        CAN_cmd_chassis((int16_t)give_current, 0, 0, 0);
    }
#endif

    /* ---- CAN2 掉线：两个电机停机 ---- */
    if (can2_last_fdb_tick == 0U ||
        (HAL_GetTick() - can2_last_fdb_tick) > FDB_TIMEOUT_MS)
    {
        PID_clear(&speed_pid_c2m1);
        PID_clear(&speed_pid_c2m2);
        speed_fdb_c2m1    = 0.0f;
        speed_fdb_c2m2    = 0.0f;
        give_current_c2m1 = 0.0f;
        give_current_c2m2 = 0.0f;
        CAN_cmd_chassis_can2(0, 0, 0, 0);

        /* 排障：掉线时刷新 CAN2 硬件状态，Watch 里看 can2_lec / can2_tec */
        can2_msr = hcan2.Instance->MSR;
        can2_esr = hcan2.Instance->ESR;
        can2_lec = (uint8_t)((can2_esr >> 4) & 0x07U);
        can2_tec = (uint8_t)((can2_esr >> 16) & 0xFFU);
        can2_rec = (uint8_t)((can2_esr >> 8) & 0xFFU);
        return;
    }

    /* ---- ID=1：和 ID=2 一样定时换向 ---- */
    if (HAL_GetTick() - can2_m1_last_flip_tick >= MOTOR_FLIP_PERIOD_MS)
    {
        can2_m1_last_flip_tick = HAL_GetTick();
        can2_m1_dir = -can2_m1_dir;
        can2_m1_flip_count++;
        PID_clear(&speed_pid_c2m1);
    }

    /* ---- CAN2 的 ID=1：目标 ±900rpm，正负由 can2_m1_dir 决定 ---- */
    speed_fdb_c2m1    = (fp32)motor_chassis_can2[CAN2_M1_INDEX].speed_rpm;
    give_current_c2m1 = PID_calc(&speed_pid_c2m1, speed_fdb_c2m1,
                                 (fp32)can2_m1_dir * CAN2_M1_TARGET_SPEED_RPM);

    /* ---- ID=2：到点就把方向翻过来，初始方向不变 ----
       换向时清一次 PID：积分项里攒的还是上一个方向的值，
       不清的话反转会被它拖慢一段时间。 */
    if (HAL_GetTick() - can2_m2_last_flip_tick >= MOTOR_FLIP_PERIOD_MS)
    {
        can2_m2_last_flip_tick = HAL_GetTick();
        can2_m2_dir = -can2_m2_dir;
        can2_m2_flip_count++;
        PID_clear(&speed_pid_c2m2);
    }

    /* ---- CAN2 的 ID=2：目标 ±900rpm，正负由 can2_m2_dir 决定 ---- */
    speed_fdb_c2m2    = (fp32)motor_chassis_can2[CAN2_M2_INDEX].speed_rpm;
    give_current_c2m2 = PID_calc(&speed_pid_c2m2, speed_fdb_c2m2,
                                 (fp32)can2_m2_dir * CAN2_M2_TARGET_SPEED_RPM);

    /* 0x200 帧里第 1 组字节给 ID=1、第 2 组给 ID=2，其余留 0 */
    CAN_cmd_chassis_can2((int16_t)give_current_c2m1,
                         (int16_t)give_current_c2m2,
                         0, 0);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
