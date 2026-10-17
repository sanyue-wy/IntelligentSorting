/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
#include "host_comm.h"
#include "motion_sys_XY.h"
#include "dji_3508.h"
#include "stepmotor.h"
#include "debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t motorCtrlTaskHandle;
const osThreadAttr_t motorCtrlTask_attributes = {
  .name = "MotorCtrlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartMotorCtrlTask(void *argument);
void StartTelemetryTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  motorCtrlTaskHandle = osThreadNew(StartMotorCtrlTask, NULL, &motorCtrlTask_attributes);
  telemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &telemetryTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread (AppFsmTask).
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  App_Init();
  /* Infinite loop */
  for(;;)
  {
    HostComm_Poll();
    App_Poll();
    osDelay(APP_POLL_INTERVAL_MS);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  1kHz (1ms) 硬实时电机控制任务: CoreXY 轨迹插补与绝对脉冲流式下发 + 大疆 3508 闭环控制
  */
void StartMotorCtrlTask(void *argument)
{
  uint32_t wake_tick = osKernelGetTickCount();
  uint32_t div3_cnt = 0;

  for (;;)
  {
    wake_tick += 1U;

    /* 1ms: CoreXY 双轴轨迹插补与步进位置流式下发 (替代原 TIM7 ISR 浮点运算) */
    MotionSys_XY_Tick();

    /* 3ms 周期: 大疆 3508 串级 TD-PID 计算及 CAN 发送 + 步进电机状态轮询 (与 DJ_CTRL_PERIOD 0.003f 精确匹配) */
    if (++div3_cnt >= 3U)
    {
      div3_cnt = 0U;
      DJ_MotorRun();
      StepMotor_Run();
    }

    osDelayUntil(wake_tick);
  }
}

/**
  * @brief  20ms 低优先级遥测与调试任务: 串口输出与调试诊断
  */
void StartTelemetryTask(void *argument)
{
  for (;;)
  {
    debug_run();
    osDelay(20U);
  }
}

/**
  * @brief  FreeRTOS 栈溢出保护钩子
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  __disable_irq();
  for (;;)
  {
    /* 硬件级死锁挂起, 防止破坏相邻内存 */
  }
}

/* USER CODE END Application */

