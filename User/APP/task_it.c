//
#include "task_it.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "dji_3508.h"

/* ========================================================================== */
/*  Chassis 模块尚未落地: 弱符号占位, 真实实现(强符号)落地后自动覆盖         */
/* ========================================================================== */
__attribute__((weak)) void Chassis_YawControl(void) {}
__attribute__((weak)) void Chassis_Run(void) {}

/**
  * @brief  统一注册定时中断: 所有周期任务都在这里挂到 drv_tim 的节拍上
  * @note   在 MX_TIM7_Init 与 Driver_CAN1_Init/Driver_UART1_Init 之后调用
  *         (见 debug_init); 分频参数 = 每多少个 1ms 节拍调用一次
  */
void Task_IT_Init(void)
{
    Driver_TIM7_Init();  /* TIM7 1ms 系统节拍(登记 -> 重算时间基 -> 开中断) */

    /* ---- 1ms 硬件定时器节拍: 仅保留轻量级 LED 心跳与驱动层通信节拍,
       所有复杂浮点控制(MS_XY_Tick / DJ_MotorRun / StepMotor_Run)均交由
       FreeRTOS 独立硬实时控制任务 MotorCtrlTask 执行, 彻底解除 ISR 阻塞 ---- */
    TIM_Timer_Subscribe(&tim7_timer, MM_TIM_Callback, 1);
    TIM_Timer_Subscribe(&tim7_timer, UART_Port_Tick, 1);   /* drv_uart 周期发送 */
    TIM_Timer_Subscribe(&tim7_timer, CAN_Bus_Tick, 1);     /* drv_can 周期发送与队列推进 */
}

/* 毫秒定时器 */
void MM_TIM_Callback(void)
{
    static uint16_t count;
    /***********0.1秒计时器***********/
    if (++count >= 99)
    {
        /***********程序运行指示灯************/
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10); /* PH10 每 0.1s 翻转, 肉眼验证 1ms 节拍 */

        count = 0;
    }
}

void Task_it_callback(void)
{
    /* 电机实时控制逻辑已迁移至 FreeRTOS MotorCtrlTask 任务 */
}
