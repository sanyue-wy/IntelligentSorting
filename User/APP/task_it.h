#ifndef task_it_H
#define task_it_H
#include  "drv_can.h"


/* 统一注册定时中断: 所有周期任务在此挂到 drv_tim 的 1ms 节拍上 */
void Task_IT_Init(void);

void MM_TIM_Callback(void);
void TASK_1MS_TIM_callback(void);
void Task_it_callback(void);

#endif //task_it_H