/**
 * @file    feedforward.h
 * @brief   二阶前馈控制器(接口层)
 *          参考 Wang Hongxi / Zhang Hongyu controller.h 中的 Feedforward_t 改写,
 *          导数估计由 OLS/ARM-DSP 改为差分 + 一阶低通, 去除外部库依赖
 * @version 1.0
 * @date    2026-09-25
 */

#ifndef __FEEDFORWARD_H__
#define __FEEDFORWARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  二阶前馈对象: Output = C0*Ref + C1*Ref_Dot + C2*Ref_Ddot
  */
typedef struct
{
  /* 传递系数 */
  float C0;
  float C1;
  float C2;

  float ControlPeriod; /* 计算周期(秒), Feedforward_Init 保证 > 0 */

  /* 目标值及其导数估计 */
  float Ref;
  float Last_Ref;
  float Ref_Dot;
  float Last_Ref_Dot;
  float Ref_Ddot;

  /* 导数一阶低通时间常数(秒), 0 = 不滤波 */
  float Dot_LPF_RC;
  float Ddot_LPF_RC;

  float MaxOut; /* 输出限幅, 0 = 不限 */
  float Output;
} Feedforward_t;

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  前馈初始化
  * @param  ffc            前馈实例, 由调用方静态分配
  * @param  c0/c1/c2       传递系数
  * @param  max_out        输出限幅, 0 = 不限
  * @param  control_period 计算周期(秒), 非法值回退为 0.001
  * @note   首拍目标值导数按相对 0 估计, 若首个目标值较大,
  *         输出会被 MaxOut 限幅属正常现象, 也可先调用 Feedforward_Reset
  */
void Feedforward_Init(Feedforward_t *ffc, float c0, float c1, float c2,
                      float max_out, float control_period);

/**
  * @brief  计算一步前馈
  * @param  ffc 前馈实例
  * @param  ref 目标值
  * @retval 输出值
  */
float Feedforward_Calculate(Feedforward_t *ffc, float ref);

/**
  * @brief  清导数历史(目标值跳变后调用可避免虚假微分)
  */
void Feedforward_Reset(Feedforward_t *ffc);

#ifdef __cplusplus
}
#endif

#endif /* __FEEDFORWARD_H__ */
