/**
 * @file    td.h
 * @brief   跟踪微分器(接口层)
 *          参考 Wang Hongxi / Zhang Hongyu controller.h 中的 TD_t 改写
 * @version 1.0
 * @date    2026-09-25
 */

#ifndef __TD_H__
#define __TD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  跟踪微分器对象: 对输入进行平滑跟踪, 同时给出一/二阶导数
  * @note   采用临界阻尼二阶线性形式 X'' = R^2*(Input - X) - 2R*X'
  *         (参考实现为非线性 fhan 形式, 需要开方库函数; 线性形式行为
  *         可预期、无超拍、无 libm 依赖)。稳定性要求 R * ControlPeriod <= 1
  */
typedef struct
{
  float Input;        /* 本拍输入(待跟踪信号) */
  float R;            /* 跟踪速度(带宽): 越大跟踪越快, 微分噪声也越大 */
  float ControlPeriod;/* 计算周期(秒), TD_Init 保证 > 0 */

  float X;            /* 跟踪值(平滑后的输入) */
  float DX;           /* 一阶导数(估计) */
  float DDX;          /* 二阶导数(估计) */
} TD_t;

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  跟踪微分器初始化
  * @param  td             实例, 由调用方静态分配
  * @param  r              跟踪速度, 需满足 r * control_period <= 1
  * @param  control_period 计算周期(秒), 非法值回退为 0.001
  */
void TD_Init(TD_t *td, float r, float control_period);

/**
  * @brief  计算一步跟踪
  * @param  td    实例
  * @param  input 本拍输入
  * @retval 平滑后的跟踪值(同 td->X)
  */
float TD_Calculate(TD_t *td, float input);

#ifdef __cplusplus
}
#endif

#endif /* __TD_H__ */
