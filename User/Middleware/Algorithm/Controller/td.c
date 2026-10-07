/**
 * @file    td.c
 * @brief   跟踪微分器(实现层)
 * @version 1.0
 * @date    2026-09-25
 */

/* Includes ------------------------------------------------------------------*/
#include "td.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  跟踪微分器初始化
  */
void TD_Init(TD_t *td, float r, float control_period)
{
  if (td == NULL)
  {
    return;
  }

  *td = (TD_t){0};

  td->R = (r > 0.0f) ? r : 1.0f;
  td->ControlPeriod = (control_period > 0.0f) ? control_period : 0.001f;
}

/**
  * @brief  计算一步跟踪
  * @note   半隐式欧拉积分: 先更新速度再更新位置, 比显式积分稳定
  */
float TD_Calculate(TD_t *td, float input)
{
  float err;

  if (td == NULL)
  {
    return 0.0f;
  }

  td->Input = input;
  err = input - td->X;

  /* 临界阻尼二阶系统: 无超拍地收敛到输入, 状态量即为平滑导数 */
  td->DDX = td->R * td->R * err - 2.0f * td->R * td->DX;
  td->DX += td->DDX * td->ControlPeriod;
  td->X += td->DX * td->ControlPeriod;

  return td->X;
}
