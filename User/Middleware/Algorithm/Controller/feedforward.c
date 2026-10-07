/**
 * @file    feedforward.c
 * @brief   二阶前馈控制器(实现层)
 * @version 1.0
 * @date    2026-09-25
 */

/* Includes ------------------------------------------------------------------*/
#include "feedforward.h"
#include "pid.h" /* FLOAT_ABS / LIMIT_MIN_MAX */

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  前馈初始化
  */
void Feedforward_Init(Feedforward_t *ffc, float c0, float c1, float c2,
                      float max_out, float control_period)
{
  if (ffc == NULL)
  {
    return;
  }

  *ffc = (Feedforward_t){0};

  ffc->C0 = c0;
  ffc->C1 = c1;
  ffc->C2 = c2;
  ffc->MaxOut = max_out;
  ffc->ControlPeriod = (control_period > 0.0f) ? control_period : 0.001f;
}

/**
  * @brief  计算一步前馈
  * @note   导数用后向差分估计(参考实现用 OLS), 可选一阶低通抑噪
  */
float Feedforward_Calculate(Feedforward_t *ffc, float ref)
{
  float dot;
  float ddot;
  float out;
  float alpha;

  if (ffc == NULL)
  {
    return 0.0f;
  }

  ffc->Ref = ref;

  /* 一阶导: 后向差分 + 可选低通 */
  dot = (ref - ffc->Last_Ref) / ffc->ControlPeriod;
  if (ffc->Dot_LPF_RC > 0.0f)
  {
    alpha = ffc->ControlPeriod / (ffc->Dot_LPF_RC + ffc->ControlPeriod);
    dot = ffc->Ref_Dot + alpha * (dot - ffc->Ref_Dot);
  }

  /* 二阶导: 对(滤波后的)一阶导再差分 + 可选低通 */
  ddot = (dot - ffc->Last_Ref_Dot) / ffc->ControlPeriod;
  if (ffc->Ddot_LPF_RC > 0.0f)
  {
    alpha = ffc->ControlPeriod / (ffc->Ddot_LPF_RC + ffc->ControlPeriod);
    ddot = ffc->Ref_Ddot + alpha * (ddot - ffc->Ref_Ddot);
  }

  ffc->Ref_Dot = dot;
  ffc->Ref_Ddot = ddot;

  out = ffc->C0 * ref + ffc->C1 * dot + ffc->C2 * ddot;
  if (ffc->MaxOut > 0.0f)
  {
    LIMIT_MIN_MAX(out, -ffc->MaxOut, ffc->MaxOut);
  }
  ffc->Output = out;

  /* 更新历史 */
  ffc->Last_Ref = ref;
  ffc->Last_Ref_Dot = dot;

  return out;
}

/**
  * @brief  清导数历史
  */
void Feedforward_Reset(Feedforward_t *ffc)
{
  if (ffc == NULL)
  {
    return;
  }

  ffc->Last_Ref = ffc->Ref;
  ffc->Last_Ref_Dot = 0.0f;
  ffc->Ref_Dot = 0.0f;
  ffc->Ref_Ddot = 0.0f;
}
