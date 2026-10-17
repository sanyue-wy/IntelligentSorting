/**
 * @file    pid.c
 * @brief   统一 PID 控制器(实现层)
 *          参考 USTC-RoboWalker alg_pid 与 Wang Hongxi / Zhang Hongyu 的
 *          controller.h 改写, 自包含、无外部库依赖
 * @version 1.0
 * @date    2026-09-25
 */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
  ******************************************************************************
  * 两份参考的取舍:
  * 1. 以 controller.h 的 Ctrl_PID 骨架为主: Improve 位掩码开关、
  *    P/I/D 分项可见、逐项限幅;
  * 2. 合入 alg_pid 的特色功能: 死区(带内目标拉到测量值防累积)、
  *    变速积分(A/B 线性衰减)、积分分离阈值、设定值步进前馈 Kf;
  * 3. alg_pid 的 D_First 与 controller.h 的 Derivative_On_Measurement
  *    是同一功能, 统一为后者一个开关;
  * 4. 显式 ControlPeriod 取代参考实现的 DWT 循环计数测 dt,
  *    结果可复现、不依赖 bsp_dwt; 未引入 arm_math / OLS / 模糊表。
  ******************************************************************************
  */

/* Private function declarations ---------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  PID 初始化(核心参数)
  */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float max_out, float max_iout, float control_period)
{
  if (pid == NULL)
  {
    return;
  }

  *pid = (PID_Controller_t){0};

  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->MaxOut = max_out;
  pid->IntegralLimit = max_iout;
  /* 时间片必须为正, 否则微分项除零 */
  pid->ControlPeriod = (control_period > 0.0f) ? control_period : 0.001f;
}

/**
  * @brief  复位 PID 运行态
  */
void PID_Clear(PID_Controller_t *pid)
{
  if (pid == NULL)
  {
    return;
  }
  pid->ITerm       = 0.0f;
  pid->Err         = 0.0f;
  pid->Last_Err    = 0.0f;
  pid->Last_Measure= 0.0f;
  pid->Last_Output = 0.0f;
  pid->Last_Dout   = 0.0f;
  pid->Pout        = 0.0f;
  pid->Dout        = 0.0f;
  pid->Fout        = 0.0f;
  pid->Output      = 0.0f;
}

/**
  * @brief  计算一步 PID
  * @param  pid     控制器实例
  * @param  measure 当前测量值
  * @param  ref     目标值
  * @retval 输出值
  */
float PID_Calculate(PID_Controller_t *pid, float measure, float ref)
{
  float error;
  float abs_error;
  float ratio;
  float p_out;
  float d_out;
  float f_out;
  float out;
  float alpha;

  if (pid == NULL)
  {
    return 0.0f;
  }

  pid->Measure = measure;
  pid->Ref = ref;
  error = ref - measure;
  abs_error = FLOAT_ABS(error);

  /* 死区: 误差置零并把目标拉到测量值, 防止带内小误差持续累积(仿 alg_pid) */
  if (pid->DeadBand > 0.0f && abs_error <= pid->DeadBand)
  {
    pid->Ref = measure;
    error = 0.0f;
    abs_error = 0.0f;
  }
  pid->Err = error;

  /* P 项 */
  if ((pid->Improve & PID_IMPROVE_PROPORTIONAL_ON_MEASUREMENT) != 0)
  {
    /* 用上拍目标值: 设定值阶跃时 P 不跳变, 消除设定值冲击 */
    p_out = pid->Kp * (pid->Last_Ref - measure);
  }
  else
  {
    p_out = pid->Kp * error;
  }

  /* I 项 */
  if (pid->I_Separate_Threshold > 0.0f && abs_error >= pid->I_Separate_Threshold)
  {
    pid->ITerm = 0.0f; /* 积分分离: 大误差时积分清零(仿 alg_pid) */
  }
  else
  {
    ratio = 1.0f;
    if ((pid->Improve & PID_IMPROVE_CHANGING_INTEGRATION_RATE) != 0 &&
        pid->I_Variable_Speed_B > 0.0f)
    {
      if (abs_error > pid->I_Variable_Speed_A + pid->I_Variable_Speed_B)
      {
        ratio = 0.0f; /* 误差过大, 停止积分 */
      }
      else if (abs_error > pid->I_Variable_Speed_A)
      {
        ratio = 1.0f - (abs_error - pid->I_Variable_Speed_A) / pid->I_Variable_Speed_B;
      }
    }

    if ((pid->Improve & PID_IMPROVE_TRAPEZOID_INTEGRAL) != 0)
    {
      pid->ITerm += pid->Ki * ratio * pid->ControlPeriod * (error + pid->Last_Err) * 0.5f;
    }
    else
    {
      pid->ITerm += pid->Ki * ratio * pid->ControlPeriod * error;
    }

    if ((pid->Improve & PID_IMPROVE_INTEGRAL_LIMIT) != 0 && pid->IntegralLimit > 0.0f)
    {
      LIMIT_MIN_MAX(pid->ITerm, -pid->IntegralLimit, pid->IntegralLimit);
    }
  }

  /* D 项 */
  if ((pid->Improve & PID_IMPROVE_DERIVATIVE_ON_MEASUREMENT) != 0)
  {
    d_out = -pid->Kd * (measure - pid->Last_Measure) / pid->ControlPeriod;
  }
  else
  {
    d_out = pid->Kd * (error - pid->Last_Err) / pid->ControlPeriod;
  }

  if ((pid->Improve & PID_IMPROVE_DERIVATIVE_FILTER) != 0 && pid->Derivative_LPF_RC > 0.0f)
  {
    alpha = pid->ControlPeriod / (pid->Derivative_LPF_RC + pid->ControlPeriod);
    d_out = pid->Last_Dout + alpha * (d_out - pid->Last_Dout);
  }
  pid->Dout = d_out;

  /* F 项: 设定值步进前馈(仿 alg_pid) */
  f_out = pid->Kf * (pid->Ref - pid->Last_Ref);
  pid->Fout = f_out;

  /* 总输出 */
  out = p_out + pid->ITerm + pid->Dout + f_out;

  if ((pid->Improve & PID_IMPROVE_OUTPUT_FILTER) != 0 && pid->Output_LPF_RC > 0.0f)
  {
    alpha = pid->ControlPeriod / (pid->Output_LPF_RC + pid->ControlPeriod);
    out = pid->Last_Output + alpha * (out - pid->Last_Output);
  }

  if (pid->MaxOut > 0.0f)
  {
    LIMIT_MIN_MAX(out, -pid->MaxOut, pid->MaxOut);
  }

  pid->Pout = p_out;
  pid->Output = out;

  /* 更新历史 */
  pid->Last_Measure = measure;
  pid->Last_Ref = pid->Ref;
  pid->Last_Err = error;
  pid->Last_Dout = pid->Dout;
  pid->Last_Output = out;

  return out;
}

/**
  * @brief  积分清零
  */
void PID_Reset_Integral(PID_Controller_t *pid)
{
  if (pid != NULL)
  {
    pid->ITerm = 0.0f;
  }
}

/**
  * @brief  获取本拍输出值
  */
float PID_Get_Out(const PID_Controller_t *pid)
{
  return (pid != NULL) ? pid->Output : 0.0f;
}

/**
  * @brief  获取积分项
  */
float PID_Get_ITerm(const PID_Controller_t *pid)
{
  return (pid != NULL) ? pid->ITerm : 0.0f;
}

/**
  * @brief  获取本拍误差
  */
float PID_Get_Err(const PID_Controller_t *pid)
{
  return (pid != NULL) ? pid->Err : 0.0f;
}
