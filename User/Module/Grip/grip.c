/**
 * @file    grip.c
 * @brief   夹爪执行机构(双舵机)实现: 机构语义 -> 角度, 角度 -> 脉宽交给 servo 层
 *
 *          舵机1 (GRIP_Z_SERVO_CH)  : 夹爪 Z 轴升降
 *          舵机2 (GRIP_JAW_SERVO_CH): 夹爪开合
 *
 *          两轴各自记"目标角度 + 动作完成时刻": 无位置反馈, 用 HAL_GetTick
 *          近似判到位(查询时惰性老化, 不订阅节拍中断)。状态只服务上层
 *          (host_comm 上报 / debug 核对), 对输出无影响 —— 输出始终是
 *          下发即改目标的开环舵机行为。
 */

/* Includes ------------------------------------------------------------------*/
#include "grip.h"
#include "servo.h"
#include "drv_tim.h"

/* ========================================================================== */
/*  私有变量                                                                   */
/* ========================================================================== */

static uint8_t s_inited;

/* ---- Z 轴(升降, 舵机1) ---- */
static uint16_t s_z_angle;       /* 最近下发的角度                      */
static uint32_t s_z_busy_until;  /* 动作完成时刻(HAL_GetTick 口径)       */
static uint8_t  s_z_busy;        /* 有动作在飞(到期由 Grip_AgeBusy 清)   */

/* ---- 夹爪(开合, 舵机2) ---- */
static uint16_t s_jaw_angle;     /* 最近下发的角度 */
static uint8_t  s_jaw_open;      /* 最近动作朝向: 1=张开, 0=闭合 */
static uint8_t  s_jaw_force;     /* 最近一次闭合力度 0~100 */
static uint32_t s_jaw_busy_until;
static uint8_t  s_jaw_busy;

/* ========================================================================== */
/*  私有函数                                                                   */
/* ========================================================================== */

/**
  * @brief  惰性老化: 到期即清忙碌标志(查询/下发前调用, 无需节拍中断)
  * @note   时间比较用有符号差值, HAL_GetTick 回绕(49.7 天)后仍正确
  */
static void Grip_AgeBusy(void)
{
  uint32_t now = SysTime_GetMs();

  if (s_z_busy && ((int32_t)(now - s_z_busy_until) >= 0))
  {
    s_z_busy = 0U;
  }
  if (s_jaw_busy && ((int32_t)(now - s_jaw_busy_until) >= 0))
  {
    s_jaw_busy = 0U;
  }
}

/* 起一次动作计时(两轴共用) */
static void Grip_ArmTimer(uint32_t *until, uint8_t *busy, uint32_t duration_ms)
{
  *until = SysTime_GetMs() + duration_ms;
  *busy  = 1U;
}

/* ========================================================================== */
/*  对外接口                                                                   */
/* ========================================================================== */

/**
  * @brief  初始化夹爪机构(幂等)
  * @note   含一次 Servo_Init; 上电姿态 = Z 抬起 + 夹爪张开
  */
void Grip_Init(void)
{
  if (s_inited)
  {
    return;
  }

  Servo_Init(); /* TIM1 CH1~CH3 起 50Hz 输出(servo 层幂等, Tray 也会调, 无副作用) */

  s_z_angle   = GRIP_Z_SAFE_DEG;
  s_z_busy    = 0U;
  s_jaw_angle = GRIP_JAW_OPEN_DEG;
  s_jaw_open  = 1U;
  s_jaw_force = 0U;
  s_jaw_busy  = 0U;
  s_inited    = 1U;

  /* 上电姿态: Z 抬起不挡 XY 行程, 夹爪张开(与 host_comm 上报初值"0=开"一致) */
  (void)Grip_MoveZ(GRIP_Z_SAFE_DEG);
  (void)Grip_Open();
}

/**
  * @brief  Z 轴移动到绝对角度(越界钳位, 与 servo 层钳位口径一致)
  */
Grip_Result_e Grip_MoveZ(uint16_t angle)
{
  if (!s_inited)
  {
    return GRIP_ERR_NOT_INIT;
  }

  if (angle > SERVO_ANGLE_270_MAX)
  {
    angle = SERVO_ANGLE_270_MAX;
  }

  Servo_SetAngle_270(GRIP_Z_SERVO_CH, (int16_t)angle);
  s_z_angle = angle;
  Grip_ArmTimer(&s_z_busy_until, &s_z_busy, GRIP_Z_MOVE_MS);
  return GRIP_OK;
}

Grip_Result_e Grip_ZUp(void)
{
  return Grip_MoveZ(GRIP_Z_UP_DEG);
}

Grip_Result_e Grip_ZDown(void)
{
  return Grip_MoveZ(GRIP_Z_DOWN_DEG);
}

/**
  * @brief  夹爪张开
  */
Grip_Result_e Grip_Open(void)
{
  if (!s_inited)
  {
    return GRIP_ERR_NOT_INIT;
  }

  Servo_SetAngle_270(GRIP_JAW_SERVO_CH, (int16_t)GRIP_JAW_OPEN_DEG);
  s_jaw_angle = GRIP_JAW_OPEN_DEG;
  s_jaw_open  = 1U;
  s_jaw_force = 0U;
  Grip_ArmTimer(&s_jaw_busy_until, &s_jaw_busy, GRIP_JAW_MOVE_MS);
  return GRIP_OK;
}

/**
  * @brief  夹爪闭合到指定力度
  * @note   力度越界不静默钳位: 拒绝下发并返回 GRIP_ERR_PARAM(**不改变当前输出**)
  */
Grip_Result_e Grip_Close(uint8_t force)
{
  uint16_t angle;

  if (!s_inited)
  {
    return GRIP_ERR_NOT_INIT;
  }
  if (force > 100U)
  {
    return GRIP_ERR_PARAM;
  }

  angle = Grip_CloseAngle(force);
  Servo_SetAngle_270(GRIP_JAW_SERVO_CH, (int16_t)angle);
  s_jaw_angle = angle;
  s_jaw_open  = 0U;
  s_jaw_force = force;
  Grip_ArmTimer(&s_jaw_busy_until, &s_jaw_busy, GRIP_JAW_MOVE_MS);
  return GRIP_OK;
}

/**
  * @brief  力度 -> 闭合角度: 行程比例 = FLOOR + (1-FLOOR)×force/100
  */
uint16_t Grip_CloseAngle(uint8_t force)
{
  float frac;
  float deg;

  if (force > 100U)
  {
    force = 100U;
  }

  frac = GRIP_JAW_FORCE_FLOOR +
         (1.0f - GRIP_JAW_FORCE_FLOOR) * ((float)force / 100.0f);
  deg = (float)GRIP_JAW_OPEN_DEG +
        ((float)GRIP_JAW_CLOSE_DEG - (float)GRIP_JAW_OPEN_DEG) * frac;
  return (uint16_t)deg;
}

bool Grip_ZIsBusy(void)
{
  Grip_AgeBusy();
  return (s_z_busy != 0U);
}

bool Grip_JawIsBusy(void)
{
  Grip_AgeBusy();
  return (s_jaw_busy != 0U);
}

bool Grip_IsBusy(void)
{
  Grip_AgeBusy();
  return ((s_z_busy != 0U) || (s_jaw_busy != 0U));
}

Grip_JawState_e Grip_GetJawState(void)
{
  Grip_AgeBusy();

  if (!s_inited)
  {
    return GRIP_JAW_UNKNOWN;
  }
  if (s_jaw_busy)
  {
    return (s_jaw_open != 0U) ? GRIP_JAW_OPENING : GRIP_JAW_CLOSING;
  }
  if (s_jaw_open)
  {
    return GRIP_JAW_OPEN;
  }
  /* 闭合到位: 力度>0 按开环口径视为"夹住", =0 只是合上(未夹持) */
  return (s_jaw_force > 0U) ? GRIP_JAW_HELD : GRIP_JAW_CLOSED;
}

uint16_t Grip_GetZAngle(void)
{
  return s_z_angle;
}

uint16_t Grip_GetJawAngle(void)
{
  return s_jaw_angle;
}