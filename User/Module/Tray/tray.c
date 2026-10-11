/**
 * @file    tray.c
 * @brief   旋转托盘模块实现: 机构语义(角度) -> 舵机角度 -> 脉宽交给 servo 层
 *
 *          单舵机驱动托盘旋转, 只有旋转一个自由度。状态只服务上层
 *          (上报 / debug 核对), 对输出无影响 —— 输出始终是"下发即改目标"的
 *          开环舵机行为: 不做加减速规划, 也不跟踪轨迹(与 MotionSys_XY 的
 *          规划+流式下发不同, 舵机由内部电路自己走完行程)。
 *
 *          到位判定: 无位置反馈, 用 HAL_GetTick 按动作时长近似
 *          (查询时惰性老化, 不订阅节拍中断)。时长随转角增大 ——
 *          固定时长在大角度下会把"还在转"误判成"已到位"。
 */

/* Includes ------------------------------------------------------------------*/
#include "tray.h"
#include "servo.h"

/* ========================================================================== */
/*  私有变量                                                                   */
/* ========================================================================== */

static uint8_t  s_inited;
static uint16_t s_angle;      /* 最近下发的目标角度(开环: 即认为到位后的角度) */
static uint8_t  s_busy;       /* 有动作在飞(到期由 Tray_AgeBusy 清)            */
static uint32_t s_busy_until; /* 动作完成时刻(HAL_GetTick 口径)                */

/* ========================================================================== */
/*  私有函数                                                                   */
/* ========================================================================== */

/**
  * @brief  惰性老化: 到期即清忙碌标志(查询/下发前调用, 无需节拍中断)
  * @note   时间比较用有符号差值, HAL_GetTick 回绕(49.7 天)后仍正确
  */
static void Tray_AgeBusy(void)
{
  uint32_t now = HAL_GetTick();

  if (s_busy && ((int32_t)(now - s_busy_until) >= 0))
  {
    s_busy = 0U;
  }
}

/* 目标角度钳位到行程(越界不静默丢弃, 收进行程内继续执行) */
static uint16_t Tray_Clamp(uint16_t deg)
{
  if (deg < TRAY_MIN_DEG)
  {
    deg = TRAY_MIN_DEG;
  }
  if (deg > TRAY_MAX_DEG)
  {
    deg = TRAY_MAX_DEG;
  }
  return deg;
}

/* 起一次动作计时: 时长随转角增大(见 tray.h 说明) */
static void Tray_ArmTimer(uint16_t delta_deg)
{
  s_busy_until = HAL_GetTick() + Tray_MoveDurationMs(delta_deg);
  s_busy       = 1U;
}

/**
  * @brief  下发目标角度(已 Init 前提)
  * @note   目标与当前一致则不下发(省一次脉宽改写); 否则改目标并按转角起计时
  */
static Tray_Result_e Tray_Apply(uint16_t deg)
{
  uint16_t delta;

  deg = Tray_Clamp(deg);
  if (deg == s_angle)
  {
    return TRAY_OK; /* 已在目标角度: 舵机保持, 不必再写一次 CCR */
  }

  delta = (deg > s_angle) ? (uint16_t)(deg - s_angle) : (uint16_t)(s_angle - deg);

  Servo_SetAngle_270(TRAY_SERVO_CH, (int16_t)deg);
  s_angle = deg;
  Tray_ArmTimer(delta);
  return TRAY_OK;
}

/* ========================================================================== */
/*  对外接口                                                                   */
/* ========================================================================== */

/**
  * @brief  初始化托盘机构(幂等)
  * @note   含一次 Servo_Init; 上电姿态 = TRAY_HOME_DEG
  */
void Tray_Init(void)
{
  if (s_inited)
  {
    return;
  }

  Servo_Init(); /* TIM1 CH1~CH3 起 50Hz 输出(servo 层幂等, Grip 也会调) */

  s_angle  = Tray_Clamp(TRAY_HOME_DEG);
  s_inited = 1U;

  /* 上电姿态: 直接下发, 不走"目标未变则跳过"的快路径 —— 上电时舵机实际
     停在哪未知, 必须真的给一次目标脉宽把它拉到 home。
     计时按满行程估(保守): 避免上层刚 Init 就以为已经到位 */
  Servo_SetAngle_270(TRAY_SERVO_CH, (int16_t)s_angle);
  Tray_ArmTimer((uint16_t)(TRAY_MAX_DEG - TRAY_MIN_DEG));
}

Tray_Result_e Tray_RotateTo(uint16_t deg)
{
  if (!s_inited)
  {
    return TRAY_ERR_NOT_INIT;
  }
  return Tray_Apply(deg);
}

Tray_Result_e Tray_RotateRel(int16_t delta_deg)
{
  int32_t target;

  if (!s_inited)
  {
    return TRAY_ERR_NOT_INIT;
  }
  if (delta_deg == 0)
  {
    return TRAY_ERR_PARAM;
  }

  /* 相对转动基于"最近下发角度"(开环口径), 结果钳位到行程内 */
  target = (int32_t)s_angle + (int32_t)delta_deg;
  if (target < (int32_t)TRAY_MIN_DEG)
  {
    target = (int32_t)TRAY_MIN_DEG;
  }
  if (target > (int32_t)TRAY_MAX_DEG)
  {
    target = (int32_t)TRAY_MAX_DEG;
  }

  return Tray_Apply((uint16_t)target);
}

Tray_Result_e Tray_Home(void)
{
  if (!s_inited)
  {
    return TRAY_ERR_NOT_INIT;
  }
  return Tray_Apply(TRAY_HOME_DEG);
}

bool Tray_IsBusy(void)
{
  Tray_AgeBusy();
  return (s_busy != 0U);
}

Tray_State_e Tray_GetState(void)
{
  Tray_AgeBusy();

  if (!s_inited)
  {
    return TRAY_IDLE;
  }
  return (s_busy != 0U) ? TRAY_MOVING : TRAY_IDLE;
}

uint16_t Tray_GetAngle(void)
{
  return s_angle;
}

/**
  * @brief  角度增量 -> 动作时长: BASE + PER_DEG × delta, 上限 MAX
  */
uint32_t Tray_MoveDurationMs(uint16_t delta_deg)
{
  uint32_t ms = (uint32_t)TRAY_MOVE_BASE_MS +
                (uint32_t)TRAY_MS_PER_DEG * (uint32_t)delta_deg;

  if (ms > (uint32_t)TRAY_MOVE_MAX_MS)
  {
    ms = (uint32_t)TRAY_MOVE_MAX_MS;
  }
  return ms;
}