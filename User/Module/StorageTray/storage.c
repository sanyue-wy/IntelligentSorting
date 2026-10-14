/**
 * @file    storage.c
 * @brief   旋转储物盘定位模块实现: 槽位 -> 角度 -> 3508 角度环
 *
 *          核心是把"转到第 N 格"翻译成 3508 的绝对角度指令, 并给出可靠的
 *          到位判定。三个关键点:
 *
 *          1) 角度口径
 *             以 3508 上电首帧编码器位置为 0°, 直接用驱动的 total_angle。
 *             槽位映射: angle = ZERO_OFFSET + DIR_SIGN × slot × 60°
 *
 *          2) 最短路径
 *             目标角度在绕圈累加口径下不唯一(±360° 的整数倍), 这里从当前
 *             角度出发选与之一致的最近等价角, 使 |Δ| <= 180°。
 *             例: 槽 5 (300°) -> 槽 0 (0°), 就近取 360° 而非 0°,
 *                 盘只反向走 60° 而不是正向 300°。
 *
 *          3) 到位判定
 *             位置误差 + 速度 + 连续保持时间三条件同时满足才认账 —— 单看
 *             位置会在过冲瞬间误判; 单看"PID 输出为 0"在无反馈时也会成立。
 *             无 CAN 反馈(no_rx_count 超时)一律判"未到位", 由超时兜底进
 *             FAULT, 绝不把"没收到反馈"报成"已到位"。
 */

/* Includes ------------------------------------------------------------------*/
#include "storage.h"
#include "drv_tim.h"

/* ========================================================================== */
/*  私有宏                                                                     */
/* ========================================================================== */

/* 槽位角: 相邻槽位间的角度(度) */
#define STORAGE_SLOT_STEP   (STORAGE_SLOT_ANGLE_DEG)
/* 整圈 */
#define STORAGE_FULL_TURN   360.0f

/* 把任意角度折算到 [-180, 180) 的等价角(用于取最短路径增量)。
   注意 floor 语义: C 的整数除法向零截断, 负数会算错(如 -360 得 -360 而非 0),
   故这里显式做向下取整除法 —— 储物盘 cur 会常态为负(槽 3 就近取 -180°,
   反向增量旋转也为负), 不能靠截断。 */
static int32_t Storage_FloorDiv(float a, float b)
{
  int32_t q = (int32_t)(a / b);
  if ((a - (float)q * b) < 0.0f)
  {
    q--; /* a/b 为负且有余数: 截断结果比真值大 1, 补回向下取整 */
  }
  return q;
}

#define STORAGE_WRAP_HALF(a) \
  ((a) - STORAGE_FULL_TURN * (float)Storage_FloorDiv((a) + 180.0f, STORAGE_FULL_TURN))

/* ========================================================================== */
/*  私有变量                                                                   */
/* ========================================================================== */

static uint8_t     s_inited;
static DJ_Motor_t *s_motor;        /* 受控电机实例(DJ_Init 过) */

static Storage_State_e s_state;
static uint8_t     s_cur_slot;     /* 逻辑当前槽位(到位时更新) */
static uint8_t     s_tgt_slot;     /* 目标槽位 */
static float       s_tgt_angle;    /* 目标电机角度(度, 已选好绕圈) */
static float       s_last_err;     /* 上一拍误差(诊断用) */

/* 到位判定的"连续保持"计时 */
static uint32_t    s_ok_since;     /* 首次进入容差的时刻(HAL_GetTick) */
static uint8_t     s_ok_active;    /* 已在容差内计时中 */

/* 旋转超时 */
static uint32_t    s_move_start;   /* 本次旋转下发时刻 */

/* ========================================================================== */
/*  私有函数                                                                   */
/* ========================================================================== */

/* 角度 -> 等价角, 落在 [0, 360) */
static float Storage_Norm0To360(float a)
{
  float r = a;
  while (r < 0.0f)
  {
    r += STORAGE_FULL_TURN;
  }
  while (r >= STORAGE_FULL_TURN)
  {
    r -= STORAGE_FULL_TURN;
  }
  return r;
}

static float Storage_AbsF(float v)
{
  return (v < 0.0f) ? -v : v;
}

/**
  * @brief  是否已有可用的角度反馈
  * @note   首帧未到(rx_once=0)或已超时(no_rx_count >= 阈值)都算无反馈。
  *         无反馈时 total_angle 冻结在旧值, 据此判到位会误判, 必须拦住
  */
static bool Storage_FeedbackValid(void)
{
  if (s_motor == NULL)
  {
    return false;
  }
  if (!s_motor->rx_once || s_motor->initialized == 0U)
  {
    return false;
  }
  return (s_motor->no_rx_count < STORAGE_RX_TIMEOUT_POLLS);
}

/**
  * @brief  卸力停机
  * @note   必须切到 SPEED_MODE + 目标速度 0, 而不是只清 setCurrent:
  *         DJ_SetAngle 会把电机置于 ANGLE_MODE, 而 DJ_MotorRun(3ms) 每拍都会
  *         用"角度环 -> 速度环"重算 setCurrent —— 只清电流会被下一拍覆盖,
  *         故障/到位后仍有活的闭环把盘推向旧目标。
  *         速度模式目标 0 且实测速度非 0 时的纠偏电流, 正是"阻尼"效果。
  *         反馈丢失时驱动内部本就置 setCurrent=0, 无需另配
  */
static void Storage_Stop(void)
{
  if (s_motor != NULL)
  {
    s_motor->setAngle = (float)s_motor->total_angle; /* 防恢复角度模式时突跳 */
    s_motor->setSpeed = 0.0f;
    s_motor->mode     = SPEED_MODE;
  }
}

/*
 * 选绕圈: 在目标角度的所有等价角中, 取离当前角度最近的那个(保证 |Δ|<=180°)
 *   cur=300°, base=0°  -> 返回 360° (Δ=+60°) 而非 0° (Δ=-300°)
 *   cur=0°,   base=300°-> 返回 300° (Δ=+300°) 与 -60° 同距, 取 0~360 内的点
 * 关闭多圈时直接返回 [0,360) 内的 base, 每格只走一个点。
 */
static float Storage_PickTargetAngle(uint8_t slot, float cur)
{
  float base = Storage_SlotToAngle(slot);

#if STORAGE_MULTITURN_ENABLE
  {
    float delta = STORAGE_WRAP_HALF(base - cur); /* 最短增量, [-180,180) */
    return cur + delta;                          /* 就近等价角(可跨圈) */
  }
#else
  (void)cur;
  return Storage_Norm0To360(base);
#endif
}

/* 角度误差(度) = 实际 - 目标 */
static float Storage_Err(void)
{
  if (s_motor == NULL)
  {
    return 0.0f;
  }
  return (float)s_motor->total_angle - s_tgt_angle;
}

/**
  * @brief  下发一次绝对角度并起超时计时
  */
static Storage_Result_e Storage_Start(float angle)
{
  if (!Storage_FeedbackValid())
  {
    return STORAGE_ERR_NO_FEEDBACK;
  }

  s_tgt_angle   = angle;
  s_ok_active   = 0U;
  s_move_start  = SysTime_GetMs();
  s_state       = STORAGE_MOVING;

  /* 角度环(TD+PID) -> 速度环 -> 电流, 全部在 DJ_MotorRun 3ms 里跑 */
  DJ_SetAngle(s_motor, angle, STORAGE_SPEED_RPM);
  return STORAGE_OK;
}

/* ========================================================================== */
/*  对外接口                                                                   */
/* ========================================================================== */

void Storage_Init(DJ_Motor_t *motor)
{
  if (s_inited)
  {
    return;
  }

  s_motor     = motor;
  s_state     = STORAGE_IDLE;
  s_cur_slot  = 0U;
  s_tgt_slot  = 0U;
  s_tgt_angle = Storage_SlotToAngle(0U);
  s_ok_active = 0U;
  s_last_err  = 0.0f;
  s_inited    = 1U;

  /* 不清编码器零点: 由 dji_3508 在首帧反馈时自动捕获(此时可能还没到),
     之后 total_angle 即"相对上电位置"的累加角度, 正是本模块要的口径。
     上电姿态按槽位 0 记账, 电机保持卸力不动, 等第一条指令。 */
  Storage_Stop();
}

Storage_Result_e Storage_RotateToSlot(uint8_t slot)
{
  if (!s_inited)
  {
    return STORAGE_ERR_NOT_INIT;
  }
  if (slot >= STORAGE_SLOT_NUM)
  {
    return STORAGE_ERR_PARAM;
  }
  if (s_state == STORAGE_FAULT)
  {
    return STORAGE_ERR_FAULT;
  }
  if (s_state == STORAGE_MOVING)
  {
    return STORAGE_ERR_BUSY;
  }

  s_tgt_slot = slot;

  /* 已在目标槽位(误差在容差内): 不动, 直接记到位 */
  if (Storage_FeedbackValid() && slot == s_cur_slot)
  {
    s_tgt_angle = Storage_PickTargetAngle(slot, (float)s_motor->total_angle);
    if (Storage_AbsF(Storage_Err()) <= STORAGE_ARRIVE_DEG)
    {
      return STORAGE_OK;
    }
  }

  /* 首次下发前必须已经有反馈, 否则"当前角度"是冻结值, 就近选点会错 */
  if (!Storage_FeedbackValid())
  {
    return STORAGE_ERR_NO_FEEDBACK;
  }

  return Storage_Start(Storage_PickTargetAngle(slot, (float)s_motor->total_angle));
}

Storage_Result_e Storage_RotateBySlot(int8_t delta_slot)
{
  float delta_deg;

  if (!s_inited)
  {
    return STORAGE_ERR_NOT_INIT;
  }
  if (s_state == STORAGE_FAULT)
  {
    return STORAGE_ERR_FAULT;
  }
  if (s_state == STORAGE_MOVING)
  {
    return STORAGE_ERR_BUSY;
  }
  if (!Storage_FeedbackValid())
  {
    return STORAGE_ERR_NO_FEEDBACK;
  }
  if (delta_slot == 0)
  {
    return STORAGE_OK;
  }

  /* 相对旋转: 不做就近选点, 严格按符号方向走, 保证转动量与方向可预期 */
  delta_deg = STORAGE_DIR_SIGN * (float)delta_slot * STORAGE_SLOT_STEP;
  s_tgt_angle = (float)s_motor->total_angle + delta_deg;

  /* 目标槽位 = 相对当前槽位的逻辑槽位(只用于查询显示) */
  {
    int16_t s = (int16_t)s_cur_slot + (int16_t)delta_slot;
    while (s < 0)
    {
      s += (int16_t)STORAGE_SLOT_NUM;
    }
    s_tgt_slot = (uint8_t)(s % (int16_t)STORAGE_SLOT_NUM);
  }

  return Storage_Start(s_tgt_angle);
}

Storage_Result_e Storage_SetReference(uint8_t slot)
{
  if (!s_inited)
  {
    return STORAGE_ERR_NOT_INIT;
  }
  if (slot >= STORAGE_SLOT_NUM)
  {
    return STORAGE_ERR_PARAM;
  }
  if (s_state == STORAGE_MOVING)
  {
    return STORAGE_ERR_BUSY;
  }
  if (!Storage_FeedbackValid())
  {
    return STORAGE_ERR_NO_FEEDBACK;
  }

  /* 把当前物理位置定义为 slot: 清零累加角度(零点=当前编码器值),
     并把驱动目标同步到 0, 避免恢复时往旧目标跑 */
  DJ_ClearAngle(s_motor);
  s_motor->setAngle = 0.0f;
  s_cur_slot  = slot;
  s_tgt_slot  = slot;
  s_tgt_angle = Storage_SlotToAngle(slot);
  s_ok_active = 0U;
  s_last_err  = 0.0f;
  s_state     = STORAGE_IDLE;
  return STORAGE_OK;
}

void Storage_ClearFault(void)
{
  if (!s_inited)
  {
    return;
  }

  Storage_Stop();
  s_ok_active = 0U;
  s_last_err  = 0.0f;
  s_state     = STORAGE_IDLE;
  /* 槽位记账保持不动: 故障是"没转到位", 盘的真实姿态未知,
     由上层回零/Storage_SetReference 重新建立口径 */
}

/* ---- 状态与到位判定(惰性推进, 主循环查询即可) ---- */

Storage_State_e Storage_GetState(void)
{
  if (!s_inited)
  {
    return STORAGE_IDLE;
  }

  if (s_state != STORAGE_MOVING)
  {
    return s_state;
  }

  /* --- 反馈丢失: 立即 FAULT 并停机 --- */
  if (!Storage_FeedbackValid())
  {
    if (s_motor != NULL && s_motor->no_rx_count >= STORAGE_RX_TIMEOUT_POLLS)
    {
      Storage_Stop();
      s_state = STORAGE_FAULT;
      return s_state;
    }
    /* 首帧还没来: 不算故障, 继续保持 MOVING 等超时兜底 */
    return s_state;
  }

  /* --- 超时: 转不到位就停机, 不无限较劲 --- */
  if ((SysTime_GetMs() - s_move_start) >= STORAGE_MOVE_TIMEOUT_MS)
  {
    Storage_Stop();
    s_state = STORAGE_FAULT;
    return s_state;
  }

  /* --- 到位三条件: 位置容差 + 速度足够小 + 连续保持 --- */
  {
    float err = Storage_Err();
    float spd = (float)s_motor->speed;

    s_last_err = err;

    if ((Storage_AbsF(err) <= STORAGE_ARRIVE_DEG) &&
        (Storage_AbsF(spd) <= STORAGE_ARRIVE_SPEED))
    {
      if (!s_ok_active)
      {
        s_ok_active = 1U;
        s_ok_since  = SysTime_GetMs();
      }
      else if ((SysTime_GetMs() - s_ok_since) >= STORAGE_ARRIVE_HOLD_MS)
      {
        /* 稳定到位: 锁存当前槽位, 卸力(角度环自带保持, 清零只去静差扭矩) */
        s_state    = STORAGE_IDLE;
        s_cur_slot = s_tgt_slot;
        Storage_Stop();
        return s_state;
      }
    }
    else
    {
      s_ok_active = 0U; /* 跳出容差, 重新计时(防过冲瞬间穿透) */
    }
  }

  return s_state;
}

bool Storage_IsBusy(void)
{
  return (Storage_GetState() == STORAGE_MOVING);
}

bool Storage_IsFault(void)
{
  return (Storage_GetState() == STORAGE_FAULT);
}

bool Storage_HasFeedback(void)
{
  return Storage_FeedbackValid();
}

bool Storage_IsSlotArrived(void)
{
  if (!s_inited)
  {
    return false;
  }
  /* 先推进判定(可能本次调用刚好收敛) */
  if (Storage_GetState() != STORAGE_IDLE)
  {
    return false;
  }
  if (!Storage_FeedbackValid())
  {
    return false; /* 无反馈不认账 */
  }
  return (Storage_AbsF(Storage_Err()) <= STORAGE_ARRIVE_DEG);
}

uint8_t Storage_GetSlot(void)
{
  return s_inited ? s_cur_slot : 0xFFU;
}

float Storage_GetTargetAngle(void)
{
  return s_tgt_angle;
}

float Storage_GetActualAngle(void)
{
  if (s_motor == NULL)
  {
    return 0.0f;
  }
  return (float)s_motor->total_angle;
}

float Storage_GetAngleError(void)
{
  return Storage_Err();
}

/* ========================================================================== */
/*  纯换算                                                                     */
/* ========================================================================== */

float Storage_SlotToAngle(uint8_t slot)
{
  if (slot >= STORAGE_SLOT_NUM)
  {
    slot = (uint8_t)(STORAGE_SLOT_NUM - 1U);
  }
  return STORAGE_ZERO_OFFSET_DEG +
         STORAGE_DIR_SIGN * (float)slot * STORAGE_SLOT_STEP;
}

uint8_t Storage_AngleToSlot(float angle)
{
  float rel = Storage_Norm0To360(STORAGE_DIR_SIGN * (angle - STORAGE_ZERO_OFFSET_DEG));
  uint8_t slot = (uint8_t)((rel / STORAGE_SLOT_STEP) + 0.5f);

  if (slot >= STORAGE_SLOT_NUM)
  {
    slot = 0U; /* 最后一格跨 360° 边界回绕 */
  }
  return slot;
}