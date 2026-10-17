/**
 * @file    motion_sys_XY.c
 * @brief   CoreXY XY 轴运动系统实现(仿 Voron 2.4 / Klipper)
 *
 *          ===== 与 Klipper 源码的对应关系(学习索引) =====
 *            A=X+Y, B=X-Y 变换      <- klippy/kinematics/corexy.py
 *                                      (chelper/kin_corexy.c: return c.x+c.y / c.x-c.y)
 *            MS_XY_CalcJunction      <- toolhead.py Move.calc_junction()
 *                                      (junction deviation 近似向心力限速)
 *            MS_XY_Replan            <- toolhead.py LookAheadQueue.flush()
 *                                      (反向扫描: 队尾假定停车 -> 逐段回推入口速度;
 *                                       正向扫描: 限制加速可达性 -> 求巡航速度)
 *            MS_XY_SetJunction       <- toolhead.py Move.set_junction()
 *                                      (加速度段/巡航段/减速度段距离与时间分解)
 *            MS_XY_Tick 求值+流式下发<- toolhead trapq + src/stepper.c 的
 *                                      "按时刻产生位置" 思想, 步进脉冲流替换为
 *                                      CAN QPos 绝对位置流(本机执行层)
 *
 *          ===== 与 Voron/Klipper 的差异(有意为之) =====
 *            1. Voron: MCU 发 step/dir 脉冲给 TMC2209, 每脉冲时刻由硬件定时;
 *               本机: STM32 -> CAN -> ZDT 闭环步进, 按 1ms 节拍流式下发绝对
 *               目标, 由驱动器闭环跟随(位置目标是绝对量, 单帧丢失下一拍自愈)。
 *            2. Klipper 规划在主机(RPi), 执行在 MCU; 本模块两层同在一个
 *               STM32: 规划在主循环(MoveTo 入队时), 执行在 TIM7 中断。
 *            3. 未实现: 回零(homing)/input_shaper/加急停止以外的限位逻辑,
 *                对应 Voron cfg 的 homing_* 与 [input_shaper] 段。
 *
 *          ===== 本机约束(务必遵守) =====
 *            - Emm_V5_* 命令构造用文件级 static 缓冲: 同一函数只能在一个
 *              上下文调用。QPos 流只在 Tick(中断)里发; Enable/Stop/Clear
 *              等只在主循环发(两者是不同函数, 不同缓冲, 可并存)。
 *            - CAN1 仅 3 个发送邮箱且非阻塞: 每拍成对下发 2 帧(留 1 帧给
 *              0x220/S_CPOS 周期帧); 发送前查空邮箱数, 不足则本拍跳过,
 *              绝对目标下下一拍自然补上。
 *            - 规划队列在主循环入队、中断出队, 两者都在 PRIMASK 临界区内
 *              操作共享状态。
 */

/* Includes ------------------------------------------------------------------*/
#include "motion_sys_XY.h"
#include "stepmotor.h"     /* StepMotor_ZDT / StepMotor_* / Emm_V5_* */
#include "drv_can.h"       /* can1_bus(邮箱空闲数查询) */
#include "drv_tim.h"       /* tim7_timer / TIM_Timer_Subscribe */

/* ========================================================================== */
/*  私有类型                                                                   */
/* ========================================================================== */

/**
  * @brief  一段笛卡尔直线运动(对应 Klipper 的 Move 结构)
  * @note   速度一律存平方(v2), 规划全程免开方, 仅分解梯形时开一次
  *         (Klipper 注释: "velocity squared ... to avoid sqrt in the planner")
  */
typedef struct
{
  /* 几何(入队时确定) */
  float start_x, start_y;   /* 段起点 (mm) */
  float end_x, end_y;       /* 段终点 (mm) */
  float axes_d_x, axes_d_y; /* 位移分量 (mm) */
  float axes_r_x, axes_r_y; /* 单位方向向量 */
  float move_d;             /* 段长 (mm) */
  float accel;              /* 本段加速度 (mm/s²) */

  /* 规划上限(入队/弯道计算时确定) */
  float max_start_v2;       /* 入口速度平方上限(junction deviation 等给出) */
  float max_cruise_v2;      /* 巡航速度平方上限 = feed² */
  float delta_v2;           /* 本段速度平方可变化量 = 2·move_d·accel */

  /* Replan 输出(junction 边界速度) */
  float start_v2, cruise_v2, end_v2;

  /* SetJunction 输出(梯形分解) */
  float start_v, cruise_v;
  float accel_d, cruise_d, decel_d;  /* 三段距离 (mm) */
  float accel_t, cruise_t, decel_t;  /* 三段时长 (s) */
  float total_t;                     /* 本段总时长 (s) */
} MS_XY_Move_t;

/* ========================================================================== */
/*  私有状态                                                                   */
/* ========================================================================== */

static volatile uint8_t  s_inited;        /* 初始化完成标志 */
static volatile uint8_t  s_state;         /* MotionSys_XY_State_e */
static uint8_t           s_enabled;       /* 两电机使能标志 */
static uint8_t           s_streamed;      /* 是否已实际流式下发过(看门狗前提) */

static MS_XY_Move_t      s_queue[MS_XY_QUEUE_LEN]; /* 规划队列(环形) */
static uint8_t           s_q_head;        /* 队首下标 */
static uint8_t           s_q_count;       /* 队列长度 */
static volatile uint8_t  s_active_valid;  /* s_active 是否有效 */
static MS_XY_Move_t      s_active;        /* 正在执行的段(冻结, 不参与重规划) */
static volatile uint16_t s_quiet_ms;      /* 距上次入队的静默时间(lookahead 窗口) */
static float             s_time;          /* 当前段内已执行时间 (s) */

static float             s_cmd_x, s_cmd_y;   /* 参考(规划)坐标 — Tick 写, 读需临界区 */
static float             s_tail_x, s_tail_y; /* 队尾坐标(MoveRel 的基准) — 仅主循环写 */
static float             s_off_a, s_off_b;   /* 脉冲零点偏置(mm): 驱动器清零时的 A/B 值 */

static int32_t           s_last_pa, s_last_pb; /* 上次下发的 A/B 绝对脉冲目标 */
static uint8_t           s_sent_valid;     /* s_last_* 是否有效(Stop 后置 0) */

static float             s_jd;             /* junction_deviation = scv²(√2−1)/accel
                                              (Klipper toolhead.py:536 同式) */
static uint16_t          s_watchdog;       /* 通信看门狗计数(仅 RUNNING 累加) */
static uint32_t          s_idle_enter_tick;/* 规划结束进入 IDLE 的时间戳 (用于 0.1mm 到位容差判定) */

/* ========================================================================== */
/*  私有辅助                                                                   */
/* ========================================================================== */

/* 环形队列取段指针 */
static MS_XY_Move_t *MS_XY_Q(uint8_t i)
{
  return &s_queue[(uint8_t)((s_q_head + i) % MS_XY_QUEUE_LEN)];
}

/* mm -> A 电机绝对脉冲(含符号电机方向与零点偏置) */
static int32_t MS_XY_PulseA(float x, float y)
{
  float p = ((x + y) - s_off_a) * MS_XY_PPS * (float)MS_XY_MOTOR_A_SIGN;
  return (int32_t)((p >= 0.0f) ? (p + 0.5f) : (p - 0.5f));
}

/* mm -> B 电机绝对脉冲 */
static int32_t MS_XY_PulseB(float x, float y)
{
  float p = ((x - y) - s_off_b) * MS_XY_PPS * (float)MS_XY_MOTOR_B_SIGN;
  return (int32_t)((p >= 0.0f) ? (p + 0.5f) : (p - 0.5f));
}

/**
  * @brief  成对下发 A/B 绝对脉冲目标(QPos 0xFC, 每条 1 个 CAN 单帧)
  * @note   两帧一次性占 2 个邮箱(共 3 个), 留 1 个给周期帧;
  *         邮箱不足则本拍不发 —— 目标是绝对量, 下一拍会带更新值补上,
  *         丢失只造成 ≤1 拍的短暂滞后, 不会累积偏差。
  * @retval true 已写入邮箱 / 与上次相同无需下发; false 邮箱不足, 下拍重试
  */
static bool MS_XY_Stream(int32_t pa, int32_t pb)
{
  if (s_sent_valid && pa == s_last_pa && pb == s_last_pb)
  {
    return true; /* 目标未变(如停在原地), 省总线 */
  }
  if (HAL_CAN_GetTxMailboxesFreeLevel(can1_bus.hcan) < 2U)
  {
    return false;
  }
  Emm_V5_QPos_Control(MS_XY_MOTOR_A_ADDR, pa);
  Emm_V5_QPos_Control(MS_XY_MOTOR_B_ADDR, pb);
  s_last_pa     = pa;
  s_last_pb     = pb;
  s_sent_valid  = 1U;
  s_streamed    = 1U;
  return true;
}

/**
  * @brief  计算两段衔接处的最大入口速度平方 —— 仿 Move.calc_junction()
  * @note   对应 toolhead.py:66-100。三级限制取最小:
  *         ① 两段各自的巡航上限; ② 前段入口+可加速量(不能倒着突破);
  *         ③ junction deviation 近似向心力(R_jd·jd·a)与半段向心限制
  *            (0.25·tan(θ/2)·delta_v2)。junction_deviation 由 square_corner_velocity
  *            导出: jd = scv²(√2−1)/accel —— Voron 无经典 jerk, 用它替代。
  */
static void MS_XY_CalcJunction(const MS_XY_Move_t *prev, MS_XY_Move_t *cur)
{
  float max_start = cur->max_cruise_v2;
  float t;

  if (prev->max_cruise_v2 < max_start)
  {
    max_start = prev->max_cruise_v2;
  }
  t = prev->max_start_v2 + prev->delta_v2;
  if (t < max_start)
  {
    max_start = t;
  }

  /* 近似向心速度限制(两段夹角越接近直角, 衔接速度越低) */
  {
    float cos_t = -(cur->axes_r_x * prev->axes_r_x +
                    cur->axes_r_y * prev->axes_r_y);          /* junction_cos_theta */
    float sin_d2 = 0.5f * (1.0f - cos_t);
    float cos_d2 = 0.5f * (1.0f + cos_t);
    if (sin_d2 < 0.0f) { sin_d2 = 0.0f; }
    if (cos_d2 < 0.0f) { cos_d2 = 0.0f; } /* 点积浮点误差可能使参数为负(Klipper 同样钳 0) */
    sin_d2 = __builtin_sqrtf(sin_d2);
    cos_d2 = __builtin_sqrtf(cos_d2);

    if ((1.0f - sin_d2) > 0.0f && cos_d2 > 0.0f)
    {
      float R_jd = sin_d2 / (1.0f - sin_d2);                 /* R_jd = sin/(1-sin) */
      float jd_v2 = R_jd * s_jd * cur->accel;                /* junction deviation 项 */
      float q     = 0.25f * sin_d2 / cos_d2;                 /* 半段向心系数 */
      float c_new = cur->delta_v2 * q;
      float c_old = prev->delta_v2 * q;

      if (jd_v2 < max_start)   { max_start = jd_v2; }
      if (c_new < max_start)   { max_start = c_new; }
      if (c_old < max_start)   { max_start = c_old; }
    }
  }

  cur->max_start_v2 = max_start;
}

/**
  * @brief  梯形速度曲线分解 —— 仿 Move.set_junction()
  * @note   对应 toolhead.py:100-113:
  *         accel_d = (cruise²−start²)/(2a), decel_d = (cruise²−end²)/(2a),
  *         cruise_d = move_d − accel_d − decel_d,
  *         各段时间 = 距离 / 该段首末平均速度。
  */
static void MS_XY_SetJunction(MS_XY_Move_t *m, float start_v2, float cruise_v2, float end_v2)
{
  float half_inv_a = 0.5f / m->accel;
  float accel_d = (cruise_v2 - start_v2) * half_inv_a;
  float decel_d = (cruise_v2 - end_v2) * half_inv_a;
  float cruise_d = m->move_d - accel_d - decel_d;
  float end_v = __builtin_sqrtf(end_v2);

  if (accel_d < 0.0f) { accel_d = 0.0f; }
  if (decel_d < 0.0f) { decel_d = 0.0f; }
  if (cruise_d < 0.0f) { cruise_d = 0.0f; } /* 浮点误差/三角形剖分钳位 */

  m->start_v2  = start_v2;
  m->cruise_v2 = cruise_v2;
  m->end_v2    = end_v2;

  m->start_v  = __builtin_sqrtf(start_v2);
  m->cruise_v = __builtin_sqrtf(cruise_v2);

  m->accel_d = accel_d;
  m->cruise_d = cruise_d;
  m->decel_d = decel_d;

  m->accel_t = accel_d / ((m->start_v + m->cruise_v) * 0.5f);
  m->cruise_t = (m->cruise_v > 1e-6f) ? (cruise_d / m->cruise_v) : 0.0f;
  m->decel_t = decel_d / ((end_v + m->cruise_v) * 0.5f);
  m->total_t = m->accel_t + m->cruise_t + m->decel_t;
}

/**
  * @brief  对规划队列整体重跑 lookahead —— 仿 LookAheadQueue.flush()
  * @note   对应 toolhead.py:135-186 的两遍扫描(此处省略 minimum_cruise_ratio
  *         平滑项, 其余逐式对应):
  *         反向: 从队尾(假定停车, j[n]=0)逐段回推
  *                j[i] = min(max_start_v2[i], j[i+1] + delta_v2[i]);
  *         队首边界强制连续: 活动段在执行中, 其出口速度已冻结,
  *                新段入口必须等于它(从静止起步则为 0)。
  *         正向: 加速度可达性 j[i+1] = min(j[i+1], j[i] + delta_v2[i]),
  *                只会下调不会破坏反向扫出的减速可达性(数学上单调)。
  *         再对每段做梯形分解。
  * @note   调用方必须已关中断(PRIMASK 临界区)。
  */
static void MS_XY_Replan(void)
{
  uint8_t n = s_q_count;
  uint8_t i;
  float j[MS_XY_QUEUE_LEN + 1]; /* j[i] = 第 i 段入口速度平方, j[n] = 队尾出口 */

  if (n == 0U)
  {
    return;
  }

  /* ---- 反向扫描: 队尾假定完全停车 ---- */
  j[n] = 0.0f;
  for (i = n; i-- > 0; )
  {
    MS_XY_Move_t *m = MS_XY_Q(i);
    float reachable = j[i + 1U] + m->delta_v2;
    j[i] = (m->max_start_v2 < reachable) ? m->max_start_v2 : reachable;
  }

  /* ---- 队首边界: 与执行中活动段的出口速度连续(或从静止起步) ---- */
  j[0] = s_active_valid ? s_active.end_v2 : 0.0f;

  /* ---- 正向扫描: 加速度可达性 ---- */
  for (i = 0; i + 1U < n; i++)
  {
    float cap = j[i] + MS_XY_Q(i)->delta_v2;
    if (j[i + 1U] > cap)
    {
      j[i + 1U] = cap;
    }
  }

  /* ---- 梯形分解 ---- */
  for (i = 0; i < n; i++)
  {
    MS_XY_Move_t *m = MS_XY_Q(i);
    float reachable = j[i + 1U] + m->delta_v2;
    float cruise2 = (j[i] + reachable) * 0.5f;  /* 三角形峰值: (s+e)/2 + a·d */
    if (cruise2 > m->max_cruise_v2)
    {
      cruise2 = m->max_cruise_v2;
    }
    MS_XY_SetJunction(m, j[i], cruise2, j[i + 1U]);
  }
}

/**
  * @brief  队首段出队进入执行(活动段)
  * @retval false 队列为空
  * @note   调用方必须已关中断; 不重置 s_time —— 由调用方决定起点
  */
static bool MS_XY_ActivateHead(void)
{
  if (s_q_count == 0U)
  {
    return false;
  }
  s_active = *MS_XY_Q(0);
  s_q_head = (uint8_t)((s_q_head + 1U) % MS_XY_QUEUE_LEN);
  s_q_count--;
  s_active_valid = 1U;
  return true;
}

/**
  * @brief  通信故障: 已下发过指令但电机长时间无应答
  * @note   只清队列、停流式下发、置 FAULT —— 不在中断里发 Stop_Now
  *         (Emm_V5 静态缓冲禁止跨上下文使用); 驱动器收不到新目标自然停在
  *         最后一拍目标上, 主循环观察到 FAULT 后调 MotionSys_XY_Stop()善后。
  */
static void MS_XY_TripFault(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_q_head = 0U;
  s_q_count = 0U;
  s_active_valid = 0U;
  s_quiet_ms = 0U;
  s_tail_x = s_cmd_x;   /* 参考终点 = 最后下发的目标 */
  s_tail_y = s_cmd_y;
  s_state = (uint8_t)MS_XY_FAULT;
  __set_PRIMASK(primask);
}

/* ========================================================================== */
/*  对外接口                                                                   */
/* ========================================================================== */

void MotionSys_XY_Init(void)
{
  if (s_inited)
  {
    return;
  }

  /* junction_deviation: jd = scv²(√2−1)/accel (Klipper toolhead.py:536) */
  s_jd = (MS_XY_SCV * MS_XY_SCV * 0.41421356f) / MS_XY_MAX_ACCEL;

  /* 注册两台 ZDT 闭环步进(内部会订阅各自的应答帧) */
  StepMotor_Init(&StepMotor_ZDT[0], MS_XY_MOTOR_A_ADDR, SM_TYPE_18DEG, MS_XY_MICROSTEP);
  StepMotor_Init(&StepMotor_ZDT[1], MS_XY_MOTOR_B_ADDR, SM_TYPE_18DEG, MS_XY_MICROSTEP);

  /* QPos 快速位置参数: raF=1 绝对位置, snF=0 立即执行(不等同步触发) */
  Emm_V5_Set_QPos_Params(MS_XY_MOTOR_A_ADDR, MS_XY_QPOS_VEL_RPM, MS_XY_QPOS_ACC, 1U, false);
  Emm_V5_Set_QPos_Params(MS_XY_MOTOR_B_ADDR, MS_XY_QPOS_VEL_RPM, MS_XY_QPOS_ACC, 1U, false);

  /* 坐标系初值 */
  s_off_a = 0.0f;
  s_off_b = 0.0f;
  s_cmd_x = s_cmd_y = 0.0f;
  s_tail_x = s_tail_y = 0.0f;
  s_last_pa = s_last_pb = 0;
  s_sent_valid = 1U;
  s_streamed = 0U;
  s_watchdog = 0U;

#if (MS_XY_ZERO_ON_INIT == 1)
  /* 两电机编码器清零: 电机当前物理位置即 (0,0) 原点, 坐标系自洽 */
  StepMotor_ClearAngle(&StepMotor_ZDT[0]);
  StepMotor_ClearAngle(&StepMotor_ZDT[1]);
#endif

  s_q_head = 0U;
  s_q_count = 0U;
  s_active_valid = 0U;
  s_quiet_ms = 0U;
  s_time = 0.0f;
  s_enabled = 0U;
  s_state = (uint8_t)MS_XY_IDLE;
  s_idle_enter_tick = 0U;
  s_inited = 1U;

  /* RTOS 多任务架构下:
     StepMotor_Run(3ms) 与 MotionSys_XY_Tick(1ms) 由 FreeRTOS 任务 MotorCtrlTask
     以精确 1kHz 周期调用, 不再挂在 TIM7 硬件定时器中断中, 彻底解除中断嵌套阻塞 */
}

void MotionSys_XY_Enable(bool en)
{
  if (!s_inited)
  {
    return;
  }
  StepMotor_Enable(&StepMotor_ZDT[0], en);
  StepMotor_Enable(&StepMotor_ZDT[1], en);
  s_enabled = en ? 1U : 0U;
}

MotionSys_XY_Result_e MotionSys_XY_MoveTo(float x, float y, float feed)
{
  uint32_t primask;
  MS_XY_Move_t *m;
  const MS_XY_Move_t *prev;
  float dx, dy, d;

  if (!s_inited)
  {
    return MS_XY_ERR_NOT_INIT;
  }
  if (x != x || y != y || feed != feed) /* NaN 检查(不引 math.h) */
  {
    return MS_XY_ERR_PARAM;
  }
  if (feed <= 0.0f)
  {
    return MS_XY_ERR_PARAM;
  }
  if (feed > MS_XY_MAX_VEL)
  {
    feed = MS_XY_MAX_VEL; /* 仿 Klipper: velocity = min(speed, max_velocity) */
  }

  /* 0.1mm 坐标分辨率网格量化对齐: 保证目标坐标落在 0.1mm 网格点上 */
  x = (float)((int32_t)(x * 10.0f + (x >= 0.0f ? 0.5f : -0.5f))) * 0.1f;
  y = (float)((int32_t)(y * 10.0f + (y >= 0.0f ? 0.5f : -0.5f))) * 0.1f;

#if (MS_XY_LIMITS_ENABLE == 1)
  /* 越界按 Klipper 方式拒绝, 不静默裁剪 */
  if (x < MS_XY_X_MIN || x > MS_XY_X_MAX || y < MS_XY_Y_MIN || y > MS_XY_Y_MAX)
  {
    return MS_XY_ERR_LIMIT;
  }
#endif

  dx = x - s_tail_x;
  dy = y - s_tail_y;
  d = __builtin_sqrtf(dx * dx + dy * dy);
  if (d < (MS_XY_RESOLUTION_MM * 0.5f))
  {
    return MS_XY_OK; /* 移动距离小于半个分辨率单位(0.05mm), 视为已在目标点, 省总线 */
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if (s_state == (uint8_t)MS_XY_FAULT)
  {
    __set_PRIMASK(primask);
    return MS_XY_ERR_BUSY; /* 先 Stop 清除故障 */
  }
  if (s_q_count >= MS_XY_QUEUE_LEN)
  {
    __set_PRIMASK(primask);
    return MS_XY_ERR_QUEUE_FULL;
  }

  /* 入队新段 */
  m = MS_XY_Q(s_q_count);
  m->start_x = s_tail_x;
  m->start_y = s_tail_y;
  m->end_x = x;
  m->end_y = y;
  m->axes_d_x = dx;
  m->axes_d_y = dy;
  m->axes_r_x = dx / d;
  m->axes_r_y = dy / d;
  m->move_d = d;
  m->accel = MS_XY_MAX_ACCEL;
  m->max_cruise_v2 = feed * feed;
  m->delta_v2 = 2.0f * d * m->accel;

  /* 弯道入口上限: 前方是执行中活动段或已入队段; 从静止起步则为 0 */
  prev = s_active_valid ? &s_active : (s_q_count > 0U ? MS_XY_Q((uint8_t)(s_q_count - 1U)) : NULL);
  if (prev != NULL)
  {
    MS_XY_CalcJunction(prev, m);
  }
  else
  {
    m->max_start_v2 = 0.0f;
  }

  s_q_count++;
  MS_XY_Replan();      /* 全队列重跑 lookahead(临界区内, ~µs 级) */
  s_quiet_ms = 0U;     /* 重新攒批窗口 */
  s_tail_x = x;        /* MoveRel 追加基准推进到新终点 */
  s_tail_y = y;
  if (s_state == (uint8_t)MS_XY_IDLE)
  {
    s_state = (uint8_t)MS_XY_RUNNING;
  }

  __set_PRIMASK(primask);

  /* 首次运动自动使能两电机(使能帧在主循环上下文发送) */
  if (!s_enabled)
  {
    MotionSys_XY_Enable(true);
  }
  return MS_XY_OK;
}

MotionSys_XY_Result_e MotionSys_XY_MoveRel(float dx, float dy, float feed)
{
  uint32_t primask;
  float bx, by;
  MotionSys_XY_Result_e ret;

  if (!s_inited)
  {
    return MS_XY_ERR_NOT_INIT;
  }
  /* 读队尾基准: s_tail 也可能是 Stop/故障路径在中断里写的, 读需临界区 */
  primask = __get_PRIMASK();
  __disable_irq();
  bx = s_tail_x;
  by = s_tail_y;
  __set_PRIMASK(primask);

  ret = MotionSys_XY_MoveTo(bx + dx, by + dy, feed);
  return ret;
}

void MotionSys_XY_Stop(void)
{
  uint32_t primask;

  if (!s_inited)
  {
    return;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  s_q_head = 0U;
  s_q_count = 0U;
  s_active_valid = 0U;
  s_quiet_ms = 0U;
  s_time = 0.0f;
  s_tail_x = s_cmd_x;   /* 后续 MoveRel 以最后下发目标为基准 */
  s_tail_y = s_cmd_y;
  s_sent_valid = 0U;    /* 下次运动强制重发首拍目标 */
  s_streamed = 0U;
  s_watchdog = 0U;
  s_state = (uint8_t)MS_XY_IDLE; /* 同时清除 FAULT */
  s_idle_enter_tick = SysTime_GetMs();
  __set_PRIMASK(primask);

  /* 双电机立即停止(主循环上下文, 独立静态缓冲, 与 Tick 的 QPos 不冲突) */
  StepMotor_Stop(&StepMotor_ZDT[0]);
  StepMotor_Stop(&StepMotor_ZDT[1]);
}

void MotionSys_XY_Tick(void)
{
  float t, dist, ax, ay;
  uint32_t primask;

  if (!s_inited || s_state == (uint8_t)MS_XY_FAULT)
  {
    return; /* 故障态停止流式下发, 等待主循环 Stop() 善后 */
  }

  /* ---- 通信看门狗: 已下发过目标但超过阈值无任何应答 -> 故障保护 ---- */
  if (s_state == (uint8_t)MS_XY_RUNNING && s_streamed)
  {
    s_watchdog++;
    if (s_watchdog >= MS_XY_FAULT_NO_RX)
    {
      s_watchdog = 0U;
      if (StepMotor_ZDT[0].no_rx_count > MS_XY_FAULT_NO_RX ||
          StepMotor_ZDT[1].no_rx_count > MS_XY_FAULT_NO_RX)
      {
        MS_XY_TripFault();
        return;
      }
    }
  }

  /* ---- 队列尚未有活动段: lookahead 攒批窗口(仿 Klipper flush 时机) ---- */
  if (!s_active_valid)
  {
    if (s_q_count > 0U)
    {
      primask = __get_PRIMASK();
      __disable_irq();
      s_quiet_ms = (uint16_t)(s_quiet_ms + MS_XY_TICK_MS);
      if (s_quiet_ms >= MS_XY_LA_WINDOW_MS)
      {
        (void)MS_XY_ActivateHead();
        s_time = 0.0f;
      }
      __set_PRIMASK(primask);
    }
    if (!s_active_valid)
    {
      return;
    }
  }

  /* ---- 推进时间并越过已完成段(跨段进位保证速度连续) ---- */
  s_time += (float)MS_XY_TICK_MS * 0.001f;
  while (s_active_valid && s_time >= s_active.total_t)
  {
    float rem = s_time - s_active.total_t;

    /* 段完成: 把精确终点补发给驱动(尽力而为; 邮箱满则绝对目标下一拍自愈) */
    (void)MS_XY_Stream(MS_XY_PulseA(s_active.end_x, s_active.end_y),
                       MS_XY_PulseB(s_active.end_x, s_active.end_y));
    s_cmd_x = s_active.end_x;
    s_cmd_y = s_active.end_y;

    s_time = rem;
    s_active_valid = 0U;
    if (s_q_count > 0U)
    {
      primask = __get_PRIMASK();
      __disable_irq();
      (void)MS_XY_ActivateHead(); /* 紧接的段立即续上(不等攒批窗口) */
      __set_PRIMASK(primask);
    }
    else
    {
      s_state = (uint8_t)MS_XY_IDLE; /* 队列排空, 回到空闲 */
      s_idle_enter_tick = SysTime_GetMs();
      break;
    }
  }
  if (!s_active_valid)
  {
    return;
  }

  /* ---- 沿梯形参考轨迹求 t 时刻位置(仿 trapq 对时间求位置) ---- */
  t = s_time;
  if (t < s_active.accel_t)
  {
    dist = s_active.start_v * t + 0.5f * s_active.accel * t * t;
  }
  else if (t < (s_active.accel_t + s_active.cruise_t))
  {
    dist = s_active.accel_d + s_active.cruise_v * (t - s_active.accel_t);
  }
  else
  {
    float t2 = t - s_active.accel_t - s_active.cruise_t;
    if (t2 > s_active.decel_t)
    {
      t2 = s_active.decel_t;
    }
    dist = s_active.accel_d + s_active.cruise_d +
           s_active.cruise_v * t2 - 0.5f * s_active.accel * t2 * t2;
  }
  if (dist > s_active.move_d)
  {
    dist = s_active.move_d;
  }

  /* CoreXY 反解到电机空间并流式下发(邮箱不足时跳过本拍, 下拍绝对值自愈) */
  ax = s_active.start_x + s_active.axes_r_x * dist;
  ay = s_active.start_y + s_active.axes_r_y * dist;
  s_cmd_x = ax;
  s_cmd_y = ay;
  (void)MS_XY_Stream(MS_XY_PulseA(ax, ay), MS_XY_PulseB(ax, ay));
}

bool MotionSys_XY_IsArrived(float target_x, float target_y, float tolerance_mm)
{
  float act_x = 0.0f, act_y = 0.0f;

  if (!s_inited)
  {
    return false;
  }
  if (tolerance_mm <= 0.0f)
  {
    tolerance_mm = MS_XY_ARRIVE_TOLERANCE_MM;
  }
  MotionSys_XY_GetActualPos(&act_x, &act_y);
  float dx = (act_x >= target_x) ? (act_x - target_x) : (target_x - act_x);
  float dy = (act_y >= target_y) ? (act_y - target_y) : (target_y - act_y);
  return (dx <= tolerance_mm && dy <= tolerance_mm);
}

bool MotionSys_XY_IsBusy(void)
{
  if (MotionSys_XY_GetState() == MS_XY_RUNNING)
  {
    return true;
  }

  /* 规划队列已排空: 检查物理实际位置是否已稳定进入 0.1mm 容差范围 */
  if (s_inited)
  {
    float target_x = s_cmd_x;
    float target_y = s_cmd_y;

    /* 若实际物理位置与目标仍有偏差超过 0.1mm，且在 500ms 等待窗口内，继续保持忙碌等待物理到位 */
    if (!MotionSys_XY_IsArrived(target_x, target_y, MS_XY_ARRIVE_TOLERANCE_MM))
    {
      uint32_t now = SysTime_GetMs();
      if ((uint32_t)(now - s_idle_enter_tick) < 500U)
      {
        return true;
      }
    }
  }

  return false;
}

MotionSys_XY_State_e MotionSys_XY_GetState(void)
{
  MotionSys_XY_State_e st;
  uint32_t primask;

  if (!s_inited)
  {
    return MS_XY_IDLE;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  st = (MotionSys_XY_State_e)s_state;
  __set_PRIMASK(primask);
  return st;
}

void MotionSys_XY_GetCmdPos(float *x, float *y)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  if (x != NULL) { *x = s_cmd_x; }
  if (y != NULL) { *y = s_cmd_y; }
  __set_PRIMASK(primask);
}

void MotionSys_XY_GetActualPos(float *x, float *y)
{
  uint32_t primask;
  float deg_a, deg_b, a, b;

  /* 反馈量由 CAN 接收中断更新, 成对读取需临界区 */
  primask = __get_PRIMASK();
  __disable_irq();
  deg_a = StepMotor_ZDT[0].total_angle;
  deg_b = StepMotor_ZDT[1].total_angle;
  __set_PRIMASK(primask);

  /* 电机轴角度 -> 皮带 mm(转一圈 = ROTATION_MM), 再叠加零点偏置与方向 */
  a = s_off_a + (float)MS_XY_MOTOR_A_SIGN * (deg_a * (MS_XY_ROTATION_MM / 360.0f));
  b = s_off_b + (float)MS_XY_MOTOR_B_SIGN * (deg_b * (MS_XY_ROTATION_MM / 360.0f));

  if (x != NULL) { *x = 0.5f * (a + b); } /* 变换逆解: X = (A+B)/2 */
  if (y != NULL) { *y = 0.5f * (a - b); } /*              Y = (A-B)/2 */
}

MotionSys_XY_Result_e MotionSys_XY_SetPosition(float x, float y)
{
  uint32_t primask;

  if (!s_inited)
  {
    return MS_XY_ERR_NOT_INIT;
  }
  if (x != x || y != y)
  {
    return MS_XY_ERR_PARAM;
  }
  if (MotionSys_XY_GetState() == MS_XY_RUNNING)
  {
    return MS_XY_ERR_BUSY;
  }

  /* 驱动器编码器清零(主循环上下文, Emm_V5_Reset_CurPos 独立静态缓冲) */
  StepMotor_ClearAngle(&StepMotor_ZDT[0]);
  StepMotor_ClearAngle(&StepMotor_ZDT[1]);

  primask = __get_PRIMASK();
  __disable_irq();
  s_off_a = x + y;      /* 记录清零时刻的 A/B 值作为脉冲零点偏置 */
  s_off_b = x - y;
  s_cmd_x = s_tail_x = x;
  s_cmd_y = s_tail_y = y;
  s_last_pa = 0;        /* 驱动器计数已清零 -> 目标 0 与当前一致 */
  s_last_pb = 0;
  s_sent_valid = 1U;
  s_streamed = 0U;
  s_watchdog = 0U;
  s_state = (uint8_t)MS_XY_IDLE; /* 顺带清除 FAULT */
  __set_PRIMASK(primask);
  return MS_XY_OK;
}
