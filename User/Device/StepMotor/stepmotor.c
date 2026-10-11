/**
 * @file    stepmotor.c
 * @brief   ZDT/Emm_V5 系列步进闭环电机控制实现
 *          底层命令走 stepmotor_ZDT(Emm_V5 协议), 对外封装电机对象接口
 */

/* Includes ------------------------------------------------------------------*/
#include "stepmotor.h"

/* ========================================================================== */
/*  私有宏 / 变量                                                              */
/* ========================================================================== */

static volatile uint8_t motorCount;                 /* 已注册电机数量 */
static StepMotor_t *Motors[STEP_MAX_MOTORS];        /* 电机对象指针数组(按注册顺序) */
static uint8_t s_poll_idx;                          /* 位置轮询游标 */

StepMotor_t StepMotor_ZDT[2];

/* S_CPOS 查询应答帧中的功能码(见 stepmotor_ZDT.c Emm_V5_Read_Sys_Params) */
#define STEP_FUNC_CPOS 0x36U
/* F1 快速位置模式参数应答帧功能码 */
#define STEP_FUNC_QPARAMS 0xF1U

/* ========================================================================== */
/*  私有辅助函数                                                               */
/* ========================================================================== */

/**
  * @brief  角度(度, 带符号) -> 脉冲数 + 方向
  * @param  m      电机对象(取细分/整步数做换算)
  * @param  angle  角度, 负值表示反转(CCW)
  * @param  dir    输出方向: 0=CW, 1=CCW
  * @param  clk    输出脉冲数(无符号绝对值)
  */
static void StepMotor_AngleToDirClk(const StepMotor_t *m, float angle, uint8_t *dir, uint32_t *clk)
{
  float pulses = (angle / 360.0f) * (float)m->steps_per_rev * (float)m->microstep;

  if (pulses < 0.0f)
  {
    *dir = 1U;
    pulses = -pulses;
  }
  else
  {
    *dir = 0U;
  }
  *clk = (uint32_t)(pulses + 0.5f);
}

/**
  * @brief  转速幅值(RPM) -> 限幅后的无符号速度值
  */
static uint16_t StepMotor_SpeedMagnitude(float speed)
{
  float v = (speed < 0.0f) ? -speed : speed;

  if (v > STEP_MAX_SPEED_RPM)
  {
    v = STEP_MAX_SPEED_RPM;
  }
  return (uint16_t)(v + 0.5f);
}

/**
  * @brief  转速(r/min, 带符号) -> 方向 + 速度幅值
  * @param  speed 转速, 负值表示反转(CCW)
  * @param  dir   输出方向: 0=CW, 1=CCW
  * @param  vel   输出速度幅值(RPM, 已限幅到 0~5000)
  */
static void StepMotor_SpeedToDirVel(float speed, uint8_t *dir, uint16_t *vel)
{
  *dir = (speed < 0.0f) ? 1U : 0U;
  *vel = StepMotor_SpeedMagnitude(speed);
}

/* ========================================================================== */
/*  CAN 接收回调(签名适配 CAN_RxCallback_t)                                    */
/* ========================================================================== */

/**
  * @brief  应答帧回调(中断上下文): 任意应答清超时计数; 0x36 实时位置抓原始
  *         字节并按手册格式解析 -> curPulse(解卷绕原始值) / total_angle(度)
  * @note   回包格式(X42S 手册 V1.0.5 §5.5.13 + §4.2.2, 地址在帧 ID 里):
  *           DLC=7: [36][符号00/01][位置 4B 大端][6B]
  *           兼容旧格式 DLC=6: [36][位置 int32 大端][6B]
  *         单位: 原始值按 STEP_CPOS_RAW_PER_REV 折算成度(默认 Emm 65536/圈);
  *         原始值按圈回绕时(0~65535)用最短增量解卷绕 —— 轮询周期 6ms,
  *         单拍位移远小于半圈, 取模增量必然安全。
  */
static void StepMotor_CAN_Callback(CAN_BusObject *bus, CAN_RxHeaderTypeDef *header, uint8_t *data)
{
  (void)bus;

  for (uint8_t i = 0; i < STEP_MAX_MOTORS; i++)
  {
    if (Motors[i] == NULL || !Motors[i]->initialized)
      continue;
    if (header->IDE != CAN_ID_EXT ||
        header->ExtId != ZDT_CAN_EXT_ID(Motors[i]->addr, 0U))
      continue;

    /* 收到该电机任意应答帧即清零超时计数 */
    Motors[i]->no_rx_count = 0;

    /* F1(QPos 参数)应答: [F1][状态02/E2/EE][6B] —— 记录状态供日志核对
       (F1 不生效时 FC 会按相对模式累加, 电机跑飞) */
    if (header->DLC >= 2U && data[0] == STEP_FUNC_QPARAMS)
    {
      Motors[i]->f1_status = data[1];
      Motors[i]->f1_count++;
    }

    if (header->DLC >= 1U && data[0] == STEP_FUNC_CPOS)
    {
      int32_t raw = 0;
      uint8_t ok  = 0;

      /* 抓原始字节(诊断): 主循环在首帧/格式变化时打 hex, 核对实际格式 */
      Motors[i]->cpos_raw_dlc = header->DLC;
      for (uint8_t b = 0U; (b < header->DLC) && (b < 8U); b++)
      {
        Motors[i]->cpos_raw_data[b] = data[b];
      }
      Motors[i]->cpos_raw_count++;

      if (header->DLC == 7U && data[6] == 0x6BU)
      {
        uint32_t v = ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16) |
                     ((uint32_t)data[4] << 8)  |  (uint32_t)data[5];
        v &= 0x7FFFFFFFu; /* 幅值不会到 2^31, 掩掉极端位型的有符号取负 UB */
        raw = (data[1] == 0x01U) ? -(int32_t)v : (int32_t)v;
        ok  = 1U;
      }
      else if (header->DLC == 6U && data[5] == 0x6BU)
      {
        raw = (int32_t)(((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) |
                        ((uint32_t)data[3] << 8)  |  (uint32_t)data[4]);
        ok  = 1U;
      }

      if (Motors[i]->cpos_skip != 0U)
      {
        /* ClearAngle 之后的守护: 驱动器复位落地前可能有在途旧回包,
           用它重建基准会把 k*每圈计数 的幽灵偏移永久锁进反馈, 丢弃 */
        Motors[i]->cpos_skip--;
      }
      else if (ok)
      {
        if (Motors[i]->cpos_base_valid == 0U)
        {
          Motors[i]->cpos_prev_raw   = raw;
          Motors[i]->curPulse        = raw;
          Motors[i]->cpos_base_valid = 1U;
        }
        else
        {
          /* 最短增量解卷绕: 兼容按圈回绕(Emm 65536/圈)与连续计数两种固件。
             模数取自 STEP_CPOS_RAW_PER_REV(切 X 固件 3600 时同步生效);
             轮询 6ms/拍、800RPM 下单拍位移 ~5k 计数, 远小于半圈(32768)。
             无符号回绕求差规避有符号溢出 UB */
          uint32_t wrap = (uint32_t)(int32_t)STEP_CPOS_RAW_PER_REV;
          uint32_t du   = (uint32_t)raw - (uint32_t)Motors[i]->cpos_prev_raw;
          int32_t  d    = (int32_t)(du % wrap);

          if (d > (int32_t)(wrap / 2U))
          {
            d -= (int32_t)wrap;
          }
          Motors[i]->cpos_prev_raw = raw;
          Motors[i]->curPulse     += d;
        }
        Motors[i]->total_angle = (float)Motors[i]->curPulse *
                                 (360.0f / STEP_CPOS_RAW_PER_REV);
      }
    }
    break;
  }
}

/* ========================================================================== */
/*  初始化                                                                     */
/* ========================================================================== */

/**
  * @brief  电机参数初始化并注册
  * @note   不可以重复初始化同一个电机; 必须在 Driver_CAN1_Init() 之后调用
  */
void StepMotor_Init(StepMotor_t *m, uint8_t addr, StepMotorType_e type, uint8_t microstep)
{
  if (m == NULL || addr == 0U || motorCount >= STEP_MAX_MOTORS)
  {
    return;
  }

  /* 跳过重复注册同一个实例 */
  for (uint8_t i = 0; i < STEP_MAX_MOTORS; i++)
  {
    if (Motors[i] == m)
    {
      return;
    }
  }

  m->addr          = addr;
  m->type          = (uint8_t)type;
  m->microstep     = (microstep == 0U) ? STEP_DEFAULT_MICROSTEP : microstep;
  m->steps_per_rev = (type == SM_TYPE_09DEG) ? STEP_STEPS_PER_REV_09 : STEP_STEPS_PER_REV_18;

  m->mode     = SM_SPEED_MODE;
  m->setSpeed = 0.0f;
  m->setAngle = 0.0f;
  m->setAcc   = STEP_DEFAULT_ACC;

  m->total_angle = 0.0f;
  m->curPulse    = 0;
  m->no_rx_count = 0;
  m->enabled     = 0;

  m->cpos_raw_dlc     = 0U;
  m->cpos_raw_count   = 0U;
  m->cpos_prev_raw    = 0;
  m->cpos_base_valid  = 0U;
  m->cpos_skip        = 0U;
  m->f1_status        = 0U;
  m->f1_count         = 0U;
  for (uint8_t b = 0U; b < 8U; b++)
  {
    m->cpos_raw_data[b] = 0U;
  }

  m->initialized = 1;
  Motors[motorCount++] = m;

  /* 订阅该电机的应答帧(扩展帧: (地址<<8)|包号0) */
  CAN_Bus_Subscribe(STEP_MOTOR_BUS, ZDT_CAN_EXT_ID(addr, 0U), StepMotor_CAN_Callback);
}

/* ========================================================================== */
/*  状态与设定接口                                                             */
/* ========================================================================== */

/* 电机使能/失能 */
void StepMotor_Enable(StepMotor_t *m, bool state)
{
  if (m == NULL || !m->initialized)
  {
    return;
  }
  Emm_V5_En_Control(m->addr, state, false);
  m->enabled = state ? 1U : 0U;
}

/* 立即停止运动 */
void StepMotor_Stop(StepMotor_t *m)
{
  if (m == NULL || !m->initialized)
  {
    return;
  }
  Emm_V5_Stop_Now(m->addr, false);
}

/* 设置电机速度(单位: r/min, 带符号: +正转CW / -反转CCW), 设置一次即可 */
void StepMotor_SetSpeed(StepMotor_t *m, float speed, uint8_t acc)
{
  uint8_t  dir;
  uint16_t vel;

  if (m == NULL || !m->initialized)
  {
    return;
  }

  StepMotor_SpeedToDirVel(speed, &dir, &vel);
  if (acc > 255U)
  {
    acc = 255U;
  }

  m->setSpeed = speed;
  m->setAcc   = acc;
  m->mode     = SM_SPEED_MODE;

  Emm_V5_Vel_Control(m->addr, dir, vel, acc, false);
}

/* 设置电机绝对角度(单位: 度), vel 为速度上限(RPM), acc 为加速度, 设置一次即可 */
void StepMotor_SetAngle(StepMotor_t *m, float angle, float vel, uint8_t acc)
{
  uint8_t  dir;
  uint16_t vel_u;
  uint32_t clk;

  if (m == NULL || !m->initialized)
  {
    return;
  }

  /* 位置模式下 vel 只取幅值, 运动方向由目标角度符号决定 */
  vel_u = StepMotor_SpeedMagnitude(vel);
  StepMotor_AngleToDirClk(m, angle, &dir, &clk);
  if (acc > 255U)
  {
    acc = 255U;
  }

  m->setAngle = angle;
  m->setAcc   = acc;
  m->mode     = SM_ANGLE_MODE;

  /* raF=1: 绝对位置运动(dir+clk 表示带符号的绝对目标位置) */
  Emm_V5_Pos_Control(m->addr, dir, vel_u, acc, clk, 1, false);
}

/* 设置电机增量角度(单位: 度, 带符号), 设置一次即可 */
void StepMotor_SetAngleInc(StepMotor_t *m, float angleInc, float vel, uint8_t acc)
{
  uint8_t  dir;
  uint16_t vel_u;
  uint32_t clk;

  if (m == NULL || !m->initialized)
  {
    return;
  }

  vel_u = StepMotor_SpeedMagnitude(vel);
  StepMotor_AngleToDirClk(m, angleInc, &dir, &clk);
  if (acc > 255U)
  {
    acc = 255U;
  }

  m->setAngle = m->total_angle + angleInc;
  m->setAcc   = acc;
  m->mode     = SM_ANGLEINC_MODE;

  /* raF=2: 相对当前电机实时位置的相对运动 */
  Emm_V5_Pos_Control(m->addr, dir, vel_u, acc, clk, 2, false);
}

/* 电机角度清零 */
void StepMotor_ClearAngle(StepMotor_t *m)
{
  if (m == NULL || !m->initialized)
  {
    return;
  }

  /* total_angle/curPulse 会被 CAN 回调更新, 临界区保证不产生半更新状态 */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  m->curPulse    = 0;
  m->total_angle = 0.0f;
  m->setAngle    = 0.0f;
  m->cpos_base_valid = 0U; /* 驱动器计数已清零, 下一帧重建解卷绕基准 */
  m->cpos_skip   = 2U;     /* 丢弃复位落地前的在途旧回包(轮询 6ms/拍, 2 帧足够),
                              否则旧值重建基准 -> 下一帧解卷绕锁进 k*每圈 偏移 */
  __set_PRIMASK(primask);

  Emm_V5_Reset_CurPos_To_Zero(m->addr);
}

/* ========================================================================== */
/*  周期维护                                                                   */
/* ========================================================================== */

/**
  * @brief  周期维护: 更新通信超时计数, 并轮询各电机实时位置(每拍只查一台)
  * @note   步进闭环由驱动器自行完成, 不需要每拍下发控制量;
  *         这里只负责反馈刷新与通信诊断, 与 DJ_MotorRun 的节拍约定保持一致
  */
void StepMotor_Run(void)
{
  for (uint8_t i = 0; i < STEP_MAX_MOTORS; i++)
  {
    if (Motors[i] == NULL || !Motors[i]->initialized)
      continue;

    if (Motors[i]->no_rx_count < 65535U)
      Motors[i]->no_rx_count++;
  }

  /* 轮询位置反馈: 每拍只查一台电机, 避免挤占总线 */
  for (uint8_t n = 0; n < STEP_MAX_MOTORS; n++)
  {
    uint8_t i = (uint8_t)((s_poll_idx + n) % STEP_MAX_MOTORS);
    if (Motors[i] != NULL && Motors[i]->initialized)
    {
      Emm_V5_Read_Sys_Params(Motors[i]->addr, S_CPOS);
      s_poll_idx = (uint8_t)(i + 1U);
      break;
    }
  }
}
