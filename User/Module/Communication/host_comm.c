/**
 * @file    host_comm.c
 * @brief   上位机通讯模块实现: 32 字节定长二进制帧 over USB CDC
 *
 *          ===== 设计参考: Voron/Klipper 上位机通讯 =====
 *          本模块的角色对应 Klipper 中"MCU 侧"那一端, 关键语义逐条对照:
 *            - 组帧/校验顺序  <- Klipper msgproto.py check_packet():
 *                                先同步字节再长度再校验; 我们无 CRC(规格用
 *                                预留位代替), 依赖 USB 传输层 CRC + SOF 同步,
 *                                32B 定长使伪同步窗口极小 —— 这是对 USB 的
 *                                有意偏离(若是裸 UART 必须加校验)。
 *            - 校验通过才 ACK <- Protocol.md: MCU 只对校验通过的块发 ack;
 *                                我们的 ACK_SEQ 同理: 受理入队才推进,
 *                                被拒帧不推进 -> 上位机据"SEQ 未被 ACK"重发。
 *            - SEQ 去重       <- 主机重发是正常行为: 重复 SEQ 静默重 ACK,
 *                                不重复执行、不记错误(与 Klipper 一致)。
 *            - 错误粘滞       <- Klipper shutdown 锁存: ERROR 置位后保持,
 *                                只有下行 FLAGS bit3(清报警)显式清除,
 *                                绝不自动恢复 —— 有运动部件的机器安全第一。
 *            - 急停锁存       <- M112 语义: 急停后除"清报警"外拒绝一切
 *                                命令(ERROR bit8 记录"急停锁存"), 恢复必须
 *                                上位机显式下发。
 *            - 链路看门狗     <- Klipper MCU 主机失联保护(HOST_LINK_TIMEOUT_MS,
 *                                默认关闭: 本协议无周期心跳约定, 避免误报)。
 *            - 混合上报       <- Moonraker: 50ms 周期心跳 + 命令受理/状态
 *                                跳变/任务结果立即补发(仅主循环发送, 避开
 *                                USB TX 忙)。
 *            - TASK_ID 关联   <- Klipper 无此项(顺序应答); 周期上报必须带
 *                                任务标识, 借鉴 Moonraker print_stats 语义。
 *
 *          ===== 任务结果自动判定(顺序模板) =====
 *            下行帧无显式"任务结束"标志, 本模块按同一 TASK_ID 的命令到达顺序
 *            推断阶段(用户规格"任务结果判定方式"):
 *              第1段运动       -> 去取件位(FAIL_STAGE 1), 到位置 PICK_REACHED
 *              夹爪闭合        -> 夹取(2)
 *              第2段运动       -> 去放置位(3/4), 到位置 PLACE_REACHED
 *              夹爪张开        -> 放置(5), 张开完成置 PLACE_CONFIRMED
 *              张开完成且设备空闲 -> 任务自然结束, 立即补发成功结果
 *              其后同 TASK_ID 运动 -> 返回段(6), 完成后再次补发结果
 *            纯移动(无夹取)任务不自动结束: 需新任务/取消/急停/超时了结。
 *            注: 夹爪空检测/脱落/放置确认需要传感器; 当前无传感器, 故
 *              GRIP_EMPTY/GRIP_SLIP 不产生, PLACE_CONFIRMED 仅表示
 *              "到达放置位并执行了张开动作"(与规格注记一致)。
 *
 *          ===== 执行分工 =====
 *            USB 中断(HostComm_OnRx): 字节组帧 -> 校验 -> 命令入 FIFO -> 更新
 *              ACK —— 只做收发与簿记, 不碰运动/舵机(缓冲区仅回调内有效)。
 *            主循环(HostComm_Poll): 排空 FIFO 分发执行(清报警/急停/取消/夹爪/
 *              运动) -> 夹爪与运动到位判定 -> 任务结果推断 -> 超时/故障善后
 *              -> 进度推算 -> 组帧上报。
 *            TIM7(HostComm_Tick, 50ms): 心跳到期 + 命令超时/链路看门狗, 只
 *              置标志不发帧。
 */

/* Includes ------------------------------------------------------------------*/
#include "host_comm.h"
#include "motion_sys_XY.h" /* MotionSys_XY_* */
#include "grip.h"          /* Grip_*: 夹爪双舵机(Z 轴升降 + 开合), 见 grip.h */
#include "drv_usb.h"       /* USB_Init / USB_Transmit_Data */
#include "drv_tim.h"       /* tim7_timer / TIM_Timer_Subscribe */
#include "stepmotor.h"     /* StepMotor_ZDT: 首次运动前 F1 应答诊断 */
#include "drv_uart.h"      /* uart1_port: 一次性诊断打印 */

/* 私有前向声明 */
static void HostComm_OnRx(uint8_t *buf, uint16_t len);
static void HostComm_OnFrame(const uint8_t *f);

/* ========================================================================== */
/*  小端读写辅助                                                               */
/* ========================================================================== */

static void H_PutU16(uint8_t *b, uint16_t v)
{
  b[0] = (uint8_t)(v & 0xFFU);
  b[1] = (uint8_t)(v >> 8);
}

static void H_PutU32(uint8_t *b, uint32_t v)
{
  b[0] = (uint8_t)(v & 0xFFU);
  b[1] = (uint8_t)((v >> 8) & 0xFFU);
  b[2] = (uint8_t)((v >> 16) & 0xFFU);
  b[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static void H_PutI32(uint8_t *b, int32_t v)
{
  H_PutU32(b, (uint32_t)v);
}

static uint16_t H_GetU16(const uint8_t *b)
{
  return (uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

static uint32_t H_GetU32(const uint8_t *b)
{
  return ((uint32_t)b[0]) | ((uint32_t)b[1] << 8) |
         ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

/* mm -> 0.001mm(带符号四舍五入) */
static int32_t H_MmToUm(float v)
{
  float s = v * 1000.0f;
  return (int32_t)((s >= 0.0f) ? (s + 0.5f) : (s - 0.5f));
}

/* ========================================================================== */
/*  私有状态                                                                   */
/* ========================================================================== */

static uint8_t s_inited;
static uint8_t s_first_move; /* 首次运动: 先显式使能电机, 防 QPos 流抢在使能前 */

/* ---- RX 组帧(仅 USB 中断上下文访问) ---- */
#define H_RX_IDLE 0U /* 未同步: 等连续 AA AA 才进 BODY(SOF 实为 AA 55, 双AA判据) */
#define H_RX_BODY 1U /* 收帧中 */
static uint8_t s_rx_state;
static uint8_t s_rx_frame[HOST_FRAME_LEN];
static uint8_t s_rx_idx;

/* ---- 命令 FIFO: 单生产者(USB 中断) 单消费者(主循环), SPSC 无锁 ---- */
static HostCmd_t s_fifo[HOST_CMD_FIFO_LEN];
static volatile uint8_t s_fifo_head; /* 中断写 */
static volatile uint8_t s_fifo_tail; /* 主循环写 */

/* ---- 应答/错误/急停 ---- */
static volatile uint16_t s_ack_seq;    /* 已受理的下行 SEQ */
static volatile uint8_t  s_seq_valid;  /* 是否受理过至少一帧 */
static volatile uint16_t s_error;      /* 粘滞错误位(HOST_ERR_*) */
static volatile uint8_t  s_estop;      /* 急停锁存 */

/* ---- 上报(仅主循环) ---- */
static uint8_t  s_tx_frame[HOST_FRAME_LEN]; /* 静态 TX 缓冲(须活到 CDC 发完) */
static uint16_t s_tx_seq;
static uint8_t  s_sent_valid;   /* 首次上报前强制发送一次 */
static uint16_t s_sent_ack;
static uint16_t s_sent_error;
static uint32_t s_sent_task;
static uint8_t  s_sent_state;
static uint8_t  s_sent_grip;
static int32_t  s_sent_x, s_sent_y; /* 已上报位置(0.001mm 整数化后比较) */
static uint32_t s_last_pos_ms;    /* 上次位置类上报时刻(限频用) */
static uint32_t s_last_try_ms;    /* 上次发送尝试时刻(失败重试限速用) */
static uint8_t  s_usb_ok;         /* USB 已配置且最近一次发送成功/忙 */

/* ---- 命令超时(主循环设, TIM7 读) ---- */
static volatile uint8_t  s_deadline_active;
static volatile uint32_t s_deadline;
static volatile uint8_t  s_timeout_flag; /* 超时到期(TIM7 置, 主循环清) */

#if (HOST_LINK_TIMEOUT_MS > 0U)
static volatile uint32_t s_last_rx_tick; /* 最后受理时刻(HAL_GetTick) */
static volatile uint8_t  s_link_flag;    /* 链路超时到期 */
#endif

/* ---- 任务/阶段(顺序模板推断; 仅主循环) ---- */
static uint32_t s_job_task;            /* 当前任务 TASK_ID */
static uint8_t  s_job_active;          /* 有定义的任务(进入首个动作后置1) */
static uint8_t  s_job_seen_close;      /* 本任务已收到夹爪闭合 */
static uint8_t  s_job_open_started;    /* 本任务已收到放置(张开)命令 */
static uint8_t  s_job_placed;          /* 放置已实际完成 */
static uint8_t  s_job_ret;             /* 已进入"返回"段 */
static uint8_t  s_job_concluded;       /* 本次任务已了结(结果已生成) */
static uint8_t  s_res_flags;           /* RESULT_FLAGS 累积 */
static uint8_t  s_res_stage;           /* FAIL_STAGE */
static uint8_t  s_res_valid;           /* 当前结果是否有效(随心跳上报) */
static uint8_t  s_res_new;             /* 新结果待补发 TASK_DONE_EVENT */
static uint8_t  s_fault_seen;          /* 本任务 FAULT 已记账 */

/* ---- 取消锁存(丢同 TASK_ID 后续动作帧) ---- */
static uint8_t  s_cancel_active;
static uint32_t s_cancel_task;

/* ---- 在飞运动/夹爪(仅主循环) ---- */
static uint8_t  s_motion_pending;      /* 已派发运动且未观测到完成 */
/* 各阶段是否有未观测完成的运动(连发时可并存: 队列排空瞬间同时判定) */
static uint8_t  s_st_pick_moving;      /* 去取件位运动在飞 */
static uint8_t  s_st_place_moving;     /* 去放置位运动在飞 */
static uint8_t  s_st_ret_moving;       /* 返回段运动在飞 */
static uint8_t  s_grip_is_open;        /* 当前夹爪动作是否为"张开" */
static float    s_prog_sx, s_prog_sy;  /* 进度基准(下发前参考位置) */
static float    s_prog_tx, s_prog_ty;  /* 进度目标 */
static uint8_t  s_progress;            /* 0~100 */

/* ---- 夹爪(仅主循环; 动作计时/到位由 grip 模块给"动作中"口径) ---- */
static uint8_t  s_grip_state;   /* HOST_GRIP_ST_* */
static uint8_t  s_grip_final;   /* 本次动作到位后的终态 */
static uint8_t  s_grip_moving;

/* ---- 统计 ---- */
static HostCommStats_t s_stats;

/* ---- 首次运动诊断(一次性, 结构体见 host_comm.h HostDiag_t) ---- */
static volatile HostDiag_t s_diag;

/* ---- 业务层命令钩子 ---- */
static HostCmdHook_fn s_cmd_hook = NULL;

/* ========================================================================== */
/*  私有辅助                                                                   */
/* ========================================================================== */

/* 粘滞错误置位: 主循环与 USB 中断都可能调用 -> PRIMASK 临界 */
static void Host_ErrSet(uint16_t bit)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_error |= bit;
  __set_PRIMASK(primask);
}

/* 关闭当前任务并生成"成功"结果 */
static void HostRes_EmitSuccess(void)
{
  if (!s_job_active || s_job_concluded)
  {
    return;
  }
  s_job_placed   = 1U;
  s_job_concluded = 1U;
  s_res_flags   |= (HOST_RES_DONE | HOST_RES_OK);
  s_res_stage    = HOST_STAGE_NONE;
  s_res_valid    = 1U;
  s_res_new      = 1U;
  s_deadline_active = 0U;
  s_progress     = 100U;
}

/* 关闭当前任务并生成"中止(超时/急停/取消)"结果 */
static void HostRes_EmitAbort(uint8_t stage)
{
  if (!s_job_active)
  {
    return;
  }
  s_job_concluded = 1U;
  s_res_flags = (uint8_t)((s_res_flags & (uint8_t)~(uint8_t)(HOST_RES_DONE | HOST_RES_OK | HOST_RES_ABORT))
                          | HOST_RES_DONE | HOST_RES_ABORT);
  s_res_stage = stage;
  s_res_valid = 1U;
  s_res_new   = 1U;
  s_deadline_active = 0U;
}

/* 关闭当前任务并生成"失败(运动中故障/入队失败)"结果 */
static void HostRes_EmitFail(uint8_t stage)
{
  if (!s_job_active)
  {
    return;
  }
  s_job_concluded = 1U;
  s_res_flags = (uint8_t)((s_res_flags & (uint8_t)~(uint8_t)(HOST_RES_DONE | HOST_RES_OK | HOST_RES_ABORT))
                          | HOST_RES_DONE);
  s_res_stage = stage;
  s_res_valid = 1U;
  s_res_new   = 1U;
  s_deadline_active = 0U;
}

/* 中止在飞运动观测(Stop 善后处统一调用): 清等待标志, 防 FAULT/取消后
   把未完成的运动误判为"到位" */
static void Host_MotionAbort(void)
{
  s_motion_pending   = 0U;
  s_st_pick_moving   = 0U;
  s_st_place_moving  = 0U;
  s_st_ret_moving    = 0U;
}

/* 开始/接替一个任务: 清空上次任务全部推断状态 */
static void HostJob_Reset(uint32_t task_id)
{
  s_job_task        = task_id;
  s_job_active      = 1U;
  s_job_seen_close  = 0U;
  s_job_open_started = 0U;
  s_job_placed      = 0U;
  s_job_ret         = 0U;
  s_job_concluded   = 0U;
  s_res_flags       = 0U;
  s_res_stage       = HOST_STAGE_NONE;
  s_res_valid       = 0U;
  s_res_new         = 0U;
  s_fault_seen      = 0U;
  s_progress        = 0U;
  Host_MotionAbort();
}

/**
  * @brief  起步前确认驱动器 QPos 参数(F1)已生效: 发送 -> 等应答 -> 校验状态
  * @retval 1 两台电机都确认(st=02); 0 重试 HOST_F1_TRY 次后仍未确认
  * @note   上电那次 F1 可能落在驱动器就绪之前被丢弃 -> 0xFC 按驱动器默认
  *         (相对)模式累加执行, 电机会跑飞(台架实测: 5cm 指令物理跑到 12~24cm,
  *         与 motion_debug 超程同根因)。这里发一次等一次应答, 丢包/被拒就
  *         重发; 等待期间 CAN RX 中断照常收应答(f1_count/f1_status 为
  *         volatile)。主循环上下文, F1 与 Tick 的 QPos 流是不同函数/静态
  *         缓冲, 可并存
  */
static uint8_t HostComm_F1Ensure(void)
{
  for (uint8_t try_i = 0U; try_i < HOST_F1_TRY; try_i++)
  {
    uint32_t n0 = StepMotor_ZDT[0].f1_count;
    uint32_t n1 = StepMotor_ZDT[1].f1_count;
    uint32_t t0 = SysTime_GetMs();

    Emm_V5_Set_QPos_Params(MS_XY_MOTOR_A_ADDR, MS_XY_QPOS_VEL_RPM, MS_XY_QPOS_ACC, 1U, false);
    Emm_V5_Set_QPos_Params(MS_XY_MOTOR_B_ADDR, MS_XY_QPOS_VEL_RPM, MS_XY_QPOS_ACC, 1U, false);

    while ((SysTime_GetMs() - t0) < HOST_F1_WAIT_MS)
    {
      if (StepMotor_ZDT[0].f1_count != n0 && StepMotor_ZDT[1].f1_count != n1)
      {
        break; /* 两台都回了, 不必等满 */
      }
    }

    if (StepMotor_ZDT[0].f1_count != n0 && StepMotor_ZDT[1].f1_count != n1 &&
        StepMotor_ZDT[0].f1_status == 0x02U && StepMotor_ZDT[1].f1_status == 0x02U)
    {
      return 1U;
    }
  }
  return 0U;
}

/* 当前协议态 STATE(急停>报警>运行>任务完成>待机; 回零/暂停预留) */
static uint8_t HostComm_CurState(void)
{
  MotionSys_XY_State_e st;

  if (s_estop)
  {
    return HOST_STATE_ESTOP;
  }
  st = MotionSys_XY_GetState();
  if (st == MS_XY_FAULT || s_error != 0U)
  {
    return HOST_STATE_ALARM;
  }
  if (s_motion_pending || s_grip_moving || MotionSys_XY_IsBusy())
  {
    return HOST_STATE_RUNNING;
  }
  if (s_job_active && !s_job_concluded)
  {
    return HOST_STATE_RUNNING;
  }
  if (s_res_valid)
  {
    return HOST_STATE_TASK_DONE;
  }
  return HOST_STATE_IDLE;
}

/* 当前推断 FAIL_STAGE: 用于失败/故障结果 */
static uint8_t HostComm_CurStage(void)
{
  if (s_job_ret)
  {
    return HOST_STAGE_RETURN;
  }
  if (s_motion_pending)
  {
    if (s_st_ret_moving)
    {
      return HOST_STAGE_RETURN;
    }
    if (s_st_place_moving)
    {
      return HOST_STAGE_PLACEGO;
    }
    if (s_st_pick_moving)
    {
      return HOST_STAGE_PICK;
    }
  }
  if (s_grip_moving)
  {
    return s_grip_is_open ? HOST_STAGE_PLACE : HOST_STAGE_GRIP;
  }
  if (s_job_open_started)
  {
    return HOST_STAGE_PLACE;
  }
  if (s_job_placed)
  {
    return HOST_STAGE_NONE;
  }
  if (s_job_seen_close)
  {
    return HOST_STAGE_PLACEGO;
  }
  return HOST_STAGE_PICK;
}

/* 急停善后: 停运动/看门狗, 夹爪动作中则报最接近终态 */
static void Host_EstopApply(void)
{
  MotionSys_XY_Stop();
  Host_MotionAbort();
  s_deadline_active = 0U;
  if (s_grip_moving)
  {
    s_grip_moving = 0U;
    s_grip_state  = s_grip_final;
  }
}

/**
  * @brief  分发一条已受理命令(主循环上下文)
  * @note   顺序: 清报警 -> 急停(同帧并存时急停生效, 安全优先)
  *         -> 急停锁存期拒绝其余命令 -> 取消任务 -> 夹爪 -> 运动。
  *         任务归属: 动作帧(MOTION/GRIP)按 TASK_ID 分组, 新 TASK_ID 视为
  *         接替上一任务(未了结则以"失败"了结)。
  */
static void HostComm_Dispatch(const HostCmd_t *cmd, uint32_t now)
{
  /* --- 若业务层注册了接管钩子, 优先由业务层处理 --- */
  if (s_cmd_hook != NULL)
  {
    if ((cmd->flags & (HOST_FLAG_MOTION | HOST_FLAG_GRIP | HOST_FLAG_PERMIT)) &&
        (!s_job_active || cmd->task_id != s_job_task))
    {
      HostJob_Reset(cmd->task_id);
    }
    if (s_cmd_hook(cmd))
    {
      return;
    }
  }

  /* --- bit3 清报警: 清 ERROR+急停锁存, FAULT 复位, 回到待机 --- */
  if (cmd->flags & HOST_FLAG_CLEAR)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_error = 0U;
    s_estop = 0U;
    __set_PRIMASK(primask);
    if (MotionSys_XY_GetState() == MS_XY_FAULT)
    {
      MotionSys_XY_Stop(); /* Stop 兼清 FAULT(主循环上下文发帧) */
    }
    Host_MotionAbort();
    s_cancel_active   = 0U;
    s_fault_seen      = 0U;
    s_deadline_active = 0U;
    s_job_active      = 0U;
    s_res_valid       = 0U;
    s_res_new         = 0U;
    s_progress        = 0U;
    return;
  }

  /* --- bit2 急停(锁存): 中止当前任务 --- */
  if (cmd->flags & HOST_FLAG_ESTOP)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_estop = 1U;
    __set_PRIMASK(primask);
    Host_ErrSet(HOST_ERR_ESTOP);
    Host_EstopApply();
    if (s_job_active && !s_job_concluded)
    {
      HostRes_EmitAbort(HOST_STAGE_ABORT);
    }
    return;
  }

  /* --- 急停锁存期: 除清报警外拒绝一切命令 --- */
  if (s_estop)
  {
    return;
  }

  /* --- bit4 取消当前任务 --- */
  if (cmd->flags & HOST_FLAG_CANCEL)
  {
    Host_EstopApply();
    if (s_job_active && !s_job_concluded)
    {
      HostRes_EmitAbort(HOST_STAGE_ABORT);
    }
    /* 主机可能已连发该任务后续段, 逐条丢弃 */
    s_cancel_active = 1U;
    s_cancel_task   = cmd->task_id;
    return;
  }

  /* --- 取消锁存期内, 丢同任务的动作帧 --- */
  if (s_cancel_active && (cmd->flags & (HOST_FLAG_MOTION | HOST_FLAG_GRIP)) &&
      cmd->task_id == s_cancel_task)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_stats.rx_drop++; /* 与 OnFrame(中断)异上下文递增, 须临界 */
    __set_PRIMASK(primask);
    return;
  }

  /* --- 动作帧(MOTION/GRIP): 任务归属 + TIMEOUT 受理 --- */
  if (cmd->flags & (HOST_FLAG_MOTION | HOST_FLAG_GRIP))
  {
    if (!s_job_active || cmd->task_id != s_job_task)
    {
      if (s_job_active && !s_job_concluded)
      {
        HostRes_EmitFail(HostComm_CurStage()); /* 上一任务被抢占, 以失败了结 */
      }
      HostJob_Reset(cmd->task_id);
    }
    s_cancel_active = 0U; /* 任一动作帧代表新意图, 解除取消锁存 */

    /* TIMEOUT: 以最新一条为准; 0=不引入也不解除既有检查
       (否则后发的 0 会抹掉前一条的保护, 挂死时永不超时) */
    if (cmd->timeout_ms > 0U)
    {
      s_deadline        = now + cmd->timeout_ms;
      s_deadline_active = 1U;
    }
  }

  /* --- bit1 夹爪: 动作期间报"动作中", 到位(按 grip 动作时长)后报终态 --- */
  if (cmd->flags & HOST_FLAG_GRIP)
  {
    switch (cmd->grip_cmd)
    {
    case HOST_GRIP_HOLD:
      break;
    case HOST_GRIP_OPEN:
      (void)Grip_Open();
      s_grip_state     = HOST_GRIP_ST_OPENING;
      s_grip_final     = HOST_GRIP_ST_OPEN;
      s_grip_is_open   = 1U;
      s_job_open_started = 1U;
      s_grip_moving    = 1U;
      break;
    case HOST_GRIP_CLOSE:
      (void)Grip_Close((uint8_t)HOST_GRIP_FORCE);
      s_grip_state     = HOST_GRIP_ST_MOVING;
      /* 力度参数>0 到位报"夹住", =0 报"合"(开环近似, 无夹持传感器) */
      s_grip_final     = (HOST_GRIP_FORCE > 0U) ? HOST_GRIP_ST_HELD : HOST_GRIP_ST_CLOSED;
      s_grip_is_open   = 0U;
      s_job_seen_close = 1U;
      s_grip_moving    = 1U;
      break;
    default:
      Host_ErrSet(HOST_ERR_PARAM); /* 校验层已拦, 防御分支 */
      break;
    }
  }

  /* --- bit0 运动: 速度由固件参数 HOST_MOVE_FEED 决定 --- */
  if (cmd->flags & HOST_FLAG_MOTION)
  {
    float x_mm = (float)cmd->x_um * 0.001f;
    float y_mm = (float)cmd->y_um * 0.001f;
    uint8_t stage;
    MotionSys_XY_Result_e ret;

    /* 阶段推断(顺序模板): 放置后再运动=返回; 已闭合后=去放置位; 否则=去取件位 */
    if (s_job_concluded && s_job_placed)
    {
      s_job_ret = 1U;
      stage = HOST_STAGE_RETURN;
    }
    else if (s_job_seen_close)
    {
      stage = HOST_STAGE_PLACEGO;
    }
    else
    {
      stage = HOST_STAGE_PICK;
    }

    /* 进度基准取下发前的参考位置 */
    MotionSys_XY_GetCmdPos(&s_prog_sx, &s_prog_sy);
    s_prog_tx = x_mm;
    s_prog_ty = y_mm;

    /* 起步前确认 QPos 参数(F1: V/acc/raF=1 绝对)已在驱动器生效; 未确认
       就拒绝本次运动(宁可不动, 也不能让 0xFC 按相对模式跑飞)。
       仅空闲起步时做, 避免在连续运动途中重设驱动器参数 */
    if (!s_motion_pending && !MotionSys_XY_IsBusy() && !HostComm_F1Ensure())
    {
      UART_Port_Printf(&uart1_port,
                       "[host] [FAIL] F1 not acked (A st=%02X n=%lu | B st=%02X n=%lu) -> move refused\r\n",
                       (unsigned)StepMotor_ZDT[0].f1_status, (unsigned long)StepMotor_ZDT[0].f1_count,
                       (unsigned)StepMotor_ZDT[1].f1_status, (unsigned long)StepMotor_ZDT[1].f1_count);
      Host_ErrSet(HOST_ERR_INTERNAL);
      if (s_job_active && !s_job_concluded)
      {
        HostRes_EmitFail(stage);
      }
      return;
    }

    /* 首次运动: MoveTo 的自动使能在入队之后才发帧, QPos 流(1ms 节拍)可能
       抢在驱动器使能完成前到达 -> 首批绝对目标被按相对/未就绪处理, 参考点
       漂移累积(台架现象: 40mm 冲到 251mm)。这里先显式使能并让出 ~10ms,
       再走正常入队路径 */
    if (s_first_move)
    {
      s_first_move = 0U;
      MotionSys_XY_Enable(true);
      for (volatile uint32_t dly = 0U; dly < 200000U; dly++)
      {
        __NOP();
      }
    }

    /* 一次性诊断: 首次运动前采集 CAN 邮箱/ESR + 两电机 F1(QPos 参数)应答。
       f1_status 02=生效 E2=参数被拒 EE=格式错; f1_count=0 说明 Init 那次
       设置根本没回(老根因: F1 上电丢失 -> 驱动器没进绝对位置模式)。
       只存不发, 由 HostComm_GetDiag 供主循环打印 */
    {
      if (!s_diag.done)
      {
        s_diag.free_mb = (uint8_t)HAL_CAN_GetTxMailboxesFreeLevel(can1_bus.hcan);
        s_diag.esr     = can1_bus.hcan->Instance->ESR;
        for (uint8_t mi = 0U; mi < 2U; mi++)
        {
          s_diag.f1_status[mi] = StepMotor_ZDT[mi].f1_status;
          s_diag.f1_count[mi]  = StepMotor_ZDT[mi].f1_count;
          s_diag.no_rx[mi]     = StepMotor_ZDT[mi].no_rx_count;
        }
        s_diag.done = 1U;
      }
    }

    ret = MotionSys_XY_MoveTo(x_mm, y_mm, HOST_MOVE_FEED);
    if (ret == MS_XY_OK)
    {
      s_motion_pending = 1U;
      switch (stage)
      {
      case HOST_STAGE_PICK:    s_st_pick_moving  = 1U; break;
      case HOST_STAGE_PLACEGO: s_st_place_moving = 1U; break;
      case HOST_STAGE_RETURN:  s_st_ret_moving   = 1U; break;
      default:                                         break;
      }
    }
    else
    {
      /* 入队失败: 记粘滞错误并了结当前任务(该段未执行)。
         UART1 诊断打印(一次性): 定位"受理但不走"的第一现场 */
      {
        static uint8_t s_movfail_printed;
        if (!s_movfail_printed)
        {
          s_movfail_printed = 1U;
          UART_Port_Printf(&uart1_port,
                           "[host] MoveTo FAIL ret=%d mst=%d | x=%ld y=%ld um\r\n",
                           (int)ret, (int)MotionSys_XY_GetState(),
                           (long)(x_mm * 1000.0f), (long)(y_mm * 1000.0f));
        }
      }
      if (ret == MS_XY_ERR_QUEUE_FULL)
      {
        Host_ErrSet(HOST_ERR_QUEUE);
      }
      else if (ret == MS_XY_ERR_BUSY || ret == MS_XY_ERR_NOT_INIT)
      {
        Host_ErrSet(HOST_ERR_INTERNAL);
      }
      else /* ERR_PARAM / ERR_LIMIT */
      {
        Host_ErrSet(HOST_ERR_PARAM);
      }
      if (s_job_active && !s_job_concluded)
      {
        HostRes_EmitFail(stage);
      }
    }
  }
}

/* 组一帧状态上报(主循环, 内容取快照保证一致)
   注: SEQ 只写入不推进 —— 发送失败会以同一 SEQ 重组重发, 仅成功发走的
   帧消耗序号, 保证上位机看到的 SEQ 连续、跳号即真丢帧 */
static void HostComm_BuildReport(uint8_t *b, uint8_t state, uint8_t grip,
                                 uint16_t error, uint16_t ack, uint32_t task,
                                 int32_t x_um, int32_t y_um, uint8_t uf)
{
  b[0] = HOST_SOF0;
  b[1] = HOST_SOF1;
  b[2] = HOST_VER;
  b[3] = HOST_TYPE_STATUS;
  b[4] = uf;
  H_PutU16(&b[5], s_tx_seq);
  H_PutU32(&b[7], task);
  H_PutI32(&b[11], x_um); /* CUR_X/CUR_Y: 闭环反馈实际位置(调用方已换算) */
  H_PutI32(&b[15], y_um);
  b[19] = state;
  b[20] = grip;
  H_PutU16(&b[21], error);
  H_PutU16(&b[23], ack);
  b[25] = s_progress;
  b[26] = s_res_flags;
  b[27] = s_res_stage;
  b[28] = 0U; /* RESERVED[4] 全零(代替 CRC) */
  b[29] = 0U;
  b[30] = 0U;
  b[31] = 0U;
}

/* ========================================================================== */
/*  对外接口                                                                   */
/* ========================================================================== */

void HostComm_Init(void)
{
  if (s_inited)
  {
    return;
  }

  /* 运动系统(幂等; 内部自订阅 3ms 反馈轮询 + 1ms 轨迹节拍) */
  MotionSys_XY_Init();

  /* 夹爪机构(幂等; 内部含 Servo_Init, 上电姿态 = Z 抬起 + 夹爪张开),
     使上报初值(0开)与实际一致 */
  Grip_Init();
  s_grip_state   = HOST_GRIP_ST_OPEN;
  s_grip_final   = HOST_GRIP_ST_OPEN;
  s_grip_moving  = 0U;
  s_grip_is_open = 0U;
  s_first_move   = 1U; /* 首次运动前显式使能(见 Dispatch) */

  /* USB CDC 绑定本模块接收回调(与 usb_debug 互斥, 见 debug.h 分层说明) */
  USB_Init(HostComm_OnRx);

  /* 心跳 + 超时看门狗节拍(必须在 Task_IT_Init 之后; 事件触发上报, 无周期帧) */
  TIM_Timer_Subscribe(&tim7_timer, HostComm_Tick, HOST_REPORT_MS);

  s_inited = 1U;
}

void HostComm_Tick(void)
{
  uint32_t now = SysTime_GetMs();

  if (!s_inited)
  {
    return;
  }

  /* 命令 TIMEOUT 看门狗: 到期由主循环善后(Stop + 中止结果), 一次性触发 */
  if (s_deadline_active && ((int32_t)(now - s_deadline) >= 0))
  {
    s_deadline_active = 0U;
    s_timeout_flag = 1U;
  }

#if (HOST_LINK_TIMEOUT_MS > 0U)
  /* 链路看门狗(仿 Klipper 主机失联保护): 无有效下行 -> 按急停处理 */
  if ((now - s_last_rx_tick) >= HOST_LINK_TIMEOUT_MS)
  {
    s_link_flag = 1U;
    s_last_rx_tick = now; /* 防重复置位刷屏 */
  }
#endif
}

void HostComm_Poll(void)
{
  uint32_t now = SysTime_GetMs();
  uint8_t state, grip;
  uint16_t error, ack;
  uint32_t task;

  if (!s_inited)
  {
    return;
  }

  /* ---- 1. 命令超时善后(主循环上下文发 Stop) ---- */
  if (s_timeout_flag)
  {
    s_timeout_flag = 0U;
    if (s_job_active && !s_job_concluded)
    {
      MotionSys_XY_Stop();
      Host_MotionAbort();
      s_deadline_active = 0U;
      Host_ErrSet(HOST_ERR_TIMEOUT | HOST_ERR_NOTREACH);
      HostRes_EmitAbort(HOST_STAGE_ABORT);
    }
    else
    {
      s_deadline_active = 0U;
    }
  }
#if (HOST_LINK_TIMEOUT_MS > 0U)
  if (s_link_flag)
  {
    s_link_flag = 0U;
    {
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      s_estop = 1U;
      __set_PRIMASK(primask);
    }
    MotionSys_XY_Stop();
    Host_MotionAbort();
    s_deadline_active = 0U;
    Host_ErrSet(HOST_ERR_COMM | HOST_ERR_NOTREACH);
    if (s_job_active && !s_job_concluded)
    {
      HostRes_EmitAbort(HOST_STAGE_ABORT);
    }
  }
#endif

  /* ---- 2. 排空命令队列并分发 ---- */
  while (s_fifo_tail != s_fifo_head)
  {
    HostCmd_t cmd = s_fifo[s_fifo_tail % HOST_CMD_FIFO_LEN];
    s_fifo_tail++;
    HostComm_Dispatch(&cmd, now);
  }

  /* ---- 3. 夹爪"动作中"到位判定(无反馈, 由 grip 模块按动作时长近似) ---- */
  if (s_grip_moving && !Grip_JawIsBusy())
  {
    s_grip_moving = 0U;
    s_grip_state  = s_grip_final;
    if (s_grip_is_open)
    {
      /* 张开(放置)完成 -> PLACE_CONFIRMED(无传感器时=到达并执行张开) */
      s_res_flags |= HOST_RES_CONFIRM;
    }
    s_grip_is_open = 0U;
  }

  /* ---- 4. 运动完成判定(IDLE=队列排空; 零长段视为已到位) ---- */
  if (s_motion_pending && !MotionSys_XY_IsBusy())
  {
    s_motion_pending = 0U;
    if (s_job_active)
    {
      if (s_st_pick_moving)
      {
        s_res_flags |= HOST_RES_PICK;   /* 到达取件位 */
      }
      if (s_st_place_moving)
      {
        s_res_flags |= HOST_RES_PLACE;  /* 到达放置位 */
      }
      if (s_st_ret_moving)
      {
        s_res_new = 1U; /* 返回段完成: 再次补发(保持既有成功结果) */
      }
    }
    Host_MotionAbort();
  }

  /* ---- 4.5 跟随误差保护: 运动中反馈实际偏离参考过大 -> 立即停止。
       失控(如驱动器按相对模式累加)时目标会以 ~500mm/s 冲出去, 正常闭环
       跟随误差 <1mm, 阈值 10mm 有足量裕度, 失控时约 20ms 内即触发 ---- */
  if (s_motion_pending && MotionSys_XY_IsBusy())
  {
    float cx, cy, ax, ay;
    int32_t ex_um, ey_um;

    MotionSys_XY_GetCmdPos(&cx, &cy);
    MotionSys_XY_GetActualPos(&ax, &ay);
    ex_um = H_MmToUm(ax - cx);
    ey_um = H_MmToUm(ay - cy);
    if (ex_um < 0) { ex_um = -ex_um; }
    if (ey_um < 0) { ey_um = -ey_um; }
    if (ex_um > (int32_t)HOST_FOLLOW_ERR_UM || ey_um > (int32_t)HOST_FOLLOW_ERR_UM)
    {
      UART_Port_Printf(&uart1_port,
                       "[host] [FAIL] follow err (%ld,%ld)um > %lum -> STOP\r\n",
                       (long)ex_um, (long)ey_um, (unsigned long)HOST_FOLLOW_ERR_UM);
      MotionSys_XY_Stop();
      Host_MotionAbort();
      s_deadline_active = 0U;
      Host_ErrSet(HOST_ERR_INTERNAL | HOST_ERR_NOTREACH);
      if (s_job_active && !s_job_concluded)
      {
        HostRes_EmitFail(HostComm_CurStage());
      }
    }
  }

  /* ---- 5. 运动系统故障观察: Stop 善后 + 任务以失败了结 ---- */
  if (MotionSys_XY_GetState() == MS_XY_FAULT)
  {
    if (!s_fault_seen)
    {
      s_fault_seen = 1U;
      Host_ErrSet(HOST_ERR_INTERNAL);
    }
    MotionSys_XY_Stop(); /* 兼清 FAULT */
    Host_MotionAbort();
    s_deadline_active = 0U;
    if (s_job_active && !s_job_concluded)
    {
      HostRes_EmitFail(HostComm_CurStage());
    }
  }

  /* ---- 6. 任务自然结束判定(放置完成且设备空闲) ---- */
  if (s_job_active && !s_job_concluded && s_job_open_started && !s_job_placed &&
      !s_grip_moving && !s_motion_pending && !MotionSys_XY_IsBusy())
  {
    HostRes_EmitSuccess();
  }

  /* ---- 7. 进度推算(参考位置沿 (起点->目标) 投影比例 0~100) ---- */
  if (s_motion_pending || MotionSys_XY_IsBusy())
  {
    float cx, cy;
    float dx = s_prog_tx - s_prog_sx;
    float dy = s_prog_ty - s_prog_sy;
    float d2 = dx * dx + dy * dy;

    MotionSys_XY_GetCmdPos(&cx, &cy);
    if (d2 < 1e-6f)
    {
      s_progress = 100U;
    }
    else
    {
      float t = ((cx - s_prog_sx) * dx + (cy - s_prog_sy) * dy) / d2;
      if (t < 0.0f) { t = 0.0f; }
      if (t > 1.0f) { t = 1.0f; }
      s_progress = (uint8_t)(t * 100.0f + 0.5f);
    }
  }
  else if (s_job_concluded)
  {
    s_progress = 100U;
  }

  /* ---- 8. 状态上报(事件触发 + 位置限频):
       语义事件(首帧/结果/ACK/ERROR/STATE/GRIPPER/TASK)不限频立即发;
       纯位置变化按 HOST_POS_REPORT_MS 限频, 防运动时刷屏占满主循环。
       上位机掉线检测: 有在途命令(TIMEOUT>0)而超时未收到结果帧 => 掉线 */
  state = HostComm_CurState();
  grip  = s_grip_state;
  {
    float ax, ay;
    int32_t x_um, y_um;

    MotionSys_XY_GetActualPos(&ax, &ay);
    x_um = H_MmToUm(ax);
    y_um = H_MmToUm(ay);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    error = s_error;   /* 快照(与构建帧一致) */
    ack   = s_seq_valid ? s_ack_seq : 0xFFFFU; /* 0xFFFF = 尚未受理过任何帧 */
    __set_PRIMASK(primask);

    task = s_job_task;

    uint8_t uf = 0U;
    if (s_res_new)                    { uf |= HOST_UF_TASK_DONE; } /* 结果事件优先 */
    if (s_res_valid)                  { uf |= HOST_UF_RESULT; }
    if (s_estop)                      { uf |= HOST_UF_ESTOP; }
    if (error != 0U)                  { uf |= HOST_UF_ALARM; }
    if (s_motion_pending || s_grip_moving || MotionSys_XY_IsBusy() ||
        (s_job_active && !s_job_concluded)) { uf |= HOST_UF_BUSY; }
    if (s_usb_ok)                     { uf |= HOST_UF_USB_OK; }

    /* 触发分类: 语义事件立即发; 位置变化受 50ms 限频(运动中最多 20Hz);
       重试限速: 上次尝试失败(USB 忙/未枚举)后至少要隔 HOST_RETRY_MS 再试,
       否则主循环会按自身速率空转刷 tx_error 与统计(台架曾见 seq≈34k 而
       实际只发出 899 帧) */
    uint8_t pos_changed = (x_um != s_sent_x || y_um != s_sent_y);
    uint8_t semantic = (!s_sent_valid || s_res_new ||
                        ack != s_sent_ack || error != s_sent_error ||
                        state != s_sent_state || grip != s_sent_grip ||
                        task != s_sent_task);
    uint8_t due = (now - s_last_pos_ms) >= HOST_POS_REPORT_MS;
    uint8_t retry_due = (now - s_last_try_ms) >= HOST_RETRY_MS;

    if ((semantic || (pos_changed && due)) && retry_due)
    {
      s_last_try_ms = now;
      HostComm_BuildReport(s_tx_frame, state, grip, error, ack, task, x_um, y_um, uf);
      switch (USB_Transmit_Data(s_tx_frame, HOST_FRAME_LEN))
      {
      case USBD_OK: /* 发送成功: 记录已发快照, SEQ 随成功发走的帧推进 */
        s_tx_seq++;
        s_stats.tx_frames++;
        s_sent_valid = 1U;
        s_sent_ack   = ack;
        s_sent_error = error;
        s_sent_state = state;
        s_sent_grip  = grip;
        s_sent_task  = task;
        s_sent_x     = x_um;
        s_sent_y     = y_um;
        s_last_pos_ms = now;
        s_res_new    = 0U;
        s_usb_ok     = 1U;
        break;
      case USBD_BUSY: /* 忙: 保留触发条件, ≥HOST_RETRY_MS 后以最新内容重试 */
        s_stats.tx_busy++;
        s_usb_ok = 1U; /* 忙=已配置, 链路仍在 */
        break;
      default: /* 未枚举/未配置: 不清触发条件, 枚举后重试 */
        s_stats.tx_busy++;
        s_usb_ok = 0U;
        break;
      }
    }
  }
}

void HostComm_GetStats(HostCommStats_t *out)
{
  if (out == NULL)
  {
    return;
  }
  out->rx_frames  = s_stats.rx_frames;
  out->rx_bad_ver = s_stats.rx_bad_ver;
  out->rx_unknown = s_stats.rx_unknown;
  out->rx_rej     = s_stats.rx_rej;
  out->rx_dup     = s_stats.rx_dup;
  out->rx_drop    = s_stats.rx_drop;
  out->tx_frames  = s_stats.tx_frames;
  out->tx_busy    = s_stats.tx_busy;
}

void HostComm_GetDiag(HostDiag_t *out)
{
  if (out == NULL)
  {
    return;
  }
  *out = s_diag; /* 单字字段, 撕裂可接受(诊断用) */
}

/* ========================================================================== */
/*  USB 中断上下文                                                             */
/* ========================================================================== */

/* USB 接收回调(中断上下文): 逐字节组帧, 一次回调可能含多帧/半帧 */
static void HostComm_OnRx(uint8_t *buf, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    uint8_t byte = buf[i];

    if (s_rx_state == H_RX_IDLE)
    {
      /* 同步: 连续 AA AA 才进 BODY —— 单 AA 后跟垃圾(如串口助手追加的
         CR/LF)不会破坏同步, 下一字节不是 AA 则继续丢弃 */
      if (byte != HOST_SOF0)
      {
        continue;
      }
      s_rx_frame[0] = byte;
      s_rx_idx = 1U;
      s_rx_state = H_RX_BODY;
    }
    else /* H_RX_BODY */
    {
      s_rx_frame[s_rx_idx++] = byte;
      if (s_rx_idx >= HOST_FRAME_LEN)
      {
        s_rx_state = H_RX_IDLE;
        HostComm_OnFrame(s_rx_frame); /* 组帧完成 -> 校验/受理 */
      }
    }
  }
}

/**
  * @brief  下行帧校验与受理(USB 中断上下文)
  * @note   顺序仿 Klipper check_packet(): VER -> TYPE -> SEQ 去重 -> 字段
  *         范围 -> 受理入队。只有受理成功才推进 ACK_SEQ;
  *         任何被拒帧只记 ERROR/统计, 不推进 ACK(上位机据 SEQ 未被 ACK
  *         决定重发)。重复 SEQ 是主机正常重发: 静默维持 ACK, 不执行不记错。
  */
static void HostComm_OnFrame(const uint8_t *f)
{
  uint16_t seq;
  HostCmd_t cmd;

  /* 1) 版本 */
  if (f[2] != HOST_VER)
  {
    s_stats.rx_bad_ver++;
    Host_ErrSet(HOST_ERR_PROTO);
    return;
  }
  /* 2) 帧类型(仅 0x01=命令; 未知类型拒绝且不 ACK) */
  if (f[3] != HOST_TYPE_CMD)
  {
    s_stats.rx_unknown++;
    Host_ErrSet(HOST_ERR_PROTO);
    return;
  }

  seq = H_GetU16(&f[5]);

  /* 3) SEQ 去重(主机重发): 静默重 ACK, 不再执行 */
  if (s_seq_valid && seq == s_ack_seq)
  {
    s_stats.rx_dup++;
#if (HOST_LINK_TIMEOUT_MS > 0U)
    s_last_rx_tick = SysTime_GetMs();
#endif
    return;
  }

  /* 4) 字段范围(拒绝即不 ACK)
     GRIP_CMD 只允许 0/1/2; 其余字段(坐标/超时)按协议定长取用,
     MOTION_VALID/GRIP_VALID 决定坐标/夹爪是否生效, 由 Dispatch 门控 */
  if (f[19] > HOST_GRIP_CLOSE)
  {
    s_stats.rx_rej++;
    Host_ErrSet(HOST_ERR_PARAM);
    return;
  }

  /* 5) 受理入队(队列满则不 ACK, 由上位机重发) */
  if ((uint8_t)(s_fifo_head - s_fifo_tail) >= HOST_CMD_FIFO_LEN)
  {
    s_stats.rx_drop++;
    return;
  }

  cmd.seq         = seq;
  cmd.task_id     = H_GetU32(&f[7]);
  cmd.x_um        = (int32_t)H_GetU32(&f[11]);
  cmd.y_um        = (int32_t)H_GetU32(&f[15]);
  cmd.flags       = f[4];
  cmd.grip_cmd    = f[19];
  cmd.timeout_ms  = H_GetU16(&f[20]);

  s_fifo[s_fifo_head % HOST_CMD_FIFO_LEN] = cmd;
  s_fifo_head++; /* 先写数据后推进 head: 消费者可见性保证(中断不嵌套本回调) */

  s_ack_seq   = seq;
  s_seq_valid = 1U;
  s_stats.rx_frames++;
#if (HOST_LINK_TIMEOUT_MS > 0U)
  s_last_rx_tick = SysTime_GetMs();
#endif
}

/* ========================================================================== */
/*  业务层 (app.c) 协同接口实现                                                 */
/* ========================================================================== */

void HostComm_RegisterCmdHook(HostCmdHook_fn hook)
{
  s_cmd_hook = hook;
}

void HostComm_ReportSuccess(void)
{
  HostRes_EmitSuccess();
}

void HostComm_ReportFail(uint8_t stage, uint16_t err)
{
  if (err != 0U)
  {
    Host_ErrSet(err);
  }
  HostRes_EmitFail(stage);
}

void HostComm_ReportAbort(uint8_t stage)
{
  HostRes_EmitAbort(stage);
}

/* ========================================================================== */
/*  上位机通用封包/解包便捷工具实现                                            */
/* ========================================================================== */

void HostComm_PackCmd(uint8_t out_frame[HOST_FRAME_LEN], const HostCmd_t *cmd)
{
  if (out_frame == NULL || cmd == NULL)
  {
    return;
  }

  memset(out_frame, 0, HOST_FRAME_LEN);
  out_frame[0] = HOST_SOF0;
  out_frame[1] = HOST_SOF1;
  out_frame[2] = HOST_VER;
  out_frame[3] = HOST_TYPE_CMD;
  out_frame[4] = cmd->flags;

  H_PutU16(&out_frame[5], cmd->seq);
  H_PutU32(&out_frame[7], cmd->task_id);
  H_PutI32(&out_frame[11], cmd->x_um);
  H_PutI32(&out_frame[15], cmd->y_um);

  out_frame[19] = cmd->grip_cmd;
  H_PutU16(&out_frame[20], cmd->timeout_ms);
}

bool HostComm_UnpackStatus(const uint8_t in_frame[HOST_FRAME_LEN], HostStatus_t *out_status)
{
  if (in_frame == NULL || out_status == NULL)
  {
    return false;
  }

  if (in_frame[0] != HOST_SOF0 || in_frame[1] != HOST_SOF1 ||
      in_frame[2] != HOST_VER || in_frame[3] != HOST_TYPE_STATUS)
  {
    return false;
  }

  out_status->flags     = in_frame[4];
  out_status->seq       = H_GetU16(&in_frame[5]);
  out_status->task_id   = H_GetU32(&in_frame[7]);
  out_status->cur_x_um  = (int32_t)H_GetU32(&in_frame[11]);
  out_status->cur_y_um  = (int32_t)H_GetU32(&in_frame[15]);
  out_status->cur_x_mm  = (float)out_status->cur_x_um * 0.001f;
  out_status->cur_y_mm  = (float)out_status->cur_y_um * 0.001f;
  out_status->state     = in_frame[19];
  out_status->gripper   = in_frame[20];
  out_status->error     = H_GetU16(&in_frame[21]);
  out_status->ack_seq   = H_GetU16(&in_frame[23]);
  out_status->progress  = in_frame[25];
  out_status->res_flags = in_frame[26];
  out_status->res_stage = in_frame[27];

  return true;
}

void HostComm_BuildMoveCmd(uint8_t out_frame[HOST_FRAME_LEN], uint16_t seq, uint32_t task_id,
                           float x_mm, float y_mm, uint16_t timeout_ms)
{
  HostCmd_t cmd;
  cmd.seq        = seq;
  cmd.task_id    = task_id;
  cmd.flags      = HOST_FLAG_MOTION;
  cmd.x_um       = (int32_t)(x_mm * 1000.0f + (x_mm >= 0.0f ? 0.5f : -0.5f));
  cmd.y_um       = (int32_t)(y_mm * 1000.0f + (y_mm >= 0.0f ? 0.5f : -0.5f));
  cmd.grip_cmd   = HOST_GRIP_HOLD;
  cmd.timeout_ms = timeout_ms;

  HostComm_PackCmd(out_frame, &cmd);
}

void HostComm_BuildPermitCmd(uint8_t out_frame[HOST_FRAME_LEN], uint16_t seq, uint32_t task_id)
{
  HostCmd_t cmd;
  cmd.seq        = seq;
  cmd.task_id    = task_id;
  cmd.flags      = HOST_FLAG_PERMIT;
  cmd.x_um       = 0;
  cmd.y_um       = 0;
  cmd.grip_cmd   = HOST_GRIP_HOLD;
  cmd.timeout_ms = 0;

  HostComm_PackCmd(out_frame, &cmd);
}

void HostComm_BuildEstopCmd(uint8_t out_frame[HOST_FRAME_LEN], uint16_t seq)
{
  HostCmd_t cmd;
  cmd.seq        = seq;
  cmd.task_id    = 0;
  cmd.flags      = HOST_FLAG_ESTOP;
  cmd.x_um       = 0;
  cmd.y_um       = 0;
  cmd.grip_cmd   = HOST_GRIP_HOLD;
  cmd.timeout_ms = 0;

  HostComm_PackCmd(out_frame, &cmd);
}

void HostComm_BuildClearCmd(uint8_t out_frame[HOST_FRAME_LEN], uint16_t seq)
{
  HostCmd_t cmd;
  cmd.seq        = seq;
  cmd.task_id    = 0;
  cmd.flags      = HOST_FLAG_CLEAR;
  cmd.x_um       = 0;
  cmd.y_um       = 0;
  cmd.grip_cmd   = HOST_GRIP_HOLD;
  cmd.timeout_ms = 0;

  HostComm_PackCmd(out_frame, &cmd);
}

void HostComm_BuildCancelCmd(uint8_t out_frame[HOST_FRAME_LEN], uint16_t seq, uint32_t task_id)
{
  HostCmd_t cmd;
  cmd.seq        = seq;
  cmd.task_id    = task_id;
  cmd.flags      = HOST_FLAG_CANCEL;
  cmd.x_um       = 0;
  cmd.y_um       = 0;
  cmd.grip_cmd   = HOST_GRIP_HOLD;
  cmd.timeout_ms = 0;

  HostComm_PackCmd(out_frame, &cmd);
}

