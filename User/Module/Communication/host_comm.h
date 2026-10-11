/**
 * @file    host_comm.h
 * @brief   上位机通讯模块(接口层): 32 字节定长二进制帧, USB CDC 收发
 *
 *          ===== 帧格式(用户协议规格, 小端) =====
 *          下行(上位机 -> STM32), 固定 32 字节:
 *            [0]  2  SOF       固定 0xAA 0x55
 *            [2]  1  VER       协议版本, 0x01
 *            [3]  1  TYPE      0x01 = 命令(坐标/夹爪/急停/清报警/取消)
 *            [4]  1  FLAGS     见下
 *            [5]  2  SEQ       上位机递增序号 0~65535 循环
 *            [7]  4  TASK_ID   任务 ID, u32(同任务的多条命令共享)
 *            [11] 4  X         目标 X, int32, 0.001mm(MOTION_VALID 时有效)
 *            [15] 4  Y         目标 Y, int32, 0.001mm
 *            [19] 1  GRIP_CMD  0保持 1张开 2闭合(GRIP_VALID 时有效)
 *            [20] 2  TIMEOUT   本条命令超时, u16, ms (0=本条不检查, 且不解除
 *                             既有检查; 显式>0 时以最新一条为准)
 *            [22] 10 RESERVED  填 0(代替 CRC)
 *          注: 速度(SPEED)与夹爪力度(GRIP_FORCE)已从协议移除, 改由
 *          下位机固件参数决定: HOST_MOVE_FEED / HOST_GRIP_FORCE。
 *
 *          上行(STM32 -> 上位机), 固定 32 字节:
 *            [0]  2  SOF       0xAA 0x55
 *            [2]  1  VER       0x01
 *            [3]  1  TYPE      0x81 = 状态/任务结果(事件触发)
 *            [4]  1  FLAGS     见下
 *            [5]  2  SEQ       STM32 自身上报序号(每成功发出一帧+1; 发失败
 *                             不推进, 上位机看到跳号即真丢帧)
 *            [7]  4  TASK_ID   当前/最近任务 ID
 *            [11] 4  CUR_X     闭环反馈实际 X, int32, 0.001mm
 *            [15] 4  CUR_Y     闭环反馈实际 Y, int32, 0.001mm
 *            [19] 1  STATE     0待机 1回零 2运行 3暂停 4报警 5急停 6任务完成
 *            [20] 1  GRIPPER   0开 1合(力0) 2闭合中 3夹住(力>0) 4张开中
 *                             5空夹 6脱落
 *            [21] 2  ERROR     粘滞错误码, 按位或(见 HOST_ERR_*)
 *            [23] 2  ACK_SEQ   已受理的下行 SEQ; 未受理过任何帧时发 0xFFFF
 *            [25] 1  PROGRESS  0~100, 参考位置沿当前运动段(起点->目标)投影比例
 *            [26] 1  RESULT_FLAGS 任务结果位(见 HOST_RES_*)
 *            [27] 1  FAIL_STAGE  失败阶段(见 HOST_STAGE_*)
 *            [28] 4  RESERVED  填 0(代替 CRC)
 *
 *          ===== 关键语义(仿 Klipper / Moonraker) =====
 *            - ACK 受理制   <- Klipper gcode: 帧校验通过并受理(入命令队列)
 *                              才更新 ACK_SEQ; 受理失败(队列满)不更新, 上位机
 *                              据"SEQ 未被 ACK"重发。
 *            - 粘滞错误     <- Klipper shutdown 锁存: ERROR 置位后保持, 只有
 *                              下行 FLAGS.CLEAR_ALARM 显式清除, 绝不自动恢复。
 *            - 急停锁存     <- M112 语义: 急停后除"清报警"外拒绝一切命令。
 *            - 任务结果     <- TASK_DONE_EVENT: 任务结束后立即补发一帧 0x81,
 *                              后续上报保持最近一次结果(RESULT_VALID=1)。
 *            - 掉线检测     <- 上下行均事件触发, 无周期心跳: 静止时完全静默。
 *                              上位机判下位机掉线: 有在途命令(TIMEOUT>0)而
 *                              超时未收到结果/超时帧 => 掉线。
 *            - 事件上报     <- 0x81 仅在事件发生时发送: 首帧 / ACK 推进 /
 *                              ERROR、STATE、GRIPPER、TASK_ID 变化 / 位置变化
 *                              (10µm 量化) / 任务结果补发。仅主循环发送。
 *
 *          ===== 任务结束自动判定(顺序模板) =====
 *            下行帧无显式"任务结束"标志, 本模块按同一 TASK_ID 的命令到达顺序
 *            推断阶段(见 host_comm.c):
 *              第1段运动       -> 去取件位(FAIL_STAGE 1), 到位置 PICK_REACHED
 *              夹爪闭合        -> 夹取(2)
 *              第2段运动       -> 去放置位(3/4), 到位置 PLACE_REACHED
 *              夹爪张开        -> 放置(5), 张开完成置 PLACE_CONFIRMED
 *              张开完成且设备空闲 -> 任务自然结束, 立即补发成功结果
 *              其后同 TASK_ID 运动 -> 返回段(6), 完成后再次补发结果
 *            纯移动(无夹取)任务不自动结束: 需新任务/取消/急停/超时了结。
 *
 *          ===== 使用约定 =====
 *            - HostComm_Init()  在 debug_init 中、Task_IT_Init 之后调用;
 *              内部会: MotionSys_XY_Init()(幂等) -> Grip_Init()(夹爪双舵机,
 *              幂等) -> USB_Init 绑定本模块回调 -> 订阅 TIM7 50ms 节拍。
 *            - HostComm_Poll()  在主循环(debug_run)每圈调用: 排空命令队列
 *              分发执行、夹爪到位判定、任务结束判定、超时善后、事件触发上报。
 *            - HostComm_Tick()  由 TIM7 自动调用: 命令超时看门狗 + 链路
 *              看门狗(只置标志, 不在中断里发帧)。
 *            - 分层互斥: host_debug=1 时不要同开 motion_debug(运动测试循环
 *              会抢运动系统)、usb_debug(回显回调会抢占 USB_Init 绑定)、
 *              grip_debug / servo_debug(重复独占同一夹爪)。
 */

#ifndef __HOST_COMM_H__
#define __HOST_COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* ========================================================================== */
/*  协议常量                                                                   */
/* ========================================================================== */

#define HOST_FRAME_LEN     32U   /* 上下行帧长(定长) */
#define HOST_VER           0x01U /* 协议版本 */
#define HOST_SOF0          0xAAU
#define HOST_SOF1          0x55U

#define HOST_TYPE_CMD      0x01U /* 下行: 命令 */
#define HOST_TYPE_STATUS   0x81U /* 上行: 状态/任务结果(事件触发) */

/* ---- 下行 FLAGS ---- */
#define HOST_FLAG_MOTION   0x01U /* bit0: MOTION_VALID, X/Y 有效, 要求运动到目标点 */
#define HOST_FLAG_GRIP     0x02U /* bit1: GRIP_VALID, GRIP_CMD 有效 */
#define HOST_FLAG_ESTOP    0x04U /* bit2: 急停(锁存) */
#define HOST_FLAG_CLEAR    0x08U /* bit3: 清报警(清除 ERROR + 急停锁存) */
#define HOST_FLAG_CANCEL   0x10U /* bit4: 取消当前任务 */

/* ---- 下行 GRIP_CMD ---- */
#define HOST_GRIP_HOLD     0U    /* 保持不动 */
#define HOST_GRIP_OPEN     1U    /* 张开 */
#define HOST_GRIP_CLOSE    2U    /* 闭合 */

/* ---- 上行 FLAGS ---- */
#define HOST_UF_TASK_DONE  0x02U /* bit1: 任务刚结束, 本帧为结果事件补发 */
#define HOST_UF_RESULT     0x04U /* bit2: RESULT_FLAGS / FAIL_STAGE 有效 */
#define HOST_UF_ESTOP      0x08U /* bit3: 急停锁存 */
#define HOST_UF_ALARM      0x10U /* bit4: ERROR 非 0 */
#define HOST_UF_BUSY       0x20U /* bit5: 正在执行任务 */
#define HOST_UF_USB_OK     0x40U /* bit6: USB CDC 已配置且可发 */
/* bit0(HEARTBEAT)保留: 上报改为纯事件触发后无周期心跳帧, 恒填 0 */

/* ---- 上行 STATE(与用户规格一致) ---- */
#define HOST_STATE_IDLE      0U  /* 待机 */
#define HOST_STATE_HOMING    1U  /* 回零(预留: 运动模块暂无回零, 永不上报) */
#define HOST_STATE_RUNNING   2U  /* 运行(任务进行中) */
#define HOST_STATE_PAUSE     3U  /* 暂停(预留: 运动模块暂无暂停, 永不上报) */
#define HOST_STATE_ALARM     4U  /* 报警(运动 FAULT / ERROR 粘滞, 锁存到清报警) */
#define HOST_STATE_ESTOP     5U  /* 急停(锁存到清报警) */
#define HOST_STATE_TASK_DONE 6U  /* 任务完成(最近任务已了结, 结果有效) */

/* ---- 上行 GRIPPER ---- */
#define HOST_GRIP_ST_OPEN    0U  /* 张开到位 */
#define HOST_GRIP_ST_CLOSED  1U  /* 闭合到位但力=0(未夹持); 无力度参数时不产生 */
#define HOST_GRIP_ST_MOVING  2U  /* 闭合动作中 */
#define HOST_GRIP_ST_HELD    3U  /* 夹住, 力>0 */
#define HOST_GRIP_ST_OPENING 4U  /* 张开动作中 */
#define HOST_GRIP_ST_EMPTY   5U  /* 空夹(需夹爪空检测传感器, 当前不产生) */
#define HOST_GRIP_ST_SLIP    6U  /* 夹取脱落(需传感器, 当前不产生) */

/* ---- 上行 ERROR 位(粘滞/按位或, 仅 FLAGS.CLEAR_ALARM 清除) ---- */
#define HOST_ERR_PROTO     0x0001U /* 协议错误(VER/TYPE 不识别), 帧被拒 */
#define HOST_ERR_PARAM     0x0002U /* 字段越界(GRIP_CMD>2 等), 帧被拒 */
#define HOST_ERR_QUEUE     0x0004U /* 命令队列满/运动规划队满, 命令被拒 */
#define HOST_ERR_TIMEOUT   0x0008U /* 命令 TIMEOUT 超时, 已停运动 */
#define HOST_ERR_NOTREACH  0x0010U /* 未到目标(运动超时/取消导致未到位) */
#define HOST_ERR_GRIPFAIL  0x0020U /* 夹取失败(需传感器, 当前不产生) */
#define HOST_ERR_GRIPSLIP  0x0040U /* 夹取脱落(需传感器, 当前不产生) */
#define HOST_ERR_PLACEFAIL 0x0080U /* 放置失败(需传感器, 当前不产生) */
#define HOST_ERR_ESTOP     0x0100U /* 急停锁存 */
#define HOST_ERR_INTERNAL  0x0200U /* 内部故障(运动系统 FAULT/通信异常) */
#define HOST_ERR_COMM      0x0400U /* 通信异常/掉线恢复(链路看门狗触发) */

/* ---- 上行 RESULT_FLAGS(任务结果位) ---- */
#define HOST_RES_DONE      0x01U /* 任务结束, 成功或失败 */
#define HOST_RES_OK        0x02U /* 任务成功 */
#define HOST_RES_PICK      0x04U /* 到达取件位 */
#define HOST_RES_PLACE     0x08U /* 到达放置位 */
#define HOST_RES_GRIPEMPTY 0x10U /* 未完成夹取/空夹(需传感器, 当前不产生) */
#define HOST_RES_GRIPSLIP  0x20U /* 夹取脱落(需传感器, 当前不产生) */
#define HOST_RES_CONFIRM   0x40U /* 已放置到指定位置确认(无传感器时=到达+张开) */
#define HOST_RES_ABORT     0x80U /* 超时/中止/急停 */

/* ---- 上行 FAIL_STAGE(失败阶段) ---- */
#define HOST_STAGE_NONE     0U /* 无/不适用 */
#define HOST_STAGE_PICK     1U /* 去取件位 */
#define HOST_STAGE_GRIP     2U /* 夹取 */
#define HOST_STAGE_TRANSPORT 3U /* 运输(与去放置位合并, 见 HOST_STAGE_PLACEGO) */
#define HOST_STAGE_PLACEGO  4U /* 去放置位 */
#define HOST_STAGE_PLACE    5U /* 放置 */
#define HOST_STAGE_RETURN   6U /* 返回 */
#define HOST_STAGE_ABORT    7U /* 急停/超时 */

/* ========================================================================== */
/*  可调配置                                                                   */
/* ========================================================================== */

#define HOST_REPORT_MS       50U   /* 事件扫描节拍(ms): 主循环每圈按事件触发上报,
                                      此值仅为 TIM7 超时看门狗精度, 非心跳周期 */
#define HOST_POS_REPORT_MS   50U   /* 位置类上报限频(ms): 运动中最多 20Hz,
                                      防刷屏占满主循环; 语义事件不受此限 */
#define HOST_RETRY_MS        1U    /* 发送失败(USB 忙/未枚举)后最小重试间隔(ms):
                                      触发条件保留不清, 到点重组最新内容重发。
                                      无此限速时主循环会按自身速率空转重试,
                                      空刷 usb tx_error 并膨胀上报 SEQ(台架实测
                                      seq≈34k 而实际只发出 899 帧的元凶) */
#define HOST_CMD_FIFO_LEN    8U    /* 已解析下行命令队列深度(中断入, 主循环出) */
#define HOST_F1_TRY          3U    /* 起步前 F1(QPos 参数)确认: 最多重发次数 */
#define HOST_F1_WAIT_MS      50U   /* 每次等待 F1 应答的时长(ms) */
#define HOST_FOLLOW_ERR_UM   10000 /* 跟随误差保护阈值(0.001mm=10mm): 运动中
                                      反馈实际偏离参考超阈值 -> 立即停止并按
                                      故障了结(防失控跑飞; 正常闭环跟随 <1mm) */
#define HOST_LINK_TIMEOUT_MS 0U    /* 链路看门狗: 超过该时长无任何有效下行帧
                                      -> 按急停处理+记录 HOST_ERR_COMM(仿 Klipper
                                      主机失联保护)。0=关闭(默认); 若上位机能
                                      保证周期发送(心跳/周期任务), 可设 5000 */

/* 运动/夹爪固件参数(SPEED 与 GRIP_FORCE 已移出协议, 在此决定) */
#define HOST_MOVE_FEED      100.0f /* 运动速度 mm/s(超 MS_XY_MAX_VEL 自动钳位) */
#define HOST_GRIP_FORCE     100U   /* 夹爪闭合力度 0~100%(闭合行程比例见 grip.h) */

/* 夹爪执行机构(双舵机: 舵机1 Z 轴升降 + 舵机2 开合)全部配置在 grip.h:
   通道映射 GRIP_*_SERVO_CH / 开合角度 GRIP_JAW_*_DEG / 力度下限
   GRIP_JAW_FORCE_FLOOR / 动作时长 GRIP_*_MOVE_MS; 本模块只下发
   Grip_Open()/Grip_Close(force), 到位口径取 Grip_JawIsBusy(), 不直接碰舵机 */

/* ========================================================================== */
/*  类型定义                                                                   */
/* ========================================================================== */

/* 下行帧解析结果(命令队列元素) */
typedef struct
{
  uint16_t seq;        /* SEQ */
  uint32_t task_id;    /* TASK_ID */
  int32_t  x_um;       /* X, 0.001mm */
  int32_t  y_um;       /* Y, 0.001mm */
  uint8_t  flags;      /* FLAGS(原始值) */
  uint8_t  grip_cmd;   /* GRIP_CMD */
  uint16_t timeout_ms; /* TIMEOUT */
} HostCmd_t;

/* 运行统计(中断写, 主循环读) */
typedef struct
{
  volatile uint32_t rx_frames;   /* 成功受理(入队)的下行帧 */
  volatile uint32_t rx_bad_ver;  /* VER 不符被丢弃(不 ACK) */
  volatile uint32_t rx_unknown;  /* 未知 TYPE 被丢弃(不 ACK) */
  volatile uint32_t rx_rej;      /* 字段超范围被丢弃(不 ACK) */
  volatile uint32_t rx_dup;      /* SEQ 重复被静默去重(ACK 维持) */
  volatile uint32_t rx_drop;     /* 命令队列满被丢弃(不 ACK, 上位机需重发) */
  volatile uint32_t tx_frames;   /* 成功发出的上行帧 */
  volatile uint32_t tx_busy;     /* USB 忙被推迟的次数 */
} HostCommStats_t;

/* 首次运动诊断快照(定位"受理但不走"): CAN 邮箱/ESR + F1(QPos 参数)应答 */
typedef struct
{
  uint8_t  done;        /* 已采集(收到过首条运动命令) */
  uint8_t  free_mb;     /* CAN 空邮箱数(>=2 健康) */
  uint32_t esr;         /* CAN ESR(bit31 ERROR_PASSIVE / bit30 WARNING) */
  uint8_t  f1_status[2];/* A/B: F1 应答状态, 02=生效 E2=被拒 EE=格式错 */
  uint32_t f1_count[2]; /* A/B: 累计 F1 应答数, 0=Init 设置无应答 */
  uint16_t no_rx[2];    /* A/B: 连续无应答计数(持续增长=电机失联) */
} HostDiag_t;

/* ========================================================================== */
/*  接口                                                                       */
/* ========================================================================== */

/**
  * @brief  初始化上位机通讯(依赖 Task_IT_Init 已启动 TIM7)
  * @note   内部顺序: MotionSys_XY_Init()(幂等) -> Grip_Init()(夹爪双舵机,
  *         幂等; 上电 Z 抬起 + 夹爪张开) -> USB_Init 绑定接收回调
  *         -> 订阅 TIM7 HOST_REPORT_MS 节拍
  */
void HostComm_Init(void);

/**
  * @brief  主循环调用: 排空命令队列(清报警/急停/取消/夹爪/运动依规格分发),
  *         夹爪到位与任务结束判定, 命令超时善后, 按事件触发发送状态上报
  * @note   每圈调用即可; 仅在事件发生时发帧(首帧/ACK/错误/状态/位置变化/
  *         任务结果), 静止时静默 —— 无周期心跳
  */
void HostComm_Poll(void);

/**
  * @brief  TIM7 节拍(中断): 命令超时看门狗 + 链路看门狗
  * @note   由 HostComm_Init 订阅, 不要手动调用; 中断内只置标志不发帧
  */
void HostComm_Tick(void);

/**
  * @brief  读运行统计快照
  */
void HostComm_GetStats(HostCommStats_t *out);

/**
  * @brief  读首次运动诊断快照(CAN/F1 应答, 定位"受理但不走")
  */
void HostComm_GetDiag(HostDiag_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __HOST_COMM_H__ */
