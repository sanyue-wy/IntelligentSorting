//
// Created by wy on 2026/9/25.
//

#include "debug.h"
#include "drv_tim.h"
#include "task_it.h"
#include <string.h>

/*==============================================================================
 * 分层检测: 从底层往上一步步来, 每次只把 debug.h 里对应宏改成 1,
 * 观察 USART1(115200) 打印的 [PASS]/[FAIL] 与统计数据。
 *
 *   第1层 uart_debug      USART1 收发自检 + 手动回显
 *   第2层 can_debug       CAN1 发送 / 接收统计 / 总线错误
 *   第3层 usb_debug       USB CDC 虚拟串口
 *   第4层 vofa_debug      VOFA+ 波形 + 在线调参
 *   第5层 stepmotor_debug ZDT 步进电机(依赖 CAN1, 电机会动!)
 *   第6层 dji_motor_debug DJI 3508: 等反馈 -> 速度正反 -> 角度±360°(电机会转!)
 *   第7层 servo_debug     舵机 0~270° 扫描: 上扫/下扫 + 越界钳位核对(舵机会动!)
 *   第10层 grip_debug     夹爪机构(双舵机): Z 轴升降 + 开合, 逐通道脉宽核对(舵机会动!)
 *   第11层 storage_debug  旋转储物盘(3508 直驱): 6 格逐格转位 + 最短路径核对(电机会转!)
 *
 * 公共部分(TIM7 1ms 节拍 + UART/CAN 驱动接入, 由 task_it.c 统一注册)不受宏控制, 始终初始化。
 *============================================================================*/

//数据结构区域
#if vofa_debug==1
static PID_Controller_t s_vofa_pid;   /* 速度环测试实例 */
static float s_vofa_speed_target = 0; /* VOFA 命令 Spe 写入这里 */
#endif

#if stepmotor_debug==1
#define ZDT_TEST_ADDR  1     /* 被测步进电机地址, 按实际改 */
#define ZDT_MICROSTEP  16    /* 细分值, 按驱动器设置改 */
#define ZDT_TEST_MOTOR (&StepMotor_ZDT[0])  /* 被测电机实例 */
typedef enum {
    ST_IDLE = 0,             /* 等待使能完成 */
    ST_ENABLE_DONE,          /* 已使能, 准备正转 */
    ST_FWD,                  /* 速度模式正转中 */
    ST_STOPPED,              /* 停止中, 准备反转 */
    ST_REV,                  /* 速度模式反转中 */
    ST_POS_F,                /* 位置模式 +360° */
    ST_POS_R,                /* 位置模式 -360° */
    ST_FINISHED              /* 一轮结束, 回到 ST_IDLE 循环 */
} StepTestState_t;
static volatile StepTestState_t s_step_state = ST_IDLE;
static uint32_t s_step_tick = 0;
#endif

#if can_debug==1 || stepmotor_debug==1
static struct {
    volatile uint32_t count;    /* 收到帧总数 */
    volatile uint16_t id;       /* 最后一帧 ID */
    volatile uint8_t  dlc;      /* 最后一帧长度 */
    uint8_t           data[8];  /* 最后一帧数据 */
} s_can_rx;
#endif

#if dji_motor_debug==1
#define DJ_TEST_MOTOR   (&DJ_Motor3508[0]) /* 被测电机实例 */
#define DJ_TEST_ID      1                  /* 电机编号 1~4: 反馈帧 0x201~0x204, 电流走 0x200 低4路 */
#define DJ_TEST_SPEED   1000.0f            /* 速度测试值 (电机侧 r/min, 输出轴约 /19) */
typedef enum {
    DJ_ST_WAIT_FB = 0,  /* 等首帧反馈: 先验通信, 不动 */
    DJ_ST_FWD,          /* 速度模式正转 */
    DJ_ST_STOP1,        /* 停止 */
    DJ_ST_REV,          /* 速度模式反转 */
    DJ_ST_STOP2,        /* 停止 */
    DJ_ST_POS_F,        /* 角度 +360° */
    DJ_ST_POS_R,        /* 角度回 0° */
    DJ_ST_DONE          /* 一轮结束, 1s 后循环 */
} DjiTestState_t;
static DjiTestState_t s_dji_state = DJ_ST_WAIT_FB;
static uint32_t s_dji_tick = 0;
#endif

#if servo_debug==1
/* ---- 第7层 servo_debug: 舵机 0~270° 扫描, 回读 CCR 核对脉宽 ---- */
#define SERVO_TEST_FREQ_HZ   50U   /* 与 servo.c 的 SERVO_FREQ_HZ 一致 */
#define SERVO_TEST_TOL_US    20U   /* 回读脉宽与期望的允许误差(µs) */
#define SERVO_TEST_STEP_DEG  15    /* 扫描步进(度), 270/15 = 18 步 */
#define SERVO_TEST_STEP_MS   150U  /* 每步间隔(ms), 全程约 5.4s */
typedef enum {
    SV_ST_START = 0,  /* 三通道送 0°, 开始上扫 */
    SV_ST_UP,         /* 0° -> 270° 每步 +15° */
    SV_ST_DOWN,       /* 270° -> 0° 每步 -15° */
    SV_ST_CLAMP_LO,   /* 送 -50°: 应被钳到 0°(500µs) */
    SV_ST_CLAMP_HI,   /* 送 320°: 应被钳到 270°(2500µs) */
    SV_ST_REPORT      /* 汇总 [PASS]/[FAIL], 1s 后开始新一轮 */
} ServoTestState_t;
static const uint32_t s_sv_ch[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
static ServoTestState_t s_sv_state = SV_ST_START;
static uint32_t s_sv_tick = 0;
static int16_t  s_sv_angle = 0;   /* 扫描步进的当前角度 */
static uint32_t s_sv_steps = 0;   /* 已核对步数 */
static uint32_t s_sv_err   = 0;   /* 有通道不一致的步数 */
#endif

#if grip_debug==1
/* ---- 第10层 grip_debug: 夹爪机构(双舵机: 舵机1 Z 轴升降 + 舵机2 开合) ----
   每步两拍: 下发后立刻核对(回报角度 / 该通道 CCR 回读脉宽 / 忙碌标志)
   -> 等动作时长走完 -> 到位核对(角度 + 脉宽 + 终态)。
   判据: 期望角度、模块回报角度、回读脉宽(±容差)三者一致, 且忙碌标志
   动作中置起、到位后自动落下 -> 轮末汇总 [PASS]/[FAIL]。(舵机会动!) */
#define GRIP_TEST_FREQ_HZ  50U    /* 与 servo.c 的 SERVO_FREQ_HZ 一致 */
#define GRIP_TEST_TOL_US   20U    /* 回读脉宽与期望值的允许误差(µs) */
#define GRIP_TEST_TO_MS    3000U  /* 单步超时(动作时长估算留足富余) */
#define GRIP_TEST_GAP_MS   2000U  /* 轮末间隔 */
typedef enum {
    GT_INIT = 0,      /* 等上电姿态就绪, 开一轮 */
    GT_Z_DOWN,        /* Z 轴降到取件位 */
    GT_Z_UP,          /* Z 轴升回安全位 */
    GT_JAW_OPEN,      /* 夹爪张开 */
    GT_JAW_CLOSE_MAX, /* 夹爪 100% 力度闭合 */
    GT_JAW_CLOSE_MIN, /* 夹爪 0% 力度闭合(保底轻夹行程) */
    GT_CLAMP,         /* 越界钳位 + 非法力度拒绝 */
    GT_REPORT         /* 轮末汇总, 间隔后开下一轮 */
} GripTestState_t;
static GripTestState_t s_gt_state = GT_INIT;
static uint32_t s_gt_tick;
static uint8_t  s_gt_sent;  /* 当前步已下发(区分"下发核对"与"到位核对"两拍) */
static uint32_t s_gt_steps; /* 一轮内核对步数 */
static uint32_t s_gt_err;   /* 一轮内不一致步数 */
#endif

#if motion_debug==1
/* ---- 第8层 motion_debug: CoreXY 运动系统循环测试 -----------------------
   流程: 每轮起点清零重同步 -> 8 条单腿(4 条轴向验证两电机同/反向 +
   4 条对角线验证 CoreXY 单电机混控) -> 正方形 4 段连发(验证 lookahead
   弯道平滑) -> 汇总 -> 3s 后开下一轮。
   判据: 参考(规划)坐标 vs 期望 <50µm —— 规划层自检;
         实际(闭环反馈)坐标 vs 期望 <1mm —— QPos 流式跟随精度。 */
#define MT_TOL_CMD_UM   50L     /* 规划坐标容差 (µm) */
#define MT_TOL_ACT_UM   1000L   /* 实际坐标容差 (µm) */
#define MT_LEG_FEED     60.0f   /* 单腿速度 mm/s */
#define MT_FAST_FEED    100.0f  /* 连发段速度 mm/s */
#define MT_SETTLE_MS    300U    /* 停稳等待: 转 IDLE 后再等该时长才测 */
#define MT_ROUND_GAP_MS 3000U   /* 每轮间隔 */
#define MT_FAULT_RETRY_MS 5000U /* FAULT 后重试间隔 */

typedef enum
{
  MT_INIT = 0,  /* 每轮起点: SetPosition 清零重同步 */
  MT_LEG,       /* 单腿段: 在飞(刷新时间戳)->停稳->检查->发下一条 */
  MT_SQUARE,    /* 正方形 4 段连发: 同上 */
  MT_DONE,      /* 本轮完成, 打汇总, 间隔后循环 */
  MT_FAULT,     /* 入队失败/通信故障: 已 Stop 善后, 延时后重试 */
} MotionTestState_t;

typedef struct
{
  float       dx, dy; /* 相对位移 (mm) */
  float       feed;   /* 速度 (mm/s) */
  const char *tag;    /* 打印标签 */
} MTLeg_t;

static const MTLeg_t s_mt_legs[] = {
    { 20.0f,   0.0f, MT_LEG_FEED, "X+ 20mm (两电机同向)" },
    {-20.0f,   0.0f, MT_LEG_FEED, "X- 20mm 回原点" },
    {  0.0f,  20.0f, MT_LEG_FEED, "Y+ 20mm (两电机反向)" },
    {  0.0f, -20.0f, MT_LEG_FEED, "Y- 20mm 回原点" },
    { 15.0f,  15.0f, MT_LEG_FEED, "对角 +X+Y (仅A电机转)" },
    {-15.0f, -15.0f, MT_LEG_FEED, "对角回原点 (仅A电机转)" },
    { 15.0f, -15.0f, MT_LEG_FEED, "对角 +X-Y (仅B电机转)" },
    {-15.0f,  15.0f, MT_LEG_FEED, "对角回原点 (仅B电机转)" },
};
#define MT_LEG_CNT ((uint8_t)(sizeof(s_mt_legs) / sizeof(s_mt_legs[0])))

static MotionTestState_t s_mt_state = MT_INIT;
static uint8_t  s_mt_idx;          /* 当前腿下标 */
static uint32_t s_mt_tick;         /* 状态时间戳(在飞时每拍刷新) */
static uint32_t s_mt_last_print;   /* 1s 状态行节拍 */
static float    s_mt_exp_x, s_mt_exp_y; /* 期望坐标 = 最后下发目标 */
static uint32_t s_mt_pass, s_mt_fail;
#endif

#if host_debug==1
/* ---- 第9层 host_debug: 上位机通讯 + 主机联调测试 -----------------------
   HostComm_Poll 排空命令队列执行并上报; 本测试模拟"上位机": 上电后按
   新协议组下行帧, 经 USB CDC 发出(本机回环), 验证完整闭环:
     发帧 -> OnRx 中断组帧/校验/入队/ACK -> Poll 分发执行(XY 会动!) ->
     0x81 事件上报(状态/位置变化 + TASK_DONE_EVENT)。
   每轮序列(TASK_ID 低字节=轮次, 逐轮+1):
     ① 清报警(复位)  ② 运动到 (40,0)  ③ 夹爪闭合  ④ 运动到 (0,40)
     ⑤ 夹爪张开      —— 设备应自动判定任务成功, 补发 TASK_DONE_EVENT
     ⑥ 返回段 (0,0)  ⑦ 3s 间隔后下一轮
   HT_AUTO_TEST: 1=上电自动跑本测试; 0=纯被动模式(等 PC 串口助手发帧),
   两者只能一个在发 —— 双"上位机"会互相清报警/抢 USB 导致无运动 */
#define HT_AUTO_TEST      0
#define HT_WAIT_MS        1500U /* 每步等待(留足运动+夹爪时间) */
#define HT_ROUND_GAP_MS   3000U /* 每轮间隔 */
#define HT_TEST_TASK_ID   0xA0000001U /* 轮次叠加进低字节 */

typedef enum
{
  HT_OFF = 0, /* 纯被动: 等 PC 串口助手发帧(手动联调) */
  HT_INIT,    /* 等首帧上报确认链路可用(USB 枚举完成) */
  HT_STEP,    /* 发送一步指令, 等待 HT_WAIT_MS */
  HT_GAP      /* 本轮结束, 间隔后开下一轮 */
} HostTestState_t;

#if (HT_AUTO_TEST == 1)
static uint8_t ht_frame[HOST_FRAME_LEN]; /* 静态缓冲(须活到 CDC 发完) */
static HostTestState_t s_ht_state = HT_INIT;
static uint8_t  s_ht_step;               /* 当前步下标 */
static uint32_t s_ht_tick;
static uint32_t s_ht_round;              /* 轮次(也做 TASK_ID 低字节) */
static uint16_t s_ht_seq;                /* 下行 SEQ(单调递增) */
#endif
#endif

#if storage_debug==1
/* ---- 第11层 storage_debug: 旋转储物盘(3508 直驱, 6 格 × 60°) --------------
   每步两拍: 下发后立刻核对(目标槽位/目标角度/状态=MOVING/驱动角度模式已在飞)
   -> 等到位(Storage_IsSlotArrived) -> 核对(到达槽位/角度误差/实际角度合理)。

   判据(全是模块自身口径的自检, 与 dji_motor 层同思路):
     1) 下发后: 目标槽位 == 请求槽位; 目标角度 == Storage_SlotToAngle(槽位)
                (或其在绕圈口径下的就近等价角); 状态变 MOVING;
                驱动 setAngle 已被写成同一目标(证明真的下发到电机层);
     2) 到位后: Storage_IsSlotArrived 为真; Storage_GetState 回 IDLE;
                Storage_GetSlot() == 目标槽位; |角度误差| <= STORAGE_ARRIVE_DEG;
     3) 路径核对: 记录下发瞬间的实际角度, 到位后算实际转动量,
                与"最短路径"理论值(<= 180°)比较 —— 转动量超 180° 说明
                就近选点没生效(会白转一圈); 方向符号相反则 STORAGE_DIR_SIGN
                需要改(-1.0f);
     4) 拒绝核对: 越界槽位 -> ERR_PARAM; 运动中重发 -> ERR_BUSY。

   注意: 本层会让盘整圈转动 —— 上机前先确认盘面负载/走线不受限。
   与 dji_motor_debug 互斥(双指挥 0x200 电流帧), 与 host_debug 互斥。 */
#define STORAGE_TEST_TO_MS   12000U /* 单步超时(大于模块自身 8s 超时, 留裕量) */
#define STORAGE_TEST_GAP_MS  3000U  /* 轮末间隔 */

typedef enum {
    SG_INIT = 0,     /* 等 CAN 反馈 + 建立参考: 把当前位置定为槽位 0 */
    SG_FWD,          /* 逐格正序 0->1->...->5 转位核对 */
    SG_FWD_WRAP,     /* 槽位 5 -> 0 回绕(最短路径应反向走 60°, 而非正向 300°) */
    SG_REV,          /* 逐格反序 5->4->...->1 */
    SG_SETREF,       /* 中间位置重定义参考核对(Storage_SetReference) */
    SG_REJECT,       /* 越界/未 Init 拒绝核对 */
    SG_REPORT        /* 轮末汇总, 间隔后开下一轮 */
} StorageTestState_t;

static StorageTestState_t s_sg_state = SG_INIT;
static uint32_t s_sg_tick;
static uint8_t  s_sg_sent;   /* 当前步已下发(区分"下发核对"与"到位核对"两拍) */
static uint8_t  s_sg_slot;   /* 本步请求的槽位 / 正反序下标 */
static float    s_sg_ang0;   /* 下发瞬间的实际角度(算实际转动量用) */
static uint32_t s_sg_steps;  /* 一轮内核对步数 */
static uint32_t s_sg_err;    /* 一轮内不一致步数 */
#endif

#if tray_debug==1
/* ---- 第12层 tray_debug: 旋转托盘(单舵机, 仅旋转) --------------------------
   每步两拍: 下发后立刻核对(回报角度 / CH3 CCR 回读脉宽 / 忙碌标志置起)
   -> 等动作时长走完 -> 到位核对(角度 + 脉宽 + 状态回 IDLE)。

   判据: 期望角度、模块回报角度、回读脉宽(±容差)三者一致, 且忙碌标志
   动作中置起、到位后自动落下 -> 轮末汇总 [PASS]/[FAIL]。
   托盘是开环舵机, 没有位置反馈, 回读 PWM 比较值是唯一能独立验证"输出
   真的变了"的手段(与 servo_debug/grip_debug 同思路)。

   注意: 本层会让托盘整程转动, 先确认托盘上无遮挡/走线不受限。
   与 servo_debug 部分重叠(都驱动 TIM1 通道), 本层只用 CH3。 */
#define TRAY_TEST_FREQ_HZ  50U    /* 与 servo.c 的 SERVO_FREQ_HZ 一致 */
#define TRAY_TEST_TOL_US   20U    /* 回读脉宽与期望值的允许误差(µs) */
#define TRAY_TEST_TO_MS    3000U  /* 单步超时(动作时长估算留足富余) */
#define TRAY_TEST_GAP_MS   2000U  /* 轮末间隔 */

typedef enum {
    TT_INIT = 0,   /* 等上电姿态(home)走完, 开一轮 */
    TT_SWEEP,      /* 0~270° 逐点扫描 */
    TT_CLAMP,      /* 越界钳位 + 相对转动非法参数核对(不产生新动作) */
    TT_REL,        /* 相对转动核对(含越界钳位) */
    TT_REPORT      /* 轮末汇总, 间隔后开下一轮 */
} TrayTestState_t;

typedef struct {
    int16_t     delta;  /* 相对转动量(度) */
    uint16_t    expect; /* 期望落到该角度 */
    const char *tag;    /* 打印标签 */
} TT_Rel_t;

static const uint16_t s_tt_angles[] = {0U, 45U, 90U, 135U, 180U, 225U, 270U};
#define TT_ANGLE_CNT ((uint8_t)(sizeof(s_tt_angles) / sizeof(s_tt_angles[0])))

static const TT_Rel_t s_tt_rels[] = {
    {-90,   180U, "rel -90 (270->180)"},
    {-1000,   0U, "rel -1000 clamp -> 0"},
    {  45,   45U, "rel +45 (0->45)"},
};
#define TT_REL_CNT ((uint8_t)(sizeof(s_tt_rels) / sizeof(s_tt_rels[0])))

static TrayTestState_t s_tt_state = TT_INIT;
static uint32_t s_tt_tick;
static uint8_t  s_tt_sent;  /* 当前步已下发(区分"下发核对"与"到位核对"两拍) */
static uint8_t  s_tt_idx;   /* 扫描点 / 相对步 下标 */
static uint32_t s_tt_steps; /* 一轮内核对步数 */
static uint32_t s_tt_err;   /* 一轮内不一致步数 */
#endif

/* Private function prototypes -----------------------------------------------*/
#if can_debug==1 || stepmotor_debug==1
static void Debug_CanRxCallback(CAN_BusObject *bus, CAN_RxHeaderTypeDef *header, uint8_t *data);
#endif
#if uart_debug==1
static void Debug_UartRxCallback(UART_PortObject *port, uint8_t *data, uint16_t size);
#endif
#if usb_debug==1
static void Debug_UsbRxCallback(uint8_t *buf, uint16_t len);
#endif
#if servo_debug==1
static uint32_t Debug_ServoReadPulseUs(uint32_t channel);
static void Debug_ServoSetAndCheck(int16_t angle);
#endif
#if grip_debug==1
static uint32_t Debug_GripReadPulseUs(uint32_t channel);
static uint32_t Debug_GripExpPulseUs(uint16_t deg);
static uint8_t  Debug_GripPulseOk(const char *tag, uint32_t channel, uint16_t deg);
static void     GripTest_Report(const char *tag, uint8_t ok);
#endif
#if storage_debug==1
static long SG_ToDeci(float deg);
static long SG_Abs(long v);
static void StorageTest_Report(const char *tag, uint8_t ok);
static void StorageTest_Status(const char *tag);
#endif
#if tray_debug==1
static uint32_t Debug_TrayReadPulseUs(uint32_t channel);
static uint32_t Debug_TrayExpPulseUs(uint16_t deg);
static uint8_t  Debug_TrayPulseOk(const char *tag, uint16_t deg);
static void     TrayTest_Report(const char *tag, uint8_t ok);
#endif
#if motion_debug==1
static long   MT_ToUm(float v);
static long   MT_Abs(long v);
static void   MotionTest_Check(const char *tag);
static void   MotionTest_Fail(const char *why);
#endif
#if host_debug==1 && HT_AUTO_TEST==1
static void HostTest_Send(uint8_t flags, int32_t x_um, int32_t y_um,
                          uint8_t grip_cmd, const char *tag);
#endif

/* 首次运动诊断行(两种模式共用): CAN 邮箱/ESR + A/B 电机 F1(QPos)应答状态 */
#if host_debug==1
static void HostComm_PrintDiag(void)
{
  static uint8_t s_printed;
  HostDiag_t d;

  if (s_printed)
  {
    return;
  }
  HostComm_GetDiag(&d);
  if (!d.done)
  {
    return; /* 还没收到过运动命令 */
  }
  s_printed = 1U;
  UART_Port_Printf(&uart1_port,
                   "[host] diag free=%u esr=0x%08lX | A f1=%02X n=%lu norx=%u | B f1=%02X n=%lu norx=%u\r\n",
                   (unsigned)d.free_mb, (unsigned long)d.esr,
                   (unsigned)d.f1_status[0], (unsigned long)d.f1_count[0], (unsigned)d.no_rx[0],
                   (unsigned)d.f1_status[1], (unsigned long)d.f1_count[1], (unsigned)d.no_rx[1]);
}
#endif

/* Exported functions ---------------------------------------------------------*/

#if can_debug==1 || stepmotor_debug==1
/**
  * @brief  CAN 兜底接收回调(中断上下文): 只缓存最新一帧, 不做任何打印
  */
static void Debug_CanRxCallback(CAN_BusObject *bus, CAN_RxHeaderTypeDef *header, uint8_t *data)
{
  (void)bus;
  s_can_rx.count++;
  s_can_rx.id  = (uint16_t)((header->IDE == CAN_ID_EXT) ? header->ExtId : header->StdId);
  s_can_rx.dlc = header->DLC;
  memcpy(s_can_rx.data, data, 8U);
}
#endif

#if uart_debug==1
/**
  * @brief  USART1 接收回调(中断上下文): 原样回显, 方便串口助手手动发字节验证
  * @note   DMA 发送非阻塞; 上一帧没发完则本段丢弃
  */
static void Debug_UartRxCallback(UART_PortObject *port, uint8_t *data, uint16_t size)
{
  (void)UART_Port_Send(port, data, size);
}
#endif

#if usb_debug==1
/**
  * @brief  USB CDC 接收回调(中断上下文): 原样回显, PC 端 COM 口发什么收什么
  * @note   上一帧没发完(CDC_Transmit_FS 返回 USBD_BUSY)则本段丢弃,
  *         由 USB_Transmit_Data 计入 stats.tx_error
  */
static void Debug_UsbRxCallback(uint8_t *buf, uint16_t len)
{
  (void)USB_Transmit_Data(buf, len);
}
#endif

#if servo_debug==1
/**
  * @brief  回读 TIM1 CHx 当前输出脉宽(µs): CCR 高电平计数 -> 微秒
  * @note   周期取 ARR 寄存器(drv_pwm 已按 50Hz 重算), 频率须与 servo.c 一致
  */
static uint32_t Debug_ServoReadPulseUs(uint32_t channel)
{
  uint32_t ticks  = __HAL_TIM_GET_COMPARE(&htim1, channel);
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

  if (period <= 1U)
  {
    return 0U;
  }
  return (uint32_t)(((uint64_t)ticks * 1000000U) / ((uint64_t)period * SERVO_TEST_FREQ_HZ));
}

/**
  * @brief  三路同送 angle, 逐通道核对实际脉宽与期望值
  * @note   期望值按 0~270 钳位后的角度计算, 与 servo.c 换算同式,
  *         因此越界入参顺带覆盖 Servo_SetAngle_270 的钳位行为;
  *         不一致的通道当场打 [FAIL], 步统计累入 s_sv_err
  */
static void Debug_ServoSetAndCheck(int16_t angle)
{
  int16_t clamped = angle;
  int32_t expected;
  uint32_t bad = 0;

  if (clamped > SERVO_ANGLE_270_MAX) clamped = SERVO_ANGLE_270_MAX;
  if (clamped < 0)                   clamped = 0;
  expected = (int32_t)SERVO_MIN_US +
             ((int32_t)clamped * ((int32_t)SERVO_MAX_US - (int32_t)SERVO_MIN_US)) / SERVO_ANGLE_270_MAX;

  for (uint8_t i = 0; i < 3U; i++)
  {
    uint32_t actual;
    int32_t  diff;

    Servo_SetAngle_270(s_sv_ch[i], angle);
    actual = Debug_ServoReadPulseUs(s_sv_ch[i]);
    diff   = (int32_t)actual - expected;
    if (diff < 0)
    {
      diff = -diff;
    }
    if (diff > (int32_t)SERVO_TEST_TOL_US)
    {
      bad++;
      UART_Port_Printf(&uart1_port,
                       "[servo] [FAIL] ch%u ang=%d exp=%ldus act=%ldus\r\n",
                       (unsigned)(i + 1U), (int)angle, (long)expected, (long)actual);
    }
  }

  s_sv_steps++;
  if (bad != 0U)
  {
    s_sv_err++;
  }
  UART_Port_Printf(&uart1_port,
                   "[servo] ang=%d exp=%ldus %s | steps=%lu err=%lu\r\n",
                   (int)angle, (long)expected, (bad == 0U) ? "[ok]" : "[BAD]",
                   (unsigned long)s_sv_steps, (unsigned long)s_sv_err);
}
#endif

#if grip_debug==1
/**
  * @brief  回读 TIM1 CHx 当前输出脉宽(µs)
  * @note   与 Debug_ServoReadPulseUs 同式: 周期取 ARR 寄存器(drv_pwm 已按
  *         50Hz 重算), 频率须与 servo.c 一致
  */
static uint32_t Debug_GripReadPulseUs(uint32_t channel)
{
  uint32_t hw_ch  = Servo_GetHwChannel((ServoChannel_e)channel);
  uint32_t ticks  = __HAL_TIM_GET_COMPARE(&htim1, hw_ch);
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

  if (period <= 1U)
  {
    return 0U;
  }
  return (uint32_t)(((uint64_t)ticks * 1000000U) / ((uint64_t)period * GRIP_TEST_FREQ_HZ));
}

/**
  * @brief  角度 -> 期望脉宽(µs), 与 servo.c 的 0~270° 换算同式
  */
static uint32_t Debug_GripExpPulseUs(uint16_t deg)
{
  if (deg > SERVO_ANGLE_270_MAX)
  {
    deg = SERVO_ANGLE_270_MAX;
  }
  return (uint32_t)((int32_t)SERVO_MIN_US +
                    ((int32_t)deg * ((int32_t)SERVO_MAX_US - (int32_t)SERVO_MIN_US)) /
                    SERVO_ANGLE_270_MAX);
}

/**
  * @brief  核对某通道回读脉宽与期望角度是否一致(不一致当场打 [FAIL] 明细)
  * @retval 1 一致(±GRIP_TEST_TOL_US); 0 不一致
  */
static uint8_t Debug_GripPulseOk(const char *tag, uint32_t channel, uint16_t deg)
{
  uint32_t exp  = Debug_GripExpPulseUs(deg);
  uint32_t act  = Debug_GripReadPulseUs(channel);
  int32_t  diff = (int32_t)act - (int32_t)exp;
  uint8_t  ok;

  if (diff < 0)
  {
    diff = -diff;
  }
  ok = (diff <= (int32_t)GRIP_TEST_TOL_US) ? 1U : 0U;
  if (!ok)
  {
    UART_Port_Printf(&uart1_port,
                     "[grip] [FAIL] %s: ch%lu deg=%u exp=%ldus act=%ldus\r\n",
                     tag, (unsigned long)(channel + 1U), (unsigned)deg,
                     (long)exp, (long)act);
  }
  return ok;
}

/* 记一次核对结果并打印(轮内计数, 轮末汇总) */
static void GripTest_Report(const char *tag, uint8_t ok)
{
  s_gt_steps++;
  if (!ok)
  {
    s_gt_err++;
  }
  UART_Port_Printf(&uart1_port, "[grip] %s %s | steps=%lu err=%lu\r\n",
                   tag, (ok != 0U) ? "[ok]" : "[FAIL]",
                   (unsigned long)s_gt_steps, (unsigned long)s_gt_err);
}
#endif

#if storage_debug==1
/* newlib-nano 无 float printf: 角度一律 x10 变整数打印(0.1° 分辨率) */
static long SG_ToDeci(float deg)
{
  float s = deg * 10.0f;
  return (long)((s >= 0.0f) ? (s + 0.5f) : (s - 0.5f));
}

static long SG_Abs(long v)
{
  return (v < 0L) ? -v : v;
}

/* 记一次核对结果并打印(轮内计数, 轮末汇总) */
static void StorageTest_Report(const char *tag, uint8_t ok)
{
  s_sg_steps++;
  if (!ok)
  {
    s_sg_err++;
  }
  UART_Port_Printf(&uart1_port, "[storage] %s %s | steps=%lu err=%lu\r\n",
                   tag, (ok != 0U) ? "[ok]" : "[FAIL]",
                   (unsigned long)s_sg_steps, (unsigned long)s_sg_err);
}

/* 状态行: 槽位/状态/角度(实际,目标)/误差 + 驱动反馈/编码器/速度/模式 ——
   卡住时用它定位(如 norx 涨 = 丢反馈; mode 不是 1 = 没进角度模式) */
static void StorageTest_Status(const char *tag)
{
  DJ_Motor_t *m = STORAGE_MOTOR;
  long ta = SG_ToDeci(Storage_GetTargetAngle());
  long aa = SG_ToDeci(Storage_GetActualAngle());
  long er = SG_ToDeci(Storage_GetAngleError());

  UART_Port_Printf(&uart1_port,
                   "[storage] %s: state=%d slot=%d tgt=%ld.%ld act=%ld.%ld err=%ld.%ld "
                   "| enc=%u spd=%d norx=%u mode=%d rx_once=%u\r\n",
                   tag,
                   (int)Storage_GetState(), (int)Storage_GetSlot(),
                   ta / 10L, SG_Abs(ta % 10L),
                   aa / 10L, SG_Abs(aa % 10L),
                   er / 10L, SG_Abs(er % 10L),
                   (unsigned)m->angle, (int)m->speed, (unsigned)m->no_rx_count,
                   (int)m->mode, (unsigned)m->rx_once);
}
#endif

#if tray_debug==1
/**
  * @brief  回读 TIM1 CHx 当前输出脉宽(µs)
  * @note   与 Debug_ServoReadPulseUs / Debug_GripReadPulseUs 同式。三处各自
  *         独立一份是沿用本工程"每层自包含、按宏裁剪"的写法(各层可单独打开,
  *         共享函数反而要在多个宏之间做组合守卫)。周期取 ARR 寄存器
  *         (drv_pwm 已按 50Hz 重算), 频率须与 servo.c 一致
  */
static uint32_t Debug_TrayReadPulseUs(uint32_t channel)
{
  uint32_t hw_ch  = Servo_GetHwChannel((ServoChannel_e)channel);
  uint32_t ticks  = __HAL_TIM_GET_COMPARE(&htim1, hw_ch);
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

  if (period <= 1U)
  {
    return 0U;
  }
  return (uint32_t)(((uint64_t)ticks * 1000000U) / ((uint64_t)period * TRAY_TEST_FREQ_HZ));
}

/**
  * @brief  角度 -> 期望脉宽(µs), 与 servo.c 的 0~270° 换算同式
  */
static uint32_t Debug_TrayExpPulseUs(uint16_t deg)
{
  if (deg > SERVO_ANGLE_270_MAX)
  {
    deg = SERVO_ANGLE_270_MAX;
  }
  return (uint32_t)((int32_t)SERVO_MIN_US +
                    ((int32_t)deg * ((int32_t)SERVO_MAX_US - (int32_t)SERVO_MIN_US)) /
                    SERVO_ANGLE_270_MAX);
}

/**
  * @brief  核对 CH3 回读脉宽与期望角度是否一致(不一致当场打 [FAIL] 明细)
  * @retval 1 一致(±TRAY_TEST_TOL_US); 0 不一致
  */
static uint8_t Debug_TrayPulseOk(const char *tag, uint16_t deg)
{
  uint32_t exp  = Debug_TrayExpPulseUs(deg);
  uint32_t act  = Debug_TrayReadPulseUs(TRAY_SERVO_CH);
  int32_t  diff = (int32_t)act - (int32_t)exp;
  uint8_t  ok;

  if (diff < 0)
  {
    diff = -diff;
  }
  ok = (diff <= (int32_t)TRAY_TEST_TOL_US) ? 1U : 0U;
  if (!ok)
  {
    UART_Port_Printf(&uart1_port,
                     "[tray] [FAIL] %s: deg=%u exp=%ldus act=%ldus\r\n",
                     tag, (unsigned)deg, (long)exp, (long)act);
  }
  return ok;
}

/* 记一次核对结果并打印(轮内计数, 轮末汇总) */
static void TrayTest_Report(const char *tag, uint8_t ok)
{
  s_tt_steps++;
  if (!ok)
  {
    s_tt_err++;
  }
  UART_Port_Printf(&uart1_port, "[tray] %s %s | steps=%lu err=%lu\r\n",
                   tag, (ok != 0U) ? "[ok]" : "[FAIL]",
                   (unsigned long)s_tt_steps, (unsigned long)s_tt_err);
}
#endif

#if motion_debug==1
/* mm -> µm(long, 四舍五入; newlib-nano 无 float printf, 一律整数打印) */
static long MT_ToUm(float v)
{
    float s = v * 1000.0f;
    return (long)((s >= 0.0f) ? (s + 0.5f) : (s - 0.5f));
}

static long MT_Abs(long v)
{
    return (v < 0L) ? -v : v;
}

/**
  * @brief  到位检查: 参考(规划)坐标严判 50µm(规划层数学自检),
  *         实际(闭环反馈)坐标判 1mm(QPos 流式跟随精度)
  */
static void MotionTest_Check(const char *tag)
{
    float cx, cy, ax, ay;
    long dcx, dcy, dax, day;
    uint8_t ok;

    MotionSys_XY_GetCmdPos(&cx, &cy);
    MotionSys_XY_GetActualPos(&ax, &ay);
    dcx = MT_Abs(MT_ToUm(cx - s_mt_exp_x));
    dcy = MT_Abs(MT_ToUm(cy - s_mt_exp_y));
    dax = MT_Abs(MT_ToUm(ax - s_mt_exp_x));
    day = MT_Abs(MT_ToUm(ay - s_mt_exp_y));
    ok = (dcx <= MT_TOL_CMD_UM && dcy <= MT_TOL_CMD_UM &&
          dax <= MT_TOL_ACT_UM && day <= MT_TOL_ACT_UM) ? 1U : 0U;

    if (ok)
    {
        s_mt_pass++;
    }
    else
    {
        s_mt_fail++;
    }
    UART_Port_Printf(&uart1_port,
                     "[motion] [%s] %s | exp=(%ld,%ld) cmd_err=(%ld,%ld) act_err=(%ld,%ld)um\r\n",
                     ok ? "PASS" : "FAIL", tag,
                     MT_ToUm(s_mt_exp_x), MT_ToUm(s_mt_exp_y),
                     dcx, dcy, dax, day);
}

/* 入队/状态失败: 记一次 FAIL, Stop 善后, 转 FAULT 重试 */
static void MotionTest_Fail(const char *why)
{
    s_mt_fail++;
    UART_Port_Printf(&uart1_port, "[motion] [FAIL] %s -> Stop\r\n", why);
    MotionSys_XY_Stop();
    s_mt_state = MT_FAULT;
    s_mt_tick = HAL_GetTick();
}
#endif

#if host_debug==1 && HT_AUTO_TEST==1
/**
  * @brief  组一帧下行命令并经 USB 发出(模拟上位机)
  * @note   小端布局与 host_comm.h 协议一致; RESERVED[10] 填 0(代替 CRC);
  *         发送失败(BUSY/未枚举)计入 usb0.stats, 打印 [FAIL]
  */
static void HostTest_Send(uint8_t flags, int32_t x_um, int32_t y_um,
                          uint8_t grip_cmd, const char *tag)
{
  ht_frame[0] = HOST_SOF0;
  ht_frame[1] = HOST_SOF1;
  ht_frame[2] = HOST_VER;
  ht_frame[3] = HOST_TYPE_CMD;
  ht_frame[4] = flags;
  ht_frame[5] = (uint8_t)(s_ht_seq & 0xFFU);          /* SEQ 低字节 */
  ht_frame[6] = (uint8_t)((s_ht_seq >> 8) & 0xFFU);
  ht_frame[7]  = (uint8_t)(s_ht_round);               /* TASK_ID 低字节=轮次 */
  ht_frame[8]  = (uint8_t)(HT_TEST_TASK_ID >> 8);
  ht_frame[9]  = (uint8_t)(HT_TEST_TASK_ID >> 16);
  ht_frame[10] = (uint8_t)(HT_TEST_TASK_ID >> 24);
  ht_frame[11]  = (uint8_t)(x_um & 0xFFU);            /* X 小端 */
  ht_frame[12]  = (uint8_t)((x_um >> 8) & 0xFFU);
  ht_frame[13]  = (uint8_t)((x_um >> 16) & 0xFFU);
  ht_frame[14]  = (uint8_t)((x_um >> 24) & 0xFFU);
  ht_frame[15]  = (uint8_t)(y_um & 0xFFU);            /* Y 小端 */
  ht_frame[16]  = (uint8_t)((y_um >> 8) & 0xFFU);
  ht_frame[17]  = (uint8_t)((y_um >> 16) & 0xFFU);
  ht_frame[18]  = (uint8_t)((y_um >> 24) & 0xFFU);
  ht_frame[19]  = grip_cmd;                           /* GRIP_CMD */
  ht_frame[20]  = 0U;                                 /* TIMEOUT 低字节: 0=不检查 */
  ht_frame[21]  = 0U;
  for (uint8_t i = 22U; i < HOST_FRAME_LEN; i++)      /* RESERVED[10] 填 0 */
  {
    ht_frame[i] = 0U;
  }

  if (USB_Transmit_Data(ht_frame, HOST_FRAME_LEN) != USBD_OK)
  {
    UART_Port_Printf(&uart1_port, "[host] [FAIL] tx busy/not-enumerated (%s)\r\n", tag);
    return;
  }
  s_ht_seq++;
  UART_Port_Printf(&uart1_port, "[host] TX seq=%u %s\r\n", (unsigned)s_ht_seq, tag);
}
#endif

#if msgcenter_debug==1
static uint32_t s_mc_pass_cnt = 0;
static uint32_t s_mc_fail_cnt = 0;

static void MC_Test_Assert(const char *name, uint8_t condition)
{
    if (condition)
    {
        s_mc_pass_cnt++;
        UART_Port_Printf(&uart1_port, "[msgcenter] %s -> [PASS]\r\n", name);
    }
    else
    {
        s_mc_fail_cnt++;
        UART_Port_Printf(&uart1_port, "[msgcenter] %s -> [FAIL]\r\n", name);
    }
}

static void Debug_MsgCenter_Init(void)
{
    UART_Port_Printf(&uart1_port, "\r\n[msgcenter] === Message Center Unit Tests Begin ===\r\n");
    s_mc_pass_cnt = 0;
    s_mc_fail_cnt = 0;

    /* TC1: 参数有效性边界 */
    uint8_t tc1_ok = (PubRegister(NULL, 4) == NULL) &&
                     (PubRegister("tc1_zero_len", 0) == NULL) &&
                     (PubRegister("tc1_name_over_32_bytes_limit_01234567890", 4) == NULL) &&
                     (SubRegister(NULL, 4) == NULL);
    MC_Test_Assert("TC1: Parameter Validation (NULL/0/overflow)", tc1_ok);

    /* TC2: 订阅先于发布注册的时序解耦 */
    Subscriber_t *sub_tc2 = SubRegister("tc2_order", sizeof(uint32_t));
    Publisher_t  *pub_tc2 = PubRegister("tc2_order", sizeof(uint32_t));
    Publisher_t  *pub_tc2_mismatch = PubRegister("tc2_order", sizeof(uint16_t));
    uint8_t tc2_ok = (sub_tc2 != NULL) && (pub_tc2 != NULL) &&
                     (pub_tc2->pub_registered_flag == 1) &&
                     (pub_tc2_mismatch == NULL);
    MC_Test_Assert("TC2: Registration Sequence & Len Check", tc2_ok);

    /* TC3: 基础 1 对 1 收发与数据正确性 */
    Publisher_t  *pub_tc3 = PubRegister("tc3_1to1", sizeof(uint32_t));
    Subscriber_t *sub_tc3 = SubRegister("tc3_1to1", sizeof(uint32_t));
    uint32_t tx_val3 = 0x12345678U;
    uint32_t rx_val3 = 0U;
    uint8_t push3_ok = PubPushMessage(pub_tc3, &tx_val3);
    uint8_t get3_ok1 = SubGetMessage(sub_tc3, &rx_val3);
    uint8_t get3_ok2 = SubGetMessage(sub_tc3, &rx_val3); /* 再次出队应为空 */
    uint8_t tc3_ok = (push3_ok == 1) && (get3_ok1 == 1) && (rx_val3 == tx_val3) && (get3_ok2 == 0);
    MC_Test_Assert("TC3: Single Topic 1-to-1 Pub/Sub", tc3_ok);

    /* TC4: 1 对多广播 (2 个订阅者独立出队) */
    Publisher_t  *pub_tc4 = PubRegister("tc4_broadcast", sizeof(uint16_t));
    Subscriber_t *sub_tc4_a = SubRegister("tc4_broadcast", sizeof(uint16_t));
    Subscriber_t *sub_tc4_b = SubRegister("tc4_broadcast", sizeof(uint16_t));
    uint16_t tx_val4 = 0xABCDU;
    uint16_t rx_val4_a = 0U, rx_val4_b = 0U;
    uint8_t push4_cnt = PubPushMessage(pub_tc4, &tx_val4);
    uint8_t get4_a = SubGetMessage(sub_tc4_a, &rx_val4_a);
    uint8_t get4_b = SubGetMessage(sub_tc4_b, &rx_val4_b);
    uint8_t tc4_ok = (push4_cnt == 2) && (get4_a == 1) && (get4_b == 1) &&
                     (rx_val4_a == tx_val4) && (rx_val4_b == tx_val4);
    MC_Test_Assert("TC4: 1-to-Many Broadcasting", tc4_ok);

    /* TC5: 多话题数据隔离性 */
    Publisher_t  *pub_tc5_x = PubRegister("tc5_topic_x", sizeof(uint8_t));
    Publisher_t  *pub_tc5_y = PubRegister("tc5_topic_y", sizeof(uint8_t));
    Subscriber_t *sub_tc5_x = SubRegister("tc5_topic_x", sizeof(uint8_t));
    Subscriber_t *sub_tc5_y = SubRegister("tc5_topic_y", sizeof(uint8_t));
    uint8_t tx_x = 0x55U, tx_y = 0xAAU;
    uint8_t rx_x = 0U, rx_y = 0U;
    PubPushMessage(pub_tc5_x, &tx_x);
    PubPushMessage(pub_tc5_y, &tx_y);
    SubGetMessage(sub_tc5_x, &rx_x);
    SubGetMessage(sub_tc5_y, &rx_y);
    uint8_t tc5_ok = (rx_x == 0x55U) && (rx_y == 0xAAU);
    MC_Test_Assert("TC5: Multi-Topic Data Isolation", tc5_ok);

    /* TC6: 满队覆盖策略 (FIFO Overwrite, 队列深度=1时覆盖前值保留最新值) */
    Publisher_t  *pub_tc6 = PubRegister("tc6_overwrite", sizeof(uint32_t));
    Subscriber_t *sub_tc6 = SubRegister("tc6_overwrite", sizeof(uint32_t));
    uint32_t v6_first = 0x11111111U;
    uint32_t v6_second = 0x22222222U;
    uint32_t rx_v6 = 0U;
    PubPushMessage(pub_tc6, &v6_first);
    PubPushMessage(pub_tc6, &v6_second);
    uint8_t get6_ok = SubGetMessage(sub_tc6, &rx_v6);
    uint8_t get6_empty = SubGetMessage(sub_tc6, &rx_v6);
    uint8_t tc6_ok = (get6_ok == 1) && (rx_v6 == v6_second) && (get6_empty == 0);
    MC_Test_Assert("TC6: FIFO Overwrite Latest Message", tc6_ok);

    /* TC7: 空队列读取边界安全 */
    Subscriber_t *sub_tc7 = SubRegister("tc7_empty", sizeof(uint32_t));
    uint32_t rx_v7 = 0x99999999U;
    uint8_t get7_ok = SubGetMessage(sub_tc7, &rx_v7);
    uint8_t tc7_ok = (get7_ok == 0) && (rx_v7 == 0x99999999U);
    MC_Test_Assert("TC7: Empty Queue Read Boundary", tc7_ok);

    UART_Port_Printf(&uart1_port,
                     "[msgcenter] === Unit Tests Finished: %lu PASS, %lu FAIL ===\r\n\r\n",
                     (unsigned long)s_mc_pass_cnt, (unsigned long)s_mc_fail_cnt);
}
#endif

void debug_init(void)
{
    /* ---- 公共底层: 所有层都依赖, 无条件接入 ---- */
#if can_loopback==1
    /* CAN 环回自检: 片上自发自收, 信号不出芯片 —— 与收发器/接线/电机无关。
       通过 = 固件与 CAN 外设完好, 问题在板子外面; 失败 = 才查固件/CAN 外设 */
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
#endif
    Driver_CAN1_Init();                       /* CAN1: 登记 -> 全收滤波 -> 启动 -> 开中断 */
    Driver_UART1_Init();                      /* USART1: DMA 空闲接收 */
    Task_IT_Init();                           /* TIM7 1ms 节拍 + 各周期任务统一注册(task_it.c) */

#if motion_debug==1
    /* CoreXY 运动系统(仿 Voron/Klipper): 注册 A/B 电机 -> QPos 参数 ->
       清零建原点 -> 订阅 StepMotor_Run(3ms) 与 MotionSys_XY_Tick(1ms)。
       必须在 Task_IT_Init 之后(依赖 TIM7 节拍已启动); 之后即可在任意
       位置调 MotionSys_XY_MoveTo(x, y, feed_mm_s) 驱动 XY 运动 */
    MotionSys_XY_Init();
    UART_Port_Printf(&uart1_port, "[motion] corexy init ok, test loop: 8 legs + square\r\n");
#endif

#if host_debug==1
    /* 上位机通讯(32B 帧协议, USB CDC): 内部完成 MotionSys_XY_Init(幂等,
       含反馈轮询+轨迹节拍订阅) -> Grip_Init(夹爪双舵机, 上电张开) ->
       USB_Init 绑定协议回调 -> 订阅 TIM7 超时看门狗节拍(事件触发上报, 无周期心跳)。
       必须在 Task_IT_Init 之后; 与 usb_debug/servo_debug/motion_debug/grip_debug 互斥 */
    HostComm_Init();
#if (HT_AUTO_TEST == 1)
    s_ht_state = HT_INIT;
    s_ht_step  = 0U;
    s_ht_tick  = HAL_GetTick();
    s_ht_round = 0U;
    s_ht_seq   = 0U;
    UART_Port_Printf(&uart1_port,
                     "[host] init ok, frame protocol v1 over usb-cdc | self-test: wait link then auto-send XY moves\r\n");
#else
    UART_Port_Printf(&uart1_port,
                     "[host] init ok, frame protocol v1 over usb-cdc | passive mode: send frames from PC serial tool\r\n");
#endif
#endif

#if can_debug==1 || stepmotor_debug==1
    can1_bus.on_rx = Debug_CanRxCallback;     /* 挂兜底回调, 收任意帧都缓存 */
#endif

#if uart_debug==1
    /* 手动验证: 串口助手(115200) 发任意字节, 应原样回显 */
    uart1_port.on_rx = Debug_UartRxCallback;
    UART_Port_Printf(&uart1_port, "[uart] init ok\r\n");
#endif

#if usb_debug==1
    /* USB 设备已在 MX_USB_DEVICE_Init 初始化(必须先于 debug_init),
       这里绑定双缓冲与回显回调; 等枚举完成后 PC 端打开 COM 口自测 */
    USB_Init(Debug_UsbRxCallback);
    UART_Port_Printf(&uart1_port, "[usb] init ok, echo on: send bytes on COM port\r\n");
#endif

#if vofa_debug==1
    Vofa_Init(&vofa, &uart1_port);            /* 接管 USART1 接收(与 uart_debug 互斥) */
    PID_Init(&s_vofa_pid, 1.0f, 0.0f, 0.0f, 1000.0f, 0.0f, 0.001f);
    Vofa_BindSpeedPid(&vofa, &s_vofa_pid, &s_vofa_speed_target);
    UART_Port_Printf(&uart1_port, "[vofa] init ok, try 'Spe=10!'\r\n");
#endif

#if stepmotor_debug==1
    StepMotor_Init(ZDT_TEST_MOTOR, ZDT_TEST_ADDR, SM_TYPE_18DEG, ZDT_MICROSTEP);
    StepMotor_Enable(ZDT_TEST_MOTOR, true);       /* 使能, 等待电机上电就绪 */
    /* STEP_CTRL_PERIOD(3ms) 中断节拍刷新位置反馈: 在中断里跑,
       主循环的阻塞打印不再打断它; 必须在 Task_IT_Init 之后订阅 */
    TIM_Timer_Subscribe(&tim7_timer, StepMotor_Run, 3);
    s_step_state = ST_IDLE;
    s_step_tick  = HAL_GetTick();
    UART_Port_Printf(&uart1_port, "[step] motor %d enable cmd sent\r\n", ZDT_TEST_ADDR);
#endif

#if dji_motor_debug==1
    /* 电机编号 1 -> 反馈帧 0x201 订阅 + 电流从 0x200 低4路第1槽发出
       (DJ_MotorRun 已由 task_it.c 挂到 TIM7 节拍, 这里只注册电机) */
    DJ_Init(DJ_TEST_MOTOR, DJ_TEST_ID, M3508, PID_METHOD);
    s_dji_state = DJ_ST_WAIT_FB;
    s_dji_tick  = HAL_GetTick();
    UART_Port_Printf(&uart1_port, "[dji] init ok, wait feedback id=0x%03X ...\r\n",
                     0x200U + DJ_TEST_ID);
#endif

#if servo_debug==1
    /* TIM1 CH1~CH3 PWM 启动(debug_init 在 MX_TIM1_Init 之后调用, 时序满足) */
    Servo_Init();
    s_sv_state = SV_ST_START;
    s_sv_tick  = HAL_GetTick();
    s_sv_steps = 0;
    s_sv_err   = 0;
    UART_Port_Printf(&uart1_port, "[servo] init ok, 0~270 sweep test start\r\n");
#endif

#if grip_debug==1
    /* 夹爪机构(TIM1 两路舵机): Grip_Init 内含 Servo_Init + 上电姿态
       (Z 抬起到安全位 + 夹爪张开), debug_init 在 MX_TIM1_Init 之后调用, 时序满足 */
    Grip_Init();
    s_gt_state = GT_INIT;
    s_gt_tick  = HAL_GetTick();
    s_gt_sent  = 0U;
    s_gt_steps = 0U;
    s_gt_err   = 0U;
    UART_Port_Printf(&uart1_port,
                     "[grip] init ok: z=ch%u jaw=ch%u | test: Z down/up + jaw open/close(100%%->0%%) + clamp\r\n",
                     (unsigned)(GRIP_Z_SERVO_CH + 1U), (unsigned)(GRIP_JAW_SERVO_CH + 1U));
#endif

#if storage_debug==1
    /* 旋转储物盘(3508 直驱): 注册电机 -> Storage_Init(不主动转, 等首帧反馈)。
       DJ_MotorRun 已由 task_it.c 挂到 TIM7 3ms 节拍, 这里只注册电机;
       必须在 Driver_CAN1_Init() 之后(debug_init 公共段已调用)。
       注意 Storage_Init 只是"记账上电姿态=槽位 0", 电机保持卸力不动,
       真正的 0° 原点由 dji_3508 在首帧反馈时捕获。
       与 dji_motor_debug 互斥: 同抢 0x200 低4路电流帧第 1 槽 */
    DJ_Init(STORAGE_MOTOR, STORAGE_MOTOR_CAN_ID, M3508, PID_METHOD);
    Storage_Init(STORAGE_MOTOR);
    s_sg_state = SG_INIT;
    s_sg_tick  = HAL_GetTick();
    s_sg_sent  = 0U;
    s_sg_slot  = 0U;
    s_sg_ang0  = 0.0f;
    s_sg_steps = 0U;
    s_sg_err   = 0U;
    UART_Port_Printf(&uart1_port,
                     "[storage] init ok: %u slots x %ld deg, dir=%s, can_id=%u | wait can feedback ...\r\n",
                     (unsigned)STORAGE_SLOT_NUM, (long)STORAGE_SLOT_ANGLE_DEG,
                     (STORAGE_DIR_SIGN > 0.0f) ? "+" : "-", (unsigned)STORAGE_MOTOR_CAN_ID);
#endif

#if tray_debug==1
    /* 旋转托盘(单舵机 CH3): Tray_Init 内含 Servo_Init + 上电姿态(home),
       debug_init 在 MX_TIM1_Init 之后调用, 时序满足。
       Servo_Init 幂等, 与 Grip 谁先调都安全 */
    Tray_Init();
    s_tt_state = TT_INIT;
    s_tt_tick  = HAL_GetTick();
    s_tt_sent  = 0U;
    s_tt_idx   = 0U;
    s_tt_steps = 0U;
    s_tt_err   = 0U;
    UART_Port_Printf(&uart1_port,
                     "[tray] init ok: ch%u home=%u deg, range %u~%u | test: sweep 0~270 + rel + clamp\r\n",
                     (unsigned)(TRAY_SERVO_CH + 1U), (unsigned)TRAY_HOME_DEG,
                     (unsigned)TRAY_MIN_DEG, (unsigned)TRAY_MAX_DEG);
#endif

#if msgcenter_debug==1
    Debug_MsgCenter_Init();
#endif
}

//死循环内运行的
void debug_run(void)
{
    uint32_t now = HAL_GetTick();
#if uart_debug==0 && can_debug==0 && usb_debug==0 && vofa_debug==0 \
    && stepmotor_debug==0 && dji_motor_debug==0 && servo_debug==0 && grip_debug==0 \
    && storage_debug==0 && tray_debug==0 && motion_debug==0 && host_debug==0 && msgcenter_debug==0
    (void)now; /* 所有层都关闭时消 unused 警告 */
#endif

#if uart_debug==1
    {
        static uint32_t s_last = 0;
        if (now - s_last >= 1000U)
        {
            s_last = now;
            /* irq_count 每秒应 +1000 左右, 否则 TIM7 中断有问题 */
            UART_Port_Printf(&uart1_port,
                             "[uart] ok | tx=%lu rx=%lu err=%lu | tick_ms=%lu\r\n",
                             (unsigned long)uart1_port.stats.tx_count,
                             (unsigned long)uart1_port.stats.rx_count,
                             (unsigned long)(uart1_port.stats.rx_error + uart1_port.stats.err_count),
                             (unsigned long)TIM_Timer_GetIrqCount(&tim7_timer));
        }
    }
#endif

#if can_debug==1
    {
        static uint8_t s_counter = 0;
        static uint32_t s_last = 0;
        uint8_t frame[8];
        if (now - s_last >= 1000U)
        {
            s_last = now;
            s_counter++;
            for (uint8_t i = 0; i < 8U; i++) { frame[i] = s_counter; }
            if (CAN_Bus_Send(&can1_bus, 0x123U, frame, 8U) != 0U)
            {
                UART_Port_Printf(&uart1_port, "[can] [FAIL] tx failed (bus off? mail full?)\r\n");
            }
            /* 无负载时 bus_error 会涨: CAN 需要 120R 终端电阻或至少一个节点 */
            /* free: 空邮箱数 —— 持续 0 且 txerr 暴涨 = 发不出去(无 ACK);
               free 正常且 tx 持续涨 = 发成功, rx=0 则问题在接收侧 */
            UART_Port_Printf(&uart1_port,
                             "[can] tx=%lu rx=%lu txerr=%lu buserr=%lu free=%lu esr=0x%08lX | last: id=0x%03X dlc=%u [%02X %02X %02X %02X]\r\n",
                             (unsigned long)can1_bus.stats.tx_count,
                             (unsigned long)can1_bus.stats.rx_count,
                             (unsigned long)can1_bus.stats.tx_error,
                             (unsigned long)can1_bus.stats.bus_error,
                             (unsigned long)HAL_CAN_GetTxMailboxesFreeLevel(can1_bus.hcan),
                             (unsigned long)can1_bus.hcan->Instance->ESR,
                             s_can_rx.id, s_can_rx.dlc,
                             s_can_rx.data[0], s_can_rx.data[1], s_can_rx.data[2], s_can_rx.data[3]);
        }
    }
#endif

#if usb_debug==1
    {
        static uint32_t s_last = 0;
        if (now - s_last >= 1000U)
        {
            s_last = now;
            /* 发送自检: PC 端 COM 口应每秒收到一帧 hello;
               未枚举/上一帧未发完 → BUSY/FAIL, 只计 tx_error 不阻塞 */
            (void)USB_Transmit_Data((uint8_t *)"hello world\n", 12U);
            UART_Port_Printf(&uart1_port,
                             "[usb] tx=%lu rx=%lu err=%lu drop=%lu | tick_ms=%lu\r\n",
                             (unsigned long)usb0.stats.tx_count,
                             (unsigned long)usb0.stats.rx_count,
                             (unsigned long)usb0.stats.tx_error,
                             (unsigned long)usb0.stats.rx_drop,
                             (unsigned long)TIM_Timer_GetIrqCount(&tim7_timer));
        }
    }
#endif

#if vofa_debug==1
    {
        static uint32_t s_last = 0;
        if (now - s_last >= 50U) /* 20Hz 波形, 与 VOFA_TX_PERIOD_MS 一致 */
        {
            s_last = now;
            float wave = (float)(TIM_Timer_GetIrqCount(&tim7_timer) % 1000U); /* 锯齿波 0~999 */
            float ch[4] = {wave, s_vofa_speed_target, s_vofa_pid.Kp, s_vofa_pid.Output};
            Vofa_SendFloats(&vofa, ch, 4);                /* VOFA+ 设成 justfloat 接收 */
        }
    }
#endif

#if stepmotor_debug==1
    /* StepMotor_Run 已按 STEP_CTRL_PERIOD(3ms) 订阅到 TIM7 中断节拍
       (debug_init 里注册), 这里只跑测试状态机 */
    switch (s_step_state)
    {
        case ST_IDLE:
            if (now - s_step_tick >= 1000U)
            {
                s_step_state = ST_ENABLE_DONE;
                s_step_tick  = now;
                UART_Port_Printf(&uart1_port, "[step] enabled, start fwd\r\n");
            }
            break;

        case ST_ENABLE_DONE:
            StepMotor_SetSpeed(ZDT_TEST_MOTOR, 10.0f, 50); /* +10 r/min 正转 */
            s_step_state = ST_FWD;
            s_step_tick  = now;
            break;

        case ST_FWD:
            if (now - s_step_tick >= 3000U)
            {
                StepMotor_Stop(ZDT_TEST_MOTOR);
                s_step_state = ST_STOPPED;
                s_step_tick  = now;
                UART_Port_Printf(&uart1_port, "[step] stop, start rev\r\n");
            }
            break;

        case ST_STOPPED:
            if (now - s_step_tick >= 500U)
            {
                StepMotor_SetSpeed(ZDT_TEST_MOTOR, -10.0f, 50); /* -10 r/min 反转 */
                s_step_state = ST_REV;
                s_step_tick  = now;
            }
            break;

        case ST_REV:
            if (now - s_step_tick >= 3000U)
            {
                StepMotor_Stop(ZDT_TEST_MOTOR);
                s_step_state = ST_POS_F;
                s_step_tick  = now;
            }
            break;

        case ST_POS_F:
            StepMotor_SetAngleInc(ZDT_TEST_MOTOR, 360.0f, 200.0f, 50); /* 相对 +360° */
            s_step_state = ST_POS_R;
            s_step_tick  = now;
            UART_Port_Printf(&uart1_port, "[step] pos +360 deg\r\n");
            break;

        case ST_POS_R:
            if (now - s_step_tick >= 3000U)
            {
                StepMotor_SetAngleInc(ZDT_TEST_MOTOR, -360.0f, 200.0f, 50); /* 相对 -360° */
                s_step_state = ST_FINISHED;
                s_step_tick  = now;
                UART_Port_Printf(&uart1_port, "[step] pos -360 deg\r\n");
            }
            break;

        case ST_FINISHED:
            if (now - s_step_tick >= 3000U)
            {
                s_step_state = ST_IDLE;
                s_step_tick  = now;
                UART_Port_Printf(&uart1_port, "[step] round done, next round in 1s\r\n");
            }
            break;

        default:
            s_step_state = ST_IDLE;
            break;
    }
    /* 顺带打印位置反馈与 CAN 收发的电机应答帧 */
    {
        static uint32_t s_last = 0;
        if (now - s_last >= 1000U)
        {
            s_last = now;
            /* total_angle 转成整数度打印: newlib-nano 未开 float printf */
            long deg = (long)(ZDT_TEST_MOTOR->total_angle + (ZDT_TEST_MOTOR->total_angle >= 0.0f ? 0.5f : -0.5f));
            UART_Port_Printf(&uart1_port,
                             "[step] pos=%ld deg (pulse=%ld) | can tx=%lu rx=%lu txerr=%lu free=%lu esr=0x%08lX | last reply: id=0x%03X dlc=%u [%02X %02X %02X %02X]\r\n",
                             deg, (long)ZDT_TEST_MOTOR->curPulse,
                             (unsigned long)can1_bus.stats.tx_count,
                             (unsigned long)can1_bus.stats.rx_count,
                             (unsigned long)can1_bus.stats.tx_error,
                             (unsigned long)HAL_CAN_GetTxMailboxesFreeLevel(can1_bus.hcan),
                             (unsigned long)can1_bus.hcan->Instance->ESR,
                             s_can_rx.id, s_can_rx.dlc,
                             s_can_rx.data[0], s_can_rx.data[1], s_can_rx.data[2], s_can_rx.data[3]);
        }
    }
#endif

#if dji_motor_debug==1
    /* DJ_MotorRun 已由 task_it.c 按 DJ_CTRL_PERIOD(3ms) 在中断里跑,
       这里只负责测试状态机: 等反馈 -> 正转 -> 停 -> 反转 -> 停 -> 角度±360° */
    switch (s_dji_state)
    {
        case DJ_ST_WAIT_FB:
            /* 首帧反馈到达前不动: 验证 CAN 订阅 0x201 是否通 */
            if (DJ_TEST_MOTOR->rx_once)
            {
                UART_Port_Printf(&uart1_port, "[dji] [PASS] fb ok, enc=%u spd=%d, start fwd\r\n",
                                 DJ_TEST_MOTOR->angle, (int)DJ_TEST_MOTOR->speed);
                DJ_SetSpeed(DJ_TEST_MOTOR, DJ_TEST_SPEED);
                s_dji_state = DJ_ST_FWD;
                s_dji_tick  = now;
            }
            else if (now - s_dji_tick >= 3000U)
            {
                s_dji_tick = now;
                /* 超时: 查电机供电/CAN 线序/120R 终端, 或编号是否真是 1 */
                UART_Port_Printf(&uart1_port,
                                 "[dji] [FAIL] no fb in 3s (rx=%lu buserr=%lu esr=0x%08lX)\r\n",
                                 (unsigned long)can1_bus.stats.rx_count,
                                 (unsigned long)can1_bus.stats.bus_error,
                                 (unsigned long)can1_bus.hcan->Instance->ESR);
            }
            break;

        case DJ_ST_FWD:
            if (now - s_dji_tick >= 2000U)
            {
                DJ_SetSpeed(DJ_TEST_MOTOR, 0.0f);
                s_dji_state = DJ_ST_STOP1;
                s_dji_tick  = now;
                UART_Port_Printf(&uart1_port, "[dji] stop, then rev\r\n");
            }
            break;

        case DJ_ST_STOP1:
            if (now - s_dji_tick >= 500U)
            {
                DJ_SetSpeed(DJ_TEST_MOTOR, -DJ_TEST_SPEED);
                s_dji_state = DJ_ST_REV;
                s_dji_tick  = now;
            }
            break;

        case DJ_ST_REV:
            if (now - s_dji_tick >= 2000U)
            {
                DJ_SetSpeed(DJ_TEST_MOTOR, 0.0f);
                s_dji_state = DJ_ST_STOP2;
                s_dji_tick  = now;
                UART_Port_Printf(&uart1_port, "[dji] stop, then angle +360\r\n");
            }
            break;

        case DJ_ST_STOP2:
            if (now - s_dji_tick >= 500U)
            {
                DJ_ClearAngle(DJ_TEST_MOTOR);
                DJ_SetAngleInc(DJ_TEST_MOTOR, 360.0f); /* 相对当前 +360°(输出轴) */
                s_dji_state = DJ_ST_POS_F;
                s_dji_tick  = now;
            }
            break;

        case DJ_ST_POS_F:
            if (now - s_dji_tick >= 4000U)
            {
                DJ_SetAngleInc(DJ_TEST_MOTOR, -360.0f); /* 回到起点 */
                s_dji_state = DJ_ST_POS_R;
                s_dji_tick  = now;
                UART_Port_Printf(&uart1_port, "[dji] angle -360 back\r\n");
            }
            break;

        case DJ_ST_POS_R:
            if (now - s_dji_tick >= 4000U)
            {
                s_dji_state = DJ_ST_DONE;
                s_dji_tick  = now;
                UART_Port_Printf(&uart1_port, "[dji] round done, next round in 1s\r\n");
            }
            break;

        case DJ_ST_DONE:
            if (now - s_dji_tick >= 1000U)
            {
                s_dji_state = DJ_ST_WAIT_FB;
                s_dji_tick  = now;
            }
            break;

        default:
            s_dji_state = DJ_ST_WAIT_FB;
            break;
    }
    /* 每秒打印一次反馈与统计(newlib-nano 无 float printf, 角度转 long) */
    {
        static uint32_t s_last = 0;
        if (now - s_last >= 1000U)
        {
            s_last = now;
            long ang = (long)(DJ_TEST_MOTOR->total_angle +
                              (DJ_TEST_MOTOR->total_angle >= 0.0f ? 0.5f : -0.5f));
            UART_Port_Printf(&uart1_port,
                             "[dji] st=%d spd=%d tgt=%ld ang=%ld norx=%u | can tx=%lu rx=%lu free=%lu\r\n",
                             (int)s_dji_state,
                             (int)DJ_TEST_MOTOR->speed,
                             (long)DJ_TEST_MOTOR->setSpeed,
                             ang,
                             (unsigned)DJ_TEST_MOTOR->no_rx_count,
                             (unsigned long)can1_bus.stats.tx_count,
                             (unsigned long)can1_bus.stats.rx_count,
                             (unsigned long)HAL_CAN_GetTxMailboxesFreeLevel(can1_bus.hcan));
        }
    }
#endif

#if servo_debug==1
    /* 舵机 0~270° 扫描: 上扫 -> 下扫 -> 越界钳位核对 -> 汇总, 循环往复。
       判据: 每步回读 CCR 折算脉宽, 与期望差 >20µs 当场 [FAIL]; 轮末 [PASS]/[FAIL] */
    // switch (s_sv_state)
    // {
    //     case SV_ST_START:
    //         Debug_ServoSetAndCheck(0);
    //         s_sv_state = SV_ST_UP;
    //         s_sv_tick  = now;
    //         UART_Port_Printf(&uart1_port, "[servo] sweep up 0 -> %d, step %d deg\r\n",
    //                          SERVO_ANGLE_270_MAX, SERVO_TEST_STEP_DEG);
    //         break;

    //     case SV_ST_UP:
    //         if (now - s_sv_tick >= SERVO_TEST_STEP_MS)
    //         {
    //             s_sv_tick  = now;
    //             s_sv_angle = (int16_t)(s_sv_angle + SERVO_TEST_STEP_DEG);
    //             if (s_sv_angle >= SERVO_ANGLE_270_MAX)
    //             {
    //                 s_sv_angle = SERVO_ANGLE_270_MAX;
    //                 Debug_ServoSetAndCheck(s_sv_angle);
    //                 s_sv_state = SV_ST_DOWN;
    //                 UART_Port_Printf(&uart1_port, "[servo] top reached, sweep down\r\n");
    //             }
    //             else
    //             {
    //                 Debug_ServoSetAndCheck(s_sv_angle);
    //             }
    //         }
    //         break;

    //     case SV_ST_DOWN:
    //         if (now - s_sv_tick >= SERVO_TEST_STEP_MS)
    //         {
    //             s_sv_tick  = now;
    //             s_sv_angle = (int16_t)(s_sv_angle - SERVO_TEST_STEP_DEG);
    //             if (s_sv_angle <= 0)
    //             {
    //                 s_sv_angle = 0;
    //                 Debug_ServoSetAndCheck(s_sv_angle);
    //                 s_sv_state = SV_ST_CLAMP_LO;
    //                 s_sv_tick  = now;
    //                 UART_Port_Printf(&uart1_port, "[servo] bottom reached, clamp checks\r\n");
    //             }
    //             else
    //             {
    //                 Debug_ServoSetAndCheck(s_sv_angle);
    //             }
    //         }
    //         break;

    //     case SV_ST_CLAMP_LO:
    //         if (now - s_sv_tick >= 500U)
    //         {
    //             Debug_ServoSetAndCheck(-50); /* 负值应被钳到 0°(500µs) */
    //             s_sv_state = SV_ST_CLAMP_HI;
    //             s_sv_tick  = now;
    //         }
    //         break;

    //     case SV_ST_CLAMP_HI:
    //         if (now - s_sv_tick >= 500U)
    //         {
    //             Debug_ServoSetAndCheck(320); /* 超上限应被钳到 270°(2500µs) */
    //             s_sv_state = SV_ST_REPORT;
    //             s_sv_tick  = now;
    //         }
    //         break;

    //     case SV_ST_REPORT:
    //         if (now - s_sv_tick >= 500U)
    //         {
    //             UART_Port_Printf(&uart1_port,
    //                              "[servo] round done: steps=%lu err=%lu [%s]\r\n",
    //                              (unsigned long)s_sv_steps,
    //                              (unsigned long)s_sv_err,
    //                              (s_sv_err == 0U) ? "PASS" : "FAIL");
    //             s_sv_steps = 0;
    //             s_sv_err   = 0;
    //             s_sv_state = SV_ST_START;
    //             s_sv_tick  = now;
    //         }
    //         break;

    //     default:
    //         s_sv_state = SV_ST_START;
    //         break;
    // }
     Debug_ServoSetAndCheck(0);
#endif

#if tray_debug==1
    /* ---- 第12层 tray_debug: 旋转托盘测试状态机(见文件头部流程说明) ----
       每步两拍: 下发后立刻核对(角度/脉宽/忙碌) -> 等到位 -> 核对(角度/脉宽/状态) */
    switch (s_tt_state)
    {
    case TT_INIT:
        if (!s_tt_sent)
        {
            s_tt_sent = 1U;
            (void)Tray_Home(); /* 每轮起点摆回已知姿态(不走动时瞬时可继续) */
            s_tt_tick = now;
            break;
        }
        if (Tray_IsBusy() && ((now - s_tt_tick) < TRAY_TEST_TO_MS))
        {
            break; /* 等上电/回零姿态走完 */
        }
        UART_Port_Printf(&uart1_port,
                         "[tray] round start: home=%u deg, sweep %u points, then rel + clamp\r\n",
                         (unsigned)TRAY_HOME_DEG, (unsigned)TT_ANGLE_CNT);
        s_tt_sent  = 0U;
        s_tt_idx   = 0U;
        s_tt_state = TT_SWEEP;
        s_tt_tick  = now;
        break;

    case TT_SWEEP:
    {
        uint16_t deg     = s_tt_angles[s_tt_idx];
        uint8_t  proceed = 0U;

        if (!s_tt_sent)
        {
            uint8_t ok = 1U;
            s_tt_sent = 1U;
            ok &= (uint8_t)((Tray_RotateTo(deg) == TRAY_OK) ? 1U : 0U);
            ok &= (uint8_t)((Tray_GetAngle() == deg) ? 1U : 0U);
            ok &= Debug_TrayPulseOk("sweep cmd", deg);
            TrayTest_Report("sweep cmd: angle+pulse", ok);
            s_tt_tick = now;
            break;
        }
        if (Tray_IsBusy())
        {
            if ((now - s_tt_tick) >= TRAY_TEST_TO_MS) /* 时长估算偏差过大 / 舵机没动 */
            {
                TrayTest_Report("sweep settle(timeout)", 0U);
                proceed = 1U;
            }
        }
        else /* 零长动作(目标与当前相同)会立刻到位 */
        {
            TrayTest_Report("sweep settled: pulse+state",
                            (uint8_t)(Debug_TrayPulseOk("sweep settled", deg) &
                                      (uint8_t)((Tray_GetState() == TRAY_IDLE) ? 1U : 0U)));
            proceed = 1U;
        }
        if (!proceed)
        {
            break;
        }

        s_tt_sent = 0U;
        s_tt_idx++;
        if (s_tt_idx >= TT_ANGLE_CNT)
        {
            s_tt_state = TT_CLAMP;
            s_tt_idx   = 0U;
        }
        s_tt_tick = now;
        break;
    }

    case TT_CLAMP:
        /* 纯核对: 上一段停在 270°, 这几个请求的目标都不变, 不产生新动作 */
        {
            uint8_t ok = 1U;
            ok &= (uint8_t)((Tray_RotateTo(400U) == TRAY_OK) ? 1U : 0U);  /* 越上限 -> 钳到 270 */
            ok &= (uint8_t)((Tray_GetAngle() == TRAY_MAX_DEG) ? 1U : 0U);
            ok &= Debug_TrayPulseOk("clamp hi", TRAY_MAX_DEG);
            ok &= (uint8_t)((Tray_RotateRel(1000) == TRAY_OK) ? 1U : 0U); /* 相对越界 -> 仍钳在 270 */
            ok &= (uint8_t)((Tray_GetAngle() == TRAY_MAX_DEG) ? 1U : 0U);
            ok &= (uint8_t)((Tray_RotateRel(0) == TRAY_ERR_PARAM) ? 1U : 0U); /* delta=0 -> 拒绝 */
            TrayTest_Report("clamp: to(400)->270, rel(+1000)->270, rel(0)->ERR_PARAM", ok);
        }
        s_tt_sent  = 0U;
        s_tt_idx   = 0U;
        s_tt_state = TT_REL;
        s_tt_tick  = now;
        break;

    case TT_REL:
    {
        int16_t  delta   = s_tt_rels[s_tt_idx].delta;
        uint16_t expect  = s_tt_rels[s_tt_idx].expect;
        uint8_t  proceed = 0U;

        if (!s_tt_sent)
        {
            uint8_t ok = 1U;
            s_tt_sent = 1U;
            ok &= (uint8_t)((Tray_RotateRel(delta) == TRAY_OK) ? 1U : 0U);
            ok &= (uint8_t)((Tray_GetAngle() == expect) ? 1U : 0U);
            ok &= Debug_TrayPulseOk(s_tt_rels[s_tt_idx].tag, expect);
            TrayTest_Report("rel cmd: angle+pulse", ok);
            s_tt_tick = now;
            break;
        }
        if (Tray_IsBusy())
        {
            if ((now - s_tt_tick) >= TRAY_TEST_TO_MS)
            {
                TrayTest_Report("rel settle(timeout)", 0U);
                proceed = 1U;
            }
        }
        else
        {
            TrayTest_Report("rel settled: pulse+state",
                            (uint8_t)(Debug_TrayPulseOk(s_tt_rels[s_tt_idx].tag, expect) &
                                      (uint8_t)((Tray_GetState() == TRAY_IDLE) ? 1U : 0U)));
            proceed = 1U;
        }
        if (!proceed)
        {
            break;
        }

        s_tt_sent = 0U;
        s_tt_idx++;
        if (s_tt_idx >= TT_REL_CNT)
        {
            s_tt_state = TT_REPORT;
            s_tt_idx   = 0U;
        }
        s_tt_tick = now;
        break;
    }

    case TT_REPORT:
        if ((now - s_tt_tick) >= TRAY_TEST_GAP_MS)
        {
            UART_Port_Printf(&uart1_port, "[tray] round done: steps=%lu err=%lu [%s]\r\n",
                             (unsigned long)s_tt_steps, (unsigned long)s_tt_err,
                             (s_tt_err == 0U) ? "PASS" : "FAIL");
            s_tt_steps = 0U;
            s_tt_err   = 0U;
            s_tt_sent  = 0U;
            s_tt_idx   = 0U;
            s_tt_state = TT_INIT;
            s_tt_tick  = now;
        }
        break;

    default:
        s_tt_state = TT_INIT;
        break;
    }
#endif

#if storage_debug==1
    /* ---- 第11层 storage_debug: 旋转储物盘测试状态机(见文件头部流程说明) ----
       每步两拍: 下发后立刻核对(目标/状态/驱动已收) -> 等到位 -> 核对(槽位/误差/转动量) */
    switch (s_sg_state)
    {
    case SG_INIT:
        /* 等首帧 CAN 反馈: 无反馈时 Storage_* 一律拒绝下发(角度口径无意义) */
        if (!Storage_HasFeedback())
        {
            if (now - s_sg_tick >= STORAGE_TEST_TO_MS)
            {
                UART_Port_Printf(&uart1_port,
                                 "[storage] [FAIL] no CAN feedback in %lums, check id=0x%03X/wiring\r\n",
                                 (unsigned long)STORAGE_TEST_TO_MS,
                                 (unsigned)(0x200U + STORAGE_MOTOR_CAN_ID));
                StorageTest_Status("no-rx");
                s_sg_tick = now; /* 继续等, 不刷屏 */
            }
            break;
        }
        /* 建立参考: 把当前位置定为槽位 0, 使本轮起点已知 */
        if (!s_sg_sent)
        {
            Storage_Result_e r = Storage_SetReference(0U);
            s_sg_sent = 1U;
            StorageTest_Report("SetReference(0) on current pos",
                               (uint8_t)((r == STORAGE_OK) ? 1U : 0U));
            s_sg_tick = now;
            break;
        }
        UART_Port_Printf(&uart1_port,
                         "[storage] round start: ref=slot0, ang=%ld.%ld | sweep 0->5, wrap 5->0, rev\r\n",
                         SG_ToDeci(Storage_GetActualAngle()) / 10L,
                         SG_Abs(SG_ToDeci(Storage_GetActualAngle()) % 10L));
        s_sg_sent  = 0U;
        s_sg_slot  = 1U; /* 0 已是起点, 从 1 开始正序 */
        s_sg_state = SG_FWD;
        s_sg_tick  = now;
        break;

    case SG_FWD:
    case SG_REV:
    case SG_FWD_WRAP:
        /* 三步共用: 请求槽位 = s_sg_slot, 下发 -> 等到位 -> 核对 */
        if (!s_sg_sent)
        {
            uint8_t  ok = 1U;
            uint8_t  req = s_sg_slot;
            float    tgt_exp;
            Storage_Result_e r;

            s_sg_sent = 1U;
            s_sg_ang0 = Storage_GetActualAngle(); /* 记录起点, 算实际转动量 */
            r = Storage_RotateToSlot(req);

            ok &= (uint8_t)((r == STORAGE_OK) ? 1U : 0U);
            ok &= (uint8_t)((Storage_GetState() == STORAGE_MOVING) ? 1U : 0U);

            /* 目标角度应等于该槽位的理论角, 或其在绕圈口径下的就近等价角
               (差值必是 360° 的整数倍) */
            tgt_exp = Storage_SlotToAngle(req);
            {
                float d = Storage_GetTargetAngle() - tgt_exp;
                while (d >= 180.0f)  { d -= 360.0f; }
                while (d < -180.0f)  { d += 360.0f; }
                ok &= (uint8_t)((SG_Abs(SG_ToDeci(d)) <= SG_ToDeci(0.5f)) ? 1U : 0U);
            }
            /* 真的下发到电机层了: 驱动 setAngle 与模块目标一致,
               且已进角度模式(1=ANGLE_MODE, 见 DJ_MotorMode_e) */
            ok &= (uint8_t)((SG_Abs(SG_ToDeci(STORAGE_MOTOR->setAngle -
                                              Storage_GetTargetAngle())) <= SG_ToDeci(0.5f))
                            ? 1U : 0U);
            ok &= (uint8_t)((STORAGE_MOTOR->mode == (uint8_t)ANGLE_MODE) ? 1U : 0U);

            StorageTest_Report("rotate: cmd+target+drv", ok);
            if (!ok)
            {
                StorageTest_Status("rotate-cmd");
            }
            s_sg_tick = now;
            break;
        }

        /* 等到位; 模块自身 8s 超时会转 FAULT, 本层超时只作兜底 */
        if (Storage_IsBusy())
        {
            if (now - s_sg_tick >= STORAGE_TEST_TO_MS)
            {
                StorageTest_Report("rotate: arrived(timeout)", 0U);
                StorageTest_Status("rotate-timeout");
                Storage_ClearFault();
                s_sg_sent = 0U;
                s_sg_tick = now;
                goto sg_step; /* 超时也往下走, 让整轮跑完 */
            }
            break;
        }

        if (!Storage_IsSlotArrived())
        {
            if (Storage_IsFault())
            {
                StorageTest_Report("rotate: FAULT (see status)", 0U);
                StorageTest_Status("rotate-fault");
                Storage_ClearFault();
            }
            else
            {
                StorageTest_Report("rotate: not arrived", 0U);
                StorageTest_Status("rotate-notarrived");
            }
            s_sg_sent = 0U;
            s_sg_tick = now;
            goto sg_step;
        }

        {
            uint8_t ok = 1U;
            long    err;
            long    moved;   /* 实际转动量(度 x10) */
            long    want;    /* 最短路径理论量(度 x10) */

            /* 1) 状态与记账 */
            ok &= (uint8_t)((Storage_GetState() == STORAGE_IDLE) ? 1U : 0U);
            ok &= (uint8_t)((Storage_GetSlot() == s_sg_slot) ? 1U : 0U);

            /* 2) 角度误差在模块容差内 */
            err = SG_Abs(SG_ToDeci(Storage_GetAngleError()));
            ok &= (uint8_t)((err <= SG_ToDeci(STORAGE_ARRIVE_DEG)) ? 1U : 0U);

            /* 3) 路径核对: 实际转动量 vs 最短路径理论量。
                  就近选点生效时 |Δ| <= 180°; 超过说明白转了一圈 */
            moved = SG_Abs(SG_ToDeci(Storage_GetActualAngle() - s_sg_ang0));
            want  = SG_Abs(SG_ToDeci(Storage_SlotToAngle(s_sg_slot) - s_sg_ang0));
            while (want > SG_ToDeci(180.0f)) { want -= SG_ToDeci(360.0f); want = SG_Abs(want); }
            ok &= (uint8_t)((moved <= SG_ToDeci(190.0f)) ? 1U : 0U); /* 留 10° 余量 */
            ok &= (uint8_t)((SG_Abs(moved - want) <= SG_ToDeci(15.0f)) ? 1U : 0U);

            UART_Port_Printf(&uart1_port,
                             "[storage] slot %u: ang=%ld.%ld err=%ld.%ld moved=%ld.%ld want=%ld.%ld (shortest)\r\n",
                             (unsigned)s_sg_slot,
                             SG_ToDeci(Storage_GetActualAngle()) / 10L,
                             SG_Abs(SG_ToDeci(Storage_GetActualAngle()) % 10L),
                             err / 10L, SG_Abs(err % 10L),
                             moved / 10L, SG_Abs(moved % 10L),
                             want / 10L, SG_Abs(want % 10L));
            StorageTest_Report("rotate: arrived(slot+err+path)", ok);
            if (!ok)
            {
                StorageTest_Status("rotate-badarrive");
            }
        }
        s_sg_sent = 0U;

sg_step:
        /* 步进: 正序 1->5 后做回绕, 回绕后反序 5->1 */
        if (s_sg_state == SG_FWD)
        {
            if (s_sg_slot + 1U < STORAGE_SLOT_NUM)
            {
                s_sg_slot++;
            }
            else
            {
                s_sg_state = SG_FWD_WRAP;
                s_sg_slot  = 0U; /* 5 -> 0 回绕: 就近应反向 60°, 不是正向 300° */
            }
        }
        else if (s_sg_state == SG_FWD_WRAP)
        {
            s_sg_state = SG_REV;
            s_sg_slot  = STORAGE_SLOT_NUM - 1U; /* 从 5 开始反序 */
        }
        else /* SG_REV: 5->4->...->1 */
        {
            if (s_sg_slot > 1U)
            {
                s_sg_slot--;
            }
            else
            {
                s_sg_state = SG_SETREF;
            }
        }
        s_sg_tick = now;
        break;

    case SG_SETREF:
        /* 参考重定义核对: 把当前位置(槽 1)重定义为槽 4, 再请求槽 4 应"不动" */
        if (!s_sg_sent)
        {
            Storage_Result_e r;
            uint8_t ok = 1U;

            s_sg_sent = 1U;
            r = Storage_SetReference(4U);
            ok &= (uint8_t)((r == STORAGE_OK) ? 1U : 0U);
            ok &= (uint8_t)((Storage_GetSlot() == 4U) ? 1U : 0U);
            /* 重定义后当前角度应约为 0(零点=当时的编码器值) */
            ok &= (uint8_t)((SG_Abs(SG_ToDeci(Storage_GetActualAngle())) <= SG_ToDeci(2.0f))
                            ? 1U : 0U);
            StorageTest_Report("SetReference(4) -> ang~0, slot=4", ok);
            s_sg_tick = now;
            break;
        }
        /* 再请求槽 4: 已在目标 -> 不下发, 直接 OK(且不产生运动) */
        {
            Storage_Result_e r;
            uint8_t ok = 1U;
            float   a0 = Storage_GetActualAngle();

            r = Storage_RotateToSlot(4U);
            ok &= (uint8_t)((r == STORAGE_OK) ? 1U : 0U);
            ok &= (uint8_t)((Storage_IsBusy() == false) ? 1U : 0U);
            ok &= (uint8_t)((SG_Abs(SG_ToDeci(Storage_GetActualAngle() - a0)) <= SG_ToDeci(2.0f))
                            ? 1U : 0U);
            StorageTest_Report("same-slot request: no move", ok);
        }
        s_sg_sent  = 0U;
        s_sg_state = SG_REJECT;
        s_sg_tick  = now;
        break;

    case SG_REJECT:
        /* 拒绝核对: 越界槽位 -> ERR_PARAM */
        if (!s_sg_sent)
        {
            Storage_Result_e r = Storage_RotateToSlot((uint8_t)STORAGE_SLOT_NUM); /* 越界 */
            s_sg_sent = 1U;
            StorageTest_Report("slot out of range -> ERR_PARAM",
                               (uint8_t)((r == STORAGE_ERR_PARAM) ? 1U : 0U));
            s_sg_tick = now;
            break;
        }
        s_sg_sent  = 0U;
        s_sg_state = SG_REPORT;
        s_sg_tick  = now;
        break;

    case SG_REPORT:
        if (now - s_sg_tick >= STORAGE_TEST_GAP_MS)
        {
            UART_Port_Printf(&uart1_port, "[storage] round done: steps=%lu err=%lu [%s]\r\n",
                             (unsigned long)s_sg_steps, (unsigned long)s_sg_err,
                             (s_sg_err == 0U) ? "PASS" : "FAIL");
            s_sg_steps = 0U;
            s_sg_err   = 0U;
            s_sg_sent  = 0U;
            s_sg_state = SG_INIT;
            s_sg_tick  = now;
        }
        break;

    default:
        s_sg_state = SG_INIT;
        break;
    }
#endif

#if grip_debug==1
    /* ---- 第10层 grip_debug: 夹爪机构(双舵机)测试状态机 ----
       每步两拍: 下发后立刻核对(回报角度/该通道脉宽/忙碌标志) ->
       等动作时长走完 -> 到位核对(角度 + 脉宽 + 终态), 轮末汇总 [PASS]/[FAIL] */
    switch (s_gt_state)
    {
    case GT_INIT:
        if (!s_gt_sent)
        {
            s_gt_sent = 1U;
            Grip_Open();  /* 每轮起点摆回已知姿态: 夹爪张开 + Z 抬起 */
            Grip_ZUp();
            s_gt_tick = now;
            break;
        }
        if (Grip_IsBusy() && (now - s_gt_tick < GRIP_TEST_TO_MS))
        {
            break; /* 等姿态到位(上一轮停在 0% 闭合 + Z 270°) */
        }
        UART_Port_Printf(&uart1_port,
                         "[grip] round start: Z -> %u deg, jaw open -> 100%%/0%% close, clamp checks\r\n",
                         (unsigned)GRIP_Z_DOWN_DEG);
        s_gt_sent  = 0U;
        s_gt_state = GT_Z_DOWN;
        s_gt_tick  = now;
        break;

    case GT_Z_DOWN:
        if (!s_gt_sent)
        {
            uint8_t ok = 1U;
            s_gt_sent = 1U;
            Grip_ZDown();
            ok &= (uint8_t)((Grip_GetZAngle() == GRIP_Z_DOWN_DEG) ? 1U : 0U);
            ok &= Debug_GripPulseOk("Z down", GRIP_Z_SERVO_CH, GRIP_Z_DOWN_DEG);
            ok &= (uint8_t)Grip_ZIsBusy();
            GripTest_Report("Z down: cmd angle+pulse+busy", ok);
            s_gt_tick = now;
            break;
        }
        if (Grip_ZIsBusy())
        {
            if (now - s_gt_tick >= GRIP_TEST_TO_MS) /* 动作时长估算偏差过大或舵机没动 */
            {
                GripTest_Report("Z down: settled(timeout)", 0U);
                s_gt_sent  = 0U;
                s_gt_state = GT_Z_UP;
            }
            break;
        }
        GripTest_Report("Z down: settled",
                        Debug_GripPulseOk("Z down settled", GRIP_Z_SERVO_CH, GRIP_Z_DOWN_DEG));
        s_gt_sent  = 0U;
        s_gt_state = GT_Z_UP;
        break;

    case GT_Z_UP:
        if (!s_gt_sent)
        {
            uint8_t ok = 1U;
            s_gt_sent = 1U;
            Grip_ZUp();
            ok &= (uint8_t)((Grip_GetZAngle() == GRIP_Z_UP_DEG) ? 1U : 0U);
            ok &= Debug_GripPulseOk("Z up", GRIP_Z_SERVO_CH, GRIP_Z_UP_DEG);
            ok &= (uint8_t)Grip_ZIsBusy();
            GripTest_Report("Z up: cmd angle+pulse+busy", ok);
            s_gt_tick = now;
            break;
        }
        if (Grip_ZIsBusy())
        {
            if (now - s_gt_tick >= GRIP_TEST_TO_MS)
            {
                GripTest_Report("Z up: settled(timeout)", 0U);
                s_gt_sent  = 0U;
                s_gt_state = GT_JAW_OPEN;
            }
            break;
        }
        GripTest_Report("Z up: settled",
                        Debug_GripPulseOk("Z up settled", GRIP_Z_SERVO_CH, GRIP_Z_UP_DEG));
        s_gt_sent  = 0U;
        s_gt_state = GT_JAW_OPEN;
        break;

    case GT_JAW_OPEN:
        if (!s_gt_sent)
        {
            uint8_t ok = 1U;
            s_gt_sent = 1U;
            Grip_Open();
            ok &= (uint8_t)((Grip_GetJawAngle() == GRIP_JAW_OPEN_DEG) ? 1U : 0U);
            ok &= (uint8_t)((Grip_GetJawState() == GRIP_JAW_OPENING) ? 1U : 0U);
            ok &= Debug_GripPulseOk("jaw open", GRIP_JAW_SERVO_CH, GRIP_JAW_OPEN_DEG);
            ok &= (uint8_t)Grip_JawIsBusy();
            GripTest_Report("jaw open: cmd angle+state+pulse+busy", ok);
            s_gt_tick = now;
            break;
        }
        if (Grip_JawIsBusy())
        {
            if (now - s_gt_tick >= GRIP_TEST_TO_MS)
            {
                GripTest_Report("jaw open: settled(timeout)", 0U);
                s_gt_sent  = 0U;
                s_gt_state = GT_JAW_CLOSE_MAX;
            }
            break;
        }
        GripTest_Report("jaw open: settled(state+pulse)",
                        (uint8_t)(((Grip_GetJawState() == GRIP_JAW_OPEN) ? 1U : 0U) &
                                  Debug_GripPulseOk("jaw open settled", GRIP_JAW_SERVO_CH,
                                                    GRIP_JAW_OPEN_DEG)));
        s_gt_sent  = 0U;
        s_gt_state = GT_JAW_CLOSE_MAX;
        break;

    case GT_JAW_CLOSE_MAX:
        if (!s_gt_sent)
        {
            uint8_t ok = 1U;
            s_gt_sent = 1U;
            Grip_Close(100U); /* 100% 力度 = 全闭角(GRIP_JAW_CLOSE_DEG) */
            ok &= (uint8_t)((Grip_GetJawAngle() == GRIP_JAW_CLOSE_DEG) ? 1U : 0U);
            ok &= (uint8_t)((Grip_GetJawState() == GRIP_JAW_CLOSING) ? 1U : 0U);
            ok &= Debug_GripPulseOk("jaw close 100%", GRIP_JAW_SERVO_CH, GRIP_JAW_CLOSE_DEG);
            ok &= (uint8_t)Grip_JawIsBusy();
            GripTest_Report("jaw close 100%: cmd angle+state+pulse+busy", ok);
            s_gt_tick = now;
            break;
        }
        if (Grip_JawIsBusy())
        {
            if (now - s_gt_tick >= GRIP_TEST_TO_MS)
            {
                GripTest_Report("jaw close 100%: settled(timeout)", 0U);
                s_gt_sent  = 0U;
                s_gt_state = GT_JAW_CLOSE_MIN;
            }
            break;
        }
        /* 力度 >0 的闭合到位按开环口径报"夹住"(HELD) */
        GripTest_Report("jaw close 100%: settled(state+pulse)",
                        (uint8_t)(((Grip_GetJawState() == GRIP_JAW_HELD) ? 1U : 0U) &
                                  Debug_GripPulseOk("jaw close 100% settled", GRIP_JAW_SERVO_CH,
                                                    GRIP_JAW_CLOSE_DEG)));
        s_gt_sent  = 0U;
        s_gt_state = GT_JAW_CLOSE_MIN;
        break;

    case GT_JAW_CLOSE_MIN:
        if (!s_gt_sent)
        {
            uint8_t  ok  = 1U;
            uint16_t exp = Grip_CloseAngle(0U); /* 力度 0% = 保底轻夹行程(非全闭) */
            s_gt_sent = 1U;
            Grip_Close(0U);
            ok &= (uint8_t)((exp != GRIP_JAW_CLOSE_DEG) ? 1U : 0U); /* 下限比例生效 */
            ok &= (uint8_t)((Grip_GetJawAngle() == exp) ? 1U : 0U);
            ok &= (uint8_t)((Grip_GetJawState() == GRIP_JAW_CLOSING) ? 1U : 0U);
            ok &= Debug_GripPulseOk("jaw close 0%", GRIP_JAW_SERVO_CH, exp);
            ok &= (uint8_t)Grip_JawIsBusy();
            GripTest_Report("jaw close 0%: cmd angle+state+pulse+busy", ok);
            s_gt_tick = now;
            break;
        }
        if (Grip_JawIsBusy())
        {
            if (now - s_gt_tick >= GRIP_TEST_TO_MS)
            {
                GripTest_Report("jaw close 0%: settled(timeout)", 0U);
                s_gt_sent  = 0U;
                s_gt_state = GT_CLAMP;
            }
            break;
        }
        /* 力度 =0 的闭合到位报"合上"(CLOSED, 未夹持) */
        GripTest_Report("jaw close 0%: settled(state+pulse)",
                        (uint8_t)(((Grip_GetJawState() == GRIP_JAW_CLOSED) ? 1U : 0U) &
                                  Debug_GripPulseOk("jaw close 0% settled", GRIP_JAW_SERVO_CH,
                                                    Grip_CloseAngle(0U))));
        s_gt_sent  = 0U;
        s_gt_state = GT_CLAMP;
        break;

    case GT_CLAMP:
        if (!s_gt_sent)
        {
            Grip_Result_e rz;
            Grip_Result_e rc;
            uint8_t ok = 1U;

            s_gt_sent = 1U;
            rz = Grip_MoveZ(300U);  /* 超上限 -> 钳到 270° */
            rc = Grip_Close(150U);  /* 力度 >100% -> 拒绝, 保持上一步输出 */
            ok &= (uint8_t)((rz == GRIP_OK) ? 1U : 0U);
            ok &= (uint8_t)((Grip_GetZAngle() == SERVO_ANGLE_270_MAX) ? 1U : 0U);
            ok &= Debug_GripPulseOk("Z clamp 300->270", GRIP_Z_SERVO_CH, SERVO_ANGLE_270_MAX);
            ok &= (uint8_t)((rc == GRIP_ERR_PARAM) ? 1U : 0U);
            ok &= (uint8_t)((Grip_GetJawAngle() == Grip_CloseAngle(0U)) ? 1U : 0U);
            GripTest_Report("Z clamp 300->270 + force>100 rejected", ok);
            s_gt_tick = now;
            break;
        }
        if (Grip_IsBusy()) /* 等 Z 轴 300->270 这一步走完再收轮 */
        {
            if (now - s_gt_tick >= GRIP_TEST_TO_MS)
            {
                s_gt_sent  = 0U;
                s_gt_state = GT_REPORT;
            }
            break;
        }
        s_gt_sent  = 0U;
        s_gt_state = GT_REPORT;
        break;

    case GT_REPORT:
        if (now - s_gt_tick >= GRIP_TEST_GAP_MS)
        {
            UART_Port_Printf(&uart1_port, "[grip] round done: steps=%lu err=%lu [%s]\r\n",
                             (unsigned long)s_gt_steps, (unsigned long)s_gt_err,
                             (s_gt_err == 0U) ? "PASS" : "FAIL");
            s_gt_steps = 0U;
            s_gt_err   = 0U;
            s_gt_sent  = 0U;
            s_gt_state = GT_INIT;
            s_gt_tick  = now;
        }
        break;

    default:
        s_gt_state = GT_INIT;
        break;
    }
#endif

#if motion_debug==1
    /* ---- 第8层 motion_debug: CoreXY 循环测试状态机(见文件头部流程说明) ---- */
    {
        MotionSys_XY_State_e mst = MotionSys_XY_GetState();

        /* 通信故障: 只在进入 FAULT 时打印并善后一次(Stop 在主循环上下文发) */
        if (mst == MS_XY_FAULT && s_mt_state != MT_FAULT)
        {
            UART_Port_Printf(&uart1_port, "[motion] [FAIL] comm FAULT -> Stop\r\n");
            MotionSys_XY_Stop();
            s_mt_fail++;
            s_mt_state = MT_FAULT;
            s_mt_tick = now;
        }

        switch (s_mt_state)
        {
        case MT_INIT:
            /* 每轮起点: 参考/实际清零重同步, 消除上一轮累计误差 */
            if (now - s_mt_tick >= 1000U)
            {
                /* 每轮重发 QPos 参数(运动模式=01 绝对): 上电那一次若丢失/被拒,
                   FC 会按相对模式累加跑飞; 应答状态由下方 F1 打印核对 */
                Emm_V5_Set_QPos_Params(MS_XY_MOTOR_A_ADDR, MS_XY_QPOS_VEL_RPM,
                                       MS_XY_QPOS_ACC, 1U, false);
                Emm_V5_Set_QPos_Params(MS_XY_MOTOR_B_ADDR, MS_XY_QPOS_VEL_RPM,
                                       MS_XY_QPOS_ACC, 1U, false);

                if (MotionSys_XY_SetPosition(0.0f, 0.0f) != MS_XY_OK)
                {
                    MotionTest_Fail("SetPosition busy");
                    break;
                }
                s_mt_exp_x = 0.0f;
                s_mt_exp_y = 0.0f;
                s_mt_idx = 0U;
                if (MotionSys_XY_MoveRel(s_mt_legs[0].dx, s_mt_legs[0].dy,
                                         s_mt_legs[0].feed) != MS_XY_OK)
                {
                    MotionTest_Fail("leg0 enqueue");
                    break;
                }
                s_mt_exp_x += s_mt_legs[0].dx;
                s_mt_exp_y += s_mt_legs[0].dy;
                s_mt_tick = now;
                s_mt_state = MT_LEG;
                UART_Port_Printf(&uart1_port, "[motion] -- round start, leg 1/8 %s\r\n",
                                 s_mt_legs[0].tag);
            }
            break;

        case MT_LEG:
            if (MotionSys_XY_IsBusy())
            {
                s_mt_tick = now; /* 在飞: 持续刷新, 停稳计时从 IDLE 后起算 */
                break;
            }
            if (now - s_mt_tick < MT_SETTLE_MS)
            {
                break; /* 等驱动器停稳再测, 避开跟随滞后 */
            }
            MotionTest_Check(s_mt_legs[s_mt_idx].tag);
            s_mt_idx++;
            if (s_mt_idx < MT_LEG_CNT)
            {
                if (MotionSys_XY_MoveRel(s_mt_legs[s_mt_idx].dx, s_mt_legs[s_mt_idx].dy,
                                         s_mt_legs[s_mt_idx].feed) != MS_XY_OK)
                {
                    MotionTest_Fail("leg enqueue");
                    break;
                }
                s_mt_exp_x += s_mt_legs[s_mt_idx].dx;
                s_mt_exp_y += s_mt_legs[s_mt_idx].dy;
                s_mt_tick = now;
                UART_Port_Printf(&uart1_port, "[motion] -- leg %u/8 %s\r\n",
                                 (unsigned)(s_mt_idx + 1U), s_mt_legs[s_mt_idx].tag);
            }
            else
            {
                /* 正方形 4 段一口气连发: 4 条全部入队后一次规划,
                   验证 lookahead 在 90° 弯角不停车(期望回到原点 0,0) */
                if (MotionSys_XY_MoveTo(25.0f,  0.0f, MT_FAST_FEED) != MS_XY_OK ||
                    MotionSys_XY_MoveTo(25.0f, 25.0f, MT_FAST_FEED) != MS_XY_OK ||
                    MotionSys_XY_MoveTo( 0.0f, 25.0f, MT_FAST_FEED) != MS_XY_OK ||
                    MotionSys_XY_MoveTo( 0.0f,  0.0f, MT_FAST_FEED) != MS_XY_OK)
                {
                    MotionTest_Fail("square enqueue");
                    break;
                }
                s_mt_tick = now;
                s_mt_state = MT_SQUARE;
                UART_Port_Printf(&uart1_port,
                                 "[motion] -- square 4-seg burst, feed %ld um/s\r\n",
                                 (long)(MT_FAST_FEED * 1000.0f));
            }
            break;

        case MT_SQUARE:
            if (MotionSys_XY_IsBusy())
            {
                s_mt_tick = now;
                break;
            }
            if (now - s_mt_tick < MT_SETTLE_MS)
            {
                break;
            }
            MotionTest_Check("square 4-seg burst (lookahead)");
            UART_Port_Printf(&uart1_port,
                             "[motion] round done: pass=%lu fail=%lu, next round in %ums\r\n",
                             (unsigned long)s_mt_pass, (unsigned long)s_mt_fail,
                             (unsigned)MT_ROUND_GAP_MS);
            s_mt_state = MT_DONE;
            s_mt_tick = now;
            break;

        case MT_DONE:
            if (now - s_mt_tick >= MT_ROUND_GAP_MS)
            {
                s_mt_tick = now;
                s_mt_state = MT_INIT;
            }
            break;

        case MT_FAULT:
            /* Stop 已在进入时发过; 5s 后回到 INIT 整轮重试 */
            if (now - s_mt_tick >= MT_FAULT_RETRY_MS)
            {
                s_mt_tick = now;
                s_mt_state = MT_INIT;
            }
            break;

        default:
            s_mt_state = MT_INIT;
            break;
        }

        /* F1 应答状态(每次重发后打一行): 02=生效 E2=参数被拒 EE=格式错
           12/22/9F=异步完成类 —— 打 hex 供人工判读 */
        {
            static uint32_t s_f1_seen[2] = {0U, 0U};

            for (uint8_t mi = 0U; mi < 2U; mi++)
            {
                uint32_t n = StepMotor_ZDT[mi].f1_count;

                if (n != s_f1_seen[mi])
                {
                    s_f1_seen[mi] = n;
                    UART_Port_Printf(&uart1_port,
                                     "[motion] F1 reply %c st=%02X n=%lu\r\n",
                                     (mi == 0U) ? 'A' : 'B',
                                     (unsigned)StepMotor_ZDT[mi].f1_status,
                                     (unsigned long)n);
                }
            }
        }

        /* 0x36 原始回包抓点: 首帧或 DLC 变化时打一行 hex —— 核对实际
           回包格式(手册 X42S V1.0.5 §5.5.13 应为 DLC=7 [36 符号 pos4 6B])。
           ISR 写、主循环读, 先在短临界区里整体快照再判断/打印, 防跨帧撕裂 */
        {
            static uint8_t s_raw_seen[2] = {0U, 0U};
            static uint8_t s_raw_dlc[2]  = {0U, 0U};

            for (uint8_t mi = 0U; mi < 2U; mi++)
            {
                uint8_t  dlc;
                uint32_t cnt;
                uint8_t  buf[8];
                uint32_t primask;

                cnt = StepMotor_ZDT[mi].cpos_raw_count; /* 单字读, 原子 */
                if (cnt == 0U)
                {
                    continue;
                }

                primask = __get_PRIMASK();
                __disable_irq();
                dlc = StepMotor_ZDT[mi].cpos_raw_dlc;
                for (uint8_t b = 0U; b < 8U; b++)
                {
                    buf[b] = StepMotor_ZDT[mi].cpos_raw_data[b];
                }
                __set_PRIMASK(primask);

                if (s_raw_seen[mi] == 0U || s_raw_dlc[mi] != dlc)
                {
                    s_raw_seen[mi] = 1U;
                    s_raw_dlc[mi]  = dlc;
                    UART_Port_Printf(&uart1_port,
                                     "[motion] CPOS raw %c dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X] n=%lu\r\n",
                                     (mi == 0U) ? 'A' : 'B',
                                     (unsigned)dlc,
                                     buf[0], buf[1], buf[2], buf[3],
                                     buf[4], buf[5], buf[6], buf[7],
                                     (unsigned long)cnt);
                }
            }
        }

        /* 1s 状态行: 状态/期望/参考/实际/原始反馈/统计 */
        if (now - s_mt_last_print >= 1000U)
        {
            float cx, cy, ax, ay;

            s_mt_last_print = now;
            MotionSys_XY_GetCmdPos(&cx, &cy);
            MotionSys_XY_GetActualPos(&ax, &ay);
            UART_Port_Printf(&uart1_port,
                             "[motion] st=%d busy=%d exp=(%ld,%ld) cmd=(%ld,%ld) act=(%ld,%ld)um raw=(%ld,%ld) P=%lu F=%lu\r\n",
                             (int)mst, (int)MotionSys_XY_IsBusy(),
                             MT_ToUm(s_mt_exp_x), MT_ToUm(s_mt_exp_y),
                             MT_ToUm(cx), MT_ToUm(cy),
                             MT_ToUm(ax), MT_ToUm(ay),
                             (long)StepMotor_ZDT[0].curPulse,
                             (long)StepMotor_ZDT[1].curPulse,
                             (unsigned long)s_mt_pass, (unsigned long)s_mt_fail);
        }
    }
#endif

#if host_debug==1
    /* ---- 第9层 host_debug: 上位机通讯主循环服务 + 主机联调测试 ----
       HostComm_Poll: 排空命令队列分发执行 -> 夹爪/任务结果判定 -> 状态上报;
       组帧/校验/ACK 在 USB 中断里完成(HostComm_OnRx), 此处只做执行侧。
       本测试模拟"上位机": 经 USB 发下行帧驱动 XY(本机回环, 电机会动!) */
    HostComm_Poll();

    /* F1(QPos 参数)应答状态: host_comm 每段运动前重发 F1, 这里在应答计数
       变化时打一行 —— st: 02=生效, E2=参数被拒, EE=格式错, 其他打 hex;
       计数一直不涨 = 驱动器压根没回(F1 丢失的老根因: 上电未就绪 -> 0xFC
       按相对模式累加跑飞)。与 motion_debug 的 F1 打印同口径 */
    {
        static uint32_t s_hf1_seen[2] = {0U, 0U};

        for (uint8_t mi = 0U; mi < 2U; mi++)
        {
            uint32_t n = StepMotor_ZDT[mi].f1_count;

            if (n != s_hf1_seen[mi])
            {
                s_hf1_seen[mi] = n;
                UART_Port_Printf(&uart1_port,
                                 "[host] F1 reply %c st=%02X n=%lu\r\n",
                                 (mi == 0U) ? 'A' : 'B',
                                 (unsigned)StepMotor_ZDT[mi].f1_status,
                                 (unsigned long)n);
            }
        }
    }

#if (HT_AUTO_TEST == 1)
    {
        static uint32_t s_ht_last_print = 0U;
        HostCommStats_t st;
        float ax, ay, cx, cy;

        switch (s_ht_state)
        {
        case HT_INIT:
            /* 等首帧上报发出 = USB 已枚举可用(Init 里 report_due=1 首圈即发) */
            if (usb0.stats.tx_count > 0U || now - s_ht_tick >= 5000U)
            {
                if (usb0.stats.tx_count == 0U)
                {
                    UART_Port_Printf(&uart1_port, "[host] [FAIL] no tx in 5s, usb not enumerated?\r\n");
                }
                s_ht_state = HT_STEP;
                s_ht_step  = 0U;
                s_ht_tick  = now;
            }
            break;

        case HT_STEP:
            if (now - s_ht_tick < HT_WAIT_MS)
            {
                break;
            }
            s_ht_tick = now;
            s_ht_round++; /* 每轮 TASK_ID 低字节+1, 避免 SEQ/TASK 复用歧义 */
            switch (s_ht_step)
            {
            case 0:
                HostTest_Send(HOST_FLAG_CLEAR, 0, 0, HOST_GRIP_HOLD, "clear alarm");
                break;
            case 1:
                HostTest_Send(HOST_FLAG_MOTION, 40000, 0, HOST_GRIP_HOLD, "move (40,0) pick");
                break;
            case 2:
                HostTest_Send(HOST_FLAG_GRIP, 0, 0, HOST_GRIP_CLOSE, "grip close");
                break;
            case 3:
                HostTest_Send(HOST_FLAG_MOTION, 0, 40000, HOST_GRIP_HOLD, "move (0,40) place");
                break;
            case 4:
                HostTest_Send(HOST_FLAG_GRIP, 0, 0, HOST_GRIP_OPEN, "grip open (place)");
                break;
            case 5:
                /* 不发任何结束标志: 等设备按顺序模板自动判成功并补发 TASK_DONE_EVENT */
                UART_Port_Printf(&uart1_port, "[host] wait auto task-done ...\r\n");
                break;
            case 6:
                HostTest_Send(HOST_FLAG_MOTION, 0, 0, HOST_GRIP_HOLD, "return (0,0)");
                break;
            default:
                s_ht_state = HT_GAP;
                s_ht_tick  = now;
                break;
            }
            s_ht_step++;
            break;

        case HT_GAP:
            if (now - s_ht_tick >= HT_ROUND_GAP_MS)
            {
                s_ht_state = HT_STEP;
                s_ht_step  = 0U;
                s_ht_tick  = now;
            }
            break;

        default:
            s_ht_state = HT_INIT;
            break;
        }

        /* 1s 状态行: 协议态/运动态/位置/统计(对照 0x81 事件帧人工判读) */
        if (now - s_ht_last_print >= 1000U)
        {
            s_ht_last_print = now;
            HostComm_GetStats(&st);
            MotionSys_XY_GetActualPos(&ax, &ay);
            MotionSys_XY_GetCmdPos(&cx, &cy);
            HostComm_PrintDiag();

            UART_Port_Printf(&uart1_port,
                             "[host] r=%u st=%u step=%u | act=(%ld,%ld) cmd=(%ld,%ld)um mst=%d busy=%d | rx=%lu drop=%lu dup=%lu | usb tx=%lu rx=%lu err=%lu\r\n",
                             (unsigned)s_ht_round, (unsigned)s_ht_state, (unsigned)s_ht_step,
                             (long)(ax * 1000.0f + (ax >= 0.0f ? 0.5f : -0.5f)),
                             (long)(ay * 1000.0f + (ay >= 0.0f ? 0.5f : -0.5f)),
                             (long)(cx * 1000.0f + (cx >= 0.0f ? 0.5f : -0.5f)),
                             (long)(cy * 1000.0f + (cy >= 0.0f ? 0.5f : -0.5f)),
                             (int)MotionSys_XY_GetState(), (int)MotionSys_XY_IsBusy(),
                             (unsigned long)st.rx_frames, (unsigned long)st.rx_drop,
                             (unsigned long)st.rx_dup,
                             (unsigned long)usb0.stats.tx_count,
                             (unsigned long)usb0.stats.rx_count,
                             (unsigned long)usb0.stats.tx_error);
        }
    }
#else
    {
        static uint32_t s_ht_last_print = 0U;
        HostCommStats_t st;
        float ax, ay, cx, cy;

        /* 被动模式: 1s 状态行(位置/运动态/受理统计), 不发任何下行帧
           act=反馈实际位置, cmd=参考(流式下发目标): 两者分离可判"驱动器
           跟不上参考"还是"参考本身走慢"(act 验证项: 停稳后 cmd≈act) */
        if (now - s_ht_last_print >= 1000U)
        {
            s_ht_last_print = now;
            HostComm_GetStats(&st);
            MotionSys_XY_GetActualPos(&ax, &ay);
            MotionSys_XY_GetCmdPos(&cx, &cy);
            HostComm_PrintDiag();

            UART_Port_Printf(&uart1_port,
                             "[host] passive | act=(%ld,%ld) cmd=(%ld,%ld)um mst=%d busy=%d | rx=%lu drop=%lu dup=%lu | usb tx=%lu rx=%lu err=%lu\r\n",
                             (long)(ax * 1000.0f + (ax >= 0.0f ? 0.5f : -0.5f)),
                             (long)(ay * 1000.0f + (ay >= 0.0f ? 0.5f : -0.5f)),
                             (long)(cx * 1000.0f + (cx >= 0.0f ? 0.5f : -0.5f)),
                             (long)(cy * 1000.0f + (cy >= 0.0f ? 0.5f : -0.5f)),
                             (int)MotionSys_XY_GetState(), (int)MotionSys_XY_IsBusy(),
                             (unsigned long)st.rx_frames, (unsigned long)st.rx_drop,
                             (unsigned long)st.rx_dup,
                             (unsigned long)usb0.stats.tx_count,
                             (unsigned long)usb0.stats.rx_count,
                             (unsigned long)usb0.stats.tx_error);
        }
    }
#endif
#endif

#if msgcenter_debug==1
    {
        static uint32_t s_last_mc = 0;
        if (now - s_last_mc >= 1000U)
        {
            s_last_mc = now;
            UART_Port_Printf(&uart1_port,
                             "[msgcenter] loop heartbeat | pass=%lu fail=%lu\r\n",
                             (unsigned long)s_mc_pass_cnt, (unsigned long)s_mc_fail_cnt);
        }
    }
#endif
}
