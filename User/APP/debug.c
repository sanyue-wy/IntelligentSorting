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

/* Private function prototypes -----------------------------------------------*/
#if can_debug==1 || stepmotor_debug==1
static void Debug_CanRxCallback(CAN_BusObject *bus, CAN_RxHeaderTypeDef *header, uint8_t *data);
#endif
#if uart_debug==1
static void Debug_UartRxCallback(UART_PortObject *port, uint8_t *data, uint16_t size);
#endif
#if servo_debug==1
static uint32_t Debug_ServoReadPulseUs(uint32_t channel);
static void Debug_ServoSetAndCheck(int16_t angle);
#endif
#if motion_debug==1
static long   MT_ToUm(float v);
static long   MT_Abs(long v);
static void   MotionTest_Check(const char *tag);
static void   MotionTest_Fail(const char *why);
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

#if can_debug==1 || stepmotor_debug==1
    can1_bus.on_rx = Debug_CanRxCallback;     /* 挂兜底回调, 收任意帧都缓存 */
#endif

#if uart_debug==1
    /* 手动验证: 串口助手(115200) 发任意字节, 应原样回显 */
    uart1_port.on_rx = Debug_UartRxCallback;
    UART_Port_Printf(&uart1_port, "[uart] init ok\r\n");
#endif

#if usb_debug==1
    /* USB 设备已在 MX_USB_DEVICE_Init 初始化, 插上电脑等枚举即可 */
    UART_Port_Printf(&uart1_port, "[usb] wait device enumerate, then check COM port\r\n");
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
}

//死循环内运行的
void debug_run(void)
{
    uint32_t now = HAL_GetTick();
#if uart_debug==0 && can_debug==0 && usb_debug==0 && vofa_debug==0 \
    && stepmotor_debug==0 && dji_motor_debug==0 && servo_debug==0
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
            CDC_Transmit_FS((uint8_t *)"hello world\n", 12U);
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

        /* 1s 状态行: 状态/期望/参考/实际/统计 */
        if (now - s_mt_last_print >= 1000U)
        {
            float cx, cy, ax, ay;

            s_mt_last_print = now;
            MotionSys_XY_GetCmdPos(&cx, &cy);
            MotionSys_XY_GetActualPos(&ax, &ay);
            UART_Port_Printf(&uart1_port,
                             "[motion] st=%d busy=%d exp=(%ld,%ld) cmd=(%ld,%ld) act=(%ld,%ld)um P=%lu F=%lu\r\n",
                             (int)mst, (int)MotionSys_XY_IsBusy(),
                             MT_ToUm(s_mt_exp_x), MT_ToUm(s_mt_exp_y),
                             MT_ToUm(cx), MT_ToUm(cy),
                             MT_ToUm(ax), MT_ToUm(ay),
                             (unsigned long)s_mt_pass, (unsigned long)s_mt_fail);
        }
    }
#endif
}
