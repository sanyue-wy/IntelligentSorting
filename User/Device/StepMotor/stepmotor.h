/**
 * @file    stepmotor.h
 * @brief   ZDT/Emm_V5 系列步进闭环电机控制接口
 *          对接 stepmotor_ZDT 底层命令层(Emm_V5 协议, 经 CAN 拆帧发送),
 *          对外提供与 dji_3508 一致风格的电机对象封装:
 *            - 发送走底层 Emm_V5_* 接口(内部已完成多帧拆包)
 *            - 接收按 ZDT_CAN_EXT_ID(addr, 0) 订阅应答帧, 解析实时位置反馈
 *            - 对外角度用"度"、速度用"r/min", 内部换算成脉冲数/RPM
 */

#ifndef __STEPMOTOR_H__
#define __STEPMOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "drv_can.h"
#include "stepmotor_ZDT.h"

/* ========================================================================== */
/*  配置宏                                                                     */
/* ========================================================================== */

#define STEP_MOTOR_BUS (&can1_bus) /* 挂载的 CAN 总线对象 */

/* StepMotor_Run 的调用周期(秒), 必须与实际主循环节拍一致 */
#define STEP_CTRL_PERIOD 0.003f

#define STEP_DEFAULT_ACC       50U   /* 默认加速度 0-255, 0 表示直接启动 */
#define STEP_DEFAULT_MICROSTEP 16U   /* 默认细分值, 需与驱动器实际设置一致 */
#define STEP_STEPS_PER_REV_18  200U  /* 1.8° 步进电机整步数/圈 */
#define STEP_STEPS_PER_REV_09  400U  /* 0.9° 步进电机整步数/圈 */
#define STEP_MAX_SPEED_RPM     5000.0f /* Emm_V5 速度参数上限(RPM) */

/* 0x36 实时位置回包原始值的每圈计数(换算 total_angle 用):
   Emm 固件 0~65535 表示一圈 -> 65536 (X42S 手册 V1.0.5 §5.5.13);
   X 固件 角度值 = 原始值/10 度 -> 3600。 读 0x1A 选项状态 bit1 可辨:
   1=Emm 固件(用本默认值), 0=X 固件(改 3600 重编译) */
#ifndef STEP_CPOS_RAW_PER_REV
#define STEP_CPOS_RAW_PER_REV 65536.0f
#endif

#define STEP_MAX_MOTORS 4U /* 最多可注册的电机实例数 */

/* ========================================================================== */
/*  枚举 / 类型                                                                 */
/* ========================================================================== */

/* 电机步距角型号 */
typedef enum
{
  SM_TYPE_18DEG = 0, /* 1.8°/整步, 200 整步/圈 */
  SM_TYPE_09DEG,     /* 0.9°/整步, 400 整步/圈 */
} StepMotorType_e;

/* 运行模式 */
typedef enum
{
  SM_SPEED_MODE = 0,
  SM_ANGLE_MODE,
  SM_ANGLEINC_MODE,
} StepMotorMode_e;

/**
  * @brief  步进电机对象
  */
typedef struct
{
  /* 身份与配置 */
  uint8_t  addr;               /* Emm_V5 电机地址(扩展帧 ID = ZDT_CAN_EXT_ID(addr,0) = 地址<<8) */
  uint8_t  type;               /* StepMotorType_e */
  uint8_t  microstep;          /* 细分值, 仅用于角度<->脉冲换算, 需与驱动器一致 */
  uint16_t steps_per_rev;      /* 整步数/圈 */
  volatile uint8_t initialized;
  volatile uint8_t enabled;    /* 使能状态 */

  /* 设定量 */
  uint8_t  mode;               /* StepMotorMode_e */
  float    setSpeed;           /* 目标转速(r/min, 带符号: +CW / -CCW) */
  float    setAngle;           /* 目标角度(度, 带符号多圈) */
  uint8_t  setAcc;             /* 加速度 0-255, 0 表示直接启动 */

  /* 反馈(CAN 中断更新) */
  float    total_angle;        /* 实时位置(度, 带符号多圈, 按 STEP_CPOS_RAW_PER_REV 折算) */
  int32_t  curPulse;           /* 0x36 原始位置值(已解卷绕累计, 单位同回包) */
  uint16_t no_rx_count;        /* 连续未收到应答计数 */

  /* 0x36 回包诊断(CAN 中断写, 主循环打印; 用于核对实际回包格式) */
  volatile uint8_t  cpos_raw_dlc;      /* 最近一帧 0x36 回包的 DLC */
  volatile uint8_t  cpos_raw_data[8];  /* 最近一帧 0x36 回包原始字节 */
  volatile uint32_t cpos_raw_count;    /* 累计收到的 0x36 回包数 */
  volatile int32_t  cpos_prev_raw;     /* 上次原始值(解卷绕基准) */
  volatile uint8_t  cpos_base_valid;   /* 卷绕基准是否已建立 */
  volatile uint8_t  cpos_skip;         /* 清零后需丢弃的在途旧回包数(防再锚定竞态) */

  /* F1(QPos 运动参数)应答状态(诊断): 02=生效 E2=参数被拒 EE=格式错 */
  volatile uint8_t  f1_status;
  volatile uint32_t f1_count;          /* 累计收到的 F1 应答数(每次重发 +1) */
} StepMotor_t;

/* ========================================================================== */
/*  导出变量                                                                   */
/* ========================================================================== */
extern StepMotor_t StepMotor_ZDT[2];

/* ========================================================================== */
/*  电机接口                                                                   */
/* ========================================================================== */

/**
  * @brief  电机参数初始化并注册(会向 can1_bus 订阅其应答帧 ID)
  * @param  m       电机对象实例(调用方静态分配)
  * @param  addr    电机地址(Emm_V5 地址, 1~255, 0 为广播地址不可用)
  * @param  type    电机型号 SM_TYPE_18DEG / SM_TYPE_09DEG
  * @param  microstep 细分值, 传 0 则取 STEP_DEFAULT_MICROSTEP
  * @note   同一电机不可重复初始化; 必须在 Driver_CAN1_Init() 之后调用
  */
void StepMotor_Init(StepMotor_t *m, uint8_t addr, StepMotorType_e type, uint8_t microstep);

/* 电机使能/失能 */
void StepMotor_Enable(StepMotor_t *m, bool state);

/* 立即停止运动 */
void StepMotor_Stop(StepMotor_t *m);

/**
  * @brief  速度模式控制(单位: r/min, 带符号: +正转CW / -反转CCW)
  * @param  acc 加速度 0-255, 0 表示直接启动
  */
void StepMotor_SetSpeed(StepMotor_t *m, float speed, uint8_t acc);

/**
  * @brief  绝对角度控制(单位: 度, 带符号多圈; vel 为速度上限 RPM, acc 为加速度)
  */
void StepMotor_SetAngle(StepMotor_t *m, float angle, float vel, uint8_t acc);

/**
  * @brief  增量角度控制(单位: 度, 带符号; 相对当前实时位置)
  */
void StepMotor_SetAngleInc(StepMotor_t *m, float angleInc, float vel, uint8_t acc);

/* 将当前位置清零(同时清除本地角度跟踪量) */
void StepMotor_ClearAngle(StepMotor_t *m);

/**
  * @brief  周期维护: 轮询各电机实时位置并维护通信超时计数, 需按 STEP_CTRL_PERIOD 周期调用
  */
void StepMotor_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __STEPMOTOR_H__ */
