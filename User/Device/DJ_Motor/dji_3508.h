/**
 * @file    dji_3508.h
 * @brief   大疆 O3/O4 系列空心杯电机(2006/3508)控制接口
 *          对接本工程 drv_can 驱动层与 pid/td 算法层:
 *            - 发送走 CAN_Bus_Send(&can1_bus, ...)
 *            - 接收回调签名适配 CAN_RxCallback_t
 *            - TDPID 用 TD_t + PID_Controller_t 组合实现(本工程无独立 TDPID 类型)
 */

#ifndef __DJI_3508_H__
#define __DJI_3508_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "drv_can.h"
#include "pid.h"
#include "td.h"

/* ========================================================================== */
/*  功能开关                                                                   */
/* ========================================================================== */
/* PID / IMPEDANCE 两种控制方法可各自裁剪; 置 0 时对应函数不参与编译 */
#define DJ_USE_PID        1
#define DJ_USE_IMPEDANCE  1

/* ========================================================================== */
/*  配置宏                                                                     */
/* ========================================================================== */

#define M3508_DEC  19 /* 3508 减速比 */
#define M2006_DEC  36 /* 2006 减速比 */

#define MAX_CURRENT  16384  /* 最大电流 (mA, 有符号) */
#define MIN_CURRENT -16384  /* 最小电流 */

/* DJ_MotorRun 的调用周期(秒), 必须与实际主循环节拍一致 */
#define DJ_CTRL_PERIOD  0.003f

/* ========================================================================== */
/*  枚举 / 类型                                                                 */
/* ========================================================================== */

/* 电机型号 */
typedef enum
{
  M2006 = 0,
  M3508,
} DJ_MotorType_e;

/* 控制方法 */
typedef enum
{
  PID_METHOD = 0,
  IMPEDANCE_METHOD,
} DJ_ControllMethod_e;

/* 运行模式 */
typedef enum
{
  SPEED_MODE = 0,
  ANGLE_MODE,
  ANGLEINC_MODE,
} DJ_MotorMode_e;

/**
  * @brief  TDPID: 跟踪微分器平滑给定 + PID 的组合控制器
  * @note   对外保留 .out / .max_out 字段以兼容原例程写法;
  *         .max_out 每拍同步到内部 PID 的输出限幅(即角度环的速度上限)
  */
typedef struct
{
  TD_t             td;  /* 给定平滑 */
  PID_Controller_t pid; /* 实际 PID */
  float            out;      /* 本拍输出 */
  float            max_out;  /* 输出限幅, 可在线改写 */
} TDPID_t;

/**
  * @brief  电机对象
  */
typedef struct
{
  /* 身份与状态 */
  uint16_t ID;           /* CAN 标准帧 ID */
  uint8_t  method;       /* DJ_ControllMethod_e */
  uint8_t  mode;         /* DJ_MotorMode_e */
  volatile uint8_t initialized;
  uint16_t dec;          /* 减速比 */

  /* 反馈(CAN 中断更新) */
  uint16_t angle;        /* 编码器原始值 0~8191 */
  uint16_t last_angle;
  uint8_t  rx_once;      /* 是否已收到首帧反馈(首帧用于捕获零点, 跳过绕圈检测) */
  int16_t  speed;        /* 实时转速(有符号) */
  int16_t  current;      /* 实时电流(有符号) */
  int32_t  round_count;  /* 累计圈数(跨 8192 翻转累加) */
  float    total_angle;  /* 折算后的总角度(度, 已除减速比) */
  uint16_t offset_angle; /* 上电时的角度偏移 */
  uint16_t no_rx_count;  /* 连续未收到帧计数 */

  /* 设定量 */
  float setSpeed;        /* 目标速度 (r/min, 未减速前) */
  float setAngle;        /* 目标角度 (度) */

  /* 控制输出 */
  int16_t setCurrent;    /* 本拍下发电流指令 */

  /* PID 方法控制器 */
#if DJ_USE_PID
  PID_Controller_t PID_Speed;          /* 速度环 */
  TDPID_t          PID_Angle;          /* 角度环(TD+PID) */
  PID_Controller_t PID_SpeedOfAngle;   /* 角度环输出 -> 速度环 */
#endif
  /* 阻抗方法 */
#if DJ_USE_IMPEDANCE
  Impedance_t imp;
#endif
} DJ_Motor_t;

/* ========================================================================== */
/*  导出变量                                                                   */
/* ========================================================================== */
extern DJ_Motor_t DJ_Motor3508[3];

/* ========================================================================== */
/*  TDPID 辅助函数                                                             */
/* ========================================================================== */
void  TDPID_Init(TDPID_t *c, float kp, float ki, float kd,
                 float max_out, float max_iout, float control_period, float td_r);
float TDPID_Calc(TDPID_t *c, float measure, float ref);

/* ========================================================================== */
/*  电机接口                                                                   */
/* ========================================================================== */

/**
  * @brief  初始化并注册一个电机(会向 can1_bus 订阅其反馈帧 ID)
  * @param  motor     电机对象实例(调用方静态分配)
  * @param  Motor_ID  电机编号 1~8, 对应帧 ID 0x200+Motor_ID
  * @param  Motor_Type M2006 / M3508
  * @param  method    PID_METHOD / IMPEDANCE_METHOD
  * @note   同一电机不可重复初始化; 必须在 Driver_CAN1_Init() 之后调用
  */
void DJ_Init(DJ_Motor_t *motor, uint8_t Motor_ID, DJ_MotorType_e Motor_Type, DJ_ControllMethod_e method);

/**
  * @brief  电机控制主函数, 需按 DJ_CTRL_PERIOD 周期调用
  * @note   计算各电机电流指令, 并按 0x200(低4路)/0x1FF(高4路) 两帧发出
  */
void DJ_MotorRun(void);

/* 电机角度和圈数清零 */
void DJ_ClearAngle(DJ_Motor_t *motor);

#if DJ_USE_PID
/* 设置电机速度(单位: 未减速前的 r/min) */
void DJ_SetSpeed(DJ_Motor_t *motor, float speed);
/* 设置电机角度(单位: 度; speed 为速度上限) */
void DJ_SetAngle(DJ_Motor_t *motor, float angle, float speed);
/* 设置电机增量角度(单位: 度) */
void DJ_SetAngleInc(DJ_Motor_t *motor, float angleInc);
#endif

#if DJ_USE_IMPEDANCE
/* 阻抗模式速度控制 */
void DJ_SetImpSpeed(DJ_Motor_t *motor, float kd, float speed, int16_t current);
/* 阻抗模式角度控制 */
void DJ_SetImpAngle(DJ_Motor_t *motor, float kp, float kd, float angle, int16_t t);
/* 阻抗模式增量角度 */
void DJ_SetImpAngleInc(DJ_Motor_t *motor, float kp, float kd, float angle, int16_t t);
/* 阻尼模式 */
void DJ_SetImpDamp(DJ_Motor_t *motor, float kd);
/* 零力矩模式 */
void DJ_SetImpZeroTorque(DJ_Motor_t *motor);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DJI_3508_H__ */
