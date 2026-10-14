/**
 * @file    tray.h
 * @brief   旋转托盘模块(单舵机直驱, 仅旋转)
 *
 *          机构: 一个 0~270° 舵机直接驱动托盘旋转, 只有旋转一个自由度
 *          (不升降/不翻转)。本模块把"机构语义"(转到角度/相对转动/回零)
 *          翻译成舵机角度, 角度 -> 脉宽由 servo 层完成, 输出经 drv_pwm
 *          落到 TIM1 —— 本模块不碰 HAL 与寄存器。
 *
 *          分层(与 Grip / MotionSys_XY 同一约定):
 *            机构语义 -> 角度 -> servo -> drv_pwm -> TIM1 PWM
 *            换通道/换定时器只改下面的 TRAY_SERVO_CH。
 *
 *          通道映射(按实际接线改; 引脚复用由 CubeMX 配置, 见 servo.h):
 *            TRAY_SERVO_CH     TIM1_CH3 (PE13)   托盘旋转舵机
 *            (CH1/CH2 已被夹爪 Grip 占用: Z 轴升降 / 开合)
 *
 *          开环说明(重要):
 *            舵机无位置反馈, 本模块**不知道托盘真实角度**。因此:
 *              - "动作中/到位"按动作时长近似(HAL_GetTick 计时, 查询时惰性老化,
 *                不占用节拍中断), 仅作状态与上报口径用, 不影响实际输出;
 *              - Tray_GetAngle() 返回的是**最近下发的目标角度**, 不是实测角度;
 *              - 动作未结束就下新目标 = 立即改目标(舵机不会等上一动作走完)。
 *
 *          行程与限位:
 *            舵机行程 0~270°, 所以托盘只能在 0~270° 内旋转(不是多圈)。
 *            越界一律钳位到 [TRAY_MIN_DEG, TRAY_MAX_DEG], 不静默丢弃;
 *            Tray_RotateRel 的越界同样钳位(结果落在行程内)。
 *
 *          使用约定:
 *            - 上电先 Tray_Init()(幂等; 内部含 Servo_Init, 多处调用无副作用),
 *              上电姿态 = TRAY_HOME_DEG;
 *            - 旋转期间用 Tray_IsBusy() 判"动作中";
 *            - 必须在 MX_TIM1_Init 之后调用。
 */

#ifndef __TRAY_H__
#define __TRAY_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "servo.h"

/* ========================================================================== */
/*  通道映射 (按接线改, 其它层只认这里的名字)                                   */
/* ========================================================================== */

#define TRAY_SERVO_CH   SERVO_CH_3 /* 托盘旋转舵机 (对应 PE13) */

/* ========================================================================== */
/*  行程 (0~270° 舵机; 角度方向按机械安装改)                                    */
/* ========================================================================== */

#define TRAY_MIN_DEG    0U    /* 行程下限(度) */
#define TRAY_MAX_DEG    270U  /* 行程上限(度), = SERVO_ANGLE_270_MAX */

/* 上电/回零位。托盘停在哪一格按机械决定; 改这里即可 */
#define TRAY_HOME_DEG   0U

/* ========================================================================== */
/*  动作时长 (开环: 按时间判"到位"; 与舵机实际速度对齐后调整)                    */
/* ========================================================================== */

/* 转动耗时随角度增大 —— 固定时长在大角度下会误判"到位" */
#define TRAY_MOVE_BASE_MS  120U   /* 起停固有延时(ms) */
#define TRAY_MS_PER_DEG    4U     /* 每度转动近似耗时(ms/°); 270° 约 1.2s */
#define TRAY_MOVE_MAX_MS   2000U  /* 单次动作计时上限(ms), 防极端值 */

/* ========================================================================== */
/*  类型定义                                                                   */
/* ========================================================================== */

/* 接口返回码 */
typedef enum
{
  TRAY_OK = 0,        /* 成功(已下发, 异步执行) */
  TRAY_ERR_NOT_INIT,  /* 未调用 Tray_Init */
  TRAY_ERR_PARAM,     /* 参数非法(如相对转动量为 0 或越界不可达) */
} Tray_Result_e;

/* 托盘状态(开环: 由动作时长推进) */
typedef enum
{
  TRAY_IDLE = 0,      /* 空闲(含动作已完成) */
  TRAY_MOVING,        /* 旋转动作中 */
} Tray_State_e;

/* ========================================================================== */
/*  接口                                                                       */
/* ========================================================================== */

/**
  * @brief  初始化托盘机构(幂等)
  * @note   含一次 Servo_Init(启动 TIM1 CH1~CH3 的 50Hz 输出), 必须在
  *         MX_TIM1_Init 之后调用; 上电姿态 = TRAY_HOME_DEG。
  *         幂等: 重复调用直接返回(servo 层同样幂等, 与 Grip 谁先调都安全)
  */
void Tray_Init(void);

/* ---- 旋转 ---- */

/**
  * @brief  旋转到绝对角度(异步)
  * @param  deg 目标角度(度), 越界钳位到 [TRAY_MIN_DEG, TRAY_MAX_DEG]
  * @retval TRAY_OK / TRAY_ERR_NOT_INIT
  * @note   与当前目标角度相同则不下发(省总线), 直接返回 TRAY_OK
  */
Tray_Result_e Tray_RotateTo(uint16_t deg);

/**
  * @brief  相对当前目标角度转动 delta 度(异步)
  * @param  delta_deg 有符号角度增量(度), 正负按机构方向
  * @retval TRAY_OK / TRAY_ERR_NOT_INIT / TRAY_ERR_PARAM(delta = 0)
  * @note   结果越界则钳位到行程内(不拒绝), 便于上层做"多转一点"的微调
  */
Tray_Result_e Tray_RotateRel(int16_t delta_deg);

/* 回到 TRAY_HOME_DEG */
Tray_Result_e Tray_Home(void);

/* ---- 状态查询 ---- */

/* 是否正在旋转(按动作时长近似) */
bool Tray_IsBusy(void);

/* 查询状态: TRAY_IDLE / TRAY_MOVING */
Tray_State_e Tray_GetState(void);

/**
  * @brief  最近下发的目标角度(度)
  * @note   开环无反馈, 这不是实测角度, 只是"最后命令到哪"
  */
uint16_t Tray_GetAngle(void);

/**
  * @brief  角度 -> 动作时长换算(纯算式, 不依赖 Init, 便于上层/测试核对)
  * @param  delta_deg 本次转动的角度增量绝对值(度), 传 0 得固有延时
  * @retval 预计动作时长(ms), 上限 TRAY_MOVE_MAX_MS
  */
uint32_t Tray_MoveDurationMs(uint16_t delta_deg);

#ifdef __cplusplus
}
#endif

#endif /* __TRAY_H__ */