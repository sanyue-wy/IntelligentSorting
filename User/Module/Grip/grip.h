/**
 * @file    grip.h
 * @brief   夹爪执行机构(双舵机): 舵机1 管夹爪 Z 轴升降, 舵机2 管夹爪开合
 *
 *          分层: 本模块只把"机构语义"(升/降/开/合/力度)翻译成角度,
 *          角度 -> 脉宽由 servo 模块完成, 输出经 drv_pwm 落到 TIM1 CHx ——
 *          模块本身不碰 HAL 与寄存器, 换通道/换定时器只改下面的映射宏。
 *
 *          通道映射(按实际接线改; 引脚 PE9/PE11/PE13 -> TIM1_CH1/2/3 由
 *          CubeMX 配置, 见 servo.h):
 *            GRIP_Z_SERVO_CH   TIM1_CH1 (PE9)   舵机1: Z 轴升降
 *            GRIP_JAW_SERVO_CH TIM1_CH2 (PE11)  舵机2: 开合
 *
 *          无位置/夹持反馈(舵机开环): "动作中/到位"按动作时长近似
 *          (HAL_GetTick 计时, 查询时惰性老化, 不需要占用节拍中断),
 *          仅作状态与上报口径用, 不影响实际输出。
 *
 *          使用约定:
 *            - 上电先 Grip_Init()(幂等; 内部含 Servo_Init, 多处调用无副作用),
 *              上电姿态为 Z 抬起 + 夹爪张开;
 *            - 动作期间用 Grip_IsBusy()/Grip_GetJawState() 判"动作中/到位";
 *            - 开环舵机: 动作未结束就下新目标 = 立即改目标, 不会等上一动作走完。
 */

#ifndef __GRIP_H__
#define __GRIP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "tim.h"           /* TIM_CHANNEL_1 ~ TIM_CHANNEL_3 */

/* ========================================================================== */
/*  通道映射 (舵机1 / 舵机2; 按接线改, 其它层只认这里的名字)                    */
/* ========================================================================== */

#define GRIP_Z_SERVO_CH    TIM_CHANNEL_1 /* 舵机1: Z 轴升降 (TIM1_CH1 / PE9)  */
#define GRIP_JAW_SERVO_CH  TIM_CHANNEL_2 /* 舵机2: 夹爪开合 (TIM1_CH2 / PE11) */

/* ========================================================================== */
/*  机构行程 (0~270° 舵机; 角度方向按机械安装改)                                */
/* ========================================================================== */

/* Z 轴升降: 角度越大越靠下(远离 XY 行程), 取值 0~270 */
#define GRIP_Z_UP_DEG     0U     /* 抬起位: 不挡 XY 运动                        */
#define GRIP_Z_DOWN_DEG   180U   /* 取件位: 降到料位                            */
#define GRIP_Z_SAFE_DEG   GRIP_Z_UP_DEG /* 上电安全位(同抬起位)                 */

/* 夹爪开合: 张开角 -> 全闭角(力度 100% 时的角度) */
#define GRIP_JAW_OPEN_DEG    10U  /* 张开角(0~270°) */
#define GRIP_JAW_CLOSE_DEG   200U /* 全闭角(0~270°) */

/* 力度下限对应的闭合行程比例: 力度 0% 仍走这么多行程(保底轻夹, 防舵机堵转) */
#define GRIP_JAW_FORCE_FLOOR 0.30f

/* ========================================================================== */
/*  动作时长 (开环: 按时间判"到位"; 与舵机实际速度对齐后调整)                    */
/* ========================================================================== */

#define GRIP_Z_MOVE_MS    700U   /* Z 轴单次动作近似时长(ms) */
#define GRIP_JAW_MOVE_MS  600U   /* 夹爪开/合单次动作近似时长(ms) */

/* ========================================================================== */
/*  类型定义                                                                   */
/* ========================================================================== */

/* 接口返回码 */
typedef enum
{
  GRIP_OK = 0,        /* 成功(已下发, 异步执行) */
  GRIP_ERR_NOT_INIT,  /* 未调用 Grip_Init */
  GRIP_ERR_PARAM,     /* 参数非法(力度 > 100) */
} Grip_Result_e;

/* 夹爪(开合舵机)状态: 无反馈, 由动作时长推进 */
typedef enum
{
  GRIP_JAW_UNKNOWN = 0, /* 未 Init */
  GRIP_JAW_OPEN,        /* 张开到位 */
  GRIP_JAW_OPENING,     /* 张开动作中 */
  GRIP_JAW_CLOSED,      /* 闭合到位(力度=0, 未夹持) */
  GRIP_JAW_CLOSING,     /* 闭合动作中 */
  GRIP_JAW_HELD,        /* 闭合到位且力度>0(按开环口径视为夹住) */
} Grip_JawState_e;

/* ========================================================================== */
/*  接口                                                                       */
/* ========================================================================== */

/**
  * @brief  初始化夹爪机构
  * @note   含一次 Servo_Init(启动 TIM1 CH1~CH3 的 50Hz 输出), 必须在
  *         MX_TIM1_Init 之后调用; 上电姿态 = Z 抬起 + 夹爪张开。
  *         幂等: 重复调用直接返回(servo 层同样幂等, 与 Tray 谁先调都安全)
  */
void Grip_Init(void);

/* ---- Z 轴(升降, 舵机1) ---- */

/**
  * @brief  Z 轴移动到绝对角度
  * @param  angle 0 ~ 270 度(越界自动钳位)
  * @retval GRIP_OK / GRIP_ERR_NOT_INIT
  */
Grip_Result_e Grip_MoveZ(uint16_t angle);

/* Z 轴抬起到安全位(GRIP_Z_UP_DEG) */
Grip_Result_e Grip_ZUp(void);

/* Z 轴降到取件位(GRIP_Z_DOWN_DEG) */
Grip_Result_e Grip_ZDown(void);

/* ---- 夹爪(开合, 舵机2) ---- */

/* 张开到 GRIP_JAW_OPEN_DEG */
Grip_Result_e Grip_Open(void);

/**
  * @brief  闭合到指定力度
  * @param  force 0~100(%), 闭合行程比例 = FLOOR + (1-FLOOR)×force/100,
  *               故 0% 仍按 GRIP_JAW_FORCE_FLOOR 走保底行程(轻夹)
  * @retval GRIP_OK / GRIP_ERR_PARAM(force>100, 拒绝下发保持原输出) /
  *         GRIP_ERR_NOT_INIT
  */
Grip_Result_e Grip_Close(uint8_t force);

/**
  * @brief  力度 -> 闭合角度换算(纯算式, 不依赖 Init, 便于上层/测试核对)
  * @param  force 0~100(%), 越界按 100 处理
  */
uint16_t Grip_CloseAngle(uint8_t force);

/* ---- 状态查询 ---- */

/* Z 轴动作是否未结束(按 GRIP_Z_MOVE_MS 近似) */
bool Grip_ZIsBusy(void);

/* 夹爪开/合动作是否未结束(按 GRIP_JAW_MOVE_MS 近似) */
bool Grip_JawIsBusy(void);

/* 任一轴动作未结束 */
bool Grip_IsBusy(void);

/* 夹爪状态: 动作中返回 *_OPENING/_CLOSING, 到位返回 OPEN/CLOSED/HELD */
Grip_JawState_e Grip_GetJawState(void);

/* 最近一次下发的 Z 轴角度(度) */
uint16_t Grip_GetZAngle(void);

/* 最近一次下发的夹爪角度(度) */
uint16_t Grip_GetJawAngle(void);

#ifdef __cplusplus
}
#endif

#endif /* __GRIP_H__ */