/**
 * @file    servo.h
 * @brief   舵机控制接口(±135°)
 *          分层: 输出经 drv_pwm 驱动层(PWM_Channel_*), 舵机模块只做角度↔脉宽换算,
 *          不直接操作 HAL; 50Hz 时间基由 drv_pwm 按定时器实际时钟重算
 *          CH1/CH2/CH3 分别映射 PE9 / PE11 / PE13(引脚复用由 CubeMX 配置)
 */

#ifndef __SERVO_H__
#define __SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "tim.h"

/* ========================================================================== */
/*  配置宏                                                                     */
/* ========================================================================== */

#define SERVO_ANGLE_MAX 135    /* ±135° 模式的角度上限(度) */
#define SERVO_ANGLE_270_MAX 270 /* 0~270° 模式的角度上限(度) */
#define SERVO_MIN_US    500    /* -135° / 0°   脉宽(µs) */
#define SERVO_CENTER_US 1500   /*    0° / 135° 脉宽(µs) */
#define SERVO_MAX_US    2500   /* +135° / 270° 脉宽(µs) */

/* ========================================================================== */
/*  舵机接口                                                                   */
/* ========================================================================== */

/**
  * @brief  启动 CH1/CH2/CH3 的 PWM 输出并恢复中位
  * @note   必须在 MX_TIM1_Init() 之后调用; CH2/CH3 的输出比较配置由本函数运行时补全
  */
void Servo_Init(void);

/**
  * @brief  ±135° 舵机角度控制
  * @param  channel TIM_CHANNEL_1 / TIM_CHANNEL_2 / TIM_CHANNEL_3
  * @param  angle   角度, -135 ~ +135 度(超限自动钳位)
  */
void Servo_SetAngle_135(uint32_t channel, int16_t angle);

/**
  * @brief  0~270° 舵机角度控制(同一行程的另一种角度原点约定)
  * @param  channel TIM_CHANNEL_1 / TIM_CHANNEL_2 / TIM_CHANNEL_3
  * @param  angle   角度, 0 ~ 270 度(超限自动钳位, 负值钳到 0)
  *                 0°→500µs, 135°→1500µs(中位), 270°→2500µs
  */
void Servo_SetAngle_270(uint32_t channel, int16_t angle);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H__ */
