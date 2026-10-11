/**
 * @file    servo.c
 * @brief   舵机控制实现: 只做 角度↔脉宽 换算, 输出全部经 drv_pwm 驱动层,
 *          不直接操作 HAL 寄存器
 *          50Hz 时间基由 drv_pwm 按定时器实际时钟重算, 不依赖 CubeMX 的 PSC/ARR;
 *          CubeMX 负责引脚复用(PE9/PE11/PE13 -> TIM1_CH1/2/3)
 */

/* Includes ------------------------------------------------------------------*/
#include "servo.h"
#include "drv_pwm.h"

/* ========================================================================== */
/*  私有宏 / 变量                                                              */
/* ========================================================================== */

/* 绑定点: 舵机挂在哪个定时器上, 换引脚/换定时器只改这里 */
#define SERVO_TIM (&htim1)

#define SERVO_FREQ_HZ 50U /* 舵机 PWM 频率(Hz) */

/* Init 初值占空比(千分比): 由中位脉宽与频率导出, 与 SetAngle 的换算同源 */
#define SERVO_DUTY_CENTER  0U

/* 三路舵机通道对象, 下标 = TIM_CHANNEL_x >> 2 */
static PWM_ChannelObject s_servo_ch[3];

/* Init 幂等标志: 现在有 Grip(夹爪 CH1/CH2) 与 Tray(托盘 CH3) 两个模块
   各自需要舵机输出, 谁先 Init 不确定 —— 用标志保证三路只真正初始化一次 */
static uint8_t s_inited;

/* ========================================================================== */
/*  舵机接口                                                                   */
/* ========================================================================== */

/**
  * @brief  启动 CH1/CH2/CH3 的 PWM 输出并恢复零位
  * @note   必须在 MX_TIM1_Init() 之后调用; 失败视为致命错误
  *         幂等: 三路同时初始化, 重复调用直接返回(可被 Grip/Tray 各自调用)
  */
void Servo_Init(void)
{
  static const uint32_t s_channels[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
  static const char *const s_names[3] = {"SERVO_CH1", "SERVO_CH2", "SERVO_CH3"};

  if (s_inited)
  {
    return;
  }

  /******舵机恢复中位*********/
  for (uint8_t i = 0; i < 3U; i++)
  {
    if (PWM_Channel_Init(&s_servo_ch[i], SERVO_TIM, s_channels[i], s_names[i],
                         SERVO_FREQ_HZ, SERVO_DUTY_CENTER) != 0U)
    {
      Error_Handler();
    }
  }

  s_inited = 1U;
}

/**
  * @brief  ±135° 舵机角度控制
  * @param  channel TIM_CHANNEL_1 / TIM_CHANNEL_2 / TIM_CHANNEL_3
  * @param  angle   角度, -135 ~ +135 度(超限自动钳位)
  */
void Servo_SetAngle_135(uint32_t channel, int16_t angle)
{
  int32_t offset;
  uint32_t pulse_us;

  /* 非法通道直接返回 */
  if (channel != TIM_CHANNEL_1 &&
      channel != TIM_CHANNEL_2 &&
      channel != TIM_CHANNEL_3)
  {
    return;
  }

  /* 限制角度范围 -135 ~ +135 */
  if (angle >  SERVO_ANGLE_MAX) angle =  SERVO_ANGLE_MAX;
  if (angle < -SERVO_ANGLE_MAX) angle = -SERVO_ANGLE_MAX;

  /* 线性映射: -135°(500µs) ~ 0°(1500µs) ~ +135°(2500µs), 全程按有符号运算 */
  offset   = (int32_t)angle * ((int32_t)SERVO_MAX_US - (int32_t)SERVO_CENTER_US) / SERVO_ANGLE_MAX;
  pulse_us = (uint32_t)((int32_t)SERVO_CENTER_US + offset);

  /* TIM_CHANNEL_x = (x-1)*4, >>2 即通道下标(HAL 的 CCR 索引同款约定) */
  PWM_Channel_SetPulse(&s_servo_ch[channel >> 2], pulse_us);
}

/**
  * @brief  0~270° 舵机角度控制(同一行程的另一种角度原点约定)
  * @param  channel TIM_CHANNEL_1 / TIM_CHANNEL_2 / TIM_CHANNEL_3
  * @param  angle   角度, 0 ~ 270 度(超限自动钳位, 负值钳到 0)
  *                 0°→500µs, 135°→1500µs(中位), 270°→2500µs
  */
void Servo_SetAngle_270(uint32_t channel, int16_t angle)
{
  int32_t offset;
  uint32_t pulse_us;

  /* 非法通道直接返回 */
  if (channel != TIM_CHANNEL_1 &&
      channel != TIM_CHANNEL_2 &&
      channel != TIM_CHANNEL_3)
  {
    return;
  }

  /* 限制角度范围 0 ~ 270 */
  if (angle > SERVO_ANGLE_270_MAX) angle = SERVO_ANGLE_270_MAX;
  if (angle < 0)                    angle = 0;

  /* 线性映射: 0°(500µs) ~ 135°(1500µs) ~ 270°(2500µs), 全程按有符号运算 */
  offset   = (int32_t)angle * ((int32_t)SERVO_MAX_US - (int32_t)SERVO_MIN_US) / SERVO_ANGLE_270_MAX;
  pulse_us = (uint32_t)((int32_t)SERVO_MIN_US + offset);

  /* TIM_CHANNEL_x = (x-1)*4, >>2 即通道下标(HAL 的 CCR 索引同款约定) */
  PWM_Channel_SetPulse(&s_servo_ch[channel >> 2], pulse_us);
}
