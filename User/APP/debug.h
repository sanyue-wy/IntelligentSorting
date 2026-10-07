//
// Created by wy on 2026/9/25.
//

#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * 分层检测开关: 从底层往上一步步验证, 建议一次只开 1 个宏,
 * 确认当前层通过后再打开下一层(下层的接入代码在上层的 #if 里自带)。
 *
 *   1.uart_debug      USART1 收发(printf + 回显)        <- 最先测
 *   2.can_debug       CAN1 收发
 *   3.usb_debug       USB CDC 虚拟串口
 *   4.vofa_debug      VOFA+ 波形 + 串口在线调参
 *   5.stepmotor_debug ZDT/Emm_V5 步进电机(依赖 CAN1)
 *   6.dji_motor_debug DJI 3508
 *   7.servo_debug     舵机 0~270° 扫描(回读脉宽核对, 舵机会动!)
 *   8.motion_debug    CoreXY 运动系统(电机会动! 8 条单腿 + 正方形循环)
 ******************************************************************************
 */

#define uart_debug  0
#define can_debug  0
#define usb_debug  0
#define vofa_debug  0
#define stepmotor_debug  0
#define dji_motor_debug  0
#define servo_debug  0
/* 8. motion_debug    CoreXY 运动系统(仿 Voron/Klipper, 挂两台 ZDT 步进)
      Init 后在 debug_run 里跑 8 条单腿 + 正方形连发循环测试;
      注意: 不要与 stepmotor_debug 同时打开(双主控抢同两台电机) */
#define motion_debug  1

/* 辅助开关: 1=CAN 片上环回自检(自发自收, 不经过收发器/总线/电机, 测固件不测硬件)
   通过判据: tx 持续增长 + rx 收到自己发的帧(如 id=0x001 dlc=3 [01 36 6B]) */
#define can_loopback  0

/* 公共打印通道(USART1)与公共总线(CAN1): 各层检测都依赖它们 */
#include "drv_uart.h"
#include "drv_can.h"
#include "tim.h"

#if vofa_debug==1
#include "vofa.h"
#endif

#if stepmotor_debug==1
#include "stepmotor.h"
#endif

#if usb_debug==1
#include "usbd_cdc_if.h"
#endif

#if dji_motor_debug==1
#include "dji_3508.h"
#endif

#if servo_debug==1
#include "servo.h"
#endif

#if motion_debug==1
#include "motion_sys_XY.h"
#endif

void debug_init(void);
void debug_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H__ */
