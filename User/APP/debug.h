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
 *   9.host_debug      上位机通讯 32B 帧协议 over USB CDC(电机会动!)
 *   10.grip_debug     夹爪机构 双舵机:舵机1 Z 轴升降 + 舵机2 开合(舵机会动!)
 *   11.storage_debug  旋转储物盘 3508 直驱: 6 格逐个转位 + 方向/最短路径核对(电机会转!)
 *   12.tray_debug     旋转托盘 单舵机: 0~270° 扫描 + 相对转动/越界钳位核对(舵机会动!)
 *   13.msgcenter_debug 消息中心 Pub/Sub 自检(1对1收发/多订阅广播/满队覆盖/参数边界)
 ******************************************************************************
 */

#define uart_debug  0
#define can_debug  0
#define usb_debug  0
#define vofa_debug  0
#define stepmotor_debug  0
#define dji_motor_debug  0
#define servo_debug  0
#define msgcenter_debug  0
/* 8. motion_debug    CoreXY 运动系统(电机会动! 8 条单腿 + 正方形循环)
      注意: 与 stepmotor_debug(双主控)、host_debug(双指挥)互斥 */
#define motion_debug  0
/* 9. host_debug      上位机通讯(32B 帧协议, USB CDC; 电机会动!)
      Init 内含 MotionSys_XY_Init + Grip_Init(夹爪) + USB_Init 绑定;
      与 motion_debug / usb_debug(回显抢占回调) / servo_debug / grip_debug
      (重复 Init 同一夹爪, 双指挥)互斥 */
#define host_debug  1
/* 10. grip_debug     夹爪机构(双舵机: 舵机1 Z 轴升降 + 舵机2 开合; 舵机会动!)
      每步下发后核对模块回报角度/状态 + 该通道 CCR 回读脉宽, 轮末 [PASS]/[FAIL];
      与 host_debug(双指挥夹爪)/servo_debug(重复 Init)互斥 */
#define grip_debug  0
/* 11. storage_debug  旋转储物盘(3508 直驱, 6 格 × 60°; 电机会转!)
      流程: 等 CAN 反馈 -> 逐格 0~5 转位(核对到达槽位/角度误差/最短路径)
      -> 反向逐格回绕 -> 越界/未 Init 拒绝核对 -> 轮末 [PASS]/[FAIL]。
      与 dji_motor_debug 互斥(同为 0x200 电流帧的双指挥, 会互相覆盖电流);
      与 host_debug 互斥(host 的协议里没有槽位字段, 无法指挥储物盘, 且
      主机联调时盘不应自转)。
      注意: 测试会让盘整圈转动, 先确认盘面/走线不受限! */
#define storage_debug  0
/* 12. tray_debug     旋转托盘(单舵机, TIM1_CH3/PE13; 舵机会动!)
      流程: 回 home -> 0~270° 逐点扫描(每点核对回报角度 + CH3 CCR 回读脉宽
      + 忙碌标志) -> 越界钳位/相对转动核对 -> 轮末 [PASS]/[FAIL]。
      与 servo_debug 部分重叠(都动 CH1~CH3, 本层只驱动 CH3), 与 grip_debug
      可共存(不同通道), 但都开时 USART1 打印会交错。
      注意: 托盘会整程转动, 先确认托盘上无遮挡/不受限! */
#define tray_debug  0

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
#include "drv_usb.h"
#endif

#if dji_motor_debug==1
#include "dji_3508.h"
#endif

#if servo_debug==1
#include "servo.h"
#endif

#if grip_debug==1
#include "grip.h"           /* Grip_*: 夹爪双舵机(Z 轴升降 + 开合) */
#include "servo.h"          /* SERVO_MIN_US/MAX_US/ANGLE_270_MAX: 脉宽核对算式 */
#endif

#if storage_debug==1
#include "storage.h"        /* Storage_*: 旋转储物盘(3508 直驱) */
#include "dji_3508.h"       /* DJ_Motor3508 / DJ_Motor_t: 直读反馈核对 */
#endif

#if tray_debug==1
#include "tray.h"           /* Tray_*: 旋转托盘(单舵机, 仅旋转) */
#include "servo.h"          /* SERVO_MIN_US/MAX_US/ANGLE_270_MAX: 脉宽核对算式 */
#endif

#if motion_debug==1
#include "motion_sys_XY.h"
#include "stepmotor.h"     /* 0x36 原始回包诊断打印需要 StepMotor_ZDT */
#endif

#if host_debug==1
#include "host_comm.h"
#include "drv_usb.h"        /* USB_Transmit_Data / usb0: 主机联调测试发帧 */
#include "motion_sys_XY.h"  /* 状态行打印运动态/位置(cmd/act) */
#include "stepmotor.h"      /* StepMotor_ZDT: F1(QPos 参数)应答打印 */
#endif

#if msgcenter_debug==1
#include "messagecenter.h"
#endif

void debug_init(void);
void debug_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H__ */
