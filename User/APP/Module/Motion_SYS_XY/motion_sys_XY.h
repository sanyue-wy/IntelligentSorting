/**
 * @file    motion_sys_XY.h
 * @brief   CoreXY XY 轴运动系统(仿 Voron 2.4 / Klipper 运动控制)
 *
 *          仿照对象与分工:
 *            - 变换层  仿 Klipper klippy/kinematics/corexy.py:
 *                      A = X + Y, B = X - Y (单位 mm)
 *            - 规划层  仿 Klipper klippy/toolhead.py:
 *                      Move.calc_junction  -> MS_XY_CalcJunction (junction deviation 过弯)
 *                      LookAheadQueue.flush-> MS_XY_Replan       (双向扫描 lookahead)
 *                      Move.set_junction   -> MS_XY_SetJunction  (梯形速度曲线分解)
 *            - 执行层  本工程自有: TIM7 1ms 节拍按时间求解参考轨迹,
 *                      每拍把 A/B 绝对脉冲目标以 Emm_V5 QPos(0xFC, 单帧)
 *                      流式下发给两台 ZDT 闭环步进(对应 Klipper MCU 按时刻发脉冲)
 *
 *          运动包络参数取自 Voron 2.4 官方配置
 *          (VoronDesign/Voron-2 @Voron2.4
 *           firmware/klipper_configurations/Octopus/Voron2_Octopus_Config.cfg):
 *            max_velocity 300mm/s, max_accel 3000mm/s², square_corner_velocity 5mm/s,
 *            rotation_distance 40mm (20T GT2 轮)
 *
 *          使用约定:
 *            - 所有接口在主循环调用; MotionSys_XY_Tick 由 TIM7 中断自动调用
 *            - 上电后先 MotionSys_XY_Init(), 首次 MoveTo 自动使能两电机
 *            - 首次运动前用 MotionSys_XY_SetPosition() 建立坐标原点
 *            - 停止后实际位置可能滞后参考位置, 重新定位前建议再调 SetPosition 同步
 *            - 不要与 stepmotor_debug 同时打开(会双主控同两台电机)
 */

#ifndef __MOTION_SYS_XY_H__
#define __MOTION_SYS_XY_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/*  运动包络 (Voron 2.4 Klipper 配置值)                                        */
/* ========================================================================== */

#define MS_XY_MAX_VEL     300.0f  /* mm/s, cfg: [printer] max_velocity        */
#define MS_XY_MAX_ACCEL   3000.0f /* mm/s², cfg: max_accel (官方注释上限4000)  */
#define MS_XY_SCV         5.0f    /* mm/s, cfg: square_corner_velocity
                                     (直角弯道允许通过的速度, 无经典 jerk 项)   */

/* ========================================================================== */
/*  机械参数 (GT2 20齿同步轮直驱, 无减速比)                                     */
/* ========================================================================== */

#define MS_XY_ROTATION_MM  40.0f  /* 皮带电机转一圈前进距离: 20齿 × 2mm 节距    */
#define MS_XY_FULL_STEPS   200U   /* 1.8° 整步电机, cfg: full_steps_per_rotation */
#define MS_XY_MICROSTEP    16U    /* ZDT 驱动器细分, 必须与驱动器实际设置一致
                                     (Voron 原厂用 32 细分 -> 160 脉冲/mm;
                                      本工程驱动默认 16 细分 -> 80 脉冲/mm)     */
/* 脉冲/mm = 整步 × 细分 / 皮带周长 = 200×16/40 = 80 */
#define MS_XY_PPS          ((float)(MS_XY_FULL_STEPS * MS_XY_MICROSTEP) / MS_XY_ROTATION_MM)

/* ========================================================================== */
/*  电机映射 (CoreXY: A = X+Y 电机, B = X-Y 电机)                               */
/* ========================================================================== */

#define MS_XY_MOTOR_A_ADDR  1U    /* A 电机(Emm_V5 地址)                      */
#define MS_XY_MOTOR_B_ADDR  2U    /* B 电机                                    */
#define MS_XY_MOTOR_A_SIGN  (+1)  /* 皮带/接线反向时装反, 改为 (-1)            */
#define MS_XY_MOTOR_B_SIGN  (+1)

/* ========================================================================== */
/*  执行器参数                                                                 */
/* ========================================================================== */

#define MS_XY_TICK_MS       1U    /* 轨迹求解节拍(ms), 订阅到 TIM7 1ms 节拍    */
#define MS_XY_QPOS_VEL_RPM  800U  /* QPos 速度上限(RPM), 需 > √2×300mm/s 折算
                                     ≈637RPM(对角线合成速度), 留裕量            */
#define MS_XY_QPOS_ACC      0U    /* QPos 加速度档 0=直接启动; 参考轨迹本身已
                                     限加速度, 令驱动器直接跟随; 若振动改 10~50 */
#define MS_XY_QUEUE_LEN     16U   /* 规划队列深度(不含正在执行段)               */
#define MS_XY_FAULT_NO_RX   100U  /* 通信看门狗阈值(StepMotor_Run 轮询计数,
                                     100×3ms=300ms 无任何电机应答 -> FAULT)     */
#define MS_XY_LA_WINDOW_MS  10U   /* lookahead 攒批窗口: 距最后一次入队静默
                                     该时长后, 队首段才开始执行(给后到的段留
                                     出并入弯道速度的机会, 对应 Klipper 的
                                     0.15s lookahead flush 思想)                */

/* 软限位: 默认关闭(先做运动验证; 上机前建议打开, 越界按 Klipper 方式报错
   而不是静默裁剪) */
#define MS_XY_LIMITS_ENABLE 0
#define MS_XY_X_MIN   0.0f
#define MS_XY_X_MAX   300.0f
#define MS_XY_Y_MIN   0.0f
#define MS_XY_Y_MAX   300.0f

/* Init 时是否把两电机当前编码器位置清零并定为 (0,0) 原点。
   1: 上电坐标系自洽(电机不动, 只重定义计数), 开箱即用
   0: 保留驱动器计数, 由调用方在建立原点后调 MotionSys_XY_SetPosition() */
#define MS_XY_ZERO_ON_INIT  1

/* ========================================================================== */
/*  类型定义                                                                   */
/* ========================================================================== */

/* 接口返回码 */
typedef enum
{
  MS_XY_OK = 0,        /* 成功 */
  MS_XY_ERR_NOT_INIT,  /* 未调用 MotionSys_XY_Init */
  MS_XY_ERR_PARAM,     /* 参数非法(NaN/feed<=0) */
  MS_XY_ERR_QUEUE_FULL,/* 规划队列满, 等 IsBusy 变 false 再发 */
  MS_XY_ERR_LIMIT,     /* 目标越出软限位(仅 MS_XY_LIMITS_ENABLE=1 时) */
  MS_XY_ERR_BUSY,      /* 运动中(或故障未清除)不允许的操作 */
} MotionSys_XY_Result_e;

/* 运动状态 */
typedef enum
{
  MS_XY_IDLE = 0,      /* 空闲(含正常完成) */
  MS_XY_RUNNING,       /* 队列有段或正在执行 */
  MS_XY_FAULT,         /* 通信故障: 已下发过指令但长时间无电机应答,
                          自动停止流式下发; 调 MotionSys_XY_Stop() 清除 */
} MotionSys_XY_State_e;

/* ========================================================================== */
/*  接口                                                                       */
/* ========================================================================== */

/**
  * @brief  初始化 CoreXY 运动系统
  * @note   必须在 debug_init 中 Driver_CAN1_Init 与 Task_IT_Init 之后调用;
  *         完成: 注册 A/B 两台电机 -> 配置 QPos 参数 -> (可选)清零建原点
  *         -> 订阅 StepMotor_Run(3ms 反馈轮询) 与 MotionSys_XY_Tick(1ms 轨迹节拍)
  */
void MotionSys_XY_Init(void);

/**
  * @brief  两电机使能/失能
  * @note   首次 MoveTo 会自动使能; 也可手动调用
  */
void MotionSys_XY_Enable(bool en);

/**
  * @brief  绝对直线移动到 (x, y)
  * @param  x,y   目标坐标, 单位 mm (CoreXY 笛卡尔空间, 非 A/B 电机空间)
  * @param  feed  进给速度 mm/s (Klipper 内部单位; G-code 的 F 是 mm/min, 注意换算),
  *               超过 MS_XY_MAX_VEL 自动钳位
  * @retval MS_XY_OK 入队成功(异步执行); 队列满/越界/参数错见返回码
  * @note   多次连续调用自动做 lookahead 弯道平滑, 段间不停车
  */
MotionSys_XY_Result_e MotionSys_XY_MoveTo(float x, float y, float feed);

/**
  * @brief  相对移动 (基于队列末段终点, 支持连续追加构成折线)
  */
MotionSys_XY_Result_e MotionSys_XY_MoveRel(float dx, float dy, float feed);

/**
  * @brief  立即停止: 清空规划队列, 双电机 Stop_Now
  * @note   停止后参考位置与实际位置可能不一致, 重新运动前建议 SetPosition 重同步;
  *         同时清除 FAULT 状态
  */
void MotionSys_XY_Stop(void);

/**
  * @brief  1ms 轨迹执行节拍 —— 由 TIM7 订阅自动调用, 用户不要直接调用
  */
void MotionSys_XY_Tick(void);

/* 是否正在运动(RUNNING) */
bool MotionSys_XY_IsBusy(void);

/* 查询状态: IDLE / RUNNING / FAULT */
MotionSys_XY_State_e MotionSys_XY_GetState(void);

/**
  * @brief  读参考(规划)坐标, 单位 mm —— 即当前流式下发的目标位置
  */
void MotionSys_XY_GetCmdPos(float *x, float *y);

/**
  * @brief  读实际坐标(由 ZDT S_CPOS 反馈换算, 单位 mm)
  * @note   依赖 Init 里订阅的 StepMotor_Run 3ms 轮询; 无反馈时返回过期值
  */
void MotionSys_XY_GetActualPos(float *x, float *y);

/**
  * @brief  建立/重同步坐标系: 把两电机当前位置定义为 (x, y)
  * @note   仅在停止状态调用; 内部将两电机编码器清零并重算脉冲偏置,
  *         同时清除 FAULT
  * @retval MS_XY_OK / MS_XY_ERR_BUSY(运动中)
  */
MotionSys_XY_Result_e MotionSys_XY_SetPosition(float x, float y);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_SYS_XY_H__ */
