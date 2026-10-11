/**
 * @file    storage.h
 * @brief   旋转储物盘定位模块(DJI 3508 减速电机直驱, 6 格 × 60°)
 *
 *          机构: 3508 经减速比 19 驱动储物盘, 盘面 6 个储料格, 每格 60°。
 *          使用时序: 识别到正确物块 -> Storage_RotateToSlot(slot) -> 等
 *          Storage_IsSlotArrived() 为真 -> 放置物块。
 *
 *          分层:
 *            机构语义(槽位) -> 角度 -> DJ_Motor 驱动 -> CAN
 *            槽位 <-> 角度换算、最短路径决策、到位判定都在本模块;
 *            角度 -> 电流由 dji_3508 的串级 PID 完成, 本模块不碰 CAN 与 HAL。
 *
 *          角度口径(本模块核心约定):
 *            - 以 3508 **上电首帧编码器位置**为 0°, 即 DJ_Motor_t.total_angle
 *              的零点(由驱动首帧反馈自动捕获, 无需外部触发);
 *            - 逆时针为正还是为负, 由 STORAGE_DIR_SIGN 决定(按机械安装改);
 *            - 映射: angle = STORAGE_ZERO_OFFSET_DEG + dir_sign × slot × 60°
 *              即 slot 0 ~ 5 对应 0° ~ 300°, 每 60° 一格。
 *
 *          注意: 上电原点 = 上电瞬间盘的实际姿态, 掉电后不保留。若需要跨上电
 *          的绝对槽位对应关系, 必须外加机械基准(限位开关/磁编码器回零) —— 本
 *          模块提供 Storage_SetReference()/Storage_GetReference() 供回零逻辑
 *          把"物理基准"重定义为 0° 并同步槽位。
 *
 *          使用约定:
 *            - 必须在 Driver_CAN1_Init() 与 DJ_Init(STORAGE_MOTOR, ...) 之后
 *              调用 Storage_Init();
 *            - DJ_MotorRun() 需按 DJ_CTRL_PERIOD(3ms) 周期调用(见 task_it.c),
 *              本模块自身不订阅节拍, 只提供查询与判定;
 *            - 旋转为异步: RotateToSlot 立即返回, 用 IsBusy/IsSlotArrived 跟进;
 *            - 未收到 CAN 反馈时(no_rx_count 超时)判到位永远为假, 会如实
 *              报"未到达", 不会误判成功(见 Storage_HasFeedback)。
 */

#ifndef __STORAGE_H__
#define __STORAGE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "dji_3508.h"

/* ========================================================================== */
/*  机械参数                                                                   */
/* ========================================================================== */

/* 盘面格子数: 每格 STORAGE_SLOT_ANGLE_DEG 度, 合 360° */
#define STORAGE_SLOT_NUM         6U

/* 每格角度(度) */
#define STORAGE_SLOT_ANGLE_DEG   60.0f

/* 电机在本模块使用的实例: DJ_Motor3508[] 的索引, 也是 CAN 电机编号(1 起) */
#define STORAGE_MOTOR_IDX        0U              /* DJ_Motor3508[0] */
#define STORAGE_MOTOR            (&DJ_Motor3508[STORAGE_MOTOR_IDX])
#define STORAGE_MOTOR_CAN_ID     1U              /* 反馈帧 0x200 + 1 */

/* 旋转方向: 槽位增大时电机角度增大为正, 反向装则改 (-1.0f) */
#define STORAGE_DIR_SIGN         (+1.0f)

/* 槽位 0 对应的电机角度(度)。上电位置为 0° 时保持 0;
   若机械上"槽位 0"与上电姿态差一个固定角, 在这里补上 */
#define STORAGE_ZERO_OFFSET_DEG  0.0f

/* ========================================================================== */
/*  运动参数                                                                   */
/* ========================================================================== */

/* 旋转速度上限 = 角度环输出(PID_Angle.max_out, 即速度环给定 r/min)上限。
   3508 减速比 19: 盘输出轴转速 = 该值 / 19 (r/min)。取 9000 时盘约 8.9rpm,
   一格 60° 约 1.1s; 台架振动作响就往下降 */
#define STORAGE_SPEED_RPM        9000.0f

/* 到位判定: |角度误差| <= 该值(度)视为到位 */
#define STORAGE_ARRIVE_DEG       6.0f

/* 到位判定: 速度低于该值(r/min, 减速前)且误差在容差内(收尾防过冲) */
#define STORAGE_ARRIVE_SPEED     400.0f

/* 到位连续保持该时长(ms)才认账, 过滤过冲/振荡瞬间穿过容差 */
#define STORAGE_ARRIVE_HOLD_MS   60U

/* 旋转超时(ms): 超出则置 STORAGE_FAULT_TIMEOUT, 停机(电流归零),
   并允许上位机重新下发。按最坏情况(相邻槽 60° + 起停)留足裕量 */
#define STORAGE_MOVE_TIMEOUT_MS  8000U

/* 反馈看门狗(拍, 与 DJ_MotorRun 调用周期同口径): 连续未收到 CAN 反馈
   超过该值 -> STORAGE_FAULT_NO_RX。100 拍 ≈ 300ms @3ms */
#define STORAGE_RX_TIMEOUT_POLLS 100U

/* ========================================================================== */
/*  多圈保护                                                                   */
/* ========================================================================== */

/* 1: 启用绕圈累加(每次旋转取最短路径, 平均转动更少);
   0: 关闭累加, 每一格只走一个 0~360° 内的点, 盘面永不绕超过一圈。
   注: 走线/气管有约束的机构建议置 0 或改用固定单向旋转 —— 最短路径会
   双向来回, 对线束不友好 */
#define STORAGE_MULTITURN_ENABLE 1

/* ========================================================================== */
/*  类型定义                                                                   */
/* ========================================================================== */

/* 接口返回码 */
typedef enum
{
  STORAGE_OK = 0,         /* 成功(已下发目标, 异步执行) */
  STORAGE_ERR_NOT_INIT,   /* 未调用 Storage_Init */
  STORAGE_ERR_PARAM,      /* 槽位越界 */
  STORAGE_ERR_BUSY,       /* 上一次旋转未结束(含故障未清除) */
  STORAGE_ERR_NO_FEEDBACK,/* 无 CAN 反馈, 拒绝下发(否则角度口径无意义) */
  STORAGE_ERR_FAULT,      /* 旋转超时/反馈丢失, 需 Storage_ClearFault */
} Storage_Result_e;

/* 旋转状态 */
typedef enum
{
  STORAGE_IDLE = 0,       /* 空闲(含正常到位) */
  STORAGE_MOVING,         /* 旋转中 */
  STORAGE_FAULT,          /* 超时 / 反馈丢失, 已停机, 待 ClearFault */
} Storage_State_e;

/* ========================================================================== */
/*  接口                                                                       */
/* ========================================================================== */

/**
  * @brief  初始化储物盘模块(幂等)
  * @param  motor  已由 DJ_Init() 初始化过的 3508 实例(通常传 STORAGE_MOTOR)
  * @note   必须在 Driver_CAN1_Init() 与 DJ_Init() 之后调用。
  *         本函数不清零编码器零点 —— 零点由 dji_3508 在**首帧反馈**时自动
  *         捕获(见 DJ_CAN_Callback), 因此 Init 早于首帧到达也能正确工作。
  *         上电姿态固定为"槽位 0", 即认为盘停在上电位置对应的格子。
  */
void Storage_Init(DJ_Motor_t *motor);

/**
  * @brief  旋转到指定槽位(绝对定位, 异步)
  * @param  slot 0 ~ STORAGE_SLOT_NUM-1, 越界返回 STORAGE_ERR_PARAM
  * @retval STORAGE_OK 已下发目标; 其余见 Storage_Result_e
  * @note   路径按 STORAGE_MULTITURN_ENABLE 决策:
  *           =1 取最短路径(可在相邻圈 ±360° 内选点, 转动量最小)
  *           =0 只走 0~360° 内的等同角(可能转过 ~300°)
  *         已在该槽位(误差在容差内)则不动, 直接置到位。
  *         旋转未结束就再次调用返回 STORAGE_ERR_BUSY(不打断上一动作)。
  */
Storage_Result_e Storage_RotateToSlot(uint8_t slot);

/**
  * @brief  相对旋转: 在当前位置基础上转动 delta_slot 格(可正可负)
  * @param  delta_slot 有符号格数, 如 +1 = 顺时针(next), -2 = 反向两格
  * @retval STORAGE_OK / STORAGE_ERR_NOT_INIT / STORAGE_ERR_BUSY /
  *         STORAGE_ERR_NO_FEEDBACK / STORAGE_ERR_FAULT
  */
Storage_Result_e Storage_RotateBySlot(int8_t delta_slot);

/**
  * @brief  按物理基准重定义坐标系: 把电机**当前**位置定义为槽位 slot
  * @param  slot 当前实际停在的槽位
  * @note   用于外部回零(限位开关/光电/人工摆正)后重建原点: 内部调
  *         DJ_ClearAngle 清零 total_angle(零点=当前编码器值), 并把当前
  *         槽位记为 slot。仅空闲时可用, 运动中返回 STORAGE_ERR_BUSY。
  */
Storage_Result_e Storage_SetReference(uint8_t slot);

/**
  * @brief  清除故障(超时/反馈丢失), 电机电流归零并回到空闲
  */
void Storage_ClearFault(void);

/* ---- 状态查询 ---- */

/* 是否正在旋转 */
bool Storage_IsBusy(void);

/* 设备是否进入 FAULT */
bool Storage_IsFault(void);

/* 是否已收到过 CAN 反馈(未收到时角度口径无意义) */
bool Storage_HasFeedback(void);

/**
  * @brief  当前槽位是否已到位(旋转完成并稳定在容差内)
  * @note   无反馈 / FAULT / 未 Init 一律返回 false
  */
bool Storage_IsSlotArrived(void);

/* 查询状态: IDLE / MOVING / FAULT */
Storage_State_e Storage_GetState(void);

/* 当前(最近一次下发的目标)槽位, 未 Init 返回 0xFF */
uint8_t Storage_GetSlot(void);

/* 目标槽位对应的电机角度(度) */
float Storage_GetTargetAngle(void);

/* 实际电机角度(度, 来自 CAN 反馈, 已除减速比), 无反馈返回 0 */
float Storage_GetActualAngle(void);

/* 角度误差(度) = 实际 - 目标 */
float Storage_GetAngleError(void);

/* ========================================================================== */
/*  纯换算(不依赖 Init, 便于上层/测试核对)                                     */
/* ========================================================================== */

/**
  * @brief  槽位 -> 电机角度(度), 不做绕圈选择
  * @note   angle = ZERO_OFFSET + DIR_SIGN × slot × 60
  */
float Storage_SlotToAngle(uint8_t slot);

/**
  * @brief  电机角度 -> 最近的槽位(用于上位机核对/回读)
  */
uint8_t Storage_AngleToSlot(float angle);

#ifdef __cplusplus
}
#endif

#endif /* __STORAGE_H__ */