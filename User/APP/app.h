/**
 * @file    app.h
 * @brief   智能分拣系统业务主控流程 (应用层顶层状态机)
 * @details 负责协调托盘自检、异物检测、XY轴运动、整机上位机权限校验、
 *          夹爪抓取/放置、异常善后与结果上报的全流程闭环控制。
 * @version 1.0
 * @date    2026-10-12
 */

#ifndef __APP_H__
#define __APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "host_comm.h"

/* ========================================================================== */
/*  【调试与异常模拟开关】便于单机无上位机/无传感器环境下的分步台架调试         */
/* ========================================================================== */

/**
 * @brief 串口调试日志打印开关 (USART1, 115200)
 *        1: 开启状态机详细流转日志(打印每次状态跃迁、耗时与坐标)
 *        0: 关闭详细日志, 仅保留异常警告
 */
#define APP_DEBUG_PRINT             1

/**
 * @brief 整机上位机许可信号校验开关 (权限门控)
 *        1: 严格校验模式: XY 轴到达目标点后, 必须等待上位机下发 HOST_FLAG_PERMIT 许可信号才执行抓取
 *        0: 调试旁路模式: XY 轴到达目标点后自动视为获得许可, 立即执行抓取 (方便脱机自测)
 */
#define APP_REQUIRE_PERMIT_SIGNAL   1

/**
 * @brief 模拟抓取失败注入开关
 *        1: 强制模拟抓取失败: 夹爪闭合后判定未抓到工件, 测试"上报失败信号 -> 原地待命等待新坐标"流程
 *        0: 正常检测模式: 默认抓取成功 (或读取物理传感器)
 */
#define APP_SIMULATE_GRIP_FAIL      0

/**
 * @brief 模拟异物检测报警注入开关
 *        1: 强制模拟检出异物: 开机自检时判定托盘有异物, 测试"触发异常流程 -> 报警等待复位"
 *        0: 正常检测模式: 默认自检通过无异物
 */
#define APP_SIMULATE_FOREIGN_OBJ    0

/* ========================================================================== */
/*  【空间坐标配置】单位: mm (基于 CoreXY 笛卡尔坐标系)                         */
/* ========================================================================== */

/**
 * @brief 机械待机 / 归位原点坐标 (Home)
 */
#define APP_HOME_X_MM               0.0f
#define APP_HOME_Y_MM               0.0f

/**
 * @brief 放置区目标坐标 (Place Zone): 物块抓取后搬运到的释放位置
 */
#define APP_PLACE_X_MM              0.0f
#define APP_PLACE_Y_MM              40.0f

/**
 * @brief 软件安全限位范围 (Soft Limits): 防止误发超程坐标导致机械撞击
 */
#define APP_XY_MIN_X_MM             0.0f
#define APP_XY_MAX_X_MM             100.0f
#define APP_XY_MIN_Y_MM             0.0f
#define APP_XY_MAX_Y_MM             100.0f

/* ========================================================================== */
/*  【运动与执行器参数配置】进给速度、夹持力度与角度                            */
/* ========================================================================== */

/**
 * @brief 前往取件点与放置区的插补进给速度 (mm/s), 上限受 MS_XY_MAX_VEL 保护
 */
#define APP_FEED_RATE_MM_S          100.0f

/**
 * @brief 搬运完成归位返回原点时的进给速度 (mm/s)
 */
#define APP_FEED_RETURN_MM_S        120.0f

/**
 * @brief 夹爪闭合夹取力度百分比: 0~100 (底层映射至 PWM 脉宽, 100% 对应全行程闭合)
 */
#define APP_GRIP_FORCE_PCT          100U

/**
 * @brief 托盘开机复位目标角度 (度): 默认 0°
 */
#define APP_TRAY_HOME_DEG           0U

/**
 * @brief 托盘开机自检旋转测试角度 (度): 旋转到此角度以确认舵机响应正常, 0 表示仅复位不转动
 */
#define APP_TRAY_CHECK_DEG          0U

/* ========================================================================== */
/*  【超时保护与时钟节拍配置】单位: ms, 动作超过此时限自动触发异常善后           */
/* ========================================================================== */

/**
 * @brief 托盘初始化与运动动作超时时限 (ms)
 */
#define APP_TIMEOUT_TRAY_MS         3000U

/**
 * @brief 等待整机上位机许可信号的最大时限 (ms)
 *        设为 0 表示无限等待 (直到上位机下发许可或下发取消/急停指令)
 */
#define APP_TIMEOUT_PERMIT_MS       15000U

/**
 * @brief XY 平台运动超时时限 (ms): 超过此时间仍未到达目标位置则判定运动异常
 */
#define APP_TIMEOUT_MOTION_MS       10000U

/**
 * @brief 夹爪下潜/抬起/开合单步动作超时时限 (ms)
 */
#define APP_TIMEOUT_GRIP_MS         2000U

/**
 * @brief 主状态机轮询节拍 (ms): 建议 5ms, 兼顾任务响应性与 FreeRTOS 任务调度开销
 */
#define APP_POLL_INTERVAL_MS        5U

/* ========================================================================== */
/*  状态机与数据结构定义                                                       */
/* ========================================================================== */

/**
 * @brief 智能分拣业务状态枚举
 */
typedef enum
{
    APP_STATE_INIT = 0,         /**< 0: 上电初始化 (托盘复位 + 异物检测自检) */
    APP_STATE_IDLE,             /**< 1: 待机就绪 (等待上位机下发 X,Y 坐标任务) */
    APP_STATE_MOVING_TO_TARGET, /**< 2: XY平台平稳移动前往目标坐标位置 */
    APP_STATE_WAIT_PERMISSION,  /**< 3: 已到位, 保持静止等待整机上位机抓取许可信号 */
    APP_STATE_GRIP_DOWN,        /**< 4: 获得许可, 夹爪下潜 */
    APP_STATE_GRIP_CLOSE,       /**< 5: 夹爪闭合抓取工件 */
    APP_STATE_GRIP_CHECK,       /**< 6: 抓取结果检测校验 (成功/失败分支) */
    APP_STATE_GRIP_UP,          /**< 7: 抓取成功, Z 轴安全抬起 */
    APP_STATE_MOVING_TO_PLACE,  /**< 8: XY 平台平稳移动前往放置区 */
    APP_STATE_RELEASE,          /**< 9: 放置区夹爪张开释放工件 */
    APP_STATE_RETURN_HOME,      /**< 10: XY 平台平稳归位返回原点 (0,0) */
    APP_STATE_GRIP_FAIL,        /**< 11: 抓取失败善后: Z抬起, 向上位机发失败信号, 待命等新指令 */
    APP_STATE_ERROR,            /**< 12: 异常故障处理 (停止机构, 报错报警, 等待清除) */
    APP_STATE_ESTOP             /**< 13: 急停锁存态 (机构全停, 坚决不执行新动作) */
} AppState_e;

/**
 * @brief 异物检测回调函数指针类型 (返回 true 表示检测到有异物阻挡, false 为正常无异物)
 */
typedef bool (*App_ForeignDetectHook_fn)(void);

/**
 * @brief 抓取检测回调函数指针类型 (返回 true 表示成功抓取工件, false 为抓空/掉落)
 */
typedef bool (*App_GraspCheckHook_fn)(void);

/* ========================================================================== */
/*  对外接口                                                                   */
/* ========================================================================== */

/**
 * @brief  业务层初始化: 绑定通信命令钩子, 启动开机自检流程
 */
void App_Init(void);

/**
 * @brief  业务层主循环轮询 (在 FreeRTOS 主任务或主循环中周期调用, 如每 5ms 一次)
 */
void App_Poll(void);

/**
 * @brief  获取当前业务主状态
 * @return 当前 AppState_e 状态枚举
 */
AppState_e App_GetState(void);

/**
 * @brief  获取当前状态的字符串描述 (便于日志打印)
 * @param  st: 状态枚举
 * @return 状态可读字符串
 */
const char *App_GetStateName(AppState_e st);

/**
 * @brief  业务层命令接收钩子 (由 host_comm 收到命令帧时回调调用)
 * @param  cmd: 上位机下行命令数据指针
 * @return true 表示业务层已接管消费该命令; false 表示由底层缺省分发
 */
bool App_OnHostCommand(const HostCmd_t *cmd);

/**
 * @brief  注册自定义异物检测函数 (若不注册则按宏开关/开环默认)
 */
void App_RegisterForeignDetectHook(App_ForeignDetectHook_fn hook);

/**
 * @brief  注册自定义抓取检测函数 (若不注册则按宏开关/开环默认)
 */
void App_RegisterGraspCheckHook(App_GraspCheckHook_fn hook);

/**
 * @brief  执行异物检测
 * @return true: 检测到异物报警; false: 正常无异物
 */
bool App_DetectForeignObject(void);

/**
 * @brief  执行抓取结果校验
 * @return true: 抓取成功; false: 抓取失败
 */
bool App_CheckGripSuccess(void);

/**
 * @brief  手动触发急停 (用于外部按键或安全互锁输入)
 */
void App_TriggerEStop(void);

/**
 * @brief  清除报警与急停 (复位回到 IDLE 待机)
 */
void App_ClearAlarm(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H__ */
