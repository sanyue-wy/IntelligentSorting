/**
 * @file    app.c
 * @brief   智能分拣系统业务主控流程实现
 * @version 1.0
 * @date    2026-10-12
 */

/* Includes ------------------------------------------------------------------*/
#include "app.h"
#include "motion_sys_XY.h"
#include "grip.h"
#include "tray.h"
#include "drv_uart.h"
#include "drv_tim.h"
#include <stdio.h>

/* Private types -------------------------------------------------------------*/
typedef struct
{
    AppState_e state;           /**< 当前业务状态 */
    AppState_e last_state;      /**< 上一业务状态 */

    uint32_t   state_tick;      /**< 进入当前状态的时间戳 (HAL_GetTick) */
    uint32_t   permit_arm_tick; /**< 到位并开始等待许可的时间戳 */

    /* 当前任务目标 */
    uint32_t   current_task_id; /**< 当前处理的任务 ID */
    float      target_x_mm;     /**< 目标 X 坐标 (mm) */
    float      target_y_mm;     /**< 目标 Y 坐标 (mm) */

    /* 控制与门控标志 */
    bool       has_task;        /**< 是否有有效任务在执行 */
    bool       permit_granted;  /**< 是否已收到整机上位机许可信号 */
    bool       estop_latched;   /**< 急停锁存标志 */
    uint16_t   last_error;      /**< 最近发生的错误码 */

    uint8_t    sub_step;        /**< 状态内部子步骤序号 */

    /* 传感器检测回调钩子 */
    App_ForeignDetectHook_fn foreign_hook;
    App_GraspCheckHook_fn    grasp_hook;
} AppContext_t;

/* Private variables ---------------------------------------------------------*/
static AppContext_t s_app;

/* Private function prototypes -----------------------------------------------*/
static void App_SetState(AppState_e new_state);

/* ========================================================================== */
/*  状态机核心实现                                                             */
/* ========================================================================== */

const char *App_GetStateName(AppState_e st)
{
    switch (st)
    {
    case APP_STATE_INIT:              return "INIT";
    case APP_STATE_IDLE:              return "IDLE";
    case APP_STATE_MOVING_TO_TARGET:  return "MOVING_TO_TARGET";
    case APP_STATE_WAIT_PERMISSION:   return "WAIT_PERMISSION";
    case APP_STATE_GRIP_DOWN:         return "GRIP_DOWN";
    case APP_STATE_GRIP_CLOSE:        return "GRIP_CLOSE";
    case APP_STATE_GRIP_CHECK:        return "GRIP_CHECK";
    case APP_STATE_GRIP_UP:           return "GRIP_UP";
    case APP_STATE_MOVING_TO_PLACE:   return "MOVING_TO_PLACE";
    case APP_STATE_RELEASE:           return "RELEASE";
    case APP_STATE_RETURN_HOME:       return "RETURN_HOME";
    case APP_STATE_GRIP_FAIL:         return "GRIP_FAIL";
    case APP_STATE_ERROR:             return "ERROR";
    case APP_STATE_ESTOP:             return "ESTOP";
    default:                          return "UNKNOWN";
    }
}

AppState_e App_GetState(void)
{
    return s_app.state;
}

static void App_SetState(AppState_e new_state)
{
    if (s_app.state == new_state)
    {
        return;
    }

    s_app.last_state = s_app.state;
    s_app.state      = new_state;
    s_app.state_tick = SysTime_GetMs();
    s_app.sub_step   = 0U;

#if (APP_DEBUG_PRINT == 1)
    UART_Port_Printf(&uart1_port, "[app] State: %s -> %s\r\n",
                     App_GetStateName(s_app.last_state),
                     App_GetStateName(new_state));
#endif
}

void App_Init(void)
{
    s_app.state           = APP_STATE_INIT;
    s_app.last_state      = APP_STATE_INIT;
    s_app.state_tick      = SysTime_GetMs();
    s_app.permit_arm_tick = 0U;
    s_app.current_task_id = 0U;
    s_app.target_x_mm     = 0.0f;
    s_app.target_y_mm     = 0.0f;
    s_app.has_task        = false;
    s_app.permit_granted  = false;
    s_app.estop_latched   = false;
    s_app.last_error      = 0U;
    s_app.sub_step        = 0U;
    s_app.foreign_hook    = NULL;
    s_app.grasp_hook      = NULL;

    /* 注册上位机命令消费钩子 */
    HostComm_RegisterCmdHook(App_OnHostCommand);

#if (APP_DEBUG_PRINT == 1)
    UART_Port_Printf(&uart1_port, "\r\n[app] === Intelligent Sorting App Init ===\r\n");
#endif
}

void App_Poll(void)
{
    uint32_t now = SysTime_GetMs();

    /* 1. 急停与底层运动故障安全门禁 */
    if (s_app.estop_latched)
    {
        if (s_app.state != APP_STATE_ESTOP)
        {
            MotionSys_XY_Stop();
            App_SetState(APP_STATE_ESTOP);
        }
        return;
    }

    if (MotionSys_XY_GetState() == MS_XY_FAULT && s_app.state != APP_STATE_ERROR)
    {
#if (APP_DEBUG_PRINT == 1)
        UART_Port_Printf(&uart1_port, "[app] MotionSys FAULT detected!\r\n");
#endif
        s_app.last_error = HOST_ERR_INTERNAL;
        HostComm_ReportFail(HOST_STAGE_NONE, HOST_ERR_INTERNAL);
        App_SetState(APP_STATE_ERROR);
        return;
    }

    /* 2. 状态机分发流转 */
    switch (s_app.state)
    {
    case APP_STATE_INIT:
        /* 阶段 1: 托盘复位与异物检测自检 */
        if (s_app.sub_step == 0U)
        {
            Tray_Init();
            Tray_Home();
            Grip_Init();
            Grip_ZUp();
            Grip_Open();
            s_app.sub_step = 1U;
            break;
        }
        if (s_app.sub_step == 1U)
        {
            if (Tray_IsBusy() || Grip_IsBusy())
            {
                if (now - s_app.state_tick >= APP_TIMEOUT_TRAY_MS)
                {
#if (APP_DEBUG_PRINT == 1)
                    UART_Port_Printf(&uart1_port, "[app] [FAIL] Tray init timeout!\r\n");
#endif
                    s_app.last_error = HOST_ERR_INTERNAL;
                    HostComm_ReportFail(HOST_STAGE_NONE, HOST_ERR_INTERNAL);
                    App_SetState(APP_STATE_ERROR);
                }
                break;
            }

            /* 托盘动作完成, 开启异物检测 */
            s_app.sub_step = 2U;
        }
        if (s_app.sub_step == 2U)
        {
            if (App_DetectForeignObject())
            {
#if (APP_DEBUG_PRINT == 1)
                UART_Port_Printf(&uart1_port, "[app] [ALARM] Foreign obstacle detected on tray!\r\n");
#endif
                s_app.last_error = HOST_ERR_FOREIGN_OBJ;
                HostComm_ReportFail(HOST_STAGE_NONE, HOST_ERR_FOREIGN_OBJ);
                App_SetState(APP_STATE_ERROR);
                break;
            }

#if (APP_DEBUG_PRINT == 1)
            UART_Port_Printf(&uart1_port, "[app] Init check PASSED! Entering IDLE standby.\r\n");
#endif
            App_SetState(APP_STATE_IDLE);
        }
        break;

    case APP_STATE_IDLE:
        /* 阶段 2: 待机等待上位机下发任务 */
        if (s_app.has_task)
        {
            /* 校验目标坐标是否落在软限位之内 */
            if (s_app.target_x_mm < APP_XY_MIN_X_MM || s_app.target_x_mm > APP_XY_MAX_X_MM ||
                s_app.target_y_mm < APP_XY_MIN_Y_MM || s_app.target_y_mm > APP_XY_MAX_Y_MM)
            {
#if (APP_DEBUG_PRINT == 1)
                UART_Port_Printf(&uart1_port, "[app] Target coordinate out of soft limits: (%.2f, %.2f)\r\n",
                                 s_app.target_x_mm, s_app.target_y_mm);
#endif
                HostComm_ReportFail(HOST_STAGE_PICK, HOST_ERR_PARAM);
                s_app.has_task = false;
                break;
            }

#if (APP_DEBUG_PRINT == 1)
            UART_Port_Printf(&uart1_port, "[app] Starting task %lu -> MoveTo(%.2f, %.2f)\r\n",
                             (unsigned long)s_app.current_task_id, s_app.target_x_mm, s_app.target_y_mm);
#endif
            s_app.permit_granted = false;
            MotionSys_XY_MoveTo(s_app.target_x_mm, s_app.target_y_mm, APP_FEED_RATE_MM_S);
            App_SetState(APP_STATE_MOVING_TO_TARGET);
        }
        break;

    case APP_STATE_MOVING_TO_TARGET:
        /* 阶段 3: XY平台移动至目标取件位 */
        if (MotionSys_XY_IsBusy())
        {
            if (now - s_app.state_tick >= APP_TIMEOUT_MOTION_MS)
            {
#if (APP_DEBUG_PRINT == 1)
                UART_Port_Printf(&uart1_port, "[app] [FAIL] Move to target timeout!\r\n");
#endif
                MotionSys_XY_Stop();
                HostComm_ReportFail(HOST_STAGE_PICK, HOST_ERR_TIMEOUT);
                App_SetState(APP_STATE_ERROR);
            }
            break;
        }

        /* 移动到位, 进入等待整机上位机许可信号状态 */
#if (APP_DEBUG_PRINT == 1)
        UART_Port_Printf(&uart1_port, "[app] Arrived at target pos. Waiting for HOST PERMIT signal...\r\n");
#endif
        s_app.permit_arm_tick = now;

#if (APP_REQUIRE_PERMIT_SIGNAL == 0)
        /* 调试旁路模式: 自动视为获得许可 */
        s_app.permit_granted = true;
#endif

        if (s_app.permit_granted)
        {
            s_app.permit_granted = false;
            App_SetState(APP_STATE_GRIP_DOWN);
        }
        else
        {
            App_SetState(APP_STATE_WAIT_PERMISSION);
        }
        break;

    case APP_STATE_WAIT_PERMISSION:
        /* 阶段 4: 权限校验门禁 - 严格等待上位机许可信号 */
        if (s_app.permit_granted)
        {
            s_app.permit_granted = false;
#if (APP_DEBUG_PRINT == 1)
            UART_Port_Printf(&uart1_port, "[app] Host PERMIT confirmed! Executing grasp...\r\n");
#endif
            App_SetState(APP_STATE_GRIP_DOWN);
            break;
        }

        /* 超时保护 (非 0 时生效) */
#if (APP_TIMEOUT_PERMIT_MS > 0U)
        if (now - s_app.permit_arm_tick >= APP_TIMEOUT_PERMIT_MS)
        {
#if (APP_DEBUG_PRINT == 1)
            UART_Port_Printf(&uart1_port, "[app] Wait PERMIT signal timeout (%ums)!\r\n", APP_TIMEOUT_PERMIT_MS);
#endif
            HostComm_ReportFail(HOST_STAGE_PICK, HOST_ERR_TIMEOUT);
            App_SetState(APP_STATE_ERROR);
            break;
        }
#endif
        break;

    case APP_STATE_GRIP_DOWN:
        /* 阶段 5: 夹爪张开并下潜 */
        if (s_app.sub_step == 0U)
        {
            Grip_Open();
            Grip_ZDown();
            s_app.sub_step = 1U;
            break;
        }
        if (Grip_IsBusy())
        {
            if (now - s_app.state_tick >= APP_TIMEOUT_GRIP_MS)
            {
#if (APP_DEBUG_PRINT == 1)
                UART_Port_Printf(&uart1_port, "[app] [FAIL] Grip Z down timeout!\r\n");
#endif
                HostComm_ReportFail(HOST_STAGE_GRIP, HOST_ERR_TIMEOUT);
                App_SetState(APP_STATE_ERROR);
            }
            break;
        }
        App_SetState(APP_STATE_GRIP_CLOSE);
        break;

    case APP_STATE_GRIP_CLOSE:
        /* 阶段 6: 夹爪闭合 */
        if (s_app.sub_step == 0U)
        {
            Grip_Close(APP_GRIP_FORCE_PCT);
            s_app.sub_step = 1U;
            break;
        }
        if (Grip_IsBusy())
        {
            if (now - s_app.state_tick >= APP_TIMEOUT_GRIP_MS)
            {
#if (APP_DEBUG_PRINT == 1)
                UART_Port_Printf(&uart1_port, "[app] [FAIL] Grip close timeout!\r\n");
#endif
                HostComm_ReportFail(HOST_STAGE_GRIP, HOST_ERR_TIMEOUT);
                App_SetState(APP_STATE_ERROR);
            }
            break;
        }
        App_SetState(APP_STATE_GRIP_CHECK);
        break;

    case APP_STATE_GRIP_CHECK:
        /* 阶段 7: 抓取结果检测校验 */
        if (App_CheckGripSuccess())
        {
#if (APP_DEBUG_PRINT == 1)
            UART_Port_Printf(&uart1_port, "[app] Grasp SUCCESS! Lifting Z...\r\n");
#endif
            App_SetState(APP_STATE_GRIP_UP);
        }
        else
        {
#if (APP_DEBUG_PRINT == 1)
            UART_Port_Printf(&uart1_port, "[app] [WARN] Grasp FAILED! Retracting and notifying host...\r\n");
#endif
            /* 向上位机上报抓取失败信号 */
            HostComm_ReportFail(HOST_STAGE_GRIP, HOST_ERR_GRIPFAIL);
            s_app.has_task = false;

            /* 夹爪安全复位 (张开并抬起) */
            Grip_Open();
            Grip_ZUp();
            App_SetState(APP_STATE_GRIP_FAIL);
        }
        break;

    case APP_STATE_GRIP_FAIL:
        /* 阶段 8: 抓取失败善后: 原地待命, 等待新的坐标指令 */
        if (Grip_IsBusy())
        {
            break;
        }
        if (s_app.has_task)
        {
#if (APP_DEBUG_PRINT == 1)
            UART_Port_Printf(&uart1_port, "[app] Received new task %lu after grasp retry!\r\n",
                             (unsigned long)s_app.current_task_id);
#endif
            s_app.permit_granted = false;
            MotionSys_XY_MoveTo(s_app.target_x_mm, s_app.target_y_mm, APP_FEED_RATE_MM_S);
            App_SetState(APP_STATE_MOVING_TO_TARGET);
        }
        break;

    case APP_STATE_GRIP_UP:
        /* 阶段 9: 抓取成功, Z 轴安全抬起 */
        if (s_app.sub_step == 0U)
        {
            Grip_ZUp();
            s_app.sub_step = 1U;
            break;
        }
        if (Grip_IsBusy())
        {
            if (now - s_app.state_tick >= APP_TIMEOUT_GRIP_MS)
            {
#if (APP_DEBUG_PRINT == 1)
                UART_Port_Printf(&uart1_port, "[app] [FAIL] Grip Z up timeout!\r\n");
#endif
                HostComm_ReportFail(HOST_STAGE_GRIP, HOST_ERR_TIMEOUT);
                App_SetState(APP_STATE_ERROR);
            }
            break;
        }

        /* Z 轴已安全抬起, 携物前往放置区 */
#if (APP_DEBUG_PRINT == 1)
        UART_Port_Printf(&uart1_port, "[app] Moving to Place Zone (%.2f, %.2f)...\r\n",
                         APP_PLACE_X_MM, APP_PLACE_Y_MM);
#endif
        MotionSys_XY_MoveTo(APP_PLACE_X_MM, APP_PLACE_Y_MM, APP_FEED_RATE_MM_S);
        App_SetState(APP_STATE_MOVING_TO_PLACE);
        break;

    case APP_STATE_MOVING_TO_PLACE:
        /* 阶段 10: XY 平台移动前往放置区 */
        if (MotionSys_XY_IsBusy())
        {
            if (now - s_app.state_tick >= APP_TIMEOUT_MOTION_MS)
            {
#if (APP_DEBUG_PRINT == 1)
                UART_Port_Printf(&uart1_port, "[app] [FAIL] Move to place timeout!\r\n");
#endif
                MotionSys_XY_Stop();
                HostComm_ReportFail(HOST_STAGE_PLACEGO, HOST_ERR_TIMEOUT);
                App_SetState(APP_STATE_ERROR);
            }
            break;
        }

        /* 放置区已到位, 准备释放工件 */
        App_SetState(APP_STATE_RELEASE);
        break;

    case APP_STATE_RELEASE:
        /* 阶段 11: 放置区夹爪张开释放工件 */
        if (s_app.sub_step == 0U)
        {
            Grip_Open();
            s_app.sub_step = 1U;
            break;
        }
        if (Grip_IsBusy())
        {
            if (now - s_app.state_tick >= APP_TIMEOUT_GRIP_MS)
            {
#if (APP_DEBUG_PRINT == 1)
                UART_Port_Printf(&uart1_port, "[app] [FAIL] Grip release timeout!\r\n");
#endif
                HostComm_ReportFail(HOST_STAGE_PLACE, HOST_ERR_TIMEOUT);
                App_SetState(APP_STATE_ERROR);
            }
            break;
        }

        /* 工件已释放, 归位返回机械原点 */
#if (APP_DEBUG_PRINT == 1)
        UART_Port_Printf(&uart1_port, "[app] Released! Returning home (%.2f, %.2f)...\r\n",
                         APP_HOME_X_MM, APP_HOME_Y_MM);
#endif
        MotionSys_XY_MoveTo(APP_HOME_X_MM, APP_HOME_Y_MM, APP_FEED_RETURN_MM_S);
        App_SetState(APP_STATE_RETURN_HOME);
        break;

    case APP_STATE_RETURN_HOME:
        /* 阶段 12: 归位原点 */
        if (MotionSys_XY_IsBusy())
        {
            if (now - s_app.state_tick >= APP_TIMEOUT_MOTION_MS)
            {
#if (APP_DEBUG_PRINT == 1)
                UART_Port_Printf(&uart1_port, "[app] [FAIL] Return home timeout!\r\n");
#endif
                MotionSys_XY_Stop();
                HostComm_ReportFail(HOST_STAGE_RETURN, HOST_ERR_TIMEOUT);
                App_SetState(APP_STATE_ERROR);
            }
            break;
        }

        /* 机构已完全归位, 任务圆满完成并上报成功 */
#if (APP_DEBUG_PRINT == 1)
        UART_Port_Printf(&uart1_port, "[app] [SUCCESS] Task %lu successfully finished!\r\n\r\n",
                         (unsigned long)s_app.current_task_id);
#endif
        HostComm_ReportSuccess();
        s_app.has_task = false;
        App_SetState(APP_STATE_IDLE);
        break;

    case APP_STATE_ERROR:
        /* 阶段 13: 异常处理流程 (停止运动, 安全复位夹爪, 等待报警清除) */
        if (s_app.sub_step == 0U)
        {
            MotionSys_XY_Stop();
            Grip_ZUp();
            Grip_Open();
            s_app.sub_step = 1U;
        }
        /* 等待上位机下发 HOST_FLAG_CLEAR 清除报警后退出此状态 */
        break;

    case APP_STATE_ESTOP:
        /* 阶段 14: 急停锁存 */
        MotionSys_XY_Stop();
        break;

    default:
        App_SetState(APP_STATE_ERROR);
        break;
    }
}

/* ========================================================================== */
/*  上位机命令交互与事件处理                                                   */
/* ========================================================================== */

bool App_OnHostCommand(const HostCmd_t *cmd)
{
    if (cmd == NULL)
    {
        return false;
    }

    /* 1. 清报警 (最高优先级) */
    if (cmd->flags & HOST_FLAG_CLEAR)
    {
        App_ClearAlarm();
        return true;
    }

    /* 2. 急停 */
    if (cmd->flags & HOST_FLAG_ESTOP)
    {
        App_TriggerEStop();
        return true;
    }

    /* 急停锁存期拒绝其他命令 */
    if (s_app.estop_latched)
    {
        return true;
    }

    /* 3. 取消任务 */
    if (cmd->flags & HOST_FLAG_CANCEL)
    {
#if (APP_DEBUG_PRINT == 1)
        UART_Port_Printf(&uart1_port, "[app] Host CANCEL signal received!\r\n");
#endif
        MotionSys_XY_Stop();
        Grip_ZUp();
        Grip_Open();
        HostComm_ReportAbort(HOST_STAGE_ABORT);
        s_app.has_task = false;
        App_SetState(APP_STATE_IDLE);
        return true;
    }

    /* 4. 整机上位机许可信号 (HOST_FLAG_PERMIT) */
    if (cmd->flags & HOST_FLAG_PERMIT)
    {
        s_app.permit_granted = true;
#if (APP_DEBUG_PRINT == 1)
        UART_Port_Printf(&uart1_port, "[app] Host PERMIT signal received for task %lu!\r\n",
                         (unsigned long)cmd->task_id);
#endif
        return true;
    }

    /* 5. 运动任务触发 (下发 X, Y 坐标) */
    if (cmd->flags & HOST_FLAG_MOTION)
    {
        s_app.target_x_mm     = (float)cmd->x_um * 0.001f;
        s_app.target_y_mm     = (float)cmd->y_um * 0.001f;
        s_app.current_task_id = cmd->task_id;
        s_app.has_task        = true;
        s_app.permit_granted  = false;

#if (APP_DEBUG_PRINT == 1)
        UART_Port_Printf(&uart1_port, "[app] Host task %lu received: (X=%.2f mm, Y=%.2f mm)\r\n",
                         (unsigned long)s_app.current_task_id, s_app.target_x_mm, s_app.target_y_mm);
#endif
        return true;
    }

    return false;
}

void App_TriggerEStop(void)
{
    s_app.estop_latched = true;
    MotionSys_XY_Stop();
#if (APP_DEBUG_PRINT == 1)
    UART_Port_Printf(&uart1_port, "[app] [ESTOP] Emergency stop latched!\r\n");
#endif
    App_SetState(APP_STATE_ESTOP);
}

void App_ClearAlarm(void)
{
    s_app.estop_latched   = false;
    s_app.has_task        = false;
    s_app.permit_granted  = false;
    s_app.last_error      = 0U;

    MotionSys_XY_Stop();
    Grip_ZUp();
    Grip_Open();

#if (APP_DEBUG_PRINT == 1)
    UART_Port_Printf(&uart1_port, "[app] Alarm cleared, returning to IDLE.\r\n");
#endif
    App_SetState(APP_STATE_IDLE);
}

/* ========================================================================== */
/*  传感器与检测钩子实现                                                       */
/* ========================================================================== */

void App_RegisterForeignDetectHook(App_ForeignDetectHook_fn hook)
{
    s_app.foreign_hook = hook;
}

void App_RegisterGraspCheckHook(App_GraspCheckHook_fn hook)
{
    s_app.grasp_hook = hook;
}

bool App_DetectForeignObject(void)
{
#if (APP_SIMULATE_FOREIGN_OBJ == 1)
    return true; /* 强制模拟检出异物报警 */
#endif

    if (s_app.foreign_hook != NULL)
    {
        return s_app.foreign_hook();
    }

    /* 硬件默认无异物阻碍 */
    return false;
}

bool App_CheckGripSuccess(void)
{
#if (APP_SIMULATE_GRIP_FAIL == 1)
    return false; /* 强制模拟抓取失败分支 */
#endif

    if (s_app.grasp_hook != NULL)
    {
        return s_app.grasp_hook();
    }

    /* 硬件未接力敏/光电微动时, 闭环动作到位即判定抓取成功 */
    return true;
}
