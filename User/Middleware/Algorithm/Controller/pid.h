/**
 * @file    pid.h
 * @brief   统一 PID 控制器(接口层)
 *          参考 USTC-RoboWalker alg_pid 与 Wang Hongxi / Zhang Hongyu 的
 *          controller.h 改写, 去除 arm_math / bsp_dwt / user_lib / OLS 依赖
 * @version 1.0
 * @date    2026-09-25
 */

#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

/* Exported macros ------------------------------------------------------------*/

/* 绝对值(避免引入 libm) */
#define FLOAT_ABS(x) (((x) >= 0.0f) ? (x) : -(x))

/* 限幅宏(参考实现中名为 LIHT_MIN_MAX, 此处修正拼写) */
#define LIMIT_MIN_MAX(x, min, max) \
    ((x) = ((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  PID 改进项开关(位或), 取值与参考实现 controller.h 一致
  * @note   参考实现的 ErrorHandle(0x80, 堵转检测)依赖具体电机策略, 暂不实现;
  *         原枚举名 NONE/Integral_Limit 等未加前缀, 在多模块工程里易命名冲突,
  *         这里统一加 PID_IMPROVE_ 前缀, 位值不变
  */
typedef enum
{
    PID_IMPROVE_NONE                        = 0x00,
    PID_IMPROVE_INTEGRAL_LIMIT              = 0x01, /* 积分限幅(需 IntegralLimit > 0) */
    PID_IMPROVE_DERIVATIVE_ON_MEASUREMENT   = 0x02, /* 微分先行(等价 alg_pid 的 D_First) */
    PID_IMPROVE_TRAPEZOID_INTEGRAL          = 0x04, /* 梯形积分 */
    PID_IMPROVE_PROPORTIONAL_ON_MEASUREMENT = 0x08, /* 设定值阶跃时 P 不跳变 */
    PID_IMPROVE_OUTPUT_FILTER               = 0x10, /* 输出一阶低通(需 Output_LPF_RC > 0) */
    PID_IMPROVE_CHANGING_INTEGRATION_RATE   = 0x20, /* 变速积分(需 A/B 且 B > 0) */
    PID_IMPROVE_DERIVATIVE_FILTER           = 0x40, /* 微分一阶低通(需 Derivative_LPF_RC > 0) */
} PID_Improve_e;

/**
  * @brief  PID 控制器对象: 数据 + PID_Calculate 方法(行为)
  * @note   结构体公开, 增益/阈值可直接赋值在线整定; 字段不加 __packed,
  *         保持自然对齐(Cortex-M4 上打包浮点只会有性能与对齐风险)
  */
typedef struct
{
    /* 当拍输入与历史 */
    float Ref;         /* 本拍目标值 */
    float Measure;     /* 本拍测量值 */
    float Err;         /* 本拍误差(死区内已置零) */
    float Last_Ref;    /* 上拍目标值 */
    float Last_Measure;/* 上拍测量值 */
    float Last_Err;    /* 上拍误差 */

    /* 增益 */
    float Kp;
    float Ki;
    float Kd;
    float Kf;          /* 设定值步进前馈(仿 alg_pid: f = Kf * (Ref - Last_Ref)) */

    /* 限幅与时间片 */
    float MaxOut;          /* 输出限幅, 0 = 不限 */
    float IntegralLimit;   /* 积分限幅, 需配合 PID_IMPROVE_INTEGRAL_LIMIT, 0 = 不限 */
    float ControlPeriod;   /* 调用周期(秒), PID_Init 保证 > 0 */

    /* 死区: |误差| <= DeadBand 时按零误差处理且目标值拉到测量值, 0 = 关闭 */
    float DeadBand;

    /* 变速积分(仿 alg_pid): |err| <= A 全速; A ~ A+B 线性衰减; >= A+B 停止积分 */
    float I_Variable_Speed_A;
    float I_Variable_Speed_B;

    /* 积分分离(仿 alg_pid): |err| >= 阈值时积分清零, 0 = 关闭 */
    float I_Separate_Threshold;

    /* 一阶低通时间常数(秒), 0 = 不滤波 */
    float Output_LPF_RC;
    float Derivative_LPF_RC;

    /* 分项与输出 */
    float Pout;       /* 比例项 */
    float ITerm;      /* 积分项(含 Ki, 直接参与求和与限幅) */
    float Dout;       /* 微分项 */
    float Fout;       /* 前馈项 */
    float Output;     /* 本拍输出 */
    float Last_Output;
    float Last_Dout;

    /* 改进项开关, 取值为 PID_Improve_e 按位或 */
    uint8_t Improve;
} PID_Controller_t;

/**
  * @brief  阻抗控制(原样保留自参考实现)
  */
typedef struct
{
    float Kp;
    float Kd;
    float dp;
    float dv;
    float t;
    float out;
} Impedance_t;

static inline float Impedance_Calc(Impedance_t *imp)
{
    imp->out = imp->Kp * imp->dp + imp->Kd * imp->dv + imp->t;
    return imp->out;
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  PID 初始化(核心参数; 死区/前馈/滤波/开关等通过结构体字段另行配置)
  * @param  pid            控制器实例, 由调用方静态分配
  * @param  kp/ki/kd       三项增益
  * @param  max_out        输出限幅, 0 = 不限
  * @param  max_iout       积分限幅(需再开 PID_IMPROVE_INTEGRAL_LIMIT), 0 = 不限
  * @param  control_period 调用周期(秒), 非法值回退为 0.001
  */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float max_out, float max_iout, float control_period);

/**
  * @brief  计算一步 PID
  * @param  pid     控制器实例
  * @param  measure 当前测量值
  * @param  ref     目标值
  * @retval 输出值
  */
float PID_Calculate(PID_Controller_t *pid, float measure, float ref);

/**
  * @brief  积分清零(积分分离/工况切换后手动复位用)
  */
void PID_Reset_Integral(PID_Controller_t *pid);

float PID_Get_Out(const PID_Controller_t *pid);
float PID_Get_ITerm(const PID_Controller_t *pid);
float PID_Get_Err(const PID_Controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
