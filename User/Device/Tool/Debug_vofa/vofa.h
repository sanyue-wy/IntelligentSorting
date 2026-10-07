/**
 * @file    vofa.h
 * @brief   VOFA+ 上位机调试工具(接口层): 波形输出 + 串口 PID 在线调参
 *          参考 dev_vofa 实现改写, 接入本工程的 drv_uart 对象层与 pid 模块
 * @version 1.0
 * @date    2026-09-25
 */

#ifndef __VOFA_H__
#define __VOFA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include "drv_uart.h"
#include "pid.h"

/* Exported macros ------------------------------------------------------------*/

/**
  * 数据格式(编译期选择):
  *   0 = firewater, 文本 CSV, VOFA+ 用 firewater 协议解析
  *   1 = justfloat, 二进制帧(4字节小端浮点 x N + 帧尾 00 00 80 7F)
  */
#ifndef VOFA_DATA_FORMAT
#define VOFA_DATA_FORMAT 0
#endif

#define VOFA_FRAME_HEAD     '='  /* 命令名与数值的分隔符, 如 P1=1.5! */
#define VOFA_FRAME_TAIL     '!'  /* 命令帧结束符 */
#define VOFA_CMD_MAX        32   /* 单帧命令缓冲长度 */
#define VOFA_TX_PERIOD_MS   50   /* 波形发送节流周期(ms) */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  在线调参的参数暂存(命令解析先写这里, 再推送到绑定对象)
  * @note   P1/I1/D1 = 速度环; P2/I2/D2 = 位置环(参考实现中注释与字段是交叉的, 此处已理顺)
  */
typedef struct
{
  float speed_kp;     /* P1 */
  float speed_ki;     /* I1 */
  float speed_kd;     /* D1 */
  float position_kp;  /* P2 */
  float position_ki;  /* I2 */
  float position_kd;  /* D2 */
  float speed_target; /* Spe */
  float angle_target; /* Ang */
} Vofa_Para_t;

/**
  * @brief  VOFA+ 调试工具对象
  */
typedef struct
{
  UART_PortObject  *port;          /* 输出/接收绑定的串口 */
  Vofa_Para_t       para;          /* 调参暂存 */

  /* 已绑定的控制对象(可为 NULL, 未绑定的命令只更新暂存) */
  PID_Controller_t *speed_pid;     /* 速度环 PID */
  PID_Controller_t *position_pid;  /* 位置环 PID */
  float            *speed_target;  /* 速度目标值变量地址 */
  float            *angle_target;  /* 角度目标值变量地址 */

  /* 命令帧装配状态 */
  uint8_t           rx_line;
  uint8_t           cmd_buff[VOFA_CMD_MAX];

  /* 波形发送节流 */
  uint32_t          tx_tick;
} Vofa_t;

/* Exported variables ---------------------------------------------------------*/

extern Vofa_t vofa;

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  初始化 VOFA 工具并接管所选串口的接收回调
  * @param  v    工具实例
  * @param  port 绑定的串口(通常 &uart1_port)
  * @note   必须在 Driver_UARTx_Init 之后调用; 会把 port->on_rx 指向 VOFA,
  *         该串口的接收从此由 VOFA 独占
  */
void Vofa_Init(Vofa_t *v, UART_PortObject *port);

/**
  * @brief  绑定速度环: 目标命令 Spe 生效到 *target, P1/I1/D1 生效到 pid
  * @note   绑定时把 pid 当前增益回填到暂存, 保证没收到命令前不会推送旧值/零值
  */
void Vofa_BindSpeedPid(Vofa_t *v, PID_Controller_t *pid, float *target);

/**
  * @brief  绑定位置环: 目标命令 Ang 生效到 *target, P2/I2/D2 生效到 pid
  */
void Vofa_BindPositionPid(Vofa_t *v, PID_Controller_t *pid, float *target);

/**
  * @brief  串口接收回调(挂到 UART_RxCallback_t, 中断上下文执行)
  * @note   Vofa_Init 已自动挂接, 一般无需手动调用
  */
void Vofa_UartRxCallback(UART_PortObject *port, uint8_t *data, uint16_t size);

/**
  * @brief  把暂存参数推送到已绑定的 PID 对象与目标变量
  * @note   收到命令后自动调用; 也可在 PID_ReInit 覆盖增益后手动重推
  */
void Vofa_ApplyParams(Vofa_t *v);

/**
  * @brief  发送一帧波形数据(内部按 VOFA_TX_PERIOD_MS 节流, 未到周期直接返回)
  * @param  position_target/actual/out  位置环: 目标/实际/输出
  * @param  speed_target/actual/out     速度环: 目标/实际/输出
  * @note   阻塞打印(firewater)与 DMA 发送(justfloat)都只能在主循环上下文调用
  */
void Vofa_Display(Vofa_t *v,
                  float position_target, float position_actual, float position_out,
                  float speed_target, float speed_actual, float speed_out);

/**
  * @brief  以 justfloat 格式发送任意个浮点数(帧尾 00 00 80 7F)
  * @param  v     工具实例
  * @param  data  浮点数组
  * @param  count 通道数 (1 ~ VOFA_MAX_CH)
  * @retval 0 成功; 1 参数错误; 2 发送失败(上一帧 DMA 未发完)
  * @note   不受 VOFA_DATA_FORMAT 与 50ms 节流影响;
  *         内部使用静态组帧缓冲(需等 DMA 发完), 主循环上下文调用
  */
uint8_t Vofa_SendFloats(Vofa_t *v, const float *data, uint16_t count);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H__ */
