/**
 * @file    drv_uart.h
 * @brief   面向对象风格的 UART 驱动(接口层)
 *          参考 SCUT-Robotlab 与 USTC-RoboWalker 两份开源 UART 驱动改写
 * @version 1.0
 * @date    2026-09-25
 */

#ifndef __DRV_UART_H__
#define __DRV_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
  ******************************************************************************
  * 建模思路与 drv_can 一致: 一个串口 = 一个 UART_PortObject 对象。
  * 数据(句柄/接收缓冲/周期发送/统计) + 行为(on_rx 回调) 封装在一起,
  * HAL 中断回调只按句柄查对象再分发, 不写 if (Instance == USARTx) 分支链;
  * 注册表静态分配、"先登记后启动", 中断触发时对象必然就绪。
  ******************************************************************************
  */

/**
  * @brief  端口运行统计(中断上下文写入, 主循环读取)
  */
typedef struct
{
  volatile uint32_t rx_count;  /* 收到数据事件次数 */
  volatile uint32_t rx_error;  /* 接收(重启 DMA 等)失败次数 */
  volatile uint32_t tx_count;  /* 成功发送次数 */
  volatile uint32_t tx_error;  /* 发送失败次数(上一次 DMA 未发完等) */
  volatile uint32_t err_count; /* HAL_UART_ErrorCallback 触发次数(噪声/溢出等) */
} UART_PortStats_t;

typedef struct UART_PortObject UART_PortObject;

/**
  * @brief  接收回调(对象的"虚函数"): 空闲中断/收满缓冲时触发
  * @param  port 收到数据的端口对象
  * @param  data 指向该端口的接收缓冲区(即 Init 时传入的缓冲)
  * @param  size 本次实际收到的字节数(从缓冲区起始算起)
  * @note   data 只在回调期间有效; 驱动在回调返回后会重新武装接收,
  *         需要留存数据请在回调内自行拷贝, 且回调应尽快返回
  */
typedef void (*UART_RxCallback_t)(UART_PortObject *port, uint8_t *data, uint16_t size);

/**
  * @brief  UART 端口对象
  */
struct UART_PortObject
{
  const char          *name;            /* 调试用名字, 如 "USART1" */
  UART_HandleTypeDef  *huart;           /* 底层 HAL 句柄 */
  UART_RxCallback_t    on_rx;           /* 接收回调 */
  uint8_t             *rx_buffer;       /* 接收缓冲区(调用方持有) */
  uint16_t             rx_buffer_size;  /* 接收缓冲区长度 */
  uint8_t             *periodic_tx_data;/* 周期发送缓冲, NULL 表示不发送 */
  uint16_t             periodic_tx_len; /* 周期发送长度 */
  UART_PortStats_t     stats;           /* 运行统计 */
};

/* Exported variables ---------------------------------------------------------*/

/* USART1 端口对象, Driver_UART1_Init 中完成初始化绑定 */
extern UART_PortObject uart1_port;

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  UART1 驱动接入入口: 绑定 huart1 与 uart1_port 并启动 DMA 空闲接收
  * @note   在 MX_USART1_UART_Init 之后调用(main.c USER CODE 2)
  */
void Driver_UART1_Init(void);

uint8_t UART_Port_Init(UART_PortObject *port, UART_HandleTypeDef *huart, const char *name,
                       UART_RxCallback_t on_rx, uint8_t *rx_buffer, uint16_t rx_buffer_size);
uint8_t UART_Port_Send(UART_PortObject *port, uint8_t *data, uint16_t length);
uint8_t UART_Port_TxBusy(UART_PortObject *port);
int    UART_Port_Printf(UART_PortObject *port, const char *fmt, ...);
void   UART_Port_SetPeriodicTx(UART_PortObject *port, uint8_t *data, uint16_t length);
void   UART_Port_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_UART_H__ */
