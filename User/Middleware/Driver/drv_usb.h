/**
 * @file    drv_usb.h
 * @brief   USB CDC 虚拟串口驱动(接口层)
 *          参照 USTC-RoboWalker 的 drv_usb.cpp 改写为 C, 并适配本工程:
 *          F407CubeMX 生成的是 FS 设备(hUsbDeviceFS / CDC_Transmit_FS),
 *          本工程无 SYS_Timestamp, 时间戳改用 HAL_GetTick()
 * @version 1.0
 * @date    2026-10-07
 */

#ifndef __DRV_USB_H__
#define __DRV_USB_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
  ******************************************************************************
  * 建模与参考实现一致: 一个 USB 设备 = 一个 USB_ManageObject 对象。
  * 自设双缓冲: 收到数据后立刻把另一块缓冲重新武装给 USB 库,
  * 业务回调在中断里处理"就绪块", 与 USB 外设的下一次写入互不干扰。
  * 参考实现的 init_finished 门闩在这里由 USB_Init 完成标记代替:
  * 枚举完成后到 USB_Init 之间到达的数据只重新武装、不进业务回调。
  ******************************************************************************
  */

/**
  * @brief  USB 接收回调(对象的"虚函数")
  * @param  buf  指向本次数据所在的就绪缓冲区(双缓冲之一)
  * @param  len  本次实际收到的字节数
  * @note   在中断上下文调用, 且 buf 只在回调期间有效 —— 驱动已把接收
  *         重定向到另一块缓冲, 但该块会在下一次收包时被覆写,
  *         需要留存数据请在回调内自行拷贝, 且回调应尽快返回
  */
typedef void (*USB_Callback)(uint8_t *buf, uint16_t len);

/**
  * @brief  USB 运行统计(主循环读取)
  * @note   rx_* 只在 USB 中断写; tx_* 会被主循环与 USB 中断两个上下文写,
  *         由 USB_Transmit_Data 的 PRIMASK 临界区串行化, 不会丢计数
  */
typedef struct
{
  volatile uint32_t rx_count;  /* 收到数据事件次数 */
  volatile uint32_t rx_drop;   /* USB_Init 前到达被丢弃的次数 */
  volatile uint32_t tx_count;  /* 成功发送次数 */
  volatile uint32_t tx_error;  /* 发送失败/忙/未枚举被拒次数 */
} USB_PortStats_t;

/**
  * @brief  USB 设备管理对象
  */
typedef struct
{
  USB_Callback     Callback_Function; /* 接收回调 */
  uint8_t         *Rx_Buffer_Active;  /* 当前武装给 USB 库的缓冲(下一次收包写入) */
  uint8_t         *Rx_Buffer_Ready;   /* 已收到数据、等待业务层处理的缓冲 */
  uint8_t         *Rx_Buffer_0;       /* 双缓冲块 0(即 CubeMX 的 UserRxBufferFS) */
  uint8_t         *Rx_Buffer_1;       /* 双缓冲块 1(驱动内部分配) */
  volatile uint32_t Rx_Time_Stamp;    /* 最后一次收包时刻(HAL_GetTick) */
  USB_PortStats_t  stats;             /* 运行统计 */
} USB_ManageObject;

/* Exported variables ---------------------------------------------------------*/

/* USB0 设备对象, USB_Init 中完成初始化绑定 */
extern USB_ManageObject usb0;

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  初始化 USB 驱动: 绑定双缓冲与接收回调
  * @param  Callback_Function 数据处理回调, 可为 NULL(收到数据只计数)
  * @note   必须在 MX_USB_DEVICE_Init 之后调用(见 debug_init);
  *         调用前到达的数据会被丢弃, CDC 枚举不受影响
  */
void USB_Init(USB_Callback Callback_Function);

/**
  * @brief  发送数据帧
  * @param  data   被发送的数据指针(发送期间须保持有效)
  * @param  length 长度
  * @retval USBD_OK 成功; USBD_BUSY 上一帧未发完; USBD_FAIL 参数错误/未就绪
  * @note   可被主循环与 USB 中断并发调用: 内部 PRIMASK 临界区保证
  *         TxState 判定-置位与统计自增的原子性; 未枚举(pClassData
  *         为 NULL)时直接返回 USBD_FAIL, 不触碰 CDC 类句柄
  */
uint8_t USB_Transmit_Data(uint8_t *data, uint16_t length);

/**
  * @brief  CDC 接收桥接: 由 usbd_cdc_if.c 的 CDC_Receive_FS 调用
  * @param  buf    本次数据所在的 CDC 接收缓冲
  * @param  length 本次实际收到的字节数
  * @note   负责双缓冲切换、重新武装接收、再分发业务回调;
  *         在中断上下文执行, 业务回调同样运行在中断上下文
  */
void USB_ReceiveCallback(uint8_t *buf, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_USB_H__ */
