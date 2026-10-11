/**
 * @file    drv_usb.c
 * @brief   USB CDC 虚拟串口驱动(实现层)
 *          参照 USTC-RoboWalker 的 drv_usb.cpp 改写为 C, 适配本工程 FS 设备
 * @version 1.0
 * @date    2026-10-07
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_usb.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
  ******************************************************************************
  * 与参考实现的差异(工程适配, 逻辑与参考一致):
  * 1. 参考面向 HS 设备(hUsbDeviceHS/CDC_Transmit_HS/UserRxBufferHS),
  *    本工程 CubeMX 生成的是 FS 设备, 对应换成 hUsbDeviceFS/CDC_Transmit_FS;
  * 2. 参考的 SYS_Timestamp 对象本工程没有, 时间戳改用 HAL_GetTick();
  * 3. 参考的 init_finished 是应用层全局门闩, 这里收进驱动: USB_Init 前
  *    到达的数据只重新武装缓冲不进业务回调(参考实现二的缺陷是初始化
  *    前到达的数据会用未初始化的双缓冲指针切换, 直接跑飞);
  * 4. 保留参考的自设双缓冲与"先武装另一块、再分发回调"的顺序, 并补上
  *    收发统计(与 drv_uart/drv_can 的 stats 口径一致);
  * 5. 发送会被主循环(host 心跳/hello)与 USB 中断(回显)并发调用, 而
  *    CDC_Transmit_FS -> USBD_CDC_TransmitPacket 的 TxState 判定-置位
  *    不是原子的, 窗口内被抢占会对同一在飞 IN 端点二次 USBD_LL_Transmit
  *    (重写 DIEPTSIZ -> 帧错乱/端点锁死); tx 统计的非原子自增同样会丢
  *    计数。发送入口整体用 PRIMASK 临界区串行化, 一并解决这两处。
  ******************************************************************************
  */

/* USB0 设备对象 */
USB_ManageObject usb0 = {0};

/* 第二块接收缓冲: 第一块复用 CubeMX 的 UserRxBufferFS(与 CDC_Init_FS
   初始武装的缓冲一致, 避免武装缓冲与驱动认知不一致) */
static uint8_t s_usb_rx_buf1[APP_RX_DATA_SIZE];

/* USB_Init 完成标记: 之前到达的数据只重新武装, 不进业务回调 */
static volatile uint8_t s_usb_init_finished = 0;

/* USB 设备句柄, 由 USB_DEVICE/Target/usbd_conf.c 定义 */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  初始化 USB 驱动: 绑定双缓冲与接收回调
  * @param  Callback_Function 数据处理回调, 可为 NULL(收到数据只计数)
  * @note   必须在 MX_USB_DEVICE_Init 之后调用; 标记最后写入,
  *         中断看到 init_finished 时缓冲指针必然已就绪
  */
void USB_Init(USB_Callback Callback_Function)
{
  usb0.Callback_Function = Callback_Function;
  usb0.Rx_Buffer_0 = UserRxBufferFS;
  usb0.Rx_Buffer_1 = s_usb_rx_buf1;
  /* CDC_Init_FS 已把接收武装在 UserRxBufferFS 上, Active 对齐它 */
  usb0.Rx_Buffer_Active = UserRxBufferFS;
  usb0.Rx_Buffer_Ready = NULL;
  usb0.Rx_Time_Stamp = 0;
  usb0.stats.rx_count = 0;
  usb0.stats.rx_drop = 0;
  usb0.stats.tx_count = 0;
  usb0.stats.tx_error = 0;

  s_usb_init_finished = 1;
}

/**
  * @brief  发送数据帧
  * @param  data   被发送的数据指针(发送期间须保持有效)
  * @param  length 长度
  * @retval USBD_OK 成功; USBD_BUSY 上一帧未发完; USBD_FAIL 参数错误/未就绪
  * @note   CDC_Transmit_FS 内部检查 TxState, 上一帧没发完返回 USBD_BUSY,
  *         调用方按返回值决定丢弃或稍后重发;
  *         整段在 PRIMASK 临界区内执行, 主循环与 USB 中断可并发调用,
  *         内部的 TxState 判定-置位与 tx 统计自增被串行化
  */
uint8_t USB_Transmit_Data(uint8_t *data, uint16_t length)
{
  uint8_t ret;
  uint32_t primask;

  if (data == NULL || length == 0U)
  {
    return USBD_FAIL;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  /* 未枚举/未配置(SET_CONFIGURATION 之前或被清除)时 pClassData 为 NULL,
     直接进 CDC_Transmit_FS 会解引用读 TxState —— 属未定义行为,
     这里提前挡掉, 兑现接口"未就绪返回 USBD_FAIL"的承诺 */
  if (hUsbDeviceFS.pClassData == NULL)
  {
    usb0.stats.tx_error++;
    ret = USBD_FAIL;
  }
  else
  {
    ret = CDC_Transmit_FS(data, length);
    if (ret == USBD_OK)
    {
      usb0.stats.tx_count++;
    }
    else
    {
      usb0.stats.tx_error++;
    }
  }

  __set_PRIMASK(primask);
  return ret;
}

/**
  * @brief  CDC 接收桥接: 双缓冲切换 -> 重新武装 -> 分发业务回调
  * @param  buf    本次数据所在的 CDC 接收缓冲(即上一次武装的那块)
  * @param  length 本次实际收到的字节数
  * @note   由 CDC_Receive_FS 在中断上下文调用;
  *         "先武装另一块再回调"保证业务回调耗时期间 USB 可继续收包,
  *         本次数据所在的块不会被 USB 库覆写(但下一次收包会覆写它)
  */
void USB_ReceiveCallback(uint8_t *buf, uint32_t length)
{
  uint8_t *other;

  /* USB_Init 尚未完成: 缓冲指针未绑定, 原样重新武装后丢弃数据 */
  if (!s_usb_init_finished)
  {
    usb0.stats.rx_drop++;
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, buf);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return;
  }

  /* 双缓冲切换: 数据落在 buf, 立刻把另一块武装给 USB 库 */
  other = (buf == usb0.Rx_Buffer_0) ? usb0.Rx_Buffer_1 : usb0.Rx_Buffer_0;
  usb0.Rx_Buffer_Ready = buf;
  usb0.Rx_Buffer_Active = other;
  usb0.Rx_Time_Stamp = HAL_GetTick();

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, usb0.Rx_Buffer_Active);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  usb0.stats.rx_count++;
  if (usb0.Callback_Function != NULL)
  {
    usb0.Callback_Function(usb0.Rx_Buffer_Ready, (uint16_t)length);
  }
}
