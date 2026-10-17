/**
 * @file    drv_can.h
 * @brief   面向对象风格的 CAN 总线驱动(接口层)
 *          参考 SCUT-Robotlab 与 USTC-RoboWalker 两份开源 CAN 驱动改写
 * @version 1.0
 * @date    2026-09-25
 */

#ifndef __DRIVER_CAN_H__
#define __DRIVER_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* Exported types ------------------------------------------------------------*/

/**
  ******************************************************************************
  * 建模思路: 一条总线 = 一个 CAN_BusObject 对象。
  * 数据(句柄/帧表/订阅表/统计) + 行为(接收回调) 封装在一起,
  * HAL 中断回调只负责"按句柄找到对象, 再调用对象的方法",
  * 消灭两份参考实现中的 if (Instance == CAN1) 分支链。
  ******************************************************************************
  */

#define CAN_BUS_MAX_SUBSCRIPTIONS 8U /* 每条总线最多按 ID 订阅 8 个接收回调 */
#define CAN_TX_FIFO_SIZE          32U /* 软件发送队列容量: 避免并发突发流量导致硬件邮箱满丢包 */

/**
  * @brief  发送队列帧项
  */
typedef struct
{
  CAN_TxHeaderTypeDef header;
  uint8_t             data[8];
} CAN_QueueFrame_t;

/**
  * @brief  软件发送队列 (环形 FIFO)
  */
typedef struct
{
  CAN_QueueFrame_t  fifo[CAN_TX_FIFO_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint16_t count;
  volatile uint32_t drop_count;
} CAN_TxQueue_t;

/**
  * @brief  总线运行统计(中断上下文写入, 主循环读取)
  */
typedef struct
{
  volatile uint32_t rx_count;  /* 成功接收帧数 */
  volatile uint32_t tx_count;  /* 成功发送帧数 */
  volatile uint32_t rx_error;  /* 接收(读 FIFO)失败次数 */
  volatile uint32_t tx_error;  /* 发送失败次数(邮箱满/总线未就绪) */
  volatile uint32_t tx_drop;   /* 软件队列满丢弃帧数 */
  volatile uint32_t bus_error; /* HAL_CAN_ErrorCallback 触发次数 */
} CAN_BusStats_t;

/**
  * @brief  周期发送帧: 由 CAN_Bus_Tick 按 period 分频自动发送 8 字节标准数据帧
  */
typedef struct
{
  uint16_t id;      /* 标准帧 ID (0x000~0x7FF) */
  uint8_t  period;  /* 分频系数: 每 period 个节拍发送一次, 0 视为 1 */
  uint8_t  counter; /* 内部节拍计数, 业务层不要修改 */
  uint8_t  data[8]; /* 发送缓冲, 业务层直接写入, 到期由节拍自动发出 */
} CAN_TxFrame_t;

typedef struct CAN_BusObject CAN_BusObject;

/**
  * @brief  接收回调(对象的"虚函数"): 不同业务挂不同实现即为多态
  * @param  bus     收到帧的总线对象
  * @param  header  帧头
  * @param  data    8 字节数据, 仅在本次回调期间有效, 留存请自行拷贝
  */
typedef void (*CAN_RxCallback_t)(CAN_BusObject *bus, CAN_RxHeaderTypeDef *header, uint8_t *data);

/**
  * @brief  按 ID 订阅项: 收到该标准帧时精确分发(观察者模式)
  */
typedef struct
{
  uint16_t         id;   /* 订阅的标准帧 ID */
  CAN_RxCallback_t cb;   /* 该 ID 的处理回调 */
  uint8_t          used; /* 占用标志, 注销后槽位可复用 */
} CAN_BusSubscription_t;

/**
  * @brief  CAN 总线对象
  */
struct CAN_BusObject
{
  const char            *name;           /* 调试用名字, 如 "CAN1" */
  CAN_HandleTypeDef     *hcan;           /* 底层 HAL 句柄 */
  CAN_RxCallback_t       on_rx;          /* 总线级兜底回调: 无 ID 订阅命中时调用 */
  CAN_TxFrame_t         *tx_frames;      /* 周期发送帧表 */
  uint8_t                tx_frame_count; /* 帧表长度 */
  CAN_BusSubscription_t  subs[CAN_BUS_MAX_SUBSCRIPTIONS]; /* ID 订阅表 */
  uint8_t                sub_count;      /* 订阅表已用长度 */
  CAN_TxQueue_t          tx_queue;       /* 软件发送队列 (防硬件邮箱占满丢包) */
  CAN_BusStats_t         stats;          /* 运行统计 */
};

/* Exported variables ---------------------------------------------------------*/

/* CAN1 对象实例, Driver_CAN1_Init 中完成初始化绑定 */
extern CAN_BusObject can1_bus;

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  CAN1 驱动接入入口: 绑定 hcan1 与 can1_bus 对象
  * @note   在 MX_CAN1_Init 之后调用(main.c USER CODE 2)
  */
void Driver_CAN1_Init(void);

uint8_t  CAN_Bus_Init(CAN_BusObject *bus, CAN_HandleTypeDef *hcan, const char *name,
                      CAN_RxCallback_t on_rx, CAN_TxFrame_t *tx_frames, uint8_t tx_frame_count);
uint8_t  CAN_Bus_Subscribe(CAN_BusObject *bus, uint16_t std_id, CAN_RxCallback_t cb);
uint8_t  CAN_Bus_Send(CAN_BusObject *bus, uint16_t std_id, uint8_t *data, uint8_t length);
uint8_t  CAN_Bus_SendExt(CAN_BusObject *bus, uint32_t ext_id, uint8_t *data, uint8_t length);
uint8_t *CAN_Bus_GetTxBuffer(CAN_BusObject *bus, uint16_t std_id);
uint8_t  CAN_Bus_ConfigFilter(CAN_BusObject *bus, uint32_t filter_bank, uint32_t fifo,
                              uint32_t std_id, uint32_t mask_id);
void     CAN_Bus_PumpTxQueue(CAN_BusObject *bus);
void     CAN_Bus_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_CAN_H__ */
