/**
 * @file    drv_can.c
 * @brief   面向对象风格的 CAN 总线驱动(实现层)
 *          参考 SCUT-Robotlab 与 USTC-RoboWalker 两份开源 CAN 驱动改写
 * @version 1.0
 * @date    2026-09-25
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"

/* Private macros ------------------------------------------------------------*/

#define CAN_BUS_MAX_INSTANCES 2U /* F4 上只有 CAN1/CAN2 */

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
  ******************************************************************************
  * 综合两份参考实现并做了如下取舍:
  * 1. 一条总线封装为一个 CAN_BusObject 对象; 中断分发按"句柄 -> 对象 -> 回调"
  *    完成, 不再像参考实现那样写 if (Instance == CAN1) 分支链;
  * 2. 参考实现一用 realloc 动态扩展回调数组, 在中断相关路径上引入堆与竞态;
  *    这里改为静态对象 + 固定容量注册表, 且"先登记、后开中断",
  *    保证中断触发时对象必然已就绪;
  * 3. 接收分发同时提供两种粒度: 按 ID 精确订阅(参考实现一的多回调能力,
  *    以静态观察者表实现)与总线级兜底回调(参考实现二的单回调用法);
  * 4. 周期发送沿用参考实现二的帧表 + 分频节拍方案, 由定时器调用 CAN_Bus_Tick 驱动。
  ******************************************************************************
  */

/* 对象注册表(静态分配, 无堆操作) */
static CAN_BusObject *s_bus_registry[CAN_BUS_MAX_INSTANCES];
static uint8_t s_bus_count = 0;

/**
  * @brief  CAN1 默认周期发送帧表(仿照参考实现: 电机帧每拍发, 超级电容 1/10 拍发)
  * @note   按实际挂载设备增删修改
  * @note   0x1FF/0x200/0x2FF 不得入表: 它们是 DJ 电机电流指令帧 ID,
  *         由 DJ_MotorRun 直发; 若留在表里, CAN_Bus_Tick 每毫秒发的全零帧
  *         会与真实电流指令抢总线/挤爆 3 个发送邮箱
  */
static CAN_TxFrame_t CAN1_TxFrames[] = {
    {.id = 0x220, .period = 10},
};

CAN_BusObject can1_bus;

/* Private function declarations ---------------------------------------------*/

static CAN_BusObject *CAN_Bus_FindByHandle(CAN_HandleTypeDef *hcan);
static void CAN_Bus_RxDispatch(CAN_HandleTypeDef *hcan, uint32_t fifo);

/* Private functions ----------------------------------------------------------*/

/**
  * @brief  按 HAL 句柄查找总线对象(中断分发用)
  * @param  hcan HAL 句柄
  * @retval 对象指针, 未登记返回 NULL
  */
static CAN_BusObject *CAN_Bus_FindByHandle(CAN_HandleTypeDef *hcan)
{
  for (uint8_t i = 0; i < s_bus_count; i++)
  {
    if (s_bus_registry[i]->hcan == hcan)
    {
      return s_bus_registry[i];
    }
  }
  return NULL;
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  CAN1 驱动接入入口: 绑定 hcan1 与 can1_bus 对象
  * @note   必须在 MX_CAN1_Init 之后调用
  * @retval 无(失败进入 Error_Handler)
  */
void Driver_CAN1_Init(void)
{
  /* 一次完成 登记 -> 全收滤波 -> 启动 -> 开中断 */
  if (CAN_Bus_Init(&can1_bus, &hcan1, "CAN1", NULL,
                   CAN1_TxFrames,
                   (uint8_t)(sizeof(CAN1_TxFrames) / sizeof(CAN1_TxFrames[0]))) != 0)
  {
    Error_Handler();
  }
}

/**
  * @brief  初始化并接入一条 CAN 总线
  * @param  bus            总线对象实例, 由调用方静态分配
  * @param  hcan           底层 HAL 句柄, 需已完成 MX_xxx_Init
  * @param  name           调试用名字
  * @param  on_rx          总线级兜底回调, 可为 NULL(收到无订阅的帧只计数)
  * @param  tx_frames      周期发送帧表, 可为 NULL(不启用周期发送)
  * @param  tx_frame_count 帧表长度
  * @retval 0 成功; 1 参数错误; 2 注册表已满; 3 配置滤波器失败;
  *         4 启动外设失败; 5 使能接收中断失败
  */
uint8_t CAN_Bus_Init(CAN_BusObject *bus, CAN_HandleTypeDef *hcan, const char *name,
                     CAN_RxCallback_t on_rx, CAN_TxFrame_t *tx_frames, uint8_t tx_frame_count)
{
  uint8_t i;

  if (bus == NULL || hcan == NULL || (tx_frames == NULL && tx_frame_count != 0U))
  {
    return 1;
  }

  /* 填充对象 */
  bus->name = name;
  bus->hcan = hcan;
  bus->on_rx = on_rx;
  bus->tx_frames = tx_frames;
  bus->tx_frame_count = tx_frame_count;
  bus->sub_count = 0;
  bus->stats.rx_count = 0;
  bus->stats.tx_count = 0;
  bus->stats.rx_error = 0;
  bus->stats.tx_error = 0;
  bus->stats.bus_error = 0;

  /* 登记: 重复 Init 则原位更新, 否则追加 */
  for (i = 0; i < s_bus_count; i++)
  {
    if (s_bus_registry[i] == bus)
    {
      break;
    }
  }
  if (i == s_bus_count)
  {
    if (s_bus_count >= CAN_BUS_MAX_INSTANCES)
    {
      return 2;
    }
    s_bus_registry[s_bus_count] = bus;
    s_bus_count++;
  }

  /* 默认全收滤波(CAN1 按惯例占用 0~13 号滤波器);
     需要按 ID 收窄时再调用 CAN_Bus_ConfigFilter */
  if (CAN_Bus_ConfigFilter(bus, (hcan->Instance == CAN1) ? 0U : 14U, CAN_RX_FIFO0, 0, 0) != 0)
  {
    return 3;
  }

  /* 启动外设并打开接收中断 —— 必须放在登记之后,
     中断一旦触发回调就能拿到完整对象 */
  if (HAL_CAN_Start(hcan) != HAL_OK)
  {
    return 4;
  }
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    return 5;
  }

  return 0;
}

/**
  * @brief  按帧 ID 订阅接收帧(观察者注册)
  * @param  bus    总线对象
  * @param  std_id 订阅键: 标准帧为 StdId (0x000~0x7FF);
  *                 扩展帧为 ExtId 的低 16 位(ZDT 的 (地址<<8)|包号 恰好在此范围)
  * @param  cb     该 ID 的处理回调; 传入 NULL 表示注销该 ID 的订阅(槽位可复用)
  * @retval 0 成功; 1 参数错误; 2 订阅表已满
  * @note   同一 ID 重复订阅视为更新回调; 收到订阅帧时只调用订阅回调,
  *         未命中订阅的帧才落到 on_rx 兜底回调
  */
uint8_t CAN_Bus_Subscribe(CAN_BusObject *bus, uint16_t std_id, CAN_RxCallback_t cb)
{
  uint8_t i;
  uint8_t empty_slot = 0xFFU;

  if (bus == NULL)
  {
    return 1;
  }

  for (i = 0; i < bus->sub_count; i++)
  {
    if (bus->subs[i].used && bus->subs[i].id == std_id)
    {
      if (cb == NULL)
      {
        bus->subs[i].used = 0; /* 注销 */
        return 0;
      }
      bus->subs[i].cb = cb; /* 更新回调 */
      return 0;
    }
    if (!bus->subs[i].used && empty_slot == 0xFFU)
    {
      empty_slot = i; /* 复用注销留下的空槽 */
    }
  }

  if (cb == NULL)
  {
    return 0; /* 本来就没订阅, 注销视为成功 */
  }
  if (empty_slot == 0xFFU)
  {
    if (bus->sub_count >= CAN_BUS_MAX_SUBSCRIPTIONS)
    {
      return 2;
    }
    empty_slot = bus->sub_count;
    bus->sub_count++;
  }

  bus->subs[empty_slot].id = std_id;
  bus->subs[empty_slot].cb = cb;
  bus->subs[empty_slot].used = 1;
  return 0;
}

/**
  * @brief  立即发送一帧标准数据帧(事件型)
  * @param  bus    总线对象
  * @param  std_id 标准帧 ID (0x000~0x7FF)
  * @param  data   数据指针
  * @param  length 长度 (0~8)
  * @retval 0 成功; 1 参数错误; 2 发送失败(邮箱满/总线未启动)
  */
uint8_t CAN_Bus_Send(CAN_BusObject *bus, uint16_t std_id, uint8_t *data, uint8_t length)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mailbox;

  if (bus == NULL || bus->hcan == NULL || data == NULL || std_id > 0x7FFU || length > 8U)
  {
    return 1;
  }

  tx_header.StdId = std_id;
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = length;

  if (HAL_CAN_AddTxMessage(bus->hcan, &tx_header, data, &tx_mailbox) != HAL_OK)
  {
    bus->stats.tx_error++;
    return 2;
  }

  bus->stats.tx_count++;
  return 0;
}

/**
  * @brief  立即发送一帧扩展数据帧(ZDT 等按扩展帧 ID 寻址的设备用)
  * @param  bus    总线对象
  * @param  ext_id 扩展帧 ID (0x00000000~0x1FFFFFFF)
  * @param  data   数据指针
  * @param  length 长度 (0~8)
  * @retval 0 成功; 1 参数错误; 2 发送失败(邮箱满/总线未启动)
  */
uint8_t CAN_Bus_SendExt(CAN_BusObject *bus, uint32_t ext_id, uint8_t *data, uint8_t length)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mailbox;

  if (bus == NULL || bus->hcan == NULL || data == NULL || ext_id > 0x1FFFFFFFU || length > 8U)
  {
    return 1;
  }

  tx_header.StdId = 0;
  tx_header.ExtId = ext_id;
  tx_header.IDE = CAN_ID_EXT;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = length;

  if (HAL_CAN_AddTxMessage(bus->hcan, &tx_header, data, &tx_mailbox) != HAL_OK)
  {
    bus->stats.tx_error++;
    return 2;
  }

  bus->stats.tx_count++;
  return 0;
}

/**
  * @brief  按 ID 获取周期帧的发送缓冲区
  * @param  bus    总线对象
  * @param  std_id 帧表中的标准帧 ID
  * @retval 8 字节缓冲区指针; 该 ID 不在帧表中返回 NULL
  * @note   业务层写入后, 由 CAN_Bus_Tick 按该帧的 period 自动发出
  */
uint8_t *CAN_Bus_GetTxBuffer(CAN_BusObject *bus, uint16_t std_id)
{
  if (bus == NULL || bus->tx_frames == NULL)
  {
    return NULL;
  }

  for (uint8_t i = 0; i < bus->tx_frame_count; i++)
  {
    if (bus->tx_frames[i].id == std_id)
    {
      return bus->tx_frames[i].data;
    }
  }
  return NULL;
}

/**
  * @brief  按掩码模式配置滤波器(只支持标准数据帧, 仿照参考实现二)
  * @param  bus         总线对象
  * @param  filter_bank 滤波器序号 0~27 (双 CAN 按惯例均分: CAN1=0~13, CAN2=14~27)
  * @param  fifo        绑定的接收 FIFO: CAN_RX_FIFO0 / CAN_RX_FIFO1
  *                     (FIFO1 需同时在 CubeMX 里使能对应 RX1 中断才能收到)
  * @param  std_id      掩码后的标准帧 ID (0x000~0x7FF)
  * @param  mask_id     屏蔽位: 1=必须匹配, 0=任意; 全 0 即接收所有帧
  * @retval 0 成功; 1 参数错误; 2 HAL 配置失败
  */
uint8_t CAN_Bus_ConfigFilter(CAN_BusObject *bus, uint32_t filter_bank, uint32_t fifo,
                             uint32_t std_id, uint32_t mask_id)
{
  CAN_FilterTypeDef filter_structure;

  if (bus == NULL || bus->hcan == NULL || filter_bank > 27U ||
      fifo > CAN_RX_FIFO1 || std_id > 0x7FFU || mask_id > 0x7FFU)
  {
    return 1;
  }

  /* 32 位掩码模式下标准 ID 位于 FxR 的 [31:21], 即高 16bit 的 [15:5] */
  filter_structure.FilterIdHigh = (uint16_t)((std_id & 0x7FFU) << 5);
  filter_structure.FilterIdLow = 0x0000;
  filter_structure.FilterMaskIdHigh = (uint16_t)((mask_id & 0x7FFU) << 5);
  filter_structure.FilterMaskIdLow = 0x0000;
  filter_structure.FilterBank = filter_bank;
  filter_structure.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_structure.FilterScale = CAN_FILTERSCALE_32BIT;
  filter_structure.FilterActivation = ENABLE;
  filter_structure.FilterFIFOAssignment = fifo;
  filter_structure.SlaveStartFilterBank = 14; /* CAN1=0~13, CAN2=14~27 */

  if (HAL_CAN_ConfigFilter(bus->hcan, &filter_structure) != HAL_OK)
  {
    return 2;
  }
  return 0;
}

/**
  * @brief  CAN 周期发送节拍, 由定时器中断按固定周期调用
  * @note   遍历所有已登记总线的帧表, 到期帧压入发送邮箱;
  *         业务层写 GetTxBuffer 得到的缓冲即可, 无需关心发送时机
  */
void CAN_Bus_Tick(void)
{
  for (uint8_t i = 0; i < s_bus_count; i++)
  {
    CAN_BusObject *bus = s_bus_registry[i];

    for (uint8_t f = 0; f < bus->tx_frame_count; f++)
    {
      CAN_TxFrame_t *frame = &bus->tx_frames[f];
      uint8_t period = (frame->period == 0U) ? 1U : frame->period;

      if (++frame->counter < period)
      {
        continue;
      }
      frame->counter = 0;
      CAN_Bus_Send(bus, frame->id, frame->data, 8);
    }
  }
}

/**
  * @brief  从 FIFO 取出一帧并按对象分发(FIFO0/FIFO1 共用实现)
  * @param  hcan HAL 句柄
  * @param  fifo CAN_RX_FIFO0 / CAN_RX_FIFO1
  * @note   无论对象是否登记都要把帧取出, 否则 FIFO 满会持续触发中断
  */
static void CAN_Bus_RxDispatch(CAN_HandleTypeDef *hcan, uint32_t fifo)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  CAN_BusObject *bus = CAN_Bus_FindByHandle(hcan);
  uint32_t rx_id;

  if (HAL_CAN_GetRxMessage(hcan, fifo, &rx_header, rx_data) != HAL_OK)
  {
    if (bus != NULL)
    {
      bus->stats.rx_error++;
    }
    return;
  }
  if (bus == NULL)
  {
    return;
  }

  bus->stats.rx_count++;

  /* 订阅键统一取"有效帧 ID": 标准帧 StdId, 扩展帧 ExtId */
  rx_id = (rx_header.IDE == CAN_ID_EXT) ? rx_header.ExtId : rx_header.StdId;

  /* ① 按帧 ID 精确匹配订阅者, 命中即分发 */
  for (uint8_t i = 0; i < bus->sub_count; i++)
  {
    if (bus->subs[i].used && bus->subs[i].id == rx_id && bus->subs[i].cb != NULL)
    {
      bus->subs[i].cb(bus, &rx_header, rx_data);
      return;
    }
  }

  /* ② 无订阅命中, 落到总线级兜底回调 */
  if (bus->on_rx != NULL)
  {
    bus->on_rx(bus, &rx_header, rx_data);
  }
}

/**
  * @brief  HAL 库 CAN 接收 FIFO0 中断回调(覆盖 HAL 弱定义)
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_Bus_RxDispatch(hcan, CAN_RX_FIFO0);
}

/**
  * @brief  HAL 库 CAN 接收 FIFO1 中断回调(覆盖 HAL 弱定义)
  * @note   需 CubeMX 使能 CAN_RX1 中断且有滤波器绑定到 FIFO1 才会触发
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_Bus_RxDispatch(hcan, CAN_RX_FIFO1);
}

/**
  * @brief  HAL 库 CAN 错误回调(覆盖 HAL 弱定义)
  * @note   需另行使能 CAN_IT_ERROR 等错误中断才会触发
  */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  CAN_BusObject *bus = CAN_Bus_FindByHandle(hcan);

  if (bus != NULL)
  {
    bus->stats.bus_error++;
  }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
