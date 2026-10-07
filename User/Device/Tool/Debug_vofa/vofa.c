/**
 * @file    vofa.c
 * @brief   VOFA+ 上位机调试工具(实现层)
 *          参考 dev_vofa 实现改写, 输出与接收全部走 drv_uart 对象层
 * @version 1.0
 * @date    2026-09-25
 */

/* Includes ------------------------------------------------------------------*/
#include "vofa.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* justfloat 组帧缓冲上限: 最大通道数(帧 = 4*count + 4 字节) */
#define VOFA_MAX_CH      16
#define VOFA_FRAME_BUF   (VOFA_MAX_CH * 4 + 4)

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Vofa_t vofa;

/**
  * @brief  命令名 -> Vofa_Para_t 字段 偏移表(表驱动, 扩展命令只需加一行)
  */
static const struct
{
  const char *cmd;
  size_t      offset;
} s_cmd_map[] = {
    {"P1", offsetof(Vofa_Para_t, speed_kp)},
    {"I1", offsetof(Vofa_Para_t, speed_ki)},
    {"D1", offsetof(Vofa_Para_t, speed_kd)},
    {"P2", offsetof(Vofa_Para_t, position_kp)},
    {"I2", offsetof(Vofa_Para_t, position_ki)},
    {"D2", offsetof(Vofa_Para_t, position_kd)},
    {"Spe", offsetof(Vofa_Para_t, speed_target)},
    {"Ang", offsetof(Vofa_Para_t, angle_target)},
    /* 后续扩展命令在这里加一行 */
};

/* Private function declarations ---------------------------------------------*/

static void Vofa_ParseCommand(Vofa_t *v);

/* Private functions ----------------------------------------------------------*/

/**
  * @brief  解析一帧完整命令(命令名=数值!), 暂存后立即推送
  * @note   在中断上下文执行: 不做任何打印(阻塞打印在中断里可能与
  *         SysTick 死锁), 参考实现逐字节打印的调试代码已移除
  */
static void Vofa_ParseCommand(Vofa_t *v)
{
  char cmd[8];
  char num[24];
  uint8_t head = 0xFFU;
  uint8_t tail = 0xFFU;
  uint8_t cmd_len;
  uint8_t num_len;
  float value;

  /* 定位帧头 '=' 与帧尾 '!' */
  for (uint8_t i = 0; i < v->rx_line; i++)
  {
    if (v->cmd_buff[i] == VOFA_FRAME_HEAD && head == 0xFFU)
    {
      head = i;
    }
    if (v->cmd_buff[i] == VOFA_FRAME_TAIL)
    {
      tail = i;
      break;
    }
  }
  if (head == 0xFFU || tail == 0xFFU || tail <= head)
  {
    return; /* 帧格式不对, 丢弃 */
  }

  cmd_len = head;             /* 帧头之前是命令名 */
  num_len = tail - head - 1U;  /* 帧头与帧尾之间是数值 */
  if (cmd_len == 0U || cmd_len >= sizeof(cmd) ||
      num_len == 0U || num_len >= sizeof(num))
  {
    return;
  }

  memcpy(cmd, v->cmd_buff, cmd_len);
  cmd[cmd_len] = '\0';
  memcpy(num, &v->cmd_buff[head + 1U], num_len);
  num[num_len] = '\0';

  value = (float)atof(num);

  /* 查表分发 */
  for (uint8_t i = 0; i < sizeof(s_cmd_map) / sizeof(s_cmd_map[0]); i++)
  {
    if (strcmp(cmd, s_cmd_map[i].cmd) == 0)
    {
      *(float *)((uint8_t *)&v->para + s_cmd_map[i].offset) = value;
      Vofa_ApplyParams(v);
      return;
    }
  }
  /* 未匹配的命令静默丢弃 */
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  初始化 VOFA 工具并接管串口接收
  */
void Vofa_Init(Vofa_t *v, UART_PortObject *port)
{
  if (v == NULL)
  {
    return;
  }

  *v = (Vofa_t){0};
  v->port = port;

  if (port != NULL)
  {
    port->on_rx = Vofa_UartRxCallback; /* 该串口接收由 VOFA 独占 */
  }
}

/**
  * @brief  绑定速度环(命令 P1/I1/D1/Spe)
  */
void Vofa_BindSpeedPid(Vofa_t *v, PID_Controller_t *pid, float *target)
{
  if (v == NULL)
  {
    return;
  }

  v->speed_pid = pid;
  v->speed_target = target;

  /* 用 PID 当前值回填暂存, 防止没收到命令就推送零值把增益清掉 */
  if (pid != NULL)
  {
    v->para.speed_kp = pid->Kp;
    v->para.speed_ki = pid->Ki;
    v->para.speed_kd = pid->Kd;
  }
}

/**
  * @brief  绑定位置环(命令 P2/I2/D2/Ang)
  */
void Vofa_BindPositionPid(Vofa_t *v, PID_Controller_t *pid, float *target)
{
  if (v == NULL)
  {
    return;
  }

  v->position_pid = pid;
  v->angle_target = target;

  if (pid != NULL)
  {
    v->para.position_kp = pid->Kp;
    v->para.position_ki = pid->Ki;
    v->para.position_kd = pid->Kd;
  }
}

/**
  * @brief  串口接收回调: 装配命令帧, 收到帧尾即解析
  * @note   中断上下文执行; 同一事件里的多个命令帧会依次处理
  */
void Vofa_UartRxCallback(UART_PortObject *port, uint8_t *data, uint16_t size)
{
  Vofa_t *v = &vofa;

  if (v->port == NULL || v->port != port || data == NULL)
  {
    return;
  }

  for (uint16_t i = 0; i < size; i++)
  {
    /* 修复参考实现的缓冲溢出: 帧尾一直不来时强制复位重新同步 */
    if (v->rx_line >= VOFA_CMD_MAX)
    {
      v->rx_line = 0;
    }

    v->cmd_buff[v->rx_line] = data[i];
    v->rx_line++;

    if (data[i] == VOFA_FRAME_TAIL)
    {
      Vofa_ParseCommand(v);
      v->rx_line = 0;
      memset(v->cmd_buff, 0, sizeof(v->cmd_buff));
      /* 不 break: 一个事件里背靠背的多帧命令继续依次解析 */
    }
  }
}

/**
  * @brief  把暂存参数推送到已绑定的 PID 与目标变量
  * @note   单个 float 在 Cortex-M4 上是原子写, 中断里推送安全
  */
void Vofa_ApplyParams(Vofa_t *v)
{
  if (v == NULL)
  {
    return;
  }

  if (v->speed_pid != NULL)
  {
    v->speed_pid->Kp = v->para.speed_kp;
    v->speed_pid->Ki = v->para.speed_ki;
    v->speed_pid->Kd = v->para.speed_kd;
  }
  if (v->position_pid != NULL)
  {
    v->position_pid->Kp = v->para.position_kp;
    v->position_pid->Ki = v->para.position_ki;
    v->position_pid->Kd = v->para.position_kd;
  }
  if (v->speed_target != NULL)
  {
    *v->speed_target = v->para.speed_target;
  }
  if (v->angle_target != NULL)
  {
    *v->angle_target = v->para.angle_target;
  }
}

/**
  * @brief  发送一帧波形数据(50ms 节流)
  */
void Vofa_Display(Vofa_t *v,
                  float position_target, float position_actual, float position_out,
                  float speed_target, float speed_actual, float speed_out)
{
  if (v == NULL || v->port == NULL)
  {
    return;
  }

  /* 50ms 节流 */
  if (uwTick - v->tx_tick < VOFA_TX_PERIOD_MS)
  {
    return;
  }
  v->tx_tick = uwTick;

#if VOFA_DATA_FORMAT == 1
  /* justfloat: 6 个小端浮点 + 帧尾 00 00 80 7F */
  {
    float values[6] = {
        position_target, position_actual, position_out,
        speed_target, speed_actual, speed_out};
    Vofa_SendFloats(v, values, 6);
  }
#else
  /* firewater: 文本 CSV, VOFA+ 按逗号分隔解析 */
  UART_Port_Printf(v->port,
                   "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                   position_target, position_actual, position_out,
                   speed_target, speed_actual, speed_out);
#endif
}

/**
  * @brief  以 justfloat 格式发送任意个浮点数
  * @note   组帧缓冲必须是静态的: UART_Port_Send 走 DMA,
  *         函数返回后 DMA 仍在读该缓冲, 栈上的临时帧会被覆写;
  *         因此上一帧未发完时直接丢弃本拍, 绝不能先改写缓冲再发送
  * @retval 0 成功; 1 参数错误; 2 上一帧仍在发送, 本拍丢弃
  */
uint8_t Vofa_SendFloats(Vofa_t *v, const float *data, uint16_t count)
{
  static uint8_t frame[VOFA_FRAME_BUF];

  if (v == NULL || v->port == NULL || data == NULL ||
      count == 0U || count > VOFA_MAX_CH)
  {
    return 1;
  }

  /* 上一帧 DMA 还在读 frame: 先判忙, 忙则丢弃, 不能 memcpy */
  if (UART_Port_TxBusy(v->port))
  {
    return 2;
  }

  memcpy(frame, data, count * 4U);
  frame[count * 4U + 0U] = 0x00;
  frame[count * 4U + 1U] = 0x00;
  frame[count * 4U + 2U] = 0x80;
  frame[count * 4U + 3U] = 0x7F;

  return UART_Port_Send(v->port, frame, (uint16_t)(count * 4U + 4U));
}
