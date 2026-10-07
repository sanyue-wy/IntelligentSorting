/**
 * @file    drv_pwm.c
 * @brief   面向对象风格的 PWM 输出驱动(实现层)
 * @version 1.0
 * @date    2026-09-26
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_pwm.h"
#include "drv_tim.h"

/* Private macros ------------------------------------------------------------*/

#define PWM_CHANNEL_MAX_INSTANCES 8U /* 按实际用到的通道数增删 */

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
  ******************************************************************************
  * 与 drv_can 同构, 并做了如下取舍:
  * 1. 一个输出通道封装为一个 PWM_ChannelObject 对象; 对外只有 Init / SetDuty /
  *    SetPulse / Start / Stop, 业务层不接触 CCR/ARR 寄存器;
  * 2. 时间基不沿用 CubeMX 写死的 PSC/ARR: 第一个接入该定时器的通道按
  *    freq_hz 与运行时实测的定时器时钟重算(复用 drv_tim 的计算工具),
  *    换主频/换板卡不需要改驱动;
  * 3. 同一定时器所有通道共用一个频率 —— 后续通道 Init 时校验频率一致,
  *    不一致直接报错, 避免后配的通道悄悄改掉先配通道的输出频率;
  * 4. 静态对象 + 固定容量注册表, 且"先登记、后启动输出"。
  ******************************************************************************
  */

/* 对象注册表(静态分配, 无堆操作) */
static PWM_ChannelObject *s_pwm_registry[PWM_CHANNEL_MAX_INSTANCES];
static uint8_t s_pwm_count = 0;

/* TIM2_CH1 对象实例 */
PWM_ChannelObject pwm_tim2_ch1;

/* Private function declarations ---------------------------------------------*/

/* Private functions ----------------------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  TIM2_CH1 驱动接入入口: 绑定 htim2 通道 1, 1kHz, 初始占空比 0
  * @note   必须在 MX_TIM2_Init 之后调用; 频率/通道按实际负载修改;
  *         引脚需在 CubeMX 配为 TIM2_CH1 复用推挽, 否则引脚上无波形
  * @retval 无(失败进入 Error_Handler)
  */
void Driver_TIM2_PWM_Init(void)
{
  if (PWM_Channel_Init(&pwm_tim2_ch1, &htim2, TIM_CHANNEL_1, "TIM2_CH1", 1000U, 0U) != 0)
  {
    Error_Handler();
  }
}

/**
  * @brief  初始化并接入一个 PWM 输出通道
  * @param  ch            通道对象实例, 由调用方静态分配
  * @param  htim          底层 HAL 句柄, 需已完成 MX_xxx_Init
  * @param  channel       通道号: TIM_CHANNEL_1 ~ TIM_CHANNEL_4
  * @param  name          调试用名字
  * @param  freq_hz       PWM 频率(Hz); 该定时器的首个通道据此重算时间基,
  *                       后续通道必须与首个通道同频率
  * @param  duty_permille 初始占空比(千分比 0~1000)
  * @retval 0 成功; 1 参数错误; 2 注册表已满或通道被其它对象占用;
  *         3 时间基计算失败(时钟未就绪/频率超出定时器范围);
  *         4 同定时器频率冲突; 5 配置通道失败; 6 启动输出失败
  * @note   重复 Init 同一对象视为原位更新; 启动输出放在登记之后
  */
uint8_t PWM_Channel_Init(PWM_ChannelObject *ch, TIM_HandleTypeDef *htim, uint32_t channel,
                         const char *name, uint32_t freq_hz, uint16_t duty_permille)
{
  PWM_ChannelObject *existing = NULL;
  uint8_t others = 0;
  uint8_t i;
  uint32_t clock_hz;
  uint16_t psc = 0;
  uint16_t arr = 0;
  TIM_OC_InitTypeDef sConfig = {0};

  if (ch == NULL || htim == NULL || name == NULL || freq_hz == 0U || duty_permille > 1000U ||
      (channel != TIM_CHANNEL_1 && channel != TIM_CHANNEL_2 &&
       channel != TIM_CHANNEL_3 && channel != TIM_CHANNEL_4))
  {
    return 1;
  }

  /* 扫描注册表: 通道占用检查 + 同定时器频率一致性检查 */
  for (i = 0; i < s_pwm_count; i++)
  {
    PWM_ChannelObject *entry = s_pwm_registry[i];

    if (entry->htim != htim)
    {
      continue;
    }
    if (entry->channel == channel)
    {
      if (entry != ch)
      {
        return 2; /* 该通道已被其它对象占用 */
      }
      existing = entry;
      continue;
    }
    if (entry->freq_hz != freq_hz)
    {
      return 4; /* 同定时器所有通道共用一个频率 */
    }
    others++;
  }

  /* 该定时器尚无其它通道在用: 本通道负责按实际时钟重算时间基 */
  clock_hz = TIM_Timer_GetClockHz(htim->Instance);
  if (clock_hz == 0U)
  {
    return 3;
  }
  if (others == 0U)
  {
    if (TIM_Timer_CalcFreq(clock_hz, freq_hz, &psc, &arr) != 0)
    {
      return 3;
    }
  }

  /* 填充对象 */
  ch->name = name;
  ch->htim = htim;
  ch->channel = channel;
  ch->freq_hz = freq_hz;
  ch->timer_clock_hz = clock_hz;
  ch->period_ticks = (others == 0U) ? ((uint32_t)arr + 1U) : (htim->Init.Period + 1U);
  ch->duty = duty_permille;
  ch->stats.set_count = 0;

  /* 登记: 重复 Init 原位更新, 否则追加 */
  if (existing == NULL)
  {
    if (s_pwm_count >= PWM_CHANNEL_MAX_INSTANCES)
    {
      return 2;
    }
    s_pwm_registry[s_pwm_count] = ch;
    s_pwm_count++;
  }

  /* 首个通道落时间基: HAL_TIM_PWM_Init 内部走 TIM_Base_SetConfig 落寄存器,
     重复调用幂等(CubeMX 已 Init 过时只刷时间基, 不重复走 MspInit) */
  if (others == 0U)
  {
    htim->Init.Prescaler = psc;
    htim->Init.Period = arr;
    if (HAL_TIM_PWM_Init(htim) != HAL_OK)
    {
      return 5;
    }
  }

  /* 配置通道: PWM1 模式, 高极性, 比较值预装载 */
  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.Pulse = (uint32_t)(((uint64_t)ch->period_ticks * duty_permille) / 1000U);
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(htim, &sConfig, channel) != HAL_OK)
  {
    return 5;
  }

  /* 启动输出 —— 必须放在登记之后 */
  if (HAL_TIM_PWM_Start(htim, channel) != HAL_OK)
  {
    return 6;
  }

  return 0;
}

/**
  * @brief  设置占空比
  * @param  ch            通道对象
  * @param  duty_permille 千分比 0~1000(1000 = 常高)
  * @retval 0 成功; 1 参数错误
  */
uint8_t PWM_Channel_SetDuty(PWM_ChannelObject *ch, uint16_t duty_permille)
{
  uint32_t compare;

  if (ch == NULL || ch->htim == NULL || duty_permille > 1000U)
  {
    return 1;
  }

  compare = (uint32_t)(((uint64_t)ch->period_ticks * duty_permille) / 1000U);
  __HAL_TIM_SET_COMPARE(ch->htim, ch->channel, compare);

  ch->duty = duty_permille;
  ch->stats.set_count++;
  return 0;
}

/**
  * @brief  按脉宽设置输出(舵机/步进驱动等按脉宽工作的负载用)
  * @param  ch      通道对象
  * @param  pulse_us 高电平脉宽(微秒), 不得超过 PWM 周期
  * @retval 0 成功; 1 参数错误或脉宽超出周期
  * @note   换算按 period_ticks × freq_hz 折算到分频后的实际 tick 率;
  *         直接乘 timer_clock_hz 只在 PSC=0 时成立, 50Hz 等低频下
  *         PSC≠0, 旧算式会算出超出周期的计数值导致恒返回错误
  */
uint8_t PWM_Channel_SetPulse(PWM_ChannelObject *ch, uint32_t pulse_us)
{
  uint32_t compare;

  if (ch == NULL || ch->htim == NULL || ch->period_ticks == 0U)
  {
    return 1;
  }

  /* 高电平计数值 = 周期计数 × 脉宽占周期的比例 */
  compare = (uint32_t)(((uint64_t)ch->period_ticks * ch->freq_hz * pulse_us) / 1000000U);
  if (compare > ch->period_ticks)
  {
    return 1;
  }

  __HAL_TIM_SET_COMPARE(ch->htim, ch->channel, compare);

  ch->duty = (uint16_t)(((uint64_t)compare * 1000U) / ch->period_ticks);
  ch->stats.set_count++;
  return 0;
}

/**
  * @brief  启动该通道 PWM 输出
  * @retval 0 成功; 1 参数错误; 2 HAL 失败
  */
uint8_t PWM_Channel_Start(PWM_ChannelObject *ch)
{
  if (ch == NULL || ch->htim == NULL)
  {
    return 1;
  }
  return (HAL_TIM_PWM_Start(ch->htim, ch->channel) == HAL_OK) ? 0 : 2;
}

/**
  * @brief  停止该通道 PWM 输出
  * @retval 0 成功; 1 参数错误; 2 HAL 失败
  * @note   同定时器还有其它通道在输出时, 定时器计数不会停
  */
uint8_t PWM_Channel_Stop(PWM_ChannelObject *ch)
{
  if (ch == NULL || ch->htim == NULL)
  {
    return 1;
  }
  return (HAL_TIM_PWM_Stop(ch->htim, ch->channel) == HAL_OK) ? 0 : 2;
}
