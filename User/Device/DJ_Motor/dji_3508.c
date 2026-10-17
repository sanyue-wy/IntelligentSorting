/**
 * @file    dji_3508.c
 * @brief   大疆 O3/O4 系列空心杯电机(2006/3508)控制实现
 *          CAN 收发对接 drv_can 驱动层, 控制器使用 pid.h / td.h
 */

/* Includes ------------------------------------------------------------------*/
#include "dji_3508.h"

/* ========================================================================== */
/*  私有宏 / 变量                                                              */
/* ========================================================================== */

#define DJ_MOTOR_BUS (&can1_bus) /* 挂载的 CAN 总线对象 */

/* 连续 DJ_RX_TIMEOUT 拍未收到反馈帧判定为通信丢失(≈100×调用周期≈300ms@3ms) */
#define DJ_RX_TIMEOUT 100U

/* 电机 ID 地址: 0x1FF = 高4路电流帧, 0x200 = 低4路电流帧, 0x201~0x208 = 反馈帧 */
enum CAN_Motor_ID
{
  DJ_H_ID  = 0x1FF,
  DJ_L_ID  = 0x200,
  DJ_M1_ID = 0x201,
  DJ_M2_ID = 0x202,
  DJ_M3_ID = 0x203,
  DJ_M4_ID = 0x204,
  DJ_M5_ID = 0x205,
  DJ_M6_ID = 0x206,
  DJ_M7_ID = 0x207,
  DJ_M8_ID = 0x208,
};

volatile static uint8_t motorCount;            /* 电机数量 */
static DJ_Motor_t *Motors[8];                  /* 电机参数结构体指针数组, 索引 = Motor_ID - 1 */

DJ_Motor_t DJ_Motor3508[3];

/* ========================================================================== */
/*  TDPID 辅助函数                                                             */
/* ========================================================================== */

void TDPID_Init(TDPID_t *c, float kp, float ki, float kd,
                float max_out, float max_iout, float control_period, float td_r)
{
  if (c == NULL)
  {
    return;
  }
  c->max_out = max_out;
  TD_Init(&c->td, td_r, control_period);
  PID_Init(&c->pid, kp, ki, kd, max_out, max_iout, control_period);
  c->pid.Improve |= PID_IMPROVE_INTEGRAL_LIMIT; /* 使 IntegralLimit 生效 */
}

void TDPID_Clear(TDPID_t *c)
{
  if (c == NULL)
  {
    return;
  }
  TD_Clear(&c->td);
  PID_Clear(&c->pid);
  c->out = 0.0f;
}

float TDPID_Calc(TDPID_t *c, float measure, float ref)
{
  if (c == NULL)
  {
    return 0.0f;
  }
  c->pid.MaxOut = c->max_out; /* 每拍同步限幅(DJ_SetAngle 可在线改写 max_out) */
  float smoothed_ref = TD_Calculate(&c->td, ref);
  c->out = PID_Calculate(&c->pid, measure, smoothed_ref);
  return c->out;
}

/* ========================================================================== */
/*  CAN 接收回调(签名适配 CAN_RxCallback_t)                                    */
/* ========================================================================== */

static void DJ_CAN_Callback(CAN_BusObject *bus, CAN_RxHeaderTypeDef *header, uint8_t *data)
{
  (void)bus;

  switch (header->StdId)
  {
  case DJ_M1_ID:
  case DJ_M2_ID:
  case DJ_M3_ID:
  case DJ_M4_ID:
  case DJ_M5_ID:
  case DJ_M6_ID:
  case DJ_M7_ID:
  case DJ_M8_ID:
  {
    uint8_t i = (uint8_t)(header->StdId - DJ_M1_ID);

    /* 跳过未注册的电机 */
    if (Motors[i] == NULL || !Motors[i]->initialized)
      break;

    Motors[i]->last_angle = Motors[i]->angle;
    Motors[i]->angle     = (uint16_t)((data[0] << 8) | data[1]);
    Motors[i]->speed     = (int16_t)((data[2] << 8) | data[3]);
    Motors[i]->current   = (int16_t)((data[4] << 8) | data[5]);

    Motors[i]->no_rx_count = 0; /* 收到 CAN 帧, 清零超时计数 */

    /* 首帧: 以当前编码器值为零点(上电位姿即 0 度), 此时不存在上一拍样本,
       跳过绕圈检测, 避免上电编码器值 >4096 被误判为翻转 */
    if (!Motors[i]->rx_once)
    {
      Motors[i]->rx_once       = 1;
      Motors[i]->offset_angle  = Motors[i]->angle;
      Motors[i]->round_count   = 0;
      Motors[i]->total_angle   = 0.0f;
      break;
    }

    /* 跨 8192 翻转累加圈数 */
    if ((int16_t)(Motors[i]->angle - Motors[i]->last_angle) > 4096)
    {
      Motors[i]->round_count--;
    }
    else if ((int16_t)(Motors[i]->angle - Motors[i]->last_angle) < -4096)
    {
      Motors[i]->round_count++;
    }
    Motors[i]->total_angle = (Motors[i]->round_count +
                              ((int32_t)Motors[i]->angle - (int32_t)Motors[i]->offset_angle) / 8192.0f) *
                             360.0f / (float)Motors[i]->dec;
    break;
  }
  default:
    break;
  }
}

/* ========================================================================== */
/*  初始化                                                                     */
/* ========================================================================== */

/**
  * @brief  电机参数初始化
  * @note   不可以重复初始化同一个电机; 必须在 Driver_CAN1_Init() 之后调用
  */
void DJ_Init(DJ_Motor_t *motor, uint8_t Motor_ID, DJ_MotorType_e Motor_Type, DJ_ControllMethod_e method)
{
  if (motor == NULL || Motor_ID < 1U || Motor_ID > 8U)
  {
    return;
  }

#if DJ_USE_PID
  if (method == PID_METHOD)
  {
    if (Motor_Type == M2006)
    {
      motor->dec = M2006_DEC;
      /* 速度环 */
      PID_Init(&motor->PID_Speed, 5.0f, 0.5f, 0.0f, 16000.0f, 1000.0f, DJ_CTRL_PERIOD);
      motor->PID_Speed.Improve |= PID_IMPROVE_INTEGRAL_LIMIT;
      /* 角度环(TD+PID) */
      TDPID_Init(&motor->PID_Angle, 150.0f, 0.5f, 10.0f, 5000.0f, 50.0f, DJ_CTRL_PERIOD, 300.0f);
      /* 角度环输出 -> 速度环 */
      PID_Init(&motor->PID_SpeedOfAngle, 5.0f, 0.0f, 0.0f, 16000.0f, 500.0f, DJ_CTRL_PERIOD);
      motor->PID_SpeedOfAngle.Improve |= PID_IMPROVE_INTEGRAL_LIMIT;
    }
    else if (Motor_Type == M3508)
    {
      motor->dec = M3508_DEC;
      PID_Init(&motor->PID_Speed, 5.0f, 0.5f, 0.0f, 16000.0f, 1000.0f, DJ_CTRL_PERIOD);
      motor->PID_Speed.Improve |= PID_IMPROVE_INTEGRAL_LIMIT;
      TDPID_Init(&motor->PID_Angle, 100.0f, 0.3f, 5.0f, 9000.0f, 50.0f, DJ_CTRL_PERIOD, 300.0f);
      PID_Init(&motor->PID_SpeedOfAngle, 5.0f, 0.0f, 0.0f, 16000.0f, 200.0f, DJ_CTRL_PERIOD);
      motor->PID_SpeedOfAngle.Improve |= PID_IMPROVE_INTEGRAL_LIMIT;
    }
  }
#endif

#if DJ_USE_IMPEDANCE
  if (method == IMPEDANCE_METHOD)
  {
    motor->dec = (Motor_Type == M2006) ? M2006_DEC : M3508_DEC;
  }
#endif

  motor->method       = (uint8_t)method;
  motor->initialized  = 1;
  motorCount++;
  motor->ID           = (uint16_t)(DJ_L_ID + Motor_ID);
  motor->setCurrent   = 0;
  motor->offset_angle = 0;
  motor->round_count  = 0;
  motor->setSpeed     = 0;
  motor->setAngle     = 0;
  motor->no_rx_count  = 0;
  motor->rx_once      = 0; /* 零点在首帧反馈时捕获(见 DJ_CAN_Callback) */
  Motors[Motor_ID - 1] = motor;

  /* 订阅该电机的反馈帧 */
  CAN_Bus_Subscribe(DJ_MOTOR_BUS, motor->ID, DJ_CAN_Callback);
}

/* ========================================================================== */
/*  控制主循环                                                                 */
/* ========================================================================== */

/**
  * @brief  大疆电机控制, 需要按 DJ_CTRL_PERIOD 周期循环执行
  */
void DJ_MotorRun(void)
{
  uint8_t data[8];

  for (uint8_t i = 0; i < 8; i++)
  {
    /* 跳过未初始化或未注册的电机 */
    if (Motors[i] == NULL || !Motors[i]->initialized)
      continue;

    if (Motors[i]->no_rx_count < 65535)
      Motors[i]->no_rx_count++;

    /* 反馈丢失保护: 超时则本拍电流归零并重置 PID 积分,
       避免断线重连后积分跑飞造成飞车 */
    if (Motors[i]->no_rx_count >= DJ_RX_TIMEOUT)
    {
      Motors[i]->setCurrent = 0;
#if DJ_USE_PID
      PID_Clear(&Motors[i]->PID_Speed);
      PID_Clear(&Motors[i]->PID_SpeedOfAngle);
      TDPID_Clear(&Motors[i]->PID_Angle);
#endif
      continue;
    }

#if DJ_USE_PID
    if (Motors[i]->method == PID_METHOD)
    {
      if (Motors[i]->mode == ANGLE_MODE || Motors[i]->mode == ANGLEINC_MODE) /* 角度控制 */
      {
        /* 串级 PID: 角度环(TD+PID) -> 速度环 -> 电流 */
        TDPID_Calc(&Motors[i]->PID_Angle, (float)Motors[i]->total_angle, Motors[i]->setAngle);
        PID_Calculate(&Motors[i]->PID_SpeedOfAngle, (float)Motors[i]->speed, Motors[i]->PID_Angle.out);
        Motors[i]->setCurrent = (int16_t)Motors[i]->PID_SpeedOfAngle.Output;
      }
      else if (Motors[i]->mode == SPEED_MODE) /* 速度控制 */
      {
        PID_Calculate(&Motors[i]->PID_Speed, (float)Motors[i]->speed, Motors[i]->setSpeed);
        Motors[i]->setCurrent = (int16_t)Motors[i]->PID_Speed.Output;
      }
    }
#endif

#if DJ_USE_IMPEDANCE
    if (Motors[i]->method == IMPEDANCE_METHOD)
    {
      Motors[i]->imp.dp = (float)(Motors[i]->setAngle - Motors[i]->total_angle);
      Motors[i]->imp.dv = (float)(Motors[i]->setSpeed - Motors[i]->speed);
      Impedance_Calc(&Motors[i]->imp);
      LIMIT_MIN_MAX(Motors[i]->imp.out, (float)MIN_CURRENT, (float)MAX_CURRENT);
      Motors[i]->setCurrent = (int16_t)Motors[i]->imp.out;
    }
#endif
  }

  /* 控制电机电流 - 低 4 个电机 (ID 1-4) */
  bool has_low = false;
  for (uint8_t i = 0; i < 4; i++)
  {
    int16_t current = 0;
    if (Motors[i] != NULL && Motors[i]->initialized)
    {
      current = Motors[i]->setCurrent;
      has_low = true;
    }
    data[i * 2]     = (uint8_t)(current >> 8);
    data[i * 2 + 1] = (uint8_t)current;
  }
  if (has_low)
  {
    CAN_Bus_Send(DJ_MOTOR_BUS, DJ_L_ID, data, 8);
  }

  /* 控制电机电流 - 高 4 个电机 (ID 5-8): 仅在存在 ID 5~8 时才发送, 避免浪费邮箱与总线带宽 */
  bool has_high = false;
  for (uint8_t i = 4; i < 8; i++)
  {
    int16_t current = 0;
    if (Motors[i] != NULL && Motors[i]->initialized)
    {
      current = Motors[i]->setCurrent;
      has_high = true;
    }
    data[(i - 4) * 2]     = (uint8_t)(current >> 8);
    data[(i - 4) * 2 + 1] = (uint8_t)current;
  }
  if (has_high)
  {
    CAN_Bus_Send(DJ_MOTOR_BUS, DJ_H_ID, data, 8);
  }
}

/* ========================================================================== */
/*  状态与设定接口                                                             */
/* ========================================================================== */

/* 电机角度和圈数清零 */
void DJ_ClearAngle(DJ_Motor_t *motor)
{
  /* total_angle 每帧由回调按 (round_count, angle, offset_angle) 重算,
     只清 total/round 会被下一帧冲掉; 必须把零点重设到当前编码器值。
     临界区保证与 CAN 回调不产生半更新状态 */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  motor->offset_angle = motor->angle;
  motor->last_angle   = motor->angle;
  motor->round_count  = 0;
  motor->total_angle  = 0.0f;
  __set_PRIMASK(primask);
}

#if DJ_USE_PID
/* 设置电机速度(单位: 未减速前的 r/min), 设置一次即可 */
void DJ_SetSpeed(DJ_Motor_t *motor, float speed)
{
  motor->setSpeed = speed;
  motor->mode     = SPEED_MODE;
}

/* 设置电机角度(单位: 度), speed 为速度上限, 设置一次即可 */
void DJ_SetAngle(DJ_Motor_t *motor, float angle, float speed)
{
  motor->setAngle     = angle;
  motor->PID_Angle.max_out = speed;
  motor->mode         = ANGLE_MODE;
}

/* 设置电机增量角度(单位: 度), 设置一次即可 */
void DJ_SetAngleInc(DJ_Motor_t *motor, float angleInc)
{
  motor->setAngle = motor->total_angle + angleInc;
  motor->mode     = ANGLEINC_MODE;
}
#endif /* DJ_USE_PID */

#if DJ_USE_IMPEDANCE
/* 阻抗模式速度控制 */
void DJ_SetImpSpeed(DJ_Motor_t *motor, float kd, float speed, int16_t current)
{
  motor->imp.Kp = 0;
  motor->imp.Kd = kd;
  motor->imp.t  = (float)current; /* 前馈电流进 imp.t; 写 setCurrent 会被 DJ_MotorRun 每拍覆盖 */
  motor->setAngle = 0;
  motor->setSpeed = speed;
}

/* 阻抗模式角度控制 */
void DJ_SetImpAngle(DJ_Motor_t *motor, float kp, float kd, float angle, int16_t t)
{
  motor->imp.Kp = kp;
  motor->imp.Kd = kd;
  motor->imp.t = t;
  motor->setAngle = angle;
  motor->setSpeed = 0;
}

/* 阻抗模式增量角度 */
void DJ_SetImpAngleInc(DJ_Motor_t *motor, float kp, float kd, float angle, int16_t t)
{
  motor->imp.Kp = kp;
  motor->imp.Kd = kd;
  motor->imp.t = t;
  motor->setAngle = motor->total_angle + angle;
  motor->setSpeed = 0;
}

/* 设置阻尼模式 */
void DJ_SetImpDamp(DJ_Motor_t *motor, float kd)
{
  motor->imp.Kp = 0;
  motor->imp.Kd = kd;
  motor->imp.t = 0;
  motor->setAngle = 0;
  motor->setSpeed = 0;
}

/* 设置零力矩模式 */
void DJ_SetImpZeroTorque(DJ_Motor_t *motor)
{
  motor->imp.Kp = 0;
  motor->imp.Kd = 0;
  motor->imp.t = 0; /* 零力矩即输出 0(原 150 会恒定输出非零电流, 与阻尼模式 DJ_SetImpDamp 的 t=0 相悖) */
  motor->setAngle = 0;
  motor->setSpeed = 0;
}
#endif /* DJ_USE_IMPEDANCE */
