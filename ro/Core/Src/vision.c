/**
  ******************************************************************************
  * @file    vision.c
  * @brief   OpenMV 视觉接收端：USART3 逐字节中断 + 状态机解帧
  *
  * 工作流程：
  *   1) vision_init() 打开 USART3 接收中断，并挂上第 1 个字节的接收；
  *   2) 每收到 1 个字节进一次 USART3 中断，HAL_UART_RxCpltCallback() 把它
  *      送给状态机解帧，然后立刻挂上下一个字节；
  *   3) 收到帧头 + 9 字节数据 + 校验都正确的一帧，就刷新全局目标信息；
  *   4) 超过 VISION_TIMEOUT_MS 没收到有效帧，目标自动判为丢失（status=0）。
  *
  * 设计约束：全程非阻塞、不占用 DMA、中断优先级低于 TIM6，
  *           因此主循环和 10ms 编码器 PID 的老代码都照常运行。
  ******************************************************************************
  */

#include "vision.h"
#include "usart.h"

#define VISION_H0          0xAAu   /* 帧头第 1 字节 */
#define VISION_H1          0x55u   /* 帧头第 2 字节 */
#define VISION_PAYLOAD_LEN 9u      /* 数据段长度：status + cx + cy + w + h */
#define VISION_STATUS_MAX  4u      /* 发送端定义的最大颜色编号 */

/* ---------------------------- 接收状态机 ---------------------------- */
static uint8_t s_rx_byte;                 /* 当前正在接收的那一个字节 */
static uint8_t s_payload[VISION_PAYLOAD_LEN];
static uint8_t s_index;
static uint8_t s_state;                   /* 0=找0xAA 1=找0x55 2=收数据 3=收校验 */

/* ---------------------------- 最新目标信息 ---------------------------- */
static volatile uint8_t  s_status;
static volatile uint8_t  s_online;
static volatile uint16_t s_cx;
static volatile uint16_t s_cy;
static volatile uint16_t s_w;
static volatile uint16_t s_h;
static volatile uint32_t s_last_tick;
static volatile uint32_t s_frames;
static volatile uint32_t s_errors;

/* ---------------------------- 内部函数 ---------------------------- */

/* 与发送端 xor_checksum() 完全一致：9 字节逐个异或 */
static uint8_t vision_xor(const uint8_t *data, uint8_t len)
{
  uint8_t chk = 0u;
  uint8_t i;

  for (i = 0u; i < len; i++)
  {
    chk ^= data[i];
  }
  return chk;
}

/* 超时检测：所有对外读取接口都会先调用它，保证拿到的都是“新鲜”数据 */
static void vision_check_timeout(void)
{
  if ((s_online != 0u) &&
      ((uint32_t)(HAL_GetTick() - s_last_tick) > VISION_TIMEOUT_MS))
  {
    s_online = 0u;
    s_status = 0u;
  }
}

/* 喂一个字节进来，走完整个解帧流程（只在串口中断里调用） */
static void vision_feed(uint8_t byte)
{
  switch (s_state)
  {
    case 0u:                                   /* 找帧头 0xAA */
      if (byte == VISION_H0)
      {
        s_state = 1u;
      }
      break;

    case 1u:                                   /* 找帧头 0x55 */
      if (byte == VISION_H1)
      {
        s_index = 0u;
        s_state = 2u;
      }
      else if (byte == VISION_H0)              /* 连续 0xAA，继续等 0x55 */
      {
        s_state = 1u;
      }
      else
      {
        s_state = 0u;
      }
      break;

    case 2u:                                   /* 收 9 字节数据段 */
      s_payload[s_index] = byte;
      s_index++;
      if (s_index >= VISION_PAYLOAD_LEN)
      {
        s_state = 3u;
      }
      break;

    default:                                   /* 收校验字节并校验 */
      if (byte == vision_xor(s_payload, VISION_PAYLOAD_LEN))
      {
        uint8_t status = s_payload[0];

        if (status > VISION_STATUS_MAX)        /* 未知颜色一律当无目标 */
        {
          status = 0u;
        }

        s_status = status;
        s_cx = (uint16_t)s_payload[1] | ((uint16_t)s_payload[2] << 8);
        s_cy = (uint16_t)s_payload[3] | ((uint16_t)s_payload[4] << 8);
        s_w  = (uint16_t)s_payload[5] | ((uint16_t)s_payload[6] << 8);
        s_h  = (uint16_t)s_payload[7] | ((uint16_t)s_payload[8] << 8);
        s_last_tick = HAL_GetTick();
        s_online = 1u;
        s_frames++;
        s_state = 0u;                          /* 一帧收完，回到找帧头 */
      }
      else
      {
        s_errors++;
        /* 校验失败：这个字节本身可能就是下一帧的 0xAA，直接拿它重新同步 */
        s_state = (byte == VISION_H0) ? 1u : 0u;
      }
      break;
  }
}

/* ---------------------------- 对外接口 ---------------------------- */

void vision_init(void)
{
  s_rx_byte = 0u;
  s_index   = 0u;
  s_state   = 0u;

  s_status    = 0u;
  s_online    = 0u;
  s_cx        = 0u;
  s_cy        = 0u;
  s_w         = 0u;
  s_h         = 0u;
  s_frames    = 0u;
  s_errors    = 0u;
  s_last_tick = HAL_GetTick();

  /* TIM6 是优先级 0 的最高优先级，这里用 1，视觉收帧不会影响 10ms 控制周期 */
  HAL_NVIC_SetPriority(VISION_UART_IRQn, VISION_UART_IRQ_PRIO, 0u);
  HAL_NVIC_EnableIRQ(VISION_UART_IRQn);

  /* 先挂 1 个字节，之后每进一次接收完成回调就再挂 1 个字节 */
  (void)HAL_UART_Receive_IT(&VISION_UART, &s_rx_byte, 1u);
}

void vision_clear(void)
{
  s_status = 0u;
  s_online = 0u;
  s_cx = 0u;
  s_cy = 0u;
  s_w  = 0u;
  s_h  = 0u;
}

void vision_get(VisionFrame *out)
{
  uint32_t primask;

  if (out == 0)
  {
    return;
  }

  vision_check_timeout();

  /* 关一下中断再整体拷贝，避免几个字段分别来自前后两帧 */
  primask = __get_PRIMASK();
  __disable_irq();
  out->status = s_status;
  out->online = s_online;
  out->cx     = s_cx;
  out->cy     = s_cy;
  out->w      = s_w;
  out->h      = s_h;
  out->frames = s_frames;
  out->errors = s_errors;
  if (primask == 0u)
  {
    __enable_irq();
  }
}

uint8_t vision_online(void)
{
  vision_check_timeout();
  return s_online;
}

uint8_t vision_status(void)
{
  vision_check_timeout();
  return s_status;
}

uint16_t vision_cx(void)
{
  return s_cx;
}

uint16_t vision_cy(void)
{
  return s_cy;
}

uint16_t vision_w(void)
{
  return s_w;
}

uint16_t vision_h(void)
{
  return s_h;
}

float vision_error_x(void)
{
  vision_check_timeout();
  if ((s_online == 0u) || (s_status == 0u))
  {
    return 0.0f;
  }
  return (float)s_cx - (float)(VISION_IMAGE_WIDTH / 2);
}

uint8_t vision_update_targets(float base_speed, float kp_turn,
                              float *target1, float *target2)
{
  float error;
  float turn;

  if ((target1 == 0) || (target2 == 0))
  {
    return 0u;
  }

  vision_check_timeout();

  /* 没目标或串口失联：左右轮都回到基础速度直行。
     这里不能沿用上一帧的差速，否则会一直原地转圈。 */
  if ((s_online == 0u) || (s_status == 0u))
  {
    *target1 = base_speed;
    *target2 = base_speed;
    return 0u;
  }

  error = (float)s_cx - (float)(VISION_IMAGE_WIDTH / 2);
  if ((error < VISION_DEADBAND_PX) && (error > -VISION_DEADBAND_PX))
  {
    error = 0.0f;
  }

  /* 目标偏右(cx>160) -> turn>0 -> 左轮加快、右轮放慢 -> 车向右转 */
  turn = kp_turn * error * VISION_TURN_SIGN;
  if (turn > VISION_MAX_TURN)
  {
    turn = VISION_MAX_TURN;
  }
  else if (turn < -VISION_MAX_TURN)
  {
    turn = -VISION_MAX_TURN;
  }

  *target1 = base_speed + turn;
  *target2 = base_speed - turn;

  if (*target1 > VISION_MAX_TARGET)  *target1 = VISION_MAX_TARGET;
  if (*target1 < -VISION_MAX_TARGET) *target1 = -VISION_MAX_TARGET;
  if (*target2 > VISION_MAX_TARGET)  *target2 = VISION_MAX_TARGET;
  if (*target2 < -VISION_MAX_TARGET) *target2 = -VISION_MAX_TARGET;

  return 1u;
}

/* ------------------------- HAL 回调（视觉收帧） ------------------------- */

/* 每收到 1 个字节进一次：解帧 + 立刻挂上下一个字节 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == VISION_UART_INSTANCE)
  {
    vision_feed(s_rx_byte);
    (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1u);
  }
}

/* 溢出/噪声等错误：清标志并重新挂接收，否则会再也收不到数据 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == VISION_UART_INSTANCE)
  {
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0u)
    {
      __HAL_UART_CLEAR_OREFLAG(huart);
    }
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1u);
  }
}
