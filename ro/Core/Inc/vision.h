#ifndef __VISION_H
#define __VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ===========================================================================
 * OpenMV 视觉接收端（接收端代码）
 * 对应发送端脚本：color_track_uart_omv_multicolor_noRed.py
 *
 * 接线：OpenMV TX  -> STM32 PB11 (USART3_RX)   <- 接收端只用到这一根
 *       OpenMV RX  <- STM32 PB10 (USART3_TX)      可不接
 *       GND        <-> GND                        必须共地
 *       波特率 115200，8 数据位，无校验，1 停止位
 *
 * 帧格式（每帧固定 12 字节，小端；发送端 struct.pack("<BHHHH", ...)）：
 *   [0]  0xAA        帧头 1
 *   [1]  0x55        帧头 2
 *   [2]  status      0=无目标 1=黄 2=蓝 3=绿 4=紫
 *   [3]  cx 低字节   目标中心 X（QVGA 下 0~319，表示左右）
 *   [4]  cx 高字节
 *   [5]  cy 低字节   目标中心 Y
 *   [6]  cy 高字节
 *   [7]  w  低字节   目标框宽
 *   [8]  w  高字节
 *   [9]  h  低字节   目标框高
 *   [10] h  高字节
 *   [11] checksum    [2]~[10] 共 9 字节逐个 XOR
 * =========================================================================*/

/* ------------------------------ 可调参数 ------------------------------ */

/* 发送端分辨率（用于把 cx 换算成“偏左/偏右”的像素偏差） */
#define VISION_IMAGE_WIDTH      320
#define VISION_IMAGE_HEIGHT     240

/* 超过这么久没收到有效帧，就认为目标丢失（自动清 status） */
#define VISION_TIMEOUT_MS       500u

/* 左右偏差死区：偏差小于这个像素数就当正对目标，免得车原地抖 */
#define VISION_DEADBAND_PX      10.0f

/* 转向方向：装完发现“越偏越转反了”就把这里改成 -1.0f */
#define VISION_TURN_SIGN        1.0f

/* 控制参数（只在调用 vision_update_targets() 时生效） */
#define VISION_BASE_SPEED       120.0f   /* 直行基础目标速度，单位=编码器计数/10ms */
#define VISION_KP_TURN          0.4f     /* 差速增益：turn = kp * (cx - 160) */
#define VISION_MAX_TURN         60.0f    /* 差速限幅 */
#define VISION_MAX_TARGET       1000.0f  /* 左右轮目标速度限幅，与 PWM 限幅保持一致 */

/* USART3 中断优先级：必须低于 TIM6(优先级 0)，保证 10ms 控制周期不被视觉打断 */
#define VISION_UART_IRQ_PRIO    1u

/* 使用的串口：默认 USART3(PB10/PB11)；若改接到 USART1(PA9/PA10) 就改这三行，
   同时把 stm32f4xx_it.c 里的 USART3_IRQHandler 换成 USART1_IRQHandler */
#define VISION_UART             huart3
#define VISION_UART_INSTANCE    USART3
#define VISION_UART_IRQn        USART3_IRQn

/* ------------------------------ 数据结构 ------------------------------ */

typedef struct
{
  uint8_t  status;   /* 0=无目标 1=黄 2=蓝 3=绿 4=紫 */
  uint8_t  online;   /* 1=在超时时间内收到过有效帧 */
  uint16_t cx;       /* 目标中心 X（左右） */
  uint16_t cy;       /* 目标中心 Y */
  uint16_t w;        /* 目标框宽 */
  uint16_t h;        /* 目标框高 */
  uint32_t frames;   /* 累计收到的有效帧数 */
  uint32_t errors;   /* 累计校验/同步失败次数（排查线路问题用） */
} VisionFrame;

/* ------------------------------ 对外接口 ------------------------------ */

/* 初始化：打开 USART3 接收中断并挂上第一个字节的接收。放在 USER CODE BEGIN 2 里调用 */
void     vision_init(void);

/* 取一份最新目标信息的快照（可随时调用，非阻塞） */
void     vision_get(VisionFrame *out);

/* 状态查询 */
uint8_t  vision_online(void);    /* 1=目标在线 */
uint8_t  vision_status(void);    /* 0=无目标 1=黄 2=蓝 3=绿 4=紫 */
uint16_t vision_cx(void);
uint16_t vision_cy(void);
uint16_t vision_w(void);
uint16_t vision_h(void);

/* 左右偏差：目标中心 - 画面中心（正=目标在右边）。无目标/失联时返回 0 */
float    vision_error_x(void);

/* 把视觉偏差换算成左右轮目标速度：
 *   target1 = base + turn，target2 = base - turn，turn = kp * (cx-160) * VISION_TURN_SIGN
 * 有目标返回 1；无目标/失联时把两轮都设成 base（回正直行）并返回 0。
 * 在 TIM6 的 10ms 中断里调用即可，主循环不用改。
 */
uint8_t  vision_update_targets(float base_speed, float kp_turn,
                               float *target1, float *target2);

/* 清空缓存的目标信息（调试用） */
void     vision_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_H */
