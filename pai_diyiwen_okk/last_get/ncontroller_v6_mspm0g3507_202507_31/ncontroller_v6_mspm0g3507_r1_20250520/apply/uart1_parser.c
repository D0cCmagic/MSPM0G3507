/***********************************************************************
 * UART1 树莓派 ASCII 协议解析器 实现
 *
 * 协议格式: a<带符号整数X>b<带符号整数Y>c
 *   示例帧:  "a-15b23c"
 *            │ │  │ ││ │
 *            │ └──┴─┘││ └─ 帧尾 'c'：提交 values
 *            │   X=-15│└─── 分隔符 'b'：X 结束，开始 Y
 *            │        └──── Y=23
 *            └──────────── 帧头 'a'：开始新帧
 *
 * 可靠性改进（相比原 parse_char）：
 *   1) 100ms 超时自动复位 —— 收到 'a' 或 'b' 后超时未完成则丢弃帧
 *   2) 非法字符复位 —— 状态机中收到非法字符立即回到 IDLE
 *   3) VisionData.valid 标志 —— 明确标记数据是否有���
 *   4) 非 ISR 上下文运行 —— 在 200Hz 任务中执行，不阻塞中断
 *
 * 作者：重构自无名创新 ncontroller_v6
 * 日期：2025-05-20
 ***********************************************************************/

#include "headfile.h"
#include "uart1_dma.h"
#include "uart1_parser.h"
#include "system.h"              /* 提供 millis() 用于超时检测 */

/***********************************************************************
 * 全局视觉数据实例
 * 由解析器（200Hz 任务）写入，由步进电机控制函数（主循环）读取。
 * 单写者单读者模型，无需锁保护（int16_t 在 M0+ 上是原子读写的）。
 ***********************************************************************/
static VisionData g_vision_data;

/***********************************************************************
 * 解析器内部状态变量（全部 static，模块外不可见）
 ***********************************************************************/
static uart1_parser_state_t g_state            = PARSE_IDLE; /* 当前解析状态      */
static int16_t              g_value_x           = 0;          /* 解析完成的 X 值   */
static int16_t              g_value_y           = 0;          /* 解析完成的 Y 值   */
static int16_t              g_value_accum       = 0;          /* 十进制累加器      */
static int8_t               g_sign              = 1;          /* 当前符号（+1/-1） */
static uint32_t             g_last_byte_time_ms = 0;          /* 上次收到字节的时间 */

/***********************************************************************
 * 函数名: void uart1_parser_init(void)
 * 说明:   将所有解析器内部状态重置为初始值。
 *         包括状态机状态、累加器、符号、时间戳和输出数据。
 * 入口:   无
 * 出口:   无
 ***********************************************************************/
void uart1_parser_init(void)
{
    g_state              = PARSE_IDLE;
    g_value_x            = 0;
    g_value_y            = 0;
    g_value_accum        = 0;
    g_sign               = 1;
    g_last_byte_time_ms  = 0;
    g_vision_data.x      = 0;
    g_vision_data.y      = 0;
    g_vision_data.timestamp_ms = 0;
    g_vision_data.valid  = false;
}

/***********************************************************************
 * 函数名: static void uart1_parser_process_byte(uint8_t c)
 * 说明:   逐字节状态机——协议解析的核心。
 *
 *         状态转移图:
 *         ┌─────────────────────────────────────┐
 *         │              超时/非法字符           │
 *         │  ┌──────────────────────────────┐   │
 *         ▼  │                              │   │
 *       PARSE_IDLE ──'a'──→ PARSE_X ──'b'──→ PARSE_Y ──'c'──→ 提交数据
 *         ▲                  │  │             │  │
 *         │               '0-9'/'-'        '0-9'/'-'
 *         │               (累加X)          (累加Y)
 *         │                                  │
 *         └────────── 非法字符 ──────────────┘
 *
 *         超时保护:
 *         当状态为 PARSE_X 或 PARSE_Y 时，如果距离上次收到字节超过
 *         UART1_PARSER_TIMEOUT_MS（100ms），状态机自动复位到 PARSE_IDLE，
 *         丢弃不完整帧。防止树莓派断电或通信中断导致永久卡死。
 *
 * 入口:   c - 当前接收到的字节
 * 出口:   无（结果通过全局 g_vision_data 传递）
 ***********************************************************************/
static void uart1_parser_process_byte(uint8_t c)
{
    uint32_t now = millis();  /* 获取系统毫秒时间戳 */

    /********************************************************************
     * 超时检测
     * 仅在非 IDLE 状态下检测：如果距离上次收到字节超过限定时间，
     * 说明帧传输中断（树莓派掉电、线缆松动、数据干扰等），
     * 立即丢弃半帧回到 IDLE，等待下一个 'a' 帧头。
     ********************************************************************/
    if (g_state != PARSE_IDLE) {
        if ((now - g_last_byte_time_ms) > UART1_PARSER_TIMEOUT_MS) {
            g_state       = PARSE_IDLE;
            g_value_accum = 0;
            g_sign        = 1;
        }
    }
    g_last_byte_time_ms = now;  /* 更新最后收到字节的时间 */

    switch (g_state) {

    /********************************************************************
     * PARSE_IDLE —— 空闲状态，等待帧头 'a'
     * 收到 'a' → 进入 PARSE_X，准备解析 X 值
     * 收到其他 → 忽略（利用 'a' 作为帧对齐锚点，自动跳过垃圾数据）
     ********************************************************************/
    case PARSE_IDLE:
        if (c == 'a') {
            g_state       = PARSE_X;
            g_value_accum = 0;
            g_sign        = 1;
        }
        break;

    /********************************************************************
     * PARSE_X —— 解析第一个带符号整数值（X 偏移量）
     * '-'  → 设置符号为负
     * '0-9' → 十进制累加（支持任意位数）
     * 'b'  → X 值解析完成（value_accum * sign），进入 PARSE_Y
     * 其他  → 非法字符，复位到 IDLE
     ********************************************************************/
    case PARSE_X:
        if (c == '-') {
            g_sign = -1;                             /* 负号标记 */
        } else if (c >= '0' && c <= '9') {
            g_value_accum = g_value_accum * 10 + (c - '0');  /* 十进制累加 */
        } else if (c == 'b') {
            g_value_x      = g_value_accum * g_sign; /* 应用符号，X 完成 */
            g_state        = PARSE_Y;                /* 切换到解析 Y */
            g_value_accum  = 0;
            g_sign         = 1;
        } else {
            /* 非法字符（如控制字符、非 ASCII 等），丢弃当前帧 */
            g_state       = PARSE_IDLE;
            g_value_accum = 0;
            g_sign        = 1;
        }
        break;

    /********************************************************************
     * PARSE_Y —— 解析第二个带符号整数值（Y 偏移量）
     * '-'  → 设置符号为负
     * '0-9' → 十进制累加
     * 'c'  → Y 值解析完成 → 帧完整 → 提交到 VisionData → 回到 IDLE
     * 其他  → 非法字符，复位到 IDLE
     ********************************************************************/
    case PARSE_Y:
        if (c == '-') {
            g_sign = -1;
        } else if (c >= '0' && c <= '9') {
            g_value_accum = g_value_accum * 10 + (c - '0');
        } else if (c == 'c') {
            /* 帧完整！提交解析结果 */
            g_value_y                 = g_value_accum * g_sign;
            g_vision_data.x           = g_value_x;
            g_vision_data.y           = g_value_y;
            g_vision_data.timestamp_ms = now;
            g_vision_data.valid       = true;         /* 标记数据有效 */
            g_state                   = PARSE_IDLE;   /* 回到空闲，等待下一帧 */
        } else {
            /* 非法字符，丢弃当前帧 */
            g_state       = PARSE_IDLE;
            g_value_accum = 0;
            g_sign        = 1;
        }
        break;
    }
}

/***********************************************************************
 * 函数名: void uart1_parser_feed(void)
 * 说明:   从环形缓冲区批量取字节喂入状态机。
 *         在 200Hz 任务中调用，紧跟 uart1_dma_process() 之后。
 *         while 循环一次性消费所有可用字节，无阻塞（空缓冲立即返回）。
 * 入口:   无
 * 出口:   无
 ***********************************************************************/
void uart1_parser_feed(void)
{
    uint8_t byte;
    /* 批量消费环形缓冲区中所有可用字节 */
    while (uart1_dma_get_byte(&byte)) {
        uart1_parser_process_byte(byte);
    }
}

/***********************************************************************
 * 函数名: const VisionData* uart1_parser_get_data(void)
 * 说明:   返回解析出的视觉数据只读指针。
 *         调用方（step_motor_ctrl_dianshe / daba）通过此指针读取
 *         靶心像素偏移量，替代原有 values[] 直接访问。
 * 入口:   无
 * 出口:   指向 VisionData 的只读指针
 * 备注:   返回的指针指向模块内部静态变量，始终有效，无需释放。
 *         通过 valid 标志判断数据是否可用：
 *         - valid=true  → 使用 x/y 驱动步进电机 PID
 *         - valid=false → x=0, y=0，保持云台当前位置不动
 ***********************************************************************/
const VisionData *uart1_parser_get_data(void)
{
    return &g_vision_data;
}
