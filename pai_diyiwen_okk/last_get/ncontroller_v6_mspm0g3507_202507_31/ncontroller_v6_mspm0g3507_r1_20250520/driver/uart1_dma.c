/***********************************************************************
 * UART1 DMA + 环形缓冲区 驱动实现
 *
 * 架构说明：
 *   UART1 RX 硬件 → DMA CH1（REPEAT_SINGLE 模式，每字节触发一次传输）
 *       ↓          DMA 自动将接收字节写入 uart1_dma_buf[]
 *   DMA 硬件缓冲区   256 字节线性缓冲，DMA 目标地址自动递增
 *       ↓          200Hz 任务调用 uart1_dma_process() 搬移数据
 *   环形缓冲区      256 字节，head=写指针 tail=读指针 count=可用数
 *       ↓          协议解析器通过 uart1_dma_get_byte() 逐字节消费
 *   协议状态机      树莓派 ASCII 协议 "a<X>b<Y>c" → VisionData
 *
 * 作者：重构自无名创新 ncontroller_v6
 * 日期：2025-05-20
 ***********************************************************************/

#include "ti_msp_dl_config.h"
#include "headfile.h"
#include "uart1_dma.h"
#include "system.h"

/***********************************************************************
 * DMA 硬件缓冲区
 * DMA CH1 将 UART1 接收的每个字节直接写入此数组。
 * REPEAT_SINGLE 模式：传输计数归零后自动重载，但目标地址不自动回绕，
 * 因此 uart1_dma_process() 每 5ms 重置一次目标地址。
 ***********************************************************************/
static volatile uint8_t uart1_dma_buf[UART1_DMA_BUF_SIZE];

/***********************************************************************
 * 软件环形缓冲区
 * 解耦 DMA（生产者）与协议解析器（消费者）。
 * head = 写入位置（DMA → rb 搬移时递增）
 * tail = 读取位置（get_byte 消费时递增）
 * count = 缓冲区中可读字节数
 * 当 head 追上 tail 时（缓冲区满），丢弃最旧的字节（tail 前移）。
 ***********************************************************************/
static uint8_t           uart1_rb_buf[UART1_DMA_BUF_SIZE];
static volatile uint16_t uart1_rb_head  = 0;   /* 写指针（生产者） */
static volatile uint16_t uart1_rb_tail  = 0;   /* 读指针（消费者） */
static volatile uint16_t uart1_rb_count = 0;   /* 当前可读字节数   */

/***********************************************************************
 * DMA CH1 配置结构体
 *
 * REPEAT_SINGLE_TRANSFER_MODE：
 *   每次 UART RX 事件触发一个字节的 DMA 传输。
 *   传输计数（DMASZ）归零后自动重载，确保 DMA 持续运行。
 *
 * srcIncrement = UNCHANGED：
 *   源地址（UART RXDATA 寄存器）是固定的硬件寄存器。
 *
 * destIncrement = INCREMENT：
 *   目标地址（uart1_dma_buf[]）每传输一字节后递增。
 ***********************************************************************/
static const DL_DMA_Config gUART1_DMA_CH1Config = {
    .transferMode  = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE, /* 每次触发传一字节，自动重载计数 */
    .extendedMode  = DL_DMA_NORMAL_MODE,       /* 普通模式，不使用 Gather/Table */
    .destIncrement = DL_DMA_ADDR_INCREMENT,    /* 目标地址递增（写入缓冲区下一位置） */
    .srcIncrement  = DL_DMA_ADDR_UNCHANGED,    /* 源地址不变（始终读 UART RXDATA 寄存器） */
    .destWidth     = DL_DMA_WIDTH_BYTE,        /* 目标数据宽度 = 1 字节 */
    .srcWidth      = DL_DMA_WIDTH_BYTE,        /* 源数据宽度 = 1 字节 */
    .trigger       = UART1_DMA_TRIGGER,        /* 触发源 = UART1 RX (19) */
    .triggerType   = DL_DMA_TRIGGER_TYPE_EXTERNAL, /* 外部硬件触发 */
};

/***********************************************************************
 * 函数名: void uart1_dma_init(void)
 * 说明:   一次性初始化 UART1 DMA 接收通道。
 *         1) 配置 DMA CH1 为 REPEAT_SINGLE 模式
 *         2) 设置源地址 = UART1 RXDATA 寄存器（固定）
 *         3) 设置目标地址 = uart1_dma_buf（递增）
 *         4) 使能 UART1 DMA 接收事件（替代 CPU RX 中断）
 *         5) 初始化环形缓冲区
 *         6) 使能 DMA 通道，开始接收
 * 入口:   无
 * 出口:   无
 * 备注:   调用后 UART1 的中断方式接收被 DMA 完全替代
 ***********************************************************************/
void uart1_dma_init(void)
{
    /* 初始化 DMA 通道 */
    DL_DMA_initChannel(DMA, UART1_DMA_CHANNEL,
                       (DL_DMA_Config *)&gUART1_DMA_CH1Config);

    /* 源地址：UART1 接收数据寄存器（固定，每个 RX 字节从此处读取） */
    DL_DMA_setSrcAddr(DMA, UART1_DMA_CHANNEL,
                      (uint32_t)&UART_1_INST->RXDATA);

    /* 目标地址：DMA 硬件缓冲区起始位置 */
    DL_DMA_setDestAddr(DMA, UART1_DMA_CHANNEL,
                       (uint32_t)uart1_dma_buf);

    /* 传输大小：缓冲区大小（256 字节后触发自动重载） */
    DL_DMA_setTransferSize(DMA, UART1_DMA_CHANNEL,
                           UART1_DMA_BUF_SIZE);

    /* 使能 UART1 DMA 接收事件（DMA_TRIG_RX.IMASK = RXINT） */
    DL_UART_enableDMAReceiveEvent(UART_1_INST,
                                  DL_UART_DMA_INTERRUPT_RX);

    /* 初始化环形缓冲区指针 */
    uart1_rb_head  = 0;
    uart1_rb_tail  = 0;
    uart1_rb_count = 0;

    /* 使能 DMA 通道，开始接收数据 */
    DL_DMA_enableChannel(DMA, UART1_DMA_CHANNEL);
}

/***********************************************************************
 * 函数名: void uart1_dma_process(void)
 * 说明:   每 5ms（200Hz）调用一次。处理流程：
 *         1) 禁用 DMA 通道 → 冻结传输状态
 *         2) 读取 DMA 剩余传输计数 → 计算新接收的字节数
 *         3) 将新字节逐个拷入环形缓冲区（满则覆盖最旧数据）
 *         4) 重置 DMA 目标地址和传输计数 → 从头开始写入
 *         5) 重新使能 DMA 通道
 *
 *         为什么需要重置 DMA 目标地址？
 *         REPEAT_SINGLE 模式会重载传输计数，但目标地址不会自动回绕。
 *         如果不重置，DMA 会持续写入 buf[256], buf[257]... 直到溢出。
 *         每 5ms 重置一次，保证 DMA 始终在 buf[0..255] 范围内写入。
 *
 * 入口:   无
 * 出口:   无
 * 备注:   200Hz 定时器中断上下文中执行，应保持短小。
 *         DMA 禁用窗口 < 10us，远小于 115200bps 单字节 87us 传输时间。
 ***********************************************************************/
void uart1_dma_process(void)
{
    uint16_t remaining;          /* DMA 剩余传输计数 */
    uint16_t bytes_available;    /* 本周期新接收的字节数 */
    uint16_t i;

    /* 1) 短暂禁用 DMA，冻结当前传输状态 */
    DL_DMA_disableChannel(DMA, UART1_DMA_CHANNEL);

    /* 2) 计算新接收的字节数 */
    remaining = DL_DMA_getTransferSize(DMA, UART1_DMA_CHANNEL);
    bytes_available = UART1_DMA_BUF_SIZE - remaining;

    /* 3) 将新字节拷入环形缓冲区 */
    for (i = 0; i < bytes_available; i++) {
        uint16_t next_head = uart1_rb_head + 1;

        /* 写指针回绕 */
        if (next_head >= UART1_DMA_BUF_SIZE)
            next_head = 0;

        /* 环形缓冲区满：丢弃最旧字节（tail 前移），覆盖写入 */
        if (next_head == uart1_rb_tail) {
            uart1_rb_tail++;
            if (uart1_rb_tail >= UART1_DMA_BUF_SIZE)
                uart1_rb_tail = 0;
        }

        uart1_rb_buf[uart1_rb_head] = uart1_dma_buf[i];
        uart1_rb_head = next_head;
        uart1_rb_count++;
    }

    /* 4) 重置 DMA 目标地址和传输计数，准备下一周期 */
    DL_DMA_setDestAddr(DMA, UART1_DMA_CHANNEL,
                       (uint32_t)uart1_dma_buf);
    DL_DMA_setTransferSize(DMA, UART1_DMA_CHANNEL,
                           UART1_DMA_BUF_SIZE);

    /* 5) 重新使能 DMA */
    DL_DMA_enableChannel(DMA, UART1_DMA_CHANNEL);
}

/***********************************************************************
 * 函数名: int uart1_dma_get_byte(uint8_t *byte)
 * 说明:   从环形缓冲区非阻塞取一字节。
 *         使用临界区保护（__disable_irq / __enable_irq）：
 *         - 生产者（uart1_dma_process）在 200Hz 定时器 ISR 中运行
 *         - 消费者（本函数）在 200Hz ISR 或主循环中运行
 *         - 临界区确保 head/tail/count 的读写原子性
 * 入口:   byte - 指向接收字节的指针
 * 出口:   1 = 成功读取一字节，0 = 缓冲区空（无数据）
 * 备注:   在 Cortex-M0+ 上，volatile uint16_t 的读写是硬件原子的，
 *         但 head 递增（读-改-写）需要临界区保护。
 ***********************************************************************/
int uart1_dma_get_byte(uint8_t *byte)
{
    int result = 0;
    uint32_t primask = __get_PRIMASK();   /* 保存当前中断屏蔽状态 */

    __disable_irq();                       /* 进入临界区 */

    if (uart1_rb_count > 0) {
        *byte = uart1_rb_buf[uart1_rb_tail];   /* 读取一字节 */
        uart1_rb_tail++;                       /* 读指针前移 */
        if (uart1_rb_tail >= UART1_DMA_BUF_SIZE)
            uart1_rb_tail = 0;                 /* 读指针回绕 */
        uart1_rb_count--;                      /* 可用计数减一 */
        result = 1;
    }

    /* 仅在调用前中断已使能的情况下恢复中断 */
    if (!(primask & 0x1))
        __enable_irq();

    return result;
}

/***********************************************************************
 * 函数名: int uart1_dma_available(void)
 * 说明:   返回环形缓冲区当前可读字节数。
 *         用于调试诊断，判断是否有数据积压。
 * 入口:   无
 * 出口:   缓冲区中待读取的字节数（0 = 空，256 = 满）
 ***********************************************************************/
int uart1_dma_available(void)
{
    return uart1_rb_count;
}
