#include "headfile.h"
#include "step_motor.h"

// 计算公式：period = (时钟源频率 / (预分频系数 * 目标频率)) - 1
// 其中：时钟源 = 40MHz, 预分频系数 = 128
#define CLK_FREQ     40000000  // 40MHz
#define PRESCALE     128       // 固定预分频

uint32_t calculate_period(uint32_t target_freq) 
{
    // 计算周期值（24位定时器最大值为0xFFFFFF）
    uint32_t period = (CLK_FREQ / (PRESCALE * target_freq)) - 1;
    
    // 边界保护（period范围0~16777215）
    if (period > 0xFFFFFF) period = 0xFFFFFF;
    if (period < 1) period = 1;  // 最小周期值
    
    return period;
}
void set_freq_pwmX(uint32_t freq) {
    // 1. 计算新周期值
    uint32_t new_period = calculate_period(freq);
    
    // 2. 更新定时器周期（立即生效）
    TIMER_setPeriod(TIMER_A1_BASE, new_period);
    
    // 3. 可选：保持原占空比比例
    // 获取原比较值并自动缩放
    uint32_t old_cc = TIMER_getCompareValue(TIMER_A1_BASE, TIMER_A);
    uint32_t new_cc = (old_cc * new_period) / 100;  // 假设原period=100
    TIMER_setCompareValue(TIMER_A1_BASE, TsIMER_A, new_cc);
}



void set_freq_pwmY(uint32_t freq) 
{
    // 1. 计算新周期值
    uint32_t new_period = calculate_period(freq);
    
    // 2. 更新定时器周期
    TIMER_setPeriod(TIMER_G7_BASE, new_period);
    
    // 3. 可选：保持原占空比比例
    uint32_t old_cc = TIMER_getCompareValue(TIMER_G7_BASE, TIMER_A);
    uint32_t new_cc = (old_cc * new_period) / 100;
    TIMER_setCompareValue(TIMER_G7_BASE, TIMER_A, new_cc);
}