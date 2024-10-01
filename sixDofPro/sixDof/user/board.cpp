#include "board.h"

/***宏定义***/
// TICK_PER_SECOND：每秒SysTick定时器中断次数
#define TICK_PER_SECOND 1000
#define TICK_US (1000000 / TICK_PER_SECOND)

/***全局变量***/
// 滴答定时器计数变量 ,49天后溢出
volatile uint32_t sysTickUptime = 0;
volatile uint32_t sysTickUptimeUs = 0;


/**
 * @brief 初始化滴答定时器
 * @note 如果修改外部晶振,记得修改 HSE_VALUE，PLL_M
 */
void sysTickInit(void)
{
    // 滴答定时器初始化
    SysTick_Config(SystemCoreClock / TICK_PER_SECOND);
    // 滴答定时器优先级初始化
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
}


/**
 * @brief 获取us级时间
 * @return    单片机从运行到此时的时间，单位us
 * @note 滴答定时器分频在此函数处理好像会损失精度
 */
uint64_t getSysTimeUs(void)
{
    uint64_t ms;
    uint64_t value;
    ms = sysTickUptime;
    value = (ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD);
    return value;
}


/**
 * @brief US级延时
 * @param us 延时时长，单位US
 * @note 使用滴答延时会造成极大的开销，时间越长，开销越大，尽量在1毫秒以内
 * @warning 此函数在会算入CPU_usage中
 */
void delayUs(uint32_t us)
{
    uint64_t from = getSysTimeUs();
    while (getSysTimeUs() < from + us)
    {
    }
}


/**
 * @brief MS级延时
 * @param ms 延时时长，单位MS
 * @return
 * @note 注意在操作系统运行下使用此函数务必进入临界区，否则精度不保证
 */
void delayMs(uint32_t ms)
{
    delayUs(ms * 1000);
}


/**
  * @brief 软件复位：单片机重启
  */
void softwareReset()
{
    __set_FAULTMASK(1); // 关闭所有中断
    NVIC_SystemReset(); // 系统复位
}
/***********End of file*****************  */
