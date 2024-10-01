/**
 * @file led.cpp
 * @author 梁文生
 * @brief
 * @version 0.1
 * @date 2021-10-09
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "led.h"

/**********************************
 * @class LED
 * @brief 对GPIO控制的LED封装，支持ID显示等
 _             _
| |    ___  __| |
| |   / _ \/ _` |
| |__|  __/ (_| |
|_____\___|\__,_|
***********************************/

/**
 * @brief Led io初始化
 **/
void Led::init(uint16_t cycleT)
{
    this->cycleT = cycleT;

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

    /**/
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

    /**/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Led::ctrl(uint8_t state)
{
    // 根据颜色,控制闪烁状态
    if (state) // 根据颜色亮灯
        GPIOC->BSRR = (uint32_t)LL_GPIO_PIN_13;
    else // 全灭
        GPIOC->BRR = (uint32_t)LL_GPIO_PIN_13;
}

/**
 * @brief led id展示
 * @param color 颜色
 * @param id LED闪烁次数
 * @note 状态1中delay:便于判断ID号较大的情况
 * @note 状态2中delay:长闪烁对应长间隔时间
 **/
void Led::show(uint8_t setId)
{
    this->setId = setId;
    delayTime += cycleT; // 时钟

    switch (showState)
    {
    case 0: // 亮灯状态
        if (delayTime > 130000)
        {
            delayTime = 0;
            showState = 1; // 进入灭灯状态
        }
        break;
    case 1: // 灭灯状态
        if (delayTime > 200000 + nowId * 20000)
        {
            delayTime = 0;
            nowId++; // 增加已展示ID数
            showState = nowId < setId ? 0 : 2;
        }
        break;
    case 2: // 长延时
        if (delayTime > 800000 + setId * 50000)
        {
            delayTime = 0;
            nowId = 0; // 开始新的循环
            showState = 0;
        }
        break;
    default:
        break;
    }
    // 根据颜色,控制闪烁状态
    ctrl(!showState);
}

// 定义板载lED灯对象
Led running;