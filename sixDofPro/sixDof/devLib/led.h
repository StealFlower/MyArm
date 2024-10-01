/*****************************************************************************
File name: TDT_Device\src\led.h
Description: LED灯
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
    ——————————————————————————————————————————————————————————————————————————
    19.11.12 首次完成
    ——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __LED_H__
#define __LED_H__

#include "board.h"

class Led
{
private:
    GPIO_TypeDef *port;      // LED所在GPIO口
    uint32_t pin;            // LED管脚
    uint32_t RCC_AHB1Periph; // LED所在GPIO时钟

public:
    // LedId
    uint8_t setId;
    // LED IO初始化
    void init(uint16_t cycleT);
    // LED 亮灭
    void ctrl(uint8_t state = 0);
    // LED Id展示
    void show(uint8_t id = 0);

private:
    uint32_t delayTime; // 延时时间
    uint8_t showState;  // 展示状态:0:亮:1:灭2:长延时
    uint8_t nowId;      // 当前已展示ID
    uint16_t cycleT;    // 运行周期时间
};
extern Led running;

/** @} */

#endif
