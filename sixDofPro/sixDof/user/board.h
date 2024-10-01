#ifndef __BOARD_H__
#define __BOARD_H__

/* ▼ 全局公共头文件引用*/
#include "stm32g4xx_hal.h"
#include "arm_math.h"
#include <string.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_iwdg.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_lpuart.h"

#define UNUSE_RC 0

//写死机器人ID，红2: 2; 蓝2： 102
#define WHO_AM_I 2

//快速调参标志位
#define Quickly_Adjust_Param 0
//联调标志位
#define VISION_DEBUG 0
#define Find_Problem 0
#define EMBD_DEBUG 0
#define AUTO_DEBUG 0

#define ABS(x) ((x) > 0 ? (x) : -(x))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

typedef struct _MitParam
{
    float p_des; float v_des; float kp; float kd; float t_ff;
}MitParam;

extern volatile uint32_t sysTickUptime;
extern volatile uint32_t sysTickUptimeUs;

void sysTickInit(void);
uint64_t getSysTimeUs(void);
void delayUs(uint32_t us);
void delayMs(uint32_t ms);
void softwareReset();

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */