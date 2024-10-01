#ifndef __SMUNIVERSAL_H
#define __SMUNIVERSAL_H
#include "stateMachine.h"
#include "devList.h"
#include "pathPlan.h"
#include "dbus.h"
#include "oreBin.h"

#define HANDPITCH_TO_KINE_RATIO 0.0125f
#define HANDROLL_TO_KINE_RATIO 0.013235294f

#define MechRatio 2.f

//校准状态机校准完成标志位
extern u8 offsetEndFlag;
//参数重加载完成标志位
extern u8 paramReloadFlag;


//各电机参数加载函数
//各状态机初始化函数
void startSM(StateMachine* stateMachine);
//各状态机结束回调函数
void endSM();
//状态机专用定时函数
u8 delay(u16 delayTimeMs);

void DeviceClear(void);

//计时判断及方差计算相关结构体
struct TimeVar
{
    float timeNow;   //当前延时时间,非延时时该值为负数
    Cycle cycle;     //延时计算相关变量
    VarCale varCale; //误差的方差计算类
};
extern struct TimeVar timeVar;  //计时判断相关变量


#endif