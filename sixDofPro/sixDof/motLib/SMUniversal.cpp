#include "SMUniversal.h"
#include "devList.h"

//校准状态机校准完成标志位
u8 offsetEndFlag = 0;
//参数重加载完成标志位
u8 paramReloadFlag = 0;
//831 1487
//-6824 -10289
//takpitch -6152 2 
//smallliftft -8358 taklift 1500
//30 96 106
struct TimeVar timeVar;           //计时判断相关变量

/**
  * @brief    状态机初始化
  * @param    stateMachine：状态机地址
  */
void startSM(StateMachine* stateMachine)
{   
    //电机参数重新加载
    loadParam();
    
    //重置电机PID方案号
    // lift.planIndex=0;...
    
    //重置到位标志和时间
    // lift.delayTime = 0;lift.hereFlag = 0;...
    
    //重置路径规划时间
    // clearTime(&liftPath);...
    

    //除自己外的所有状态机都置为不可运行状态
    for (uint8_t i = 0; i < FSMNum; i++)
    {
        if (FSMList[i] == stateMachine)
            FSMList[i]->runFlag = 1;
        else
            FSMList[i]->runFlag = 0;
    }
    
    
	//延时等参数清空
	timeVar.timeNow = -1; //特殊处理
    timeVar.varCale.clear();
}


/**
  * @brief    一个状态机结束时将所有状态机都置为可运行状态
  */
void endSM()
{
    for (uint8_t i = 0; i < FSMNum; i++)
    {
        FSMList[i]->runFlag = 1;
    }
}


/**
  * @brief    当经过delayTimeMs个ms时返回1
  * @param    delayTimeMs：计时时间，单位ms
  * @retval   计时时间到时返回1
  */
u8 delay(u16 delayTimeMs)
{
    if (timeVar.timeNow < 0) //开始延时
    {
        timeVar.timeNow = 0;
        timeVar.cycle.getCycleT(); //清空时间 
    }
    else
        timeVar.timeNow += timeVar.cycle.getCycleT() * 1000;
    if (timeVar.timeNow > delayTimeMs)
    {
        timeVar.timeNow = -1;
        return 1;
    }
    return 0;
}
