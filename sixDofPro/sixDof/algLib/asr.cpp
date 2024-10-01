#include "asr.h"


//驱动防滑控制原理：
//车辆在正常行驶时，驱动力作用在车轮上，通过摩擦力将车轮与地面连接起来，并提供足够的牵引力。
//当车辆需要快速加速或爬坡时，驱动力可能超过了轮胎与地面之间的摩擦力，车轮与地面之间的摩擦
//力不足以提供足够的牵引力，导致车轮打滑。
//因此，驱动防滑控制的措施即限制车轮起步的最大加速度

//光滑加速曲线：
//为了获得确定的光滑加速曲线，至少需要对v(t)施加4个约束条件。
//由初始值和最终值可得到对函数值的两个约束条件：v(0)=v0, v(t)=vf
//为确保车轮加速度连续可得到另外的两个约束条件：a(0)=0 , a(t)=0
//次数至少为3的多项式才能满足这4个约束条件，一个三次多项式有4个系数，4个方程解4个未知数

//已知最大加速度acc, v0=nowIndex.fb, vf=tarIndex.fb
//可求解得加速时间段tf=1.5*ABS(tarIndex.fb - nowIndex.fb)/ABS(acc)


Asr::Asr()
{
    clear();
}


/**
 * @brief    设定目标值
 * @param    now：当前值
 * @param    tar：目标值
 * @param    time：过程时间，单位/s
 * */
void Asr::setTar(float now, float tar, float time)
{
    /*计算三次四项式系数*/
    this->tar = tar;
    this->time = time;
    parm[0] = now;
    parm[1] = 0;
    parm[2] = 3 * (tar - now) / pow(time, 2);
    parm[3] = -2 * (tar - now) / pow(time, 3);
    cycle.getCycleT(); //更新计时
    nowTime = 0;
}


/**
 * @brief 更新输出值
 * */
float Asr::update()
{
    float nowValue = 0;
    //根据模型计算设定值
    nowTime += cycle.getCycleT();
    if (nowTime < time)
    {
        for (u8 i = 0; i < 4; i++)
            nowValue += parm[i] * pow(nowTime, i);
    }
    else
        nowValue = tar;

    return nowValue;
}


/**
 * @brief 清除
 * */
void Asr::clear()
{
    time = 0; //目标时间
    tar = 0;  //目标值
    memset(parm, 0, sizeof(parm));
    nowTime = 0; //当前时间
    cycle.getCycleT();
}