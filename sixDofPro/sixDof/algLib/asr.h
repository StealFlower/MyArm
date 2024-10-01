#ifndef __ASR_H
#define __ASR_H
#include "board.h"
#include "cycle.h"

//驱动防滑系统，三次四项式曲线
struct Asr
{
public:
    float time;    //目标时间
    float tar;     //目标值
    float parm[4]; //三次四项式参数表
    float nowTime; //当前时间
    Cycle cycle;   //获取间隔时间用

public:
    //构造器
    Asr();
    //设置目标值
    void setTar(float now, float tar, float time);
    //更新输出值
    float update();
    //清除
    void clear();
};

#endif