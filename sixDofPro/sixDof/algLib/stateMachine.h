#ifndef __STATEMACHINE_H__
#define __STATEMACHINE_H__

#include "board.h"

// 状态
class State
{
public:
    // 构造器
    State(State **nowStatePointer)
    {
        this->nowStatePointer = nowStatePointer;
    }
    // 状态更新
    virtual void update(){};
    // 状态退出
    virtual void exit(){};
    // 小状态计数
    uint8_t count = 0;

    // 状态切换
    void convertTo(State *next);

private:
    State **nowStatePointer;
};

// 状态机： 抽象
struct StateMachine
{
public:
    State *nowState = NULL; // 当前状态
    // 使能标志
    uint8_t enableFlag;
    // 运行标志
    uint8_t runFlag;
    StateMachine();

    // 状态机主更新函数
    void update();
};

extern StateMachine **FSMList;
extern int FSMNum;
#endif //__STATEMACHINE_H__