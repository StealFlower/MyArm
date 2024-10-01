#ifndef ONLINE_H__
#define ONLINE_H__
#include "board.h"

struct Online
{
private:
    uint16_t onlineCnt;
    uint8_t onlineFlag;

public:
    Online();

    // 离线检测 每个运行周期调用一次
    void check();
    // 刷新检测 每次收到数据反馈时调用一次
    void update();

    
    bool isOnline()
    {
        return onlineFlag;
    }
    bool isOffLine()
    {
        return !onlineFlag;
    }
};

extern Online **onlinePtrList;
extern uint8_t onlineNum;

#endif