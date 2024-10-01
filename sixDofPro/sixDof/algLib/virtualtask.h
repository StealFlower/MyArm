#ifndef __TASKVIRTUAL_H__
#define __TASKVIRTUAL_H__

#include "board.h"

enum TaskStatus
{
    taskStateRun = 0,
    taskStateStop = 1,

};

// 虚任务抽象
class VirtualTask
{
private:
public:
    TaskStatus status;
    // 构造函数
    VirtualTask();

    // 遥控更新回调
    virtual void rcUpdateCallBack(){};
    // 任务使能回调
    virtual void enableCallBack(){};
    // 任务失能回调
    virtual void disableCallBack(){};

    // 初始化
    virtual void init() = 0;
    // 主函数
    virtual void run() = 0;

    // 任务列表
    static VirtualTask **taskList;
    // 任务数量
    static int taskNum;
    void setAlwaysRun(bool aRFlag);
    bool alwaysRunFlag;
};

#endif