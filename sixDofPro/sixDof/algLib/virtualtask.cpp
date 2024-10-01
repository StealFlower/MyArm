#include "virtualtask.h"

// 任务列表外部声明
VirtualTask **VirtualTask::taskList = 0;
// 任务数量外部声明
int VirtualTask::taskNum = 0;

// 虚任务构造器
VirtualTask::VirtualTask()
{
    if (taskList == 0)
        taskList = (VirtualTask **)malloc(sizeof(VirtualTask *));
    else
        taskList = (VirtualTask **)realloc(taskList, sizeof(VirtualTask *) * taskNum + 1);

    // 记录任务指针
    taskList[taskNum] = this;
    taskNum += 1;

    alwaysRunFlag = false;  // 默认脱力后暂停运行
    status = taskStateStop; // 默认不运行
}

// 设定为持续运行,同时直接开始运行
void VirtualTask::setAlwaysRun(bool aRFlag)
{
    status = taskStateRun;  // 立刻开始运行
    alwaysRunFlag = aRFlag; // 设定为持续运行
}