#include "stateMachine.h"

// 状态机列表外部声明
StateMachine **FSMList = 0;
// 状态机数量外部声明
int FSMNum = 0;

void State::convertTo(State *next)
{
    // 切换至下一状态
    *nowStatePointer = next;
    // 调用上一状态退出函数
    this->exit();
}

StateMachine::StateMachine()
{
    if (FSMList == 0)
        FSMList = (StateMachine **)malloc(sizeof(StateMachine *));
    else
        FSMList = (StateMachine **)realloc(FSMList, sizeof(StateMachine *) * (FSMNum + 1));

    // 记录状态机指针
    FSMList[FSMNum] = this;
    FSMNum += 1;

    runFlag = 1;
    enableFlag = 1;
}

// 状态机更新函数
void StateMachine::update()
{
    // 调用对应状态的更新函数
    nowState->update();
}
