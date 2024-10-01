#include "offsetStateMachine.h"
#include "devList.h"
#include "SMUniversal.h"


// 状态机：校准状态机
StateMachine offSetSM;

//校准状态
OffsetState offsetState(&offSetSM.nowState);
OffsetState::OffsetState(State **nowStatePointer) : State(nowStatePointer)
{
    convertTo(this);
}
void OffsetState::update()
{

}
void OffsetState::exit()
{
    count = 0;
}
