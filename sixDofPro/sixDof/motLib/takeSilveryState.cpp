#include "oreMotionStateMachine.h"
#include "devList.h"


uint8_t SilverErrModeFlag;

TakeSilveryState takeSilveryState(&takeOreSM.nowState);
TakeSilveryState::TakeSilveryState(State **nowStatePointer) : State(nowStatePointer)
{
}
void TakeSilveryState::update()
{
    //退出状态检测
    if((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.B) 
    ||(rcCtrl.rc.sw1Tick==RCS::Mid_Down && rcCtrl.rc.sw2==RCS::Mid))
    {
        convertTo(&exitPerfectState);
        return;
    }
}

void TakeSilveryState::exit()
{
    count=0;
}
