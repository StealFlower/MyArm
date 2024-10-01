#include "oreMotionStateMachine.h"
#include "judgement.h"
#include "devList.h"

CheckState checkstate(&takeOreSM.nowState);
CheckState::CheckState(State **nowStatePointer) : State(nowStatePointer)
{
}
void CheckState::update()
{
    //退出
    if((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.B)
    ||(rcCtrl.rc.sw1Tick==RCS::Mid_Down && rcCtrl.rc.sw2==RCS::Mid))
    {
        convertTo(&exitPerfectState);
        return;
    }
}

void CheckState::exit()
{
    count=0;
}
