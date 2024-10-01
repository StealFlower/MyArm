#include "oreMotionStateMachine.h"
#include "judgement.h"
#include "devList.h"

HidState hidState(&takeOreSM.nowState);
HidState::HidState(State **nowStatePointer) : State(nowStatePointer)
{
}
void HidState::update()
{
    //退出
    if((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.B)
    ||(rcCtrl.rc.sw1Tick==RCS::Mid_Down && rcCtrl.rc.sw2==RCS::Mid))
    {
        convertTo(&exitPerfectState);
        return;
    }

}

void HidState::exit()
{
    count=0;
}
