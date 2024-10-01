#include "oreMotionStateMachine.h"
#include "judgement.h"
#include "devList.h"

CatchState catchState(&takeOreSM.nowState);
CatchState::CatchState(State **nowStatePointer) : State(nowStatePointer)
{
}
void CatchState::update()
{
    //退出
    if((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.B)
    ||(rcCtrl.rc.sw1Tick==RCS::Mid_Down && rcCtrl.rc.sw2==RCS::Mid))
    {
        convertTo(&exitPerfectState);
        return;
    }


    switch(count)
    {
        case 0:
        break;
        case 1:
        break;

        //结束空接状态
        case 203:
            convertTo(&takeInitState);
        break;
    }
}
void CatchState::exit()
{
    count=0;
}
