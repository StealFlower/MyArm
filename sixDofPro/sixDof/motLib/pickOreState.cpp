#include "oreMotionStateMachine.h"
#include "devList.h"

TakeFloorState takeFloorState(&takeOreSM.nowState);
TakeFloorState::TakeFloorState(State **nowStatePointer) : State(nowStatePointer)
{
}
void TakeFloorState::update()
{
    //退出状态检测
    if((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.B) 
    ||(rcCtrl.rc.sw1Tick==RCS::Mid_Down && rcCtrl.rc.sw2==RCS::Mid))
    {
        convertTo(&exitPerfectState);
        return;
    }

}

void TakeFloorState::exit()
{   
    count=0;
}
