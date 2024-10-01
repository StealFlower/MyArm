#include "oreMotionStateMachine.h"
#include "devList.h"
#include "kinematics.h"

TakeGoldState takeGoldState(&takeOreSM.nowState);
TakeGoldState::TakeGoldState(State **nowStatePointer) : State(nowStatePointer)
{
}
void TakeGoldState::update()
{
    // 退出状态检测
    if((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.B)
    ||(rcCtrl.rc.sw1Tick==RCS::Mid_Down && rcCtrl.rc.sw2==RCS::Mid))
    {
        convertTo(&exitPerfectState);
        return;
    }
    else if((rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.B)
    ||(rcCtrl.rc.sw1Tick==RCS::Mid_Up && rcCtrl.rc.sw2==RCS::Mid))
    {
        count = 100;
        //取矿中途不想再取，回家
    }
}

void TakeGoldState::exit()
{
    count = 0;
}
