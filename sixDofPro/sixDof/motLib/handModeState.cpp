#include "oreMotionStateMachine.h"
#include "devList.h"
#include "kinematics.h"
#include "SMUniversal.h"

float AngleGetted,PitchGetted;

extern EndPoint offsetEndPoint,ctrlPoint,offsetCtrlPoint;
HandModeState handModeState(&takeOreSM.nowState);
HandModeState::HandModeState(State **nowStatePointer) : State(nowStatePointer)
{
}
void HandModeState::update()
{
    // 退出取矿检测
    if((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.B) 
    ||(rcCtrl.rc.sw1Tick==RCS::Mid_Down && rcCtrl.rc.sw2==RCS::Mid))
    {
        convertTo(&exitPerfectState);
        return;
    }

}

void HandModeState::exit()
{
    count = 0;
}