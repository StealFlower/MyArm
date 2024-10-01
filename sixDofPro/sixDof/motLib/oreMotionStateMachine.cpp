#include "oreMotionStateMachine.h"
#include "devList.h"
#include "kinematics.h"


StateMachine takeOreSM; //矿石相关动作状态机
u8 LastSMisExc;

u8 ForceErrMode;
//初始状态
TakeInitState takeInitState(&takeOreSM.nowState);
TakeInitState::TakeInitState(State **nowStatePointer) : State(nowStatePointer)
{
    convertTo(this);
}
void TakeInitState::update()
{
    switch (count)
    {
    case 0:
        ///TODO:定位导航开启模式待定,半自动模式有必要加入新文件进行分类
        if ((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.Q)
        || (rcCtrl.rc.sw1Tick == RCS::Mid_Up && rcCtrl.rc.sw2 == RCS::Up))
        {//装甲板遮挡
            if(ore[inArm].OreNum){
                startSM(&takeOreSM);
                convertTo(&hidState);
            }
        }

        /*金矿E*/
        if ((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.E)
        || (rcCtrl.rc.sw1Tick == RCS::Mid_Down && rcCtrl.rc.sw2 == RCS::Up))
        {
            if(ore[inThree].OreNum>=2 && ore[inTak].OreNum){
                return;
            }
            startSM(&takeOreSM);
            convertTo(&takeGoldState);
        }
        /*Err下的金矿CTRL+E*/
        if(rcCtrl.key.CTRL && rcCtrl.key.E && !rcCtrl.key.SHIFT)
        {
            if(ore[inThree].OreNum>=2 && ore[inTak].OreNum){
                return;
            }
            ForceErrMode = 1;
            startSM(&takeOreSM);
            convertTo(&takeGoldState);
        }

        /*银矿R*/
        if ((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.R)
        || (rcCtrl.rc.sw1Tick == RCS::Mid_Up && rcCtrl.rc.sw2 == RCS::Mid))
        {
            if(ore[inThree].OreNum){
                return;
            }
            startSM(&takeOreSM);
            convertTo(&takeSilveryState);
        }
        /*Err下的银矿Ctrl+R*/
        if(rcCtrl.key.CTRL && rcCtrl.key.R && !rcCtrl.key.SHIFT)
        {
            if(ore[inThree].OreNum){
                return;
            }
            ForceErrMode = 1;
            startSM(&takeOreSM);
            convertTo(&takeSilveryState);
        }

        /*捡矿F*/
        if((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.F)
        || (rcCtrl.rc.sw2 == RCS::Up //下外八
        &&  rcCtrl.rc.ch[0] == 660 &&rcCtrl.rc.ch[1] == -660
        &&  rcCtrl.rc.ch[2] == -660 &&rcCtrl.rc.ch[3] == -660))
        {
            if(ore[inArm].OreNum){
                return;
            }
            startSM(&takeOreSM);
            convertTo(&takeFloorState);
        }


        /*兑换C*/
        if ((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.C)
        || (rcCtrl.rc.sw2 == RCS::Up
        && rcCtrl.rc.ch[0] == -660 && rcCtrl.rc.ch[1] == 660
        && rcCtrl.rc.ch[4] == -660))
        {
            // // 矿石数量小于1时退出兑换
            if(!ore[inArm].OreNum && !ore[inThree].OreNum && !ore[inTak].OreNum){
                #if VISION_DEBUG
                #else
                    return;
                #endif

            }
            startSM(&takeOreSM);
            convertTo(&exchangeState);
        }
        /*使用取矿推出进行低等级矿石兑换*/
        if(rcCtrl.key.CTRL && rcCtrl.key.C && !rcCtrl.key.SHIFT)
        {
            // // 矿石数量小于1时退出兑换
            if(!ore[inArm].OreNum && !ore[inThree].OreNum && !ore[inTak].OreNum){
                #if VISION_DEBUG
                #else
                    return;
                #endif
            }
            ForceErrMode = 1;
            startSM(&takeOreSM);
            convertTo(&exchangeState);
        }

        /*手动*/
        if (rcCtrl.rc.sw2 == RCS::Up //下内八
        && rcCtrl.rc.ch[0] == -660 && rcCtrl.rc.ch[1] == -660
        && rcCtrl.rc.ch[2] == 660  && rcCtrl.rc.ch[3] == -660)
        {
            startSM(&takeOreSM);
            convertTo(&handModeState);
        }
        /*检录模式*/
        if (rcCtrl.rc.sw2 == RCS::Up //上内八
        && rcCtrl.rc.ch[0] == -660 && rcCtrl.rc.ch[1] == 660
        && rcCtrl.rc.ch[2] == 660  && rcCtrl.rc.ch[3] == 660)
        {
            startSM(&takeOreSM);
            convertTo(&checkstate);
        }
    }
}
void TakeInitState::exit()
{
    count = 0;
}


uint8_t MechExitCount,ArmExitCount;
//退出状态
ExitPerfectState exitPerfectState(&takeOreSM.nowState);
ExitPerfectState::ExitPerfectState(State **nowStatePointer) : State(nowStatePointer)
{
}
void ExitPerfectState::update()
{
}

void ExitPerfectState::exit()
{
    count = 0;
    endSM();
}

void updateNowPoint(void)
{
}
