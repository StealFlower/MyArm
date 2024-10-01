#include "oreMotionStateMachine.h"
#include "kinematics.h"
#include "devList.h"
#include "vision.h"

extern uint8_t ClampGoldFlag;

extern EndPoint offsetEndPoint,ctrlPoint,offsetCtrlPoint;

enum ValveIndex : uint8_t{
    left = 0,
    middle = 1,
    right = 2,
    tak = 3,
    arm = 4,
};

bool CANSOLVE,INRANGE;
uint8_t MAYCRASH;

uint8_t mechCount;

uint8_t Level5Mode;

struct{
    float yaw0;
    float yaw1;
    float yaw2;
    float pitch1;
    float roll1;
    float wristPitch;
    float wristRoll;
    float wristRollOffset;
}tempMotorAngle;
/*
工程机器人将其携带的矿石按照正确的方向（条形码朝下）放置在己方兑换站的矿石识别区，
随后将其推入兑换站，可进行矿石兑换。
*/

ExchangeState exchangeState(&takeOreSM.nowState);
ExchangeState::ExchangeState(State **nowStatePointer) : State(nowStatePointer)
{
}
void ExchangeState::update()
{
    // 退出状态检测
    if((!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.B) 
    ||(rcCtrl.rc.sw1Tick==RCS::Mid_Down && rcCtrl.rc.sw2==RCS::Mid))
    {
        //退出前执行
        //...

        convertTo(&exitPerfectState);
        return;
    }
    //首先进行机械臂交接抬升，抬升到位之后进行矿仓有矿判断，如果有矿则实行交接

}
void ExchangeState::exit()
{
    count = 0;
}
