#ifndef __AIR_H__
#define __AIR_H__

#include "fdcan.h"
#include "board.h"
#include "online.h"
#include "motor.h"
#include "gugu.h"

enum valveIndex{
    PitchLeft = 2,
    PitchMiddle = 1,
    PitchRight = 0,
};
enum pumpspeed{pumpstop=4000,pumpcheck=5000,fullspeed=8000,};//全速8000，停止4000

enum AirStateIndex{
    airTakGold = 0,
    airTakSilver = 1,
    airExchange = 2,
    airExit = 3,
};

class Air{
private:

public:
    Gugu gugu;
    CanSendMsg valveMsg = CanSendMsg(&hfdcan3);
    CanSendMsg armValve = CanSendMsg(&hfdcan2);
    CanSendMsg pumpMsg = CanSendMsg(&hfdcan1);
    bool airvalvestate[8];
    bool armvalvestate;
    Air();
    int16_t MechPumpSpd , ArmPumpSpd , TakPumpSpd;
    void SendDataUpdate();
    void Stop();

    void StateConvert(AirStateIndex state);

};

#endif