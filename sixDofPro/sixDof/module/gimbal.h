#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "board.h"
#include "motor.h"
#include "fdcan.h"

enum GimbalYaw : int
{
    gimYawForward = 0,
    gimYawLeft = -90,
    gimYawBack = 180,
    gimYawRight = 90,
    gimYawCheck = 120,//
};
enum GimbalPitch : int
{
    gimPitLowest = -24,
    gimPitMiddle = 0,
    gimPitHighest = 15,
};
enum GimbalLift : int
{
    gimLiftLowest = 100,
    gimLiftHighest = 6800,
};


struct Gimbal
{
public:
    //Motor lift  = Motor(RMDX4, &hfdcan1, 0x207);
    Motor yaw   = Motor(GM3510, &hfdcan3, 0x208);
    //Motor pitch = Motor(GM3510, &hfdcan1, 0x209);

    PidParam pitchSpdParam[1];
    PidParam pitchPosParam[1];
    PidParam yawSpdParam[1];
    PidParam yawPosParam[1];
    PidParam liftSpdParam[1];
    PidParam liftPosParam[1];


    void init();//初始化
    void update();//控制
};


#endif 
