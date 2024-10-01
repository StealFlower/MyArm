#include "gimbal.h"
#include "dbus.h"
#include "oreMotionStateMachine.h"


/**
  * @brief 云台初始化：电机参数加载
  */
void Gimbal::init()
{
//    lift.setPlan(0, spd, &liftSpdParam[0]);
//    lift.setParam(0, spd, 0, 0, 0, 0, 7000);
//    lift.setPlan(0, pos, &liftPosParam[0]);
//    lift.setParam(0, pos, 0, 0, 0, 0, 3000 * 6);

    yaw.canInfo.offsetEncoder = 5620;//云台电机零点偏移量
    yaw.setPlan(0, spd, &yawSpdParam[0]);
    yaw.setParam(0, spd, 20, 0, 0, 0, 16000);
    yaw.setPlan(0, pos, &yawPosParam[0]);
    yaw.setParam(0, pos, 100, 0, 0.8, 0, 400);

//    pitch.canInfo.offsetEncoder = 0;//云台电机零点偏移量
//    pitch.setPlan(0, spd, &pitchSpdParam[0]);
//    pitch.setParam(0, spd, 0, 0, 0, 0, 16000);
//    pitch.setPlan(0, pos, &pitchPosParam[0]);
//    pitch.setParam(0, pos, 0, 0, 0, 0, 16000);
}


/**
  * @brief 云台更新：云台电机控制
  */
void Gimbal::update()
{
    //云台抬升电机校准完成后才能进行控制
    // if(Lift.offsetOkFlag)
    // {
    //     Lift.setPosition = LIMIT(Lift.setPosition,LiftHighest,LiftLowest);
    //     Lift.ctrlPosition(Lift.setPosition);
    // }

    //云台Yaw：V键切换转向
    if(!rcCtrl.key.CTRL && !rcCtrl.key.SHIFT && rcCtrl.keyPress.V)
        yaw.setPosition = yaw.setPosition==gimYawForward?gimYawRight:(yaw.setPosition==gimYawRight?gimYawBack:gimYawForward);
    yaw.ctrlPosition(yaw.setPosition);

    //云台Pitch,此版车不存在
//    pitch.setPosition += rcCtrl.mouse.vy*0.003;
//    pitch.setPosition = LIMIT(pitch.setPosition,gimPitLowest,gimPitHighest);
//    pitch.ctrlSinglePosition(pitch.setPosition);

}


