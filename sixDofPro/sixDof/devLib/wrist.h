#ifndef _WRIST_H_
#define _WRIST_H_

#include "doubleMotor.h"

#define Roll_KINE_TO_MECH_RATIO  44
#define Pitch_KINE_TO_MECH_RATIO 34

class Wrist : public DoubleMotor
{
public:
    Motor *motorPtr[2];
    float FbRoll,FbPitch;
    Wrist(Motor *motor1Ptr,Motor *motor2Ptr);

public:
    u8 planIndex;    //当前PID方案索引
    //设置pid方案
    void setPlan(int planIndex, u8 spdOrPos, PidParam *paramPtr);
    //设置pid参数
    void setParam(int planIndex, u8 spdOrPos, float kp,float ki,float kd,float intLimit,float outLimit);
    //设置速度限幅
	void setSpeedLimit(float speedMax);
    //设置输出限幅
    void setOutLimit(float outLimit);

    //控制速度

    //控制位置
    float setRoll,setPitch;   
    void ctrlPosition(float SetRoll,float SetPitch);

    //零点校准
    uint8_t offsetOkFlag;
    uint8_t motorOffsetOkFlag[2] = {0};                //电机校准标志位
    uint8_t ctrlMotorOffset(float reSetSpeed, float maxErr, float outLimit);
    
    //反馈
    float getRollAngle();
    float getPitchAngle();
    
    //判断到位
    Cycle delay;
    VarCale varCale;
    float delayTime;
    float err,var;
    uint8_t hereFlag;
    u8 exitStatus(float maxErr, float maxVar, float deathRoomTime, float outTime, float targetPos=-1);
    uint8_t lastExitStatus;
    float lastDelayTime;
    
    // 判断堵转
    VarCale currentVarCale;
    float curVar;
    uint8_t lockState;
    uint16_t lockCnt;
    
    void clear(void);

};
#endif
