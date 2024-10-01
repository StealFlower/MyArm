#ifndef __DOUBLEMOTOR_
#define __DOUBLEMOTOR_
#include "motor.h"

enum Direction
{
    same = 1,
    opposite = -1
};

struct DoubleMotor
{
public:
    Motor *motorPtr[2];     //双电机指针
    int8_t direction;       //方向：1同-1反向

    DoubleMotor(Motor *motor1Ptr, Motor *motor2Ptr, int8_t direction);
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
    float setSpeed;   
    void ctrlSpeed(float setSpd);

    //控制位置
    float setPosition;   
    void ctrlPosition(float setPos);

    //零点校准
    uint8_t offsetOkFlag;
    uint8_t motorOffsetOkFlag[2] = {0};                //电机校准标志位
    uint8_t ctrlMotorOffset(float reSetSpeed, float maxErr, float outLimit);
    
    //反馈
    float getSpeed();
    float getPosition();
    
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
    //切换方向
    void changeDirection(int8_t direction);
    //切换模式
    u8 ctrlMode;
    void toggleCtrlMode(u8 spdOrPos);

};




#endif