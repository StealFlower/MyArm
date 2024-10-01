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
    u8 planIndex;    //��ǰPID��������
    //����pid����
    void setPlan(int planIndex, u8 spdOrPos, PidParam *paramPtr);
    //����pid����
    void setParam(int planIndex, u8 spdOrPos, float kp,float ki,float kd,float intLimit,float outLimit);
    //�����ٶ��޷�
	void setSpeedLimit(float speedMax);
    //��������޷�
    void setOutLimit(float outLimit);

    //�����ٶ�

    //����λ��
    float setRoll,setPitch;   
    void ctrlPosition(float SetRoll,float SetPitch);

    //���У׼
    uint8_t offsetOkFlag;
    uint8_t motorOffsetOkFlag[2] = {0};                //���У׼��־λ
    uint8_t ctrlMotorOffset(float reSetSpeed, float maxErr, float outLimit);
    
    //����
    float getRollAngle();
    float getPitchAngle();
    
    //�жϵ�λ
    Cycle delay;
    VarCale varCale;
    float delayTime;
    float err,var;
    uint8_t hereFlag;
    u8 exitStatus(float maxErr, float maxVar, float deathRoomTime, float outTime, float targetPos=-1);
    uint8_t lastExitStatus;
    float lastDelayTime;
    
    // �ж϶�ת
    VarCale currentVarCale;
    float curVar;
    uint8_t lockState;
    uint16_t lockCnt;
    
    void clear(void);

};
#endif
