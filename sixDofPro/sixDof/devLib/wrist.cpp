#include "wrist.h"

Wrist::Wrist(Motor *motor1Ptr,Motor*motor2Ptr) : DoubleMotor(motor1Ptr,motor2Ptr,1)
{
    motorPtr[0] = motor1Ptr;
    motorPtr[1] = motor2Ptr;
}
//PID设置
void Wrist::setPlan(int planIndex,u8 spdOrPos, PidParam *paramPtr)
{
    //电机1和电机2的PID方向指针指向同一个地址
    //双电机共用一套方案
    motorPtr[0]->setPlan(planIndex, spdOrPos, paramPtr);
    motorPtr[1]->setPlan(planIndex, spdOrPos, paramPtr);
}
void Wrist::setParam(int planIndex, u8 spdOrPos, float kp,float ki,float kd,float intLimit,float outLimit)
{
    motorPtr[0]->setParam(planIndex,spdOrPos,kp,ki,kd,intLimit,outLimit);
    motorPtr[1]->setParam(planIndex,spdOrPos,kp,ki,kd,intLimit,outLimit);
}
void Wrist::setSpeedLimit(float speedMax)
{
    motorPtr[0]->pidPos.paramList[planIndex]->outLimit = speedMax;
    motorPtr[1]->pidPos.paramList[planIndex]->outLimit = speedMax;
}
void Wrist::setOutLimit(float outLimit)
{
    motorPtr[0]->pidSpd.paramList[planIndex]->outLimit = outLimit;
    motorPtr[1]->pidSpd.paramList[planIndex]->outLimit = outLimit;
}
void Wrist::clear(void)
{
    this->setPosition=0;this->setSpeed=0;
    memset(&motorPtr[0]->canInfo, 0, sizeof(motorPtr[0]->canInfo));
    memset(&motorPtr[1]->canInfo, 0, sizeof(motorPtr[1]->canInfo));
    motorPtr[0]->pidSpd.clear();motorPtr[1]->pidSpd.clear();
    motorPtr[0]->pidPos.clear();motorPtr[1]->pidPos.clear();
}
//零点校准
uint8_t Wrist::ctrlMotorOffset(float reSetSpeed, float maxErr, float outLimit)
{
    
    //两个电机独立控制
    if (!motorOffsetOkFlag[0])
        motorOffsetOkFlag[0] = motorPtr[0]->ctrlMotorOffset(reSetSpeed, maxErr, outLimit);
    if (!motorOffsetOkFlag[1])
        motorOffsetOkFlag[1] = motorPtr[1]->ctrlMotorOffset(-reSetSpeed, maxErr, outLimit);
        
    if(motorOffsetOkFlag[0] && motorOffsetOkFlag[1])
    {
        memset(motorOffsetOkFlag,0,2);
        return 1;
    }
    return 0;
}

//位置控制
void Wrist::ctrlPosition(float SetRoll,float SetPitch)
{
    float setRollMech = SetRoll * Roll_KINE_TO_MECH_RATIO;
    float setPitchMech = SetPitch * Pitch_KINE_TO_MECH_RATIO;
    
    motorPtr[0]->planIndex = this->planIndex;
    motorPtr[1]->planIndex = this->planIndex;
    
    motorPtr[0]->setPosition = setRollMech + setPitchMech;
    motorPtr[1]->setPosition = setRollMech - setPitchMech;
    motorPtr[0]->ctrlPosition(motorPtr[0]->setPosition);
    motorPtr[1]->ctrlPosition(motorPtr[1]->setPosition);
}

//反馈
float Wrist::getRollAngle()
{
    FbRoll = ((motorPtr[0]->canInfo.totalAngle_f)+(motorPtr[1]->canInfo.totalAngle_f))/ (2.0f * Roll_KINE_TO_MECH_RATIO)  ;
	return FbRoll;
}
float Wrist::getPitchAngle()
{
    FbPitch = ((motorPtr[0]->canInfo.totalAngle_f)-(motorPtr[1]->canInfo.totalAngle_f))/ (2.0f * Pitch_KINE_TO_MECH_RATIO) ;
	return FbPitch;
}


/**
  * @brief    当双电机位置 “稳定在允许误差范围内” 时返回1
  * @param    maxErr：最大误差；maxVar：最大方差；
  * @param    deathRoomTime：死区时间；outTime：超时时间
  * @retval   双电机位置 “稳定在允许误差范围内” 时返回1，否则返回0
  */
u8 Wrist::exitStatus(float maxErr, float maxVar, float deathRoomTime, float outTime, float targetPos)
{
    if(ABS(targetPos+1)<0.001)
    {
        err = (ABS(motorPtr[0]->pidPos.error) + ABS(motorPtr[1]->pidPos.error)) / 2;
    }
    else
    {
        err = (ABS(targetPos-motorPtr[0]->getPosition()) + ABS(targetPos*direction-motorPtr[1]->getPosition()))/2;
    }
    if (delayTime == 0)    //首次开始
	{
		varCale.clear();
        delay.getCycleT();    //清除
	}
    var = varCale.caleVar(err);
//    delayTime += delay.getCycleT() * 1000; //单位转换为ms
    //getCycleT()函数有问题
    delayTime += 2; //默认2ms
    if (err < maxErr && var < maxVar && delayTime > deathRoomTime)
    {
        lastDelayTime = delayTime;
        delayTime = 0;
        lastExitStatus = 1;
        return 1;
    }
	if(delayTime > outTime)
	{
        lastDelayTime = delayTime;
        delayTime = 0;
        lastExitStatus = 2;
        return 1;
	}
    lastExitStatus = 0;
    return 0;
}


