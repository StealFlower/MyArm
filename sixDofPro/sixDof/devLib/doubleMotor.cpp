#include "doubleMotor.h"


//构造函数
DoubleMotor::DoubleMotor(Motor *motor1Ptr, Motor *motor2Ptr, int8_t direction)
{
    motorPtr[0] = motor1Ptr;
    motorPtr[1] = motor2Ptr;
    this->direction = direction;    //双电机转向
}


//PID设置
void DoubleMotor::setPlan(int planIndex,u8 spdOrPos, PidParam *paramPtr)
{
    //电机1和电机2的PID方向指针指向同一个地址
    //双电机共用一套方案
    motorPtr[0]->setPlan(planIndex, spdOrPos, paramPtr);
    motorPtr[1]->setPlan(planIndex, spdOrPos, paramPtr);
}
void DoubleMotor::setParam(int planIndex, u8 spdOrPos, float kp,float ki,float kd,float intLimit,float outLimit)
{
    motorPtr[0]->setParam(planIndex,spdOrPos,kp,ki,kd,intLimit,outLimit);
}
void DoubleMotor::setSpeedLimit(float speedMax)
{
    motorPtr[0]->pidPos.paramList[planIndex]->outLimit = speedMax;
}
void DoubleMotor::setOutLimit(float outLimit)
{
    motorPtr[0]->pidSpd.paramList[planIndex]->outLimit = outLimit;
}


//切换转向
void DoubleMotor::changeDirection(int8_t direction)
{
    if(this->direction == direction)
        return;
    //清除Position和PID
    clear();
    //切换方向
    this->direction = direction;
}
//切换模式
void DoubleMotor::toggleCtrlMode(u8 spdOrPos)
{
    if(this->ctrlMode == spdOrPos)
        return;
    //清除Position和PID
    clear();
    //切换方向
    this->ctrlMode = spdOrPos;
}
void DoubleMotor::clear(void)
{
    this->setPosition=0;this->setSpeed=0;
    memset(&motorPtr[0]->canInfo, 0, sizeof(motorPtr[0]->canInfo));
    memset(&motorPtr[1]->canInfo, 0, sizeof(motorPtr[1]->canInfo));
    motorPtr[0]->pidSpd.clear();motorPtr[1]->pidSpd.clear();
    motorPtr[0]->pidPos.clear();motorPtr[1]->pidPos.clear();
}


//零点校准
uint8_t DoubleMotor::ctrlMotorOffset(float reSetSpeed, float maxErr, float outLimit)
{
    //两个电机独立控制
    if (!motorOffsetOkFlag[0])
        motorOffsetOkFlag[0] = motorPtr[0]->ctrlMotorOffset(reSetSpeed, maxErr, outLimit);
    if (!motorOffsetOkFlag[1])
        motorOffsetOkFlag[1] = motorPtr[1]->ctrlMotorOffset(reSetSpeed*direction, maxErr, outLimit);
        
    if(motorOffsetOkFlag[0] && motorOffsetOkFlag[1])
    {
        memset(motorOffsetOkFlag,0,2);
        return 1;
    }
    return 0;
}


//速度控制
void DoubleMotor::ctrlSpeed(float setSpd)
{
    this->setSpeed = setSpd;
    motorPtr[0]->planIndex = this->planIndex;
    motorPtr[1]->planIndex = this->planIndex;

    motorPtr[0]->ctrlSpeed(setSpeed);
    motorPtr[1]->ctrlSpeed(setSpeed*direction);
}
//位置控制
void DoubleMotor::ctrlPosition(float setPos)
{
    this->setPosition=setPos;
    
    motorPtr[0]->planIndex = this->planIndex;
    motorPtr[1]->planIndex = this->planIndex;

    motorPtr[0]->ctrlPosition(setPosition);
    motorPtr[1]->ctrlPosition(setPosition*direction);
}


//反馈
float DoubleMotor::getPosition()
{
	return ((motorPtr[0]->canInfo.totalAngle_f)+direction*(motorPtr[1]->canInfo.totalAngle_f))/2.0f;
}
float DoubleMotor::getSpeed()
{
	return ((motorPtr[0]->canInfo.dps)+direction*(motorPtr[1]->canInfo.dps))/2.0f;
}


/**
  * @brief    当双电机位置 “稳定在允许误差范围内” 时返回1
  * @param    maxErr：最大误差；maxVar：最大方差；
  * @param    deathRoomTime：死区时间；outTime：超时时间
  * @retval   双电机位置 “稳定在允许误差范围内” 时返回1，否则返回0
  */
u8 DoubleMotor::exitStatus(float maxErr, float maxVar, float deathRoomTime, float outTime, float targetPos)
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


