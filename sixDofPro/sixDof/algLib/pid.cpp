#include "pid.h"

/**
 * @brief 构造器
 */
Pid::Pid() : planNum(0), lastPlanIndex(-1), fbValuePtr(0)
{
}

void Pid::setParam(int planIndex, float kp, float ki, float kd, float intLimit, float outLimit)
{
    paramList[planIndex]->kp = kp;
    paramList[planIndex]->ki = ki;
    paramList[planIndex]->kd = kd;
    paramList[planIndex]->intLimit = intLimit;
    paramList[planIndex]->outLimit = outLimit;
}

void Pid::setParam(int planIndex, float kp, float ki, float kd, float intLimit, float outLimitLower, float outLimitUpper)
{
    paramList[planIndex]->kp = kp;
    paramList[planIndex]->ki = ki;
    paramList[planIndex]->kd = kd;
    paramList[planIndex]->intLimit = intLimit;
    paramList[planIndex]->resultMaxSides[0] = outLimitLower;
    paramList[planIndex]->resultMaxSides[1] = outLimitUpper;
}

/**
 * @brief 装载PID参数
 * @param planIndex 取值0,1,2...
 * @param PidParamStruct PidParam结构体指针
 * @note
 */
void Pid::setPlan(int planIndex, PidParam *paramPtr)
{
    if (planIndex >= planNum)
    {
        planNum = planIndex + 1;
        if (planNum == 0)
            paramList = (PidParam **)malloc(sizeof(PidParam *));
        else
            paramList = (PidParam **)realloc(paramList, sizeof(PidParam *) * planNum); // 为指针paramList分配planNum个PidParam大小的空间
    }
    paramList[planIndex] = paramPtr;
}

/**
 * @brief Pid 传入参数指针
 * @param set 设定值指针
 * @note
 */
void Pid::setDataPtr(float *fb, int planIndex)
{
    paramList[planIndex]->fbValuePtr = fb;
}

/**
 * @brief 清零具有累加性质的PID变量
 */
void Pid::clear()
{
    lastError = 0;
    intError = 0;
    deltaError = 0;
    // 清空时间
    pidCycle.getCycleT();
}

/**
 * @brief 检查PID参数是否加载
 * @return [bool]参数是否加载
 * @note 实际上只检查了输出限幅和积分限幅
 */
bool Pid::paramCheck(uint8_t planIndex)
{
    // 检查参数号和参数指针
    if (planIndex + 1 > planNum || fbValuePtr == 0 || fbValuePtr == 0)
        return false;
    // 检查参数数值
    if (paramList[planIndex]->outLimit == 0 && paramList[planIndex]->kp && paramList[planIndex]->ki)
        return false;
    else
        return true;
}

void Pid::setPositiveValue(uint8_t planIndex, float positiveValue)
{
    paramList[planIndex]->advancedPara.positiveFBValue = positiveValue;
}

/**
 * @brief PID计算函数
 * @return [bool]参数是否加载
 * @note 实际上只检查了输出限幅和积分限幅
 */
float Pid::calculate(uint8_t planIndex)
{
    float dt = pidCycle.getCycleT();
    // 未填充参数直接退出
    if (!paramCheck(planIndex))
        return 0;
    // 方案号更改时
    if (planIndex != lastPlanIndex)
    {
        if (paramList[planIndex]->fbValuePtr)
            fbValuePtr = paramList[planIndex]->fbValuePtr;
        lastPlanIndex = planIndex;
        clear();
        return 0;
    }
    /* 偏差 = 设定值 - 反馈值 */
    error = *setValuePtr - *fbValuePtr;

    // 如果间隔时间过长, 清空积分
    if (dt > 0.1f) // 100ms
        clear();
    /* 偏差进行积分 */
    // 正常积分
    if (paramList[planIndex]->advancedPara.integralMethod == 0)
    {
        intError += 0.5f * (error + lastError) * dt;
    }
    // 积分分离
    else if (paramList[planIndex]->advancedPara.integralMethod == 1)
    {
        if (ABS(error) < paramList[planIndex]->advancedPara.integralThreshold)
        {
            intError += 0.5f * (error + lastError) * dt;
        }
        else
        {
            intError = 0;
            if (paramList[planIndex]->advancedPara.kpAmpForIntegralSep > 0)
                error *= ABS(error) / paramList[planIndex]->advancedPara.integralThreshold * paramList[planIndex]->advancedPara.kpAmpForIntegralSep;
        }
    }
    // 变积分
    else if (paramList[planIndex]->advancedPara.integralMethod == 2)
    {
        intError += 0.5f * (error + lastError) * dt * LIMIT(1 - error / paramList[planIndex]->advancedPara.VIntegralIndex, 0, 1);
    }

    /* 偏差的积分进行限制 -抗饱和*/
    intError = LIMIT(intError, -paramList[planIndex]->intLimit, paramList[planIndex]->intLimit);

    /* 偏差进行微分 */
    // 普通微分
    if (paramList[planIndex]->advancedPara.differentialMethod == 0)
    {
        /* 偏差增量 */
        deltaError = (error - lastError) / dt;
        /* 记录本次误差 */
        lastError = error;
    }
    // 微分先行
    else if (paramList[planIndex]->advancedPara.differentialMethod == 1)
    {
        /* 偏差增量 */
        deltaError = (lastError - *fbValuePtr) / dt;
        /* 记录本次误差 */
        lastError = *fbValuePtr;
    }
    else if (paramList[planIndex]->advancedPara.differentialMethod == 2)
    {
        deltaError = ((1 - paramList[planIndex]->advancedPara.DLPFIndex) * deltaError + paramList[planIndex]->advancedPara.DLPFIndex * lastDError);
        lastDError = deltaError;
    }

    /* 总的输出 = 比例项的输出 + 积分项的输出 + 微分项的输出 */
    result = paramList[planIndex]->kp * error + paramList[planIndex]->ki * intError + paramList[planIndex]->kd * deltaError;

    // 正反馈补偿
    result += paramList[planIndex]->advancedPara.positiveFBValue;

    /* 总的输出不能超出电机给定值的范围 */
    if (paramList[planIndex]->resultMaxSides[0] == 0 && paramList[planIndex]->resultMaxSides[1] == 0)
        result = LIMIT(result, -paramList[planIndex]->outLimit, paramList[planIndex]->outLimit);
    else // 双边独立限幅模式
        result = LIMIT(result, -paramList[planIndex]->resultMaxSides[0], paramList[planIndex]->resultMaxSides[1]);

    // 输出
    return result;
}
