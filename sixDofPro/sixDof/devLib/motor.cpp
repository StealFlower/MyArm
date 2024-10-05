#include "motor.h"
#include "dbus.h"

/// 电机电流输出限制表
///@note 顺序对应枚举变量MotorType里的电机类型排列-M2006,M3508,M3510,GM3510,GM6020,RMDX4
uint16_t motorCurrentLimitList[6] = {10000, 16384, 32760, 29000, 30000, 16384};
// 电机速度输出限制表-取决于各类电机的最高转速
///@note 顺序对应枚举变量MotorType里的电机类型排列-M2006,M3508,M3510,GM3510,GM6020,RMDX4
uint16_t motorSpeedLimitList[6] = {16000, 9158, 9600, 1200, 320, 3000};
// 电机绕组最高允许温度列表
///@note 顺序对应枚举变量MotorType里的电机类型排列-M2006,M3508,M3510,GM3510,GM6020
uint16_t motorMaxTempList[6] = {0xff, 125, 0xff, 100, 125, 100};

/// 大疆电机对象列表
MotorList motorList[3][12];//3条CAN，每条CAN可挂载12个电机
/// 大疆电机CAN发送缓冲区
CanSendMsg Motor::motorSendMsg[3][3] = {
    //      0x200           0x1ff                   0x1fe
    {CanSendMsg(&hfdcan1), CanSendMsg(&hfdcan1), CanSendMsg(&hfdcan1)}, // CAN1
    {CanSendMsg(&hfdcan2), CanSendMsg(&hfdcan2), CanSendMsg(&hfdcan2)}, // CAN2
    {CanSendMsg(&hfdcan3), CanSendMsg(&hfdcan3), CanSendMsg(&hfdcan3)}  // CAN3
};

// 对象列表外部初始化
Motor **Motor::objectPtrList = 0;
// 对象数量外部初始化
int Motor::objectNum = 0;

/**
 * @param    motorType：M2006,M3508,M3510,GM3510,GM6020
 * @param    _Canx：CAN1/CAN2
 * @param    _Std_ID：          CAN标准标识符
 */
Motor::Motor(MotorType motorType, FDCAN_HandleTypeDef *hfdcan, uint32_t _Std_ID)
{
    //为对象申请内存
    if (objectPtrList == 0)
        objectPtrList = (Motor **)malloc(sizeof(Motor *));
    else
        objectPtrList = (Motor **)realloc(objectPtrList, sizeof(Motor *) * objectNum + 1);

    // 记录对象指针
    objectPtrList[objectNum] = this;
    objectNum += 1;
    
    // 电机基本信息填充
    motorInfo.type = motorType;
    motorInfo.stdID = _Std_ID;
    motorInfo.canx = (hfdcan == &hfdcan1 ? can1 : (hfdcan == &hfdcan2 ? can2 : can3));

    // 电机额外信息填充
    otherInfo.currentLimit = motorCurrentLimitList[motorType]; // 输出限幅（内环输出限幅）
    otherInfo.speedLimit = motorSpeedLimitList[motorType];     // 速度限幅（外环输出限幅）
    otherInfo.tempLimit = motorMaxTempList[motorType];         // 绕组最高允许温度
    otherInfo.criticalTemp = 0.7f * otherInfo.tempLimit;       // 过热保护临界温度
    otherInfo.maxOverTemp = 0.8f * otherInfo.tempLimit;        // 过热保护截止温度，高于此温度电机输出为0

    // pid默认反馈值填充
    pidSpd.fbValuePtr = &canInfo.dps;
    pidPos.fbValuePtr = &canInfo.totalAngle_f;
    pidSpd.setValuePtr = &setSpeed;
    pidPos.setValuePtr = &setPosition;

    // 进行初始化
    motorInit();
}

/**
 * @details 该函数
 * @note 该函数在构造时会自动调用，因此正常情况（使用大疆电机）不需要手动调用
 * @warning 该函数未重写
 */
void Motor::motorInit(void)
{
    // 判断ID格式
    if (motorInfo.stdID < 0x201 || motorInfo.stdID > 0x20C)
    {
        // 电机ID异常
        return;
    }

    // 首次定义该电机
    if (motorList[(uint8_t)motorInfo.canx][motorInfo.stdID - 0x201].motorPoint == 0)
    { 
        // 能够被初始化
        enableFlag = 1;
        // 更新电机列表
        motorList[(uint8_t)motorInfo.canx][motorInfo.stdID - 0x201].motorPoint = this;
    }
    else
    {                   // 电机重定义
        enableFlag = 0; // 无法被使能
    }
}


//motorSendMsg数组初始化，非类内定义的变量禁止在类的构造函数中赋值初始
//化,倘若编译时类的定义顺序较变量更靠前，则构造函数中的赋值初始化是无效的
void Motor::canMsgInit(void)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            motorSendMsg[i][j].txHeader.DataLength = FDCAN_DLC_BYTES_8;
            motorSendMsg[i][j].txHeader.Identifier = 0x200 - j;
        }
    }
    
    //控制帧0x1ff和0x200的控制帧为电调C610和C620的控制帧，默认开启
    //控制帧0x1fe只适用于银河电调，不需要时关闭，否则会占用CAN总线频率
    
    // motorSendMsg[0][2].enableFlag=false;//CAN1 控制帧0x1fe 失能
    motorSendMsg[1][2].enableFlag=false;//CAN2 控制帧0x1fe 失能
    motorSendMsg[2][2].enableFlag=false;//CAN3 控制帧0x1fe 失能
}


//设置PID方案
void Motor::setPlan(int planIndex, uint8_t spdOrPos, PidParam *paramPtr)
{
    if (spdOrPos == spd)
        pidSpd.setPlan(planIndex, paramPtr);
    else
        pidPos.setPlan(planIndex, paramPtr);
}


//设置PID参数，输出限幅的上下限均为outLimit
void Motor::setParam(int planIndex, uint8_t spdOrPos, float kp, float ki, float kd, float intLimit, float outLimit)
{
    if (spdOrPos == spd)
        pidSpd.setParam(planIndex, kp, ki, kd, intLimit, outLimit);
    else
        pidPos.setParam(planIndex, kp, ki, kd, intLimit, outLimit);
}


//设置PID参数，输出限幅的上限为outLimitUpper，下限为outLimitLower
void Motor::setParam(int planIndex, uint8_t spdOrPos, float kp, float ki, float kd, float intLimit, float outLimitLower, float outLimitUpper)
{
    if (spdOrPos == spd)
        pidSpd.setParam(planIndex, kp, ki, kd, intLimit, outLimitLower, outLimitUpper);
    else
        pidPos.setParam(planIndex, kp, ki, kd, intLimit, outLimitLower, outLimitUpper);
}


//设置速度限幅，即位置外环的输出限幅
void Motor::setSpdLimit(float spdLimit)
{
    pidPos.paramList[planIndex]->outLimit = spdLimit;
}


//设置输出限幅，即速度内环的输出限幅
void Motor::setOutLimit(float outLimit)
{
    pidSpd.paramList[planIndex]->outLimit = outLimit;
}


/**
 * @details 对输入的电流进行限幅，包括温度限幅、最大值限幅；并将电流值填入canBuff的缓存区中
 * @param  current          电流值
 * @param  sendFlag         是否将限幅后的输出值填入canBuff对应的缓存区中
 * @return float 经过温度限幅、功率限幅、最大值限幅的电流值
 */
void Motor::ctrlCurrent(float current)
{
    // 入值限幅
    current = LIMIT(current, -motorCurrentLimitList[motorInfo.type], motorCurrentLimitList[motorInfo.type]);
    // 过热保护
    this->overHeatProtect(canInfo.temperature);
    current *= otherInfo.overHeatKp;

    this->setCurrent = current;
    // 电流输出
    motorPowerOut(current);
}


/**
 * @details 将设定的速度值通过pid计算后，将pid输出值传入ctrlCurrent
 * @param  speed            速度设定值
 * @param  planIndex        方案号
 * @param  sendFlag         是否自动调用发送函数（传入sendFlag中）
 * @return float 经过内环pid控制，以及温度限幅、功率限幅、最大值限幅的 \b 电流值
 */
void Motor::ctrlSpeed(float speed)
{
    this->setSpeed = speed;
    /*PID计算，默认输出到对象的result变量*/
    pidSpd.calculate(planIndex);
    // 电流环
    ctrlCurrent(pidSpd.result);
}


/**
 * @details 将设定的位置通过pid计算后，将pid输出值传入ctrlSpeed
 * @param  position         位置设定值
 * @param  planIndex        方案号
 * @param  sendFlag         是否自动调用发送函数
 * @return float 经过双环pid控制，以及温度限幅、功率限幅、最大值限幅的电流值
 */
void Motor::ctrlPosition(float position)
{
    this->setPosition = position;
    /*PID计算，默认输出到对象的result变量*/
    pidPos.calculate(planIndex);
    /*速度环*/
    ctrlSpeed(pidPos.result);
}


//单级PID控制，应对速度反馈不准确的情况
void Motor::ctrlSinglePosition(float position)
{
    this->setPosition = position;
    /*PID计算，默认输出到对象的result变量*/
    pidPos.calculate(planIndex);
    /*速度环*/
    ctrlCurrent(pidPos.result);
}


//添加重力补偿的单级PID位置控制
void Motor::ctrlGraPosition(float position,float gravity)
{
    this->setPosition = position;
    /*PID计算，默认输出到对象的result变量*/
    pidPos.calculate(planIndex);
    /*速度环*/
    ctrlCurrent(pidPos.result+gravity);
}


/**
 * @param  temp 当前温度
 */
void Motor::overHeatProtect(int16_t temp)
{
    otherInfo.overHeatKp = (float)(1.0f - ((temp - otherInfo.criticalTemp) / (otherInfo.maxOverTemp - otherInfo.criticalTemp)));
    otherInfo.overHeatKp = LIMIT(otherInfo.overHeatKp, 0, 1);
}


/**
 * @param  canResult 电流值
 */
void Motor::motorPowerOut(float canResult)
{
    uint8_t motorIndex = motorInfo.stdID - 0x201;
    // 根据can口和id填充缓冲区并清空计数器和标志位
    if (motorIndex < 4)
    {
        motorSendMsg[motorInfo.canx][0].txData[motorIndex * 2] = (uint8_t)((int16_t)canResult >> 8) & 0xff;
        motorSendMsg[motorInfo.canx][0].txData[motorIndex * 2 + 1] = (uint8_t)((int16_t)canResult) & 0xff;
    }
    else if (motorIndex < 8)
    {
        motorIndex -= 4;
        motorSendMsg[motorInfo.canx][1].txData[motorIndex * 2] = (uint8_t)((int16_t)canResult >> 8) & 0xff;
        motorSendMsg[motorInfo.canx][1].txData[motorIndex * 2 + 1] = (uint8_t)((int16_t)canResult) & 0xff;
    }
    else
    {
        motorIndex -= 8;
        motorSendMsg[motorInfo.canx][2].txData[motorIndex * 2] = (uint8_t)((int16_t)canResult >> 8) & 0xff;
        motorSendMsg[motorInfo.canx][2].txData[motorIndex * 2 + 1] = (uint8_t)((int16_t)canResult) & 0xff;
    }
}


/**
 * @brief    等待电机转动至限位堵转停下，并设置当前位置为电机零点值
 * @param    reSetSpeed：速度增量；maxErr：最大误差；outLimit：输出限幅
 * @retval   堵转时(校准结束)返回1
 * @warning  只对方案0适用
 * @warning  务必判断返回的堵转标志位，堵转立刻退出
 */
uint8_t Motor::ctrlMotorOffset(float reSetSpeed, float maxErr, float outLimit, float outTime)
{
    delayTime += delay.getCycleT() * 1000; // 单位为ms
    switch (otherInfo.posOffSetFlag)
    {
    case 0:
        // 记录方案初始输出限幅以便校准完后恢复
        otherInfo.outLimitTemp = pidSpd.paramList[0]->outLimit;
        // 获取外环信息
        otherInfo.offSetPos = *pidPos.fbValuePtr;
        otherInfo.posOffSetFlag++;
        break;
    case 1:
        // 更改PID输出限幅为校零模式
        pidSpd.paramList[0]->outLimit = outLimit;
        // 给定校准速度,开始旋转校准
        otherInfo.offSetPos += reSetSpeed;
        ctrlPosition(otherInfo.offSetPos);
        // 检测堵转
        if (ABS(pidPos.error) > maxErr)
            otherInfo.posOffSetFlag++;
        break;
    case 2:
        //重置电机零点
        memset(&canInfo, 0, sizeof(CanInfo));

        pidPos.clear();
        ctrlPosition(0);
        otherInfo.posOffSetFlag = 0;

        delayTime = 0;
        // 恢复PID输出限幅为正常模式
        pidSpd.paramList[0]->outLimit = otherInfo.outLimitTemp;
        return 1;
    }
    
    //超时直接退出
    if (delayTime > outTime)
    {
        delayTime = 0;
        return 1;
    }
    return 0;
}


// 获取电机反馈位置
float Motor::getPosition()
{
    return canInfo.totalAngle_f;
}


//获取电机反馈速度
int16_t Motor::getSpeed()
{
    return canInfo.speed;
}


//获取电机类型速度限幅
float Motor::getMotorSpeedLimit()
{
    return motorSpeedLimitList[motorInfo.type];
}


//获取电机类型输出限幅
float Motor::getMotorCurrentLimit()
{
    return motorCurrentLimitList[motorInfo.type];
}


/**
 * @brief DJI电机上电初始化
 * @param[in] can_x CAN1/CAN2
 * @param[in] _CanRxMsg Can原始数据地址
 */
void Motor::canOffset(uint8_t *pRxData)
{
    // 非云台电机才认为上电点为零点
    if (motorInfo.type != GM6020 && motorInfo.type != GM3510)
    {
        // 获取上电初始角度
        canInfo.offsetEncoder = (int16_t)(pRxData[0] << 8 | pRxData[1]);
    }
    // 初始角度
    canInfo.encoder = (int16_t)((pRxData[0] << 8) | (pRxData[1])); // 机械角度
    canInfo.lastEncoder = canInfo.encoder;                         // 解决上电圈数不对问题
    canInfo.totalRound = 0;                                        // 电机角度归零时用
    canInfo.totalAngle = 0;
    canInfo.totalEncoder = 0;

    canInfo.encoderCalibration = canInfo.encoder - canInfo.offsetEncoder;
    canInfo.lastEncoderCalibration = canInfo.encoderCalibration;
}

/**
 * @brief DJI电机can返回信息计算
 * @param[in] _CanRxMsg Can原始数据地址
 */
void Motor::canHandle(uint8_t *pRxData)
{
    // 初始化校准
    if (++canInfo.msgCnt < 3)
    {
        canOffset(pRxData);
        return;
    }

    //CAN报文间隔时间
    canInfo.intervalTime = (float)canInfo.cycle.getCycleT() * 1000;

    /*电机原始反馈数据处理*/
    // 云台电机
    if (motorInfo.type == GM3510 || motorInfo.type == RMDX4)
    {
        canInfo.encoder = (int16_t)((pRxData[0] << 8) | (pRxData[1]));     // 机械角度
        canInfo.trueCurrent = (int16_t)((pRxData[2] << 8) | (pRxData[3])); // 实际电流
    }
    // 底盘电机
    else if (motorInfo.type == GM6020 || motorInfo.type == M2006 || motorInfo.type == M3508 || motorInfo.type == M3510)
    {
        canInfo.encoder = (int16_t)((pRxData[0] << 8) | (pRxData[1]));     // 机械角度
        canInfo.speed = (int16_t)((pRxData[2] << 8) | (pRxData[3]));       // 速度
        canInfo.trueCurrent = (int16_t)((pRxData[4] << 8) | (pRxData[5])); // 实际电流
        canInfo.temperature = (int16_t)(pRxData[6]);                       // 温度
    }

    /*计算校准后的机械角度*/
    canInfo.encoderCalibration = canInfo.encoder - canInfo.offsetEncoder;

    if(motorInfo.type == GM6020)
        canInfo.totalRound =0;
    /*计算电机旋转总圈数*/
    if (canInfo.encoderCalibration - canInfo.lastEncoderCalibration > 4096)
    {
        canInfo.totalRound--;
    }
    else if (canInfo.encoderCalibration - canInfo.lastEncoderCalibration < -4096)
    {
        canInfo.totalRound++;
    }
    /*计算电机旋转总机械角度值*/
    canInfo.totalEncoder = (canInfo.totalRound * 8192 + canInfo.encoderCalibration);
    /*计算电机旋转总角度*/
    canInfo.totalAngle = canInfo.totalEncoder * 0.0439453125f;   //(强制转换成整型)
    canInfo.totalAngle_f = canInfo.totalEncoder * 0.0439453125f; //(浮点型)

    /*计算云台电机旋转速度=机械角度值之差与RPM换算*/
    if (motorInfo.type == GM3510 || motorInfo.type == RMDX4)
    {
        canInfo.speed = (canInfo.totalEncoder - canInfo.lastTotalEncoder) / 2;
        canInfo.speed = dpsKf.KalmanFilter(canInfo.speed, 0.01, 5, 0);
        canInfo.dps = canInfo.speed * 6.0f;
    }
    else if (motorInfo.type == GM6020 || motorInfo.type == M2006 || motorInfo.type == M3508 || motorInfo.type == M3510)
    {
        // RPM*6 -> dps
        canInfo.dps = canInfo.speed * 6.0f; //(浮点型)
    }
    canInfo.angleF = canInfo.encoderCalibration * 0.0439453125f;
    
    /*记录此次机械角度*/
    canInfo.lastEncoder = canInfo.encoder;
    canInfo.lastTotalEncoder = canInfo.totalEncoder;
    canInfo.lastEncoderCalibration = canInfo.encoderCalibration;
    /*电机在线更新*/
    online.update();
}


/**
 * @brief    当电机位置 “稳定在允许误差范围内” 时返回1
 * @param    maxErr：最大误差；maxVar：最大方差；
 * @param    deathRoomTime：死区时间；outTime：超时时间
 * @retval   电机位置 “稳定在允许误差范围内” 时返回1，否则返回0
 */
uint8_t Motor::exitStatus(float maxErr, float maxVar, float deathRoomTime, float outTime, float tarPos)
{
    //默认err为pid位置环的误差，有输入值时err为设定位置与当前值的误差
    if(ABS(tarPos+1)<0.001)
    {
        err = ABS(pidPos.error);
    }
    else
    {
        err = ABS(tarPos-getPosition());//设定值-反馈值
    }
    if (delayTime == 0) // 首次开始
    {
        varCale.clear();
        delay.getCycleT(); // 清除
    }

    var = varCale.caleVar(err);

//    delayTime += delay.getCycleT() * 1000; //单位转换为ms
    //getCycleT()函数有问题
    delayTime += 2; //若任务运行频率为500Hz则默认为2ms
    if (ABS(err) < maxErr && var < maxVar && delayTime > deathRoomTime)
    {
        delayTime = 0;
        return 1;
    }
    if (delayTime > outTime)
    {
        delayTime = 0;
        return 1;
    }
    return 0;
}
