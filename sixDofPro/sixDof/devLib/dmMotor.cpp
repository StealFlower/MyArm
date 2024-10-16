#include "dmMotor.h"

// 指示灯状态

// 额定转速 120rpm => 4PI rad/s => 12.56637 rad/s
// 空载最大转速 200rpm => 6.67PI rad/s => 41.8879 rad/s
// 额定扭矩 3NM
// 峰值扭矩 7NM

//调参：
//MIT模式：增大Kp，位置更精准，同时更容易引起超调

// 对象列表外部初始化
DmMotor **DmMotor::objectPtrList = 0;
// 对象数量外部初始化
int DmMotor::objectNum = 0;

DmMotor::DmMotor(FDCAN_HandleTypeDef *hfdcan,uint8_t _CanId,uint16_t _MasterId)
{
    //为对象申请内存
    if (objectPtrList == 0)
        objectPtrList = (DmMotor **)malloc(sizeof(DmMotor *));
    else
        objectPtrList = (DmMotor **)realloc(objectPtrList, sizeof(DmMotor *) * objectNum + 1);

    // 记录对象指针
    objectPtrList[objectNum] = this;
    objectNum += 1;
    
    motorInfo.hfdcan = hfdcan;
    motorInfo.canx = (hfdcan == &hfdcan1 ? can1 : (hfdcan == &hfdcan2 ? can2 : can3));
    motorInfo.canId = _CanId;
    motorInfo.masterId = _MasterId;
    
    motorSendMsg = new CanSendMsg(hfdcan);
    motorSendMsg->txHeader.Identifier = motorInfo.canId;
}


/// @brief  CAN接收数据处理
/// @return 0：错误 1：正常
bool DmMotor::canHandle(uint8_t *pRxData)
{
    canInfo.masterId = (pRxData[0]) & 0x0F;
    canInfo.errCode = (pRxData[0]) >> 4;
    canInfo.pos_rad = uint_to_float(int((pRxData[1] << 8) | pRxData[2]), P_MIN, P_MAX, 16);         // (-12.5,12.5)
    canInfo.vel_rads = uint_to_float(int((pRxData[3] << 4) | (pRxData[4] >> 4)), V_MIN, V_MAX, 12); // (-45.0,45.0)
    canInfo.toq_nm = uint_to_float(int(((pRxData[4] & 0xF) << 8) | pRxData[5]), T_MIN, T_MAX, 12);  // (-18.0,18.0)
    canInfo.tMos_c = (float)(pRxData[6]);
    canInfo.tRotor_c = (float)(pRxData[7]);
    
    canInfo.pos_dps = canInfo.pos_rad*57.295779513;
    canInfo.vel_dps = canInfo.vel_rads*57.295779513;
    
    /// 更新在线状态
	this->online.update();
    if(canInfo.errCode>=8)return 0;
    else return 1;
}

/// @brief MIT控制模式
/// @param p_des,v_des,t_ff：范围可由调试助手进行设定，
/// @param kp,kd：Kp的范围为[0,500]，Kd的范围为[0,5]。
/// @note  对位置进行控制时， kd 不能赋 0， 否则会造成电机震荡， 甚至失控。
void DmMotor::mitCtrl(float p_des, float v_des, float kp, float kd, float t_ff)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(p_des, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(v_des, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    motorSendMsg->txHeader.DataLength = FDCAN_DLC_BYTES_8;
    motorSendMsg->txData[0] = (pos_tmp >> 8);
    motorSendMsg->txData[1] = pos_tmp;
    motorSendMsg->txData[2] = (vel_tmp >> 4);
    motorSendMsg->txData[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    motorSendMsg->txData[4] = kp_tmp;
    motorSendMsg->txData[5] = (kd_tmp >> 4);
    motorSendMsg->txData[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    motorSendMsg->txData[7] = tor_tmp;
}


/// @brief 进入电机
void DmMotor::motorEnable()
{
    motorSendMsg->txHeader.DataLength = FDCAN_DLC_BYTES_8;
    for (uint8_t i = 0; i < 7; i++)
    {
        motorSendMsg->txData[i] = 0xff;
    }
    motorSendMsg->txData[7] = 0xfc;
    
    MyCanTxStruct txMessage;
    txMessage.txHeader=motorSendMsg->txHeader;
    memcpy(txMessage.txData,motorSendMsg->txData,8);

    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);
}

/// @brief 退出电机
void DmMotor::motorDisable()
{
    motorSendMsg->txHeader.DataLength = FDCAN_DLC_BYTES_8;
    for (uint8_t i = 0; i < 7; i++)
    {
        motorSendMsg->txData[i] = 0xff;
    }
    motorSendMsg->txData[7] = 0xfd;
    
    MyCanTxStruct txMessage;
    txMessage.txHeader=motorSendMsg->txHeader;
    memcpy(txMessage.txData,motorSendMsg->txData,8);

    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);
}

/// @brief 保存位置零点
void DmMotor::saveZero()
{
    MyCanTxStruct txMessage;
    txMessage.txHeader.Identifier = motorInfo.canId;
    for (uint8_t i = 0; i < 7; i++)
    {
        txMessage.txData[i] = 0xff;
    }
    txMessage.txData[7] = 0xfe;

    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);
}
void DmMotor::clearErr()
{
    MyCanTxStruct txMessage;
    txMessage.txHeader.Identifier = motorInfo.canId;
    for (uint8_t i = 0; i < 7; i++)
    {
        txMessage.txData[i] = 0xff;
    }
    txMessage.txData[7] = 0xfb;

    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);
}

void DmMotor::setPlan(int planIndex, MitParam *paramPtr)
{
    if (planIndex >= planNum)
    {
        planNum = planIndex + 1;
        if (planNum == 0)
            paramList = (MitParam **)malloc(sizeof(MitParam *));
        else
            paramList = (MitParam **)realloc(paramList, sizeof(MitParam *) * planNum); // 为指针paramList分配planNum个PidParam大小的空间
    }
    paramList[planIndex] = paramPtr;
}

void DmMotor::setParam(int planIndex, float p_des, float v_des, float kp, float kd, float t_ff)
{
    paramList[planIndex]->p_des = p_des;
    paramList[planIndex]->v_des = v_des;
    paramList[planIndex]->kp = kp;
    paramList[planIndex]->kd = kd;
    paramList[planIndex]->t_ff = t_ff;
}


void DmMotor::ctrlPosition(float setPosition,float tff,float vdes)
{
    this->setPosition = setPosition;
    posErr = setPosition-canInfo.pos_dps;
    float setPos_rad = setPosition/57.295779513f;
    if(fabs(tff)<0.0001f && fabs(vdes)<0.0001f)
        mitCtrl(setPos_rad, paramList[planIndex]->v_des, paramList[planIndex]->kp, paramList[planIndex]->kd, paramList[planIndex]->t_ff);
    else 
        mitCtrl(setPos_rad, vdes, paramList[planIndex]->kp, paramList[planIndex]->kd, tff);       
}

float DmMotor::getPosition()
{
    return canInfo.pos_dps;
}

uint8_t DmMotor::exitStatus(float maxErr, float maxVar, float deathRoomTime, float outTime, float tarPos)
{
    if (ABS(tarPos+1)<0.001)
    {
        err = ABS(posErr);
    }
    else
    {
        err = ABS(tarPos - getPosition()); // 设定值-反馈值
    }
    if (delayTime == 0) // 首次开始
    {
        varCale.clear();
        delay.getCycleT(); // 清除
    }

    var = varCale.caleVar(err);

//    delayTime += delay.getCycleT() * 1000; //单位转换为ms
    //getCycleT()函数有问题
    delayTime += 2; //默认2ms
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

