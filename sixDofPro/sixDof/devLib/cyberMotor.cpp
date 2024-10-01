/******************************
File name: TDT_devLib\cybermotor.cpp
@description: 小米电机CyberMotor框架
@function:
	——————————————————————————————————————————————————————————————————————————
	void CyberMotor::acquireInfo()
	——————————————————————————————————————————————————————————————————————————
	void CyberMotor::acquireParam(uint16_t paramIndex)
	——————————————————————————————————————————————————————————————————————————
	void CyberMotor::motorEnable()
	——————————————————————————————————————————————————————————————————————————
	void CyberMotor::motorDisable()
	——————————————————————————————————————————————————————————————————————————
	void CyberMotor::setZero()
	——————————————————————————————————————————————————————————————————————————
	void CyberMotor::setId(uint8_t id)
	——————————————————————————————————————————————————————————————————————————
    bool CyberMotor::canHandle(uint32_t *pRxIdinfo,uint8_t *pRxData)
    ——————————————————————————————————————————————————————————————————————————
    void CyberMotor::mitCtrl(float p_des,float v_des,float kp,float kd,float t_ff)
    ——————————————————————————————————————————————————————————————————————————
    void CyberMotor::setPlan(int planIndex, MitParam *paramPtr)
    ——————————————————————————————————————————————————————————————————————————
    void CyberMotor::setParam(int planIndex, float p_des, float v_des, float kp, float kd, float t_ff)
    ——————————————————————————————————————————————————————————————————————————
    void CyberMotor::ctrlPosition(float setPosition)
    ——————————————————————————————————————————————————————————————————————————
    float CyberMotor::getPosition()
    ——————————————————————————————————————————————————————————————————————————
    void CyberMotor::changeParam(uint16_t paramIndex,float paramValue)
    ——————————————————————————————————————————————————————————————————————————
    void CyberMotor::changeMode(uint8_t mode)
    ——————————————————————————————————————————————————————————————————————————
    void CyberMotor::fillParam(uint16_t paramIndex,uint8_t paramValue)//存在三个重载
    ——————————————————————————————————————————————————————————————————————————
    void CyberMotor::spdCtrl(float spd_ref,float limit_cur)
    ——————————————————————————————————————————————————————————————————————————
    uint8_t CyberMotor::exitStatus(float maxErr, float maxVar, float deathRoomTime, float outTime, float tarPos)
    ——————————————————————————————————————————————————————————————————————————
    uint8_t CyberMotor::ctrlMotorOffset(float reSetSpeed, float maxErr, float outLimit, float outTime)
    ——————————————————————————————————————————————————————————————————————————
@author: 吴滨航
@date: 23/11/23
@history:
    ——————————————————————————————————————————————————————————————————————————
    24.1.20修改小米电机校准函数以及电机失能处理，以降低CAN帧率
    ——————————————————————————————————————————————————————————————————————————
    24.1.10 测试初步完成
    ——————————————————————————————————————————————————————————————————————————
    23.12.29 完成所有功能，加入参数读写功能，待测试
	——————————————————————————————————————————————————————————————————————————
    23.11.23 迁移至G4框架中
	——————————————————————————————————————————————————————————————————————————
	23.11.15 首次完成(F4)
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "cyberMotor.h"

uint8_t MCUId[8];
//对象列表外部初始化
CyberMotor **CyberMotor::objectPtrList = 0;
//对象数量外部初始化
int CyberMotor::objectNum = 0;

CyberMotor::CyberMotor(FDCAN_HandleTypeDef *hfdcan,uint8_t _CanId,uint16_t _MasterId){//masterid出厂为0

    if(objectPtrList == 0){
        objectPtrList = (CyberMotor **)malloc(sizeof(CyberMotor *));
    }
    else{
        objectPtrList = (CyberMotor **)realloc(objectPtrList,sizeof(CyberMotor *) * objectNum + 1);
    }

    objectPtrList[objectNum] = this;
    objectNum += 1;

    motorInfo.hfdcan = hfdcan;
    motorInfo.canx = (hfdcan == &hfdcan1 ? can1 : (hfdcan == &hfdcan2 ? can2 : can3));
    motorInfo.masterId = _MasterId;
    motorInfo.idInfo.id = _CanId;


    motorSendMsg = new CanSendMsg(hfdcan,true,FDCAN_EXTENDED_ID);//小米电机默认使用扩展帧

    // pid默认反馈值填充
    pidSpd.fbValuePtr = &canInfo.pos_dps;
    pidPos.fbValuePtr = &caledPos;
    pidSpd.setValuePtr = &setSpeed;
    pidPos.setValuePtr = &setPosition;
}
/// @brief 获取电机ID和MCU唯一标识符
void CyberMotor::acquireInfo(){
    this->motorInfo.idInfo.mode = 0;

    this->motorInfo.idInfo.res = 0;
    this->motorInfo.idInfo.data = this->motorInfo.masterId;
    for(uint8_t i=0;i<8;i++){
        motorSendMsg->txData[i] = 0;
    }

    MyCanTxStruct txMessage;

    memcpy(&motorSendMsg->txHeader.Identifier,&this->motorInfo.idInfo,sizeof(motorSendMsg->txHeader.Identifier));

}
/// @brief 单个参数读取,立即执行
/// @param paramIndex 见参数列表
void CyberMotor::acquireParam(uint16_t paramIndex){
    this->motorInfo.idInfo.mode = 17;
    this->motorInfo.idInfo.data = this->motorInfo.masterId;
    this->motorInfo.idInfo.res = 0;

    MyCanTxStruct tmpMessage(FDCAN_EXTENDED_ID);//小米电机默认使用扩展帧
    tmpMessage.txData[0] = (paramIndex & 0xFF);
    tmpMessage.txData[1] = (paramIndex >> 8);

    memcpy(&tmpMessage.txHeader.Identifier,&this->motorInfo.idInfo,sizeof(tmpMessage.txHeader.Identifier));
    self_CAN_Transmit(motorInfo.hfdcan,&tmpMessage);

}
/// @brief 修改电机参数
/// @param paramIndex 参数索引
/// @param paramValue 传入参数值
void CyberMotor::changeParam(uint16_t paramIndex,float paramValue){
    fillParam(paramIndex,paramValue);
    memcpy(&motorSendMsg->txHeader.Identifier,&this->motorInfo.idInfo,sizeof(motorSendMsg->txHeader.Identifier));
}
void CyberMotor::changeParam(uint16_t paramIndex,float paramValue, CanSendMsg *msg){
    fillParam(paramIndex,paramValue,msg);
    memcpy(&msg->txHeader.Identifier,&this->motorInfo.idInfo,sizeof(motorSendMsg->txHeader.Identifier));
}
void CyberMotor::changeParam(uint16_t paramIndex,uint8_t paramValue,MyCanTxStruct *msg){
    fillParam(paramIndex,paramValue,msg);
    memcpy(&msg->txHeader.Identifier,&this->motorInfo.idInfo,sizeof(motorSendMsg->txHeader.Identifier));
}
/// @brief 电机使能
void CyberMotor::motorEnable(){
    this->motorInfo.idInfo.mode = 3;

    this->motorInfo.idInfo.res = 0;
    this->motorInfo.idInfo.data = this->motorInfo.masterId;

	MyCanTxStruct txMessage(FDCAN_EXTENDED_ID);
    memset(txMessage.txData,0,8);


	memcpy(&txMessage.txHeader.Identifier,&this->motorInfo.idInfo,sizeof(txMessage.txHeader.Identifier));

    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);

}
/// @brief 电机失能
void CyberMotor::motorDisable(){
    this->motorInfo.idInfo.mode = 4;

    this->motorInfo.idInfo.res = 0;
    this->motorInfo.idInfo.data = this->motorInfo.masterId;

    MyCanTxStruct txMessage(FDCAN_EXTENDED_ID);//正常运行的时候data区需要清零，data[0]=1清故障

	memcpy(&txMessage.txHeader.Identifier,&this->motorInfo.idInfo,sizeof(txMessage.txHeader.Identifier));//调用失能回调函数时只发送txMessage不额外发送消息
    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);
}
/// @brief 设置电机机械零点,立即执行
void CyberMotor::setZero(){
    exCanIdInfo tmpInfo;
    uint8_t tmpData[8];
    tmpInfo.mode = 6;
    tmpInfo.id = this->motorInfo.idInfo.id;
    tmpInfo.res = 0;
    tmpInfo.data = this->motorInfo.masterId;
    memset(tmpData,0,8);

    tmpData[0] = 1;


    MyCanTxStruct txMessage(FDCAN_EXTENDED_ID);

    memcpy(&txMessage.txHeader.Identifier,&tmpInfo,sizeof(txMessage.txHeader.Identifier));

    memcpy(txMessage.txData,tmpData,8);

    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);
}
/// @brief 运控模式下电机的校准函数
/// @param resetSpeed 电机校准速度 单位degree/s
/// @param kd 电机的运控参数kd
/// @return 返回校准结果
uint8_t CyberMotor::mitCalibra(float resetSpeed,float kd,float spdJudIndex,float reachIndex,float outTime){

    this->delayTime += delay.getCycleT() * 1000; // 单位为ms
    this->mitCtrl(0, (resetSpeed / RAD_TO_DEGREE), 0, kd, 0);
    if((ABS(this->canInfo.pos_dps) >= ABS(resetSpeed / spdJudIndex)) && (reachFlag == 0)){
        reachFlag = 1;
    }
    if(reachFlag && (ABS(this->canInfo.pos_dps) <= ABS(resetSpeed / reachIndex))){
        this->setZero();
        this->setZero();
        reachFlag = 0;
        resetSpeed = 0;
        return 1;
    }
    if (this->delayTime > (outTime * 1000))
    {
        this->setZero();
        this->setZero();
        this->delayTime = 0;
        return 1;
    }
    return 0;
}

/// @brief 设置电机ID
/// @param id 要设置的ID
/// @note 只能在电机未使能的情况下执行，电机使能状态该操作并不会改变电机ID
void CyberMotor::setId(uint8_t id){
    this->motorInfo.idInfo.mode = 7;

    this->motorInfo.idInfo.res = 0;
    this->motorInfo.idInfo.data = (this->motorInfo.masterId & 0xFF) | (id << 8);
    for(uint8_t i=0;i<8;i++){
        motorSendMsg->txData[i] = 0;
    }

	memcpy(&motorSendMsg->txHeader.Identifier,&this->motorInfo.idInfo,sizeof(motorSendMsg->txHeader.Identifier));
    this->motorInfo.idInfo.id = id;

}
/// @brief CAN接收数据处理
/// @return 0:错误 1:正常
bool CyberMotor::canHandle(uint32_t *pRxIdinfo,uint8_t *pRxData){

	if(((*pRxIdinfo >> 24)&0x1F) == 2){//反馈帧模式位为2
        this->recvIdInfo.data = (*pRxIdinfo >> 8)&0xFFFF;
        this->recvIdInfo.mode = (*pRxIdinfo >> 24)&0x1F;//0x1F:11111
		this->recvIdInfo.id = this->recvIdInfo.data & 0xFF;
		if(this->recvIdInfo.id != this->motorInfo.idInfo.id){
			return 0;
		}
        //扩展帧部分数据处理
        this->canInfo.masterId = *pRxIdinfo & 0xFF;
        this->canInfo.canId = this->recvIdInfo.data & 0xFF;
        this->canInfo.modeState = (this->recvIdInfo.data >> 14)&3;

        if(((this->recvIdInfo.data >> 8)&0x3F) == 0){//无错误
            this->canInfo.errInfoFlag = 0;
        }
        else{//错误处理
            this->canInfo.errInfoFlag = 1;
            this->canInfo.errInfo.lackVol = this->recvIdInfo.data&1;
            this->canInfo.errInfo.OverCur = (this->recvIdInfo.data>>1)&1;
            this->canInfo.errInfo.OverTemp = (this->recvIdInfo.data>>2)&1;
            this->canInfo.errInfo.MagCodeErr = (this->recvIdInfo.data>>3)&1;
            this->canInfo.errInfo.HALLCodeErr = (this->recvIdInfo.data>>4)&1;
            this->canInfo.errInfo.calibraFlag = (this->recvIdInfo.data>>5)&1;
        }
        //数据区数据处理
        this->canInfo.pos_rad = uint_to_float(int((pRxData[0] << 8) | pRxData[1]), CM_P_MIN, CM_P_MAX, 16);
        this->canInfo.pos_dps = uint_to_float(int((pRxData[2] << 8) | pRxData[3]), CM_V_MIN, CM_V_MAX, 16) * RAD_TO_DEGREE;//转化为度/s
        this->canInfo.toq_nm = uint_to_float(int((pRxData[4] << 8) | pRxData[5]), CM_T_MIN, CM_T_MAX, 16);
        this->canInfo.currenT = float((pRxData[6] << 8) | pRxData[7])/10.f;
        //弧度角度转化
        this->canInfo.pos_deg = this->canInfo.pos_rad * RAD_TO_DEGREE;
        //过圈检测
        if(canInfo.pos_rad - canInfo.lastPos_rad > PI){
            caltInfo.totalRound--;
        }
        else if(canInfo.pos_rad - canInfo.lastPos_rad < -PI){
            caltInfo.totalRound++;
        }
        caltInfo.totalAngle = (canInfo.pos_rad + caltInfo.totalRound * (2*PI)) * RAD_TO_DEGREE;
        caltInfo.totalAngle_f = (canInfo.pos_rad + caltInfo.totalRound * (2*PI)) * RAD_TO_DEGREE;
        //记录此次角度
        canInfo.lastPos_rad = canInfo.pos_rad;

        return 1;
    }
    else if((*pRxIdinfo >> 24) == 0){//反馈帧模式位为0(获取设备ID)
        if((*pRxIdinfo & 0xFF) != 0xFE){
            return 0;
        }
        //扩展帧部分数据处理
        this->recvIdInfo.data = (*pRxIdinfo >> 8)&0xFFFF;
        this->recvIdInfo.mode = (*pRxIdinfo >> 24)&0x1F;//0x1F:11111
		this->canInfo.canId = this->recvIdInfo.data & 0xFF;

		if(this->canInfo.canId != this->motorInfo.idInfo.id){
			return 0;
		}

        //数据区数据处理,传递64位MCU唯一标识符
        for(u8 i=0;i<8;i++){
            MCUId[i] = pRxData[i];
        }
        return 1;
    }
    else if(((*pRxIdinfo >> 24)&0x1F) == 17){//反馈帧模式位为17(获取参数)
        //扩展帧部分数据处理
        this->recvIdInfo.mode = (*pRxIdinfo >>24)&0x1F;
        this->recvIdInfo.data = (*pRxIdinfo >>8)&0xFFFF;
        this->recvIdInfo.id = *pRxIdinfo & 0xFF;

        if((this->recvIdInfo.id != motorInfo.masterId) || (this->recvIdInfo.data != motorInfo.idInfo.id)){
            return 0;
        }
        else {
            //数据区参数处理
            uint16_t paramIndex;
            paramIndex = (pRxData[1] << 8) | pRxData[0];
            switch(paramIndex){
                case run_mode_index:
                    //电机运行模式读取
                    paramInfo.run_mode = (modeList)pRxData[4];
                break;
                case iq_ref_index:
                    //电流环模式下的Iq指令读取
                    memcpy(&paramInfo.iq_ref,&pRxData[4],4);
                break;
                case spd_ref_index:
                    //速度环模式下的速度指令读取
                    memcpy(&paramInfo.spd_ref,&pRxData[4],4);
                break;
                case limit_torque_index:
                    //转矩限制
                    memcpy(&paramInfo.limit_torque,&pRxData[4],4);
                break;
                case cur_kp_index:
                    //电流环模式下的Kp读取
                    memcpy(&paramInfo.cur_kp,&pRxData[4],4);
                break;
                case cur_ki_index:
                    //电流环模式下的Ki读取
                    memcpy(&paramInfo.cur_ki,&pRxData[4],4);
                break;
                case cur_filt_gain_index:
                    //电流环模式下的滤波系数读取
                    memcpy(&paramInfo.cur_filt_gain,&pRxData[4],4);
                break;
                case loc_ref_index:
                    //位置环模式下的位置指令读取
                    memcpy(&paramInfo.loc_ref,&pRxData[4],4);
                break;
                case limit_spd_index:
                    //位置环模式下的速度限制读取
                    memcpy(&paramInfo.limit_spd,&pRxData[4],4);
                break;
                case limit_cur_index:
                    //速度位置模式下的电流限制读取
                    memcpy(&paramInfo.limit_cur,&pRxData[4],4);
                break;
                case mechPos_index:
                    //负载端计圈机械角度读取
                    memcpy(&paramInfo.mechPos,&pRxData[4],4);
                    paramInfo.mechPos_deg = paramInfo.mechPos * RAD_TO_DEGREE;
                    this->caledPos = paramInfo.mechPos_deg - motorPosOffset;
                break;
                case iqf_index:
                    //iq滤波值读取
                    memcpy(&paramInfo.iqf,&pRxData[4],4);
                break;
                case mechVel_index:
                    //负载端转速读取
                    memcpy(&paramInfo.mechVel,&pRxData[4],4);
                break;
                case VBUS_index:
                    //电压读取
                    memcpy(&paramInfo.VBUS,&pRxData[4],4);
                break;
                case rotation_index:
                    //圈数读取
                    memcpy(&paramInfo.rotation,&pRxData[4],4);
                break;
            }
        }
        return 1;
    }
    else {
        return 0;
    }
}
/// @brief 电机模式写入（通信类型18）掉电丢失,立即执行
/// @param mode 0:运控模式1:位置模式2:速度模式3:电流模式
/// @note 更改电机模式的时候，实际上不需要像说明书一样先更改模式再使能电机，在电机使能的状态下更改模式也是有效的
void CyberMotor::changeMode(uint8_t mode){

    MyCanTxStruct txMessage(FDCAN_EXTENDED_ID);
    fillParam(run_mode_index,mode,&txMessage);

    memcpy(&txMessage.txHeader.Identifier,&this->motorInfo.idInfo,sizeof(motorSendMsg->txHeader.Identifier));

    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);
}

void CyberMotor::fillParam(uint16_t paramIndex, uint8_t paramValue, CanSendMsg *msg){
    this->motorInfo.idInfo.data = motorInfo.masterId;
    this->motorInfo.idInfo.res = 0;
    this->motorInfo.idInfo.mode = 18;//单个参数写入时通信类型为18

    memset(msg->txData,0,8);
	msg->txData[0] = (paramIndex & 0xFF);
    msg->txData[1] = (paramIndex >> 8);

    memcpy(&msg->txData[4],&paramValue,1);
}

void CyberMotor::fillParam(uint16_t paramIndex,int16_t paramValue, CanSendMsg *msg){
    this->motorInfo.idInfo.data = motorInfo.masterId;
    this->motorInfo.idInfo.res = 0;
    this->motorInfo.idInfo.mode = 18;//单个参数写入时通信类型为18

    memset(msg->txData,0,8);
    msg->txData[0] = (paramIndex & 0xFF);
    msg->txData[1] = (paramIndex >> 8);

    memcpy(&msg->txData[4],&paramValue,2);
}

void CyberMotor::fillParam(uint16_t paramIndex,float paramValue, CanSendMsg *msg){
    this->motorInfo.idInfo.data = motorInfo.masterId;
    this->motorInfo.idInfo.res = 0;
    this->motorInfo.idInfo.mode = 18;//单个参数写入时通信类型为18

    memset(msg->txData,0,8);
    msg->txData[0] = (paramIndex & 0xFF);
    msg->txData[1] = (paramIndex >> 8);

    memcpy(&msg->txData[4],&paramValue,4);
}

void CyberMotor::fillParam(uint16_t paramIndex,float paramValue, MyCanTxStruct *msg){
    this->motorInfo.idInfo.data = motorInfo.masterId;
    this->motorInfo.idInfo.res = 0;
    this->motorInfo.idInfo.mode = 18;//单个参数写入时通信类型为18

    memset(msg->txData,0,8);
    msg->txData[0] = (paramIndex & 0xFF);
    msg->txData[1] = (paramIndex >> 8);

    memcpy(&msg->txData[4],&paramValue,4);
}

void CyberMotor::fillParam(uint16_t paramIndex,int16_t paramValue, MyCanTxStruct *msg){
    this->motorInfo.idInfo.data = motorInfo.masterId;
    this->motorInfo.idInfo.res = 0;
    this->motorInfo.idInfo.mode = 18;//单个参数写入时通信类型为18

    memset(msg->txData,0,8);
    msg->txData[0] = (paramIndex & 0xFF);
    msg->txData[1] = (paramIndex >> 8);

    memcpy(&msg->txData[4],&paramValue,2);
}

void CyberMotor::fillParam(uint16_t paramIndex,uint8_t paramValue, MyCanTxStruct *msg){
    this->motorInfo.idInfo.data = motorInfo.masterId;
    this->motorInfo.idInfo.res = 0;
    this->motorInfo.idInfo.mode = 18;//单个参数写入时通信类型为18

    memset(msg->txData,0,8);
    msg->txData[0] = (paramIndex & 0xFF);
    msg->txData[1] = (paramIndex >> 8);

    memcpy(&msg->txData[4],&paramValue,1);
}

/// @brief MIT控制模式
/// @param p_des 期望位置 单位rad
/// @param v_des 期望角速度 单位rad/s
/// @param kp_kd 线性系数和微分系数
/// @param t_ff 扭矩,常用作前馈
/// @note 对位置进行控制时， kd 不能赋 0， 否则会造成电机震荡， 甚至失控。
void CyberMotor::mitCtrl(float p_des,float v_des,float kp,float kd,float t_ff){
    this->motorInfo.idInfo.mode = 1;

    this->motorInfo.idInfo.res = 0;

    uint16_t p_des_tmp,v_des_tmp,kp_tmp,kd_tmp,t_ff_tmp;
    p_des_tmp = float_to_uint(p_des,CM_P_MIN,CM_P_MAX,16);
    v_des_tmp = float_to_uint(v_des,CM_V_MIN,CM_V_MAX,16);
    kp_tmp = float_to_uint(kp,CM_KP_MIN,CM_KP_MAX,16);
    kd_tmp = float_to_uint(kd,CM_KD_MIN,CM_KD_MAX,16);
    t_ff_tmp = float_to_uint(t_ff,CM_T_MIN,CM_T_MAX,16);

    this->motorInfo.idInfo.data = t_ff_tmp;

    motorSendMsg->txData[0] = p_des_tmp >> 8;
    motorSendMsg->txData[1] = p_des_tmp & 0xFF;
    motorSendMsg->txData[2] = v_des_tmp >> 8;
    motorSendMsg->txData[3] = v_des_tmp & 0xFF;
    motorSendMsg->txData[4] = kp_tmp >> 8;
    motorSendMsg->txData[5] = kp_tmp & 0xFF;
    motorSendMsg->txData[6] = kd_tmp >> 8;
    motorSendMsg->txData[7] = kd_tmp & 0xFF;


	memcpy(&motorSendMsg->txHeader.Identifier,&this->motorInfo.idInfo,sizeof(motorSendMsg->txHeader.Identifier));

}
/// @brief 速度控制模式
/// @param spd_ref 期望速度
/// @param limit_cur 速度位置模式下电流限制
void CyberMotor::spdCtrl(float spd_ref,float limit_cur){
    static uint8_t ctrlstep = 0;

    if(ctrlstep == 0){
        changeParam(limit_cur_index,limit_cur);
        ctrlstep++;
        return;
    }
    else if(ctrlstep == 1){
        changeParam(spd_ref_index,spd_ref);
        ctrlstep = 0;
        return;
    }
}
/// @brief 电流控制，需要切换至电流控制模式
/// @param iq_ref 期望电流值
void CyberMotor::curCtrl(float iq_ref){
    changeParam(iq_ref_index,iq_ref);
    return;
}
/// @brief 位置控制，立即执行
/// @param limit_spd 位置模式下电机转至期望角度的速度 0~30rad/s
/// @param loc_ref 期望角度(度为单位)
void CyberMotor::posCtrl(float limit_spd,float loc_ref){

    MyCanTxStruct txMessage(FDCAN_EXTENDED_ID);

    changeParam(limit_spd_index,limit_spd,&txMessage);
    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);
    changeParam(loc_ref_index,(loc_ref / RAD_TO_DEGREE),&txMessage);
    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);

}
/// @brief 位置控制,立即执行
/// @param limit_spd 位置模式下电机转至期望角度的速度 0~30rad/s
/// @param loc_ref 期望角度(弧度为单位)
/// @param limit_cur 速度位置模式下的电流限制
void CyberMotor::posCtrl(float limit_spd,float loc_ref,float limit_cur){
    MyCanTxStruct txMessage(FDCAN_EXTENDED_ID);

    changeParam(limit_spd_index,limit_spd,&txMessage);
    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);

    changeParam(limit_cur_index,limit_cur,&txMessage);
    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);

    changeParam(loc_ref_index,(loc_ref / RAD_TO_DEGREE),&txMessage);
    self_CAN_Transmit(motorInfo.hfdcan,&txMessage);
}
/// @brief 小米电机参数结构体初始化
/// @param planIndex 方案号
/// @param paramPtr 指向参数结构体的指针
void CyberMotor::setPlan(int planIndex, MitParam *paramPtr)
{
    if (planIndex >= planNum)
    {
        planNum = planIndex + 1;
        if (planNum == 0)
            paramList = (MitParam **)malloc(sizeof(MitParam *));
        else
            paramList = (MitParam **)realloc(paramList, sizeof(MitParam *) * planNum); // 为指针paramList分配planNum个MitParam大小的空间
    }
    paramList[planIndex] = paramPtr;
}
/// @brief 小米电机参数结构体填充
/// @param planIndex 方案号
/// @param p_des 位置
/// @param v_des 速度
/// @param kp 位置系数
/// @param kd 速度系数
/// @param t_ff 扭矩前馈
/// @note 该函数在setPlan函数后调用，且一般不必将p_des、v_des赋为除0以外的值
void CyberMotor::setParam(int planIndex, float p_des, float v_des, float kp, float kd, float t_ff)
{
    paramList[planIndex]->p_des = p_des;
    paramList[planIndex]->v_des = v_des;
    paramList[planIndex]->kp = kp;
    paramList[planIndex]->kd = kd;
    paramList[planIndex]->t_ff = t_ff;
}

void CyberMotor::ctrlPosition(float setPosition)
{
    this->setPosition = setPosition;
    posErr = setPosition-(canInfo.pos_rad * RAD_TO_DEGREE);
    float setPos_rad = setPosition/RAD_TO_DEGREE;//弧度角度转化
    mitCtrl(setPos_rad, paramList[planIndex]->v_des, paramList[planIndex]->kp, paramList[planIndex]->kd, paramList[planIndex]->t_ff);
}

void CyberMotor::ctrlPosition(float setPosition,float preTorque)//加入力矩前馈的运控控制
{
    this->setPosition = setPosition;
    posErr = setPosition-(canInfo.pos_rad * RAD_TO_DEGREE);
    float setPos_rad = setPosition/RAD_TO_DEGREE;//弧度角度转化
    mitCtrl(setPos_rad, paramList[planIndex]->v_des, paramList[planIndex]->kp, paramList[planIndex]->kd, paramList[planIndex]->t_ff);
}
float CyberMotor::getPosition()
{
    return (canInfo.pos_rad * RAD_TO_DEGREE);
}
float CyberMotor::getPidPos(){
    return this->caledPos;
}
/// @brief 小米电机到位检测
/// @param maxErr 最大误差
/// @param maxVar 最大方差
/// @param deathRoomTime 死区时间
/// @param outTime 溢出时间
/// @param tarPos 目标位置
/// @return 到位返回1，否则返回0
/// @note 该函数默认以500Hz运行，在修改任务运行频率时需要同时修改delayTime为执行周期
uint8_t CyberMotor::exitStatus(float maxErr, float maxVar, float deathRoomTime, float outTime, float tarPos)
{
    if (ABS(tarPos+1)<0.001)
    {
        err = ABS(this->setPosition - this->caledPos);
    }
    else
    {
        err = ABS(tarPos - this->caledPos); // 设定值-反馈值
    }
    if (delayTime == 0) // 首次开始
    {
        varCale.clear();
        delay.getCycleT(); // 清除
    }

    var = varCale.caleVar(err);

//    delayTime += delay.getCycleT() * 1000; //单位转换为ms
    //getCycleT()函数有问题
    delayTime += 2; //默认2ms,小米电机应当注意时间周期
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
/// @brief 小米电机校准
/// @param reSetSpeed 校准速度
/// @param maxErr 最大误差
/// @param outTime 溢出时间
/// @param outLimit 电机电流限制
/// @return 电机堵转（校准结束）返回1
uint8_t CyberMotor::ctrlMotorOffset(float reSetSpeed, float maxErr, float outLimit, float outTime){
    delayTime += delay.getCycleT() * 1000; // 单位为ms
    switch (otherInfo.posOffSetFlag){
        case 0:
        // 记录方案初始输出限幅以便校准完后恢复
        otherInfo.currentLimit = pidSpd.paramList[0]->outLimit;
        // 获取外环信息
        otherInfo.offSetPos = *pidPos.fbValuePtr;
        //改变电机运动模式为电流模式
        changeMode(curMode);
        //下一步
        otherInfo.posOffSetFlag++;
        break;
        case 1:
        // 更改PID输出限幅为校零模式
        pidSpd.paramList[0]->outLimit = outLimit;

        // 给定校准速度,开始旋转校准
        otherInfo.offSetPos += reSetSpeed;
        PosPid(otherInfo.offSetPos);
        // 检测堵转
        if (ABS(posErr) > maxErr)
            otherInfo.posOffSetFlag++;
        break;
        case 2:
        //重置电机零点
		this->setZero();
        this->motorPosOffset = this->paramInfo.mechPos_deg;

        otherInfo.posOffSetFlag = 0;

        delayTime = 0;
        // 恢复输出限幅为正常模式
        pidSpd.paramList[0]->outLimit = otherInfo.currentLimit;

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
/// @brief PID位置环
/// @param position 目标位置
void CyberMotor::PosPid(float position){
	this->setPosition = position;
    posErr = position - *pidPos.fbValuePtr;
    /*PID计算，默认输出到对象的result变量*/
    pidPos.calculate(planIndex);
    /*速度环*/
    SpdPid(pidPos.result);
}
/// @brief PID速度环
/// @param speed 期望速度
void CyberMotor::SpdPid(float speed){
    this->setSpeed = speed;
    /*PID计算，默认输出到对象的result变量*/
    pidSpd.calculate(planIndex);

    pidSpd.result = LIMIT(pidSpd.result, -23, 23);
    // 电流环
    curCtrl(pidSpd.result);
}
/// @brief PID参数初始化
/// @param planIndex 方案号
/// @param spdOrPos 位置or速度
/// @param paramPtr 参数指针，指向结构体
void CyberMotor::setPidPlan(int planIndex, uint8_t spdOrPos, PidParam *paramPtr){
    if (spdOrPos == spd)
        pidSpd.setPlan(planIndex, paramPtr);
    else
        pidPos.setPlan(planIndex, paramPtr);
}
/// @brief PID参数填充
/// @param planIndex 方案号
/// @param spdOrPos 速度or位置
/// @param kp\ki\kd
/// @param intLimit 积分限幅
/// @param outLimit 输出限幅
void CyberMotor::setPidParam(int planIndex, uint8_t spdOrPos, float kp, float ki, float kd, float intLimit, float outLimit){
    if (spdOrPos == spd)
        pidSpd.setParam(planIndex, kp, ki, kd, intLimit, outLimit);
    else
        pidPos.setParam(planIndex, kp, ki, kd, intLimit, outLimit);
}
/// @brief PID参数填充
/// @param planIndex 方案号
/// @param spdOrPos 速度or位置
/// @param kp\ki\kd
/// @param intLimit 积分限幅
/// @param outLimitLower 输出下限
/// @param outLimitUpper 输出上限
void CyberMotor::setPidParam(int planIndex, uint8_t spdOrPos, float kp, float ki, float kd, float intLimit, float outLimitLower, float outLimitUpper){
    if (spdOrPos == spd)
        pidSpd.setParam(planIndex, kp, ki, kd, intLimit, outLimitLower, outLimitUpper);
    else
        pidPos.setParam(planIndex, kp, ki, kd, intLimit, outLimitLower, outLimitUpper);
}
