#include "utMotor.h"

//对象列表外部初始化
utMotor **utMotor::objectPtrList = 0;
//对象数量外部初始化
int utMotor::objectNum = 0;

utMotor::utMotor(FDCAN_HandleTypeDef *hfdcan,uint8_t Id,uint8_t ch,int8_t direc){
    if(objectPtrList == 0){
        objectPtrList = (utMotor **)malloc(sizeof(utMotor *));
    }
    else{
        objectPtrList = (utMotor **)realloc(objectPtrList,sizeof(utMotor *) * objectNum + 1);
    }

    objectPtrList[objectNum] = this;
    objectNum += 1;

    motorInfo.hfdcan = hfdcan;
    motorInfo.canx = (hfdcan == &hfdcan1 ? can1 : (hfdcan == &hfdcan2 ? can2 : can3));
    motorInfo.Id = Id;
    motorInfo.ch = ch;
    motorInfo.direc = direc;

    motorSendMsg = new CanSendMsg(hfdcan,true,FDCAN_STANDARD_ID);//宇树电机使用串口进行通信
}
/// @brief 电机上力
void utMotor::motorEnable(){
    this->motorInfo.mode = 10;
}
/// @brief 电机脱力
void utMotor::motorDisable(){
    this->motorInfo.mode = 0;
	motorSendMsg->txHeader.Identifier = 0x107;

	memcpy(&motorSendMsg->txData[4],&this->motorInfo.mode,2);

	motorSendMsg->txData[6] = 0;
	motorSendMsg->txData[7] = 0;
}
/// @brief 电机运控模式控制
/// @param p_des 目标位置
/// @param v_des 速度规划
/// @param kp 刚度系数
/// @param kd 速度系数(阻尼)
/// @param t_ff 力矩前馈
void utMotor::mitCtrl(float p_des,float v_des,float kp,float kd,float t_ff){


    static uint8_t ctrlstep = 0;
	static bool sendingFlag = 0;


    if(ctrlstep == 0){
        motorSendMsg->txHeader.Identifier = 0x105;

		memcpy(&motorSendMsg->txData[0],&t_ff,4);

		memcpy(&motorSendMsg->txData[4],&v_des,4);


        ctrlstep++;
    }
    else if(ctrlstep == 1){
        motorSendMsg->txHeader.Identifier = 0x106;

		memcpy(&motorSendMsg->txData[0],&p_des,4);

		memcpy(&motorSendMsg->txData[4],&kp,4);

        ctrlstep++;
    }
	else if(ctrlstep == 2){
		motorSendMsg->txHeader.Identifier = 0x107;

		memcpy(&motorSendMsg->txData[0],&kd,4);

		memcpy(&motorSendMsg->txData[4],&this->motorInfo.mode,2);

		motorSendMsg->txData[6] = 0;
		motorSendMsg->txData[7] = 0;

		ctrlstep = 0;
	}
}
/// @brief 电机配置方案
/// @param planIndex 参数方案号
/// @param paramPtr 参数方案指针
void utMotor::setPlan(int planIndex, MitParam *paramPtr){

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
/// @brief 向指定方案中填充参数
/// @param planIndex 方案号
/// @param p_des 目标位置
/// @param v_des 速度规划
/// @param kp 刚度系数
/// @param kd 速度系数
/// @param t_ff 力矩前馈
/// @note 宇树电机在mit模式下控制位置的时候速度系数需要大于刚度系数
void utMotor::setParam(int planIndex, float p_des, float v_des, float kp, float kd, float t_ff){

    paramList[planIndex]->p_des = p_des;
    paramList[planIndex]->v_des = v_des;
    paramList[planIndex]->kp = kp;
    paramList[planIndex]->kd = kd;
    paramList[planIndex]->t_ff = t_ff;

}
/// @brief 使用mit模式控制宇树电机
/// @param setPosition 期望角度
void utMotor::ctrlPosition(float setPosition){

    posErr = setPosition - this->caledPos;
    float setPos_rad = ((setPosition + this->motorPosOffset) * REDUCTION_RATIO  * 2 * PI)/RAD_TO_DEGREE;//弧度角度转化
    mitCtrl(setPos_rad, paramList[planIndex]->v_des, paramList[planIndex]->kp, paramList[planIndex]->kd, paramList[planIndex]->t_ff);

}
/// @brief 主控接收来自副控的反馈信息
/// @param pRxHeader 反馈帧ID
/// @param pRxData 反馈数据帧
/// @return 返回是否正确处理，若未能正确处理则返回0，否则返回1
bool utMotor::canHandle(uint32_t *pRxHeader,uint8_t *pRxData){
    switch(*pRxHeader){
        case info_FB:
            this->motorRecvMsg.motor_id = pRxData[0];
            this->motorRecvMsg.mode = pRxData[1];
            this->motorRecvMsg.MError = pRxData[2];
            this->motorInfo.ch = pRxData[3];
            this->motorInfo.direc = pRxData[4];
            this->onforceFlag = pRxData[5];
        break;
        case TW_FB:
            memcpy(&motorRecvMsg.T,pRxData,4);
            memcpy(&motorRecvMsg.W,&pRxData[4],4);
        break;
        case LWACC_FB:
            memcpy(&motorRecvMsg.LW,pRxData,4);
            memcpy(&motorRecvMsg.Acc,&pRxData[4],4);
        break;
        case TempPos_FB:
            memcpy(&motorRecvMsg.Temp,pRxData,4);
            memcpy(&motorRecvMsg.Pos,&pRxData[4],4);
            motorRecvMsg.Pos_Deg = motorRecvMsg.Pos * RAD_TO_DEGREE;
            caledPos = motorRecvMsg.Pos_Deg - motorPosOffset;
        break;
        case gyro_FB_01:
            memcpy(&motorRecvMsg.gyro[0],pRxData,4);
            memcpy(&motorRecvMsg.gyro[1],&pRxData[4],4);
        break;
        case gyro_acc_FB:
            memcpy(&motorRecvMsg.gyro[2],pRxData,4);
            memcpy(&motorRecvMsg.acc[0],&pRxData[4],4);
        break;
        case acc_FB_12:
            memcpy(&motorRecvMsg.acc[1],pRxData,4);
            memcpy(&motorRecvMsg.acc[2],&pRxData[4],4);
        break;
        default:
            return 0;
    }
	return 1;
}
/// @brief 返回宇树电机角度反馈
/// @return 返回宇树电机校准后的角度(单位以主控为准)
float utMotor::getPosition(){
    return this->caledPos;
}
float utMotor::getOutTorque(){
    return this->motorRecvMsg.T;
}
void utMotor::setOutLimit(float outLimit){

}
#if USE_UT_Encoder
uint8_t utMotor::caleMotor(float bias,float MechRatio){
    this->motorPosOffset = (this->Encoder_Angle - bias) * MechRatio + motorRecvMsg.Pos_Deg;
    return 1;
}
#else
uint8_t utMotor::ctrlMotorOffset(float resetspeed,float kd,float outTime){

    delayTime += delay.getCycleT() * 1000; // 单位为ms
    if((ABS(this->motorRecvMsg.W) >= ABS(resetspeed / 2.f)) && (!this->reachFlag)){
        this->reachFlag = 1;
    }
    if((ABS(this->motorRecvMsg.W) <= ABS(resetspeed / 3.f)) && this->reachFlag){
        motorPosOffset = motorRecvMsg.Pos_Deg;
        this->reachFlag = 0;
        mitCtrl(0, 0, 0, 0, 0);
        return 1;
    }
    else{

        mitCtrl(0, resetspeed, 0, kd, 0);
    }

    if(delayTime > outTime){
        delayTime = 0;
        mitCtrl(0, 0, 0, 0, 0);
        return 1;
    }
    return 0;
}
#endif
/// @brief 判断宇树电机是否到位
/// @param maxErr 到位判断的最大误差
/// @param maxVar 到位判断的最大方差
/// @param deathRoomTime 死区时间
/// @param outTime 溢出时间
/// @param tarPos 目标位置
/// @return 返回是否到位，若到位则返回1，否则返回0
/// @note 由于宇树电机的反馈帧频率大约在160Hz左右，因此即使task是以500Hz跑的，但实际上方差计算并不准确，待矫正
uint8_t utMotor::exitStatus(float maxErr, float maxVar, float deathRoomTime, float outTime, float tarPos){
    //默认err为pid位置环的误差，有输入值时err为设定位置与当前值的误差
    if(ABS(tarPos+1)<0.001)
    {
        err = ABS(posErr);
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