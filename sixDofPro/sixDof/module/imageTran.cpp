#include "imageTran.h"
#include "kinematics.h"


/**
  * @brief 图传链路CAN回传数据处理
  */
void ImageTran::canHandle(MyCanRxStruct *rxMessage)
{
    if(rxMessage->rxHeader.Identifier>0x400 && rxMessage->rxHeader.Identifier<0x405)//0x401~0x404
    {
        uint8_t index=rxMessage->rxHeader.Identifier-0x401;
        memcpy(customCtrlData.data[index],rxMessage->rxData,8);
    }
    
    if(rxMessage->rxHeader.Identifier>0x500 && rxMessage->rxHeader.Identifier<0x503)//0x501~0x502
    {
        uint8_t index=rxMessage->rxHeader.Identifier-0x501;
        memcpy(keyMouseData.data[index],rxMessage->rxData,8);
    }
}



void ImageTran::CtrlerDataUpdate()
{
    if(customCtrlData.deltaInfo.onlineFlag == 1){//是delta控制器
        this->deltaDataUpdate();
    }
    else if(customCtrlData.teachInfo.onlineFlag == 2){//是示教器
        this->teachDataUpdate();
    }
}




float KfQ=0.01,KfR=20.0;
EndPoint ctrlPoint; //自定义控制器操作臂位姿
EndPoint offsetEndPoint,offsetCtrlPoint;
void ImageTran::deltaDataUpdate()
{
    //自定义控制器数据反馈低，通过卡尔曼滤波平滑处理
    deltaData.x     = deltaFliter[0].KalmanFilter(customCtrlData.deltaInfo.x, KfQ, KfR, 0);
    deltaData.y     = deltaFliter[1].KalmanFilter(customCtrlData.deltaInfo.y, KfQ, KfR, 0);
    deltaData.z     = deltaFliter[2].KalmanFilter(customCtrlData.deltaInfo.z, KfQ, KfR, 0);
    deltaData.pitch = deltaFliter[3].KalmanFilter(customCtrlData.deltaInfo.pitch, KfQ, KfR, 0);
    deltaData.roll  = deltaFliter[4].KalmanFilter(customCtrlData.deltaInfo.roll, KfQ, KfR, 0);
    deltaData.yaw   = deltaFliter[5].KalmanFilter(customCtrlData.deltaInfo.yaw, KfQ, KfR, 0);

    //更新自定义控制器Delta3+3操作臂位姿
    ctrlPoint.xPos  = deltaData.y     ;
    ctrlPoint.yPos  = -deltaData.x     ;
    ctrlPoint.zPos  = deltaData.z     ;
    ctrlPoint.pitch = -deltaData.pitch ;
    ctrlPoint.roll  =  deltaData.roll ;
    ctrlPoint.yaw   = deltaData.yaw   ;
}

float lastRoll;
int16_t RollRound;
void ImageTran::teachDataUpdate()
{
    teachData.yaw0 = teachFliter[0].KalmanFilter(customCtrlData.teachInfo.yaw0, KfQ, KfR, 0);
    teachData.yaw1 = teachFliter[1].KalmanFilter(customCtrlData.teachInfo.yaw1, KfQ, KfR, 0);
    teachData.yaw2 = teachFliter[2].KalmanFilter(customCtrlData.teachInfo.yaw2, KfQ, KfR, 0);
    teachData.pitch1 = teachFliter[3].KalmanFilter(customCtrlData.teachInfo.pitch1, KfQ, KfR, 0);
    teachData.roll1 = teachFliter[4].KalmanFilter(customCtrlData.teachInfo.roll1, KfQ, KfR, 0);
    teachData.wristPitch = teachFliter[5].KalmanFilter(customCtrlData.teachInfo.wristPitch, KfQ, KfR, 0);

    lastRoll = teachData.wristRoll;
    teachData.wristRoll = teachFliter[6].KalmanFilter(customCtrlData.teachInfo.wristRoll, KfQ, KfR, 0);
    if(teachData.wristRoll - lastRoll > 180.f)
    {
        RollRound--;
    }
    else if(teachData.wristRoll - lastRoll < -180.f)
    {
        RollRound++;
    }
    teachData.wristRollTotal = teachData.wristRoll + RollRound * 360.f;
}