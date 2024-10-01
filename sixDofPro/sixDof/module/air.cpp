#include "air.h"


Air::Air() : valveMsg(&hfdcan3,1),pumpMsg(&hfdcan1,1),armValve(&hfdcan2,1){
    valveMsg.txHeader.Identifier = 0x180;
    valveMsg.txHeader.IdType=FDCAN_STANDARD_ID;
    //valveMsg.txHeader.TxFrameType=FDCAN_DATA_FRAME;
    valveMsg.txHeader.DataLength=FDCAN_DLC_BYTES_8;
    //valveMsg.txHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;

    armValve.txHeader.Identifier = 0x180;
    armValve.txHeader.IdType=FDCAN_STANDARD_ID;
    //armValve.txHeader.TxFrameType=FDCAN_DATA_FRAME;
    armValve.txHeader.DataLength=FDCAN_DLC_BYTES_8;
    //armValve.txHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;

    pumpMsg.txHeader.Identifier = 0x100;
    pumpMsg.txHeader.IdType=FDCAN_STANDARD_ID;
    //pumpMsg.txHeader.TxFrameType=FDCAN_DATA_FRAME;
    pumpMsg.txHeader.DataLength=FDCAN_DLC_BYTES_8;
    //pumpMsg.txHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;

}
void Air::SendDataUpdate(void){
    valveMsg.txData[1] = (airvalvestate[0])|(airvalvestate[1]<<1)|(airvalvestate[2]<<2)|(airvalvestate[3]<<3)|(airvalvestate[4]<<4)|(airvalvestate[5]<<5)|(airvalvestate[6]<<6)|(airvalvestate[7]<<7);
    armValve.txData[1] = armvalvestate;
    pumpMsg.txData[0] = MechPumpSpd>>8;
    pumpMsg.txData[1] = MechPumpSpd&0xff;
    pumpMsg.txData[2] = MechPumpSpd>>8;
    pumpMsg.txData[3] = MechPumpSpd&0xff;
    this->gugu.excGugu = ArmPumpSpd;
    this->gugu.takGugu = TakPumpSpd;
    this->gugu.update();
}
void Air::Stop(void){
    //气阀全部关闭
    for(u8 i=0;i<8;i++){
        airvalvestate[i]=false;
    }//所有气阀关断，三矿空接阀切换到空接，所有继电器打开

    //气泵全部停止
	ArmPumpSpd=guguStop;
	MechPumpSpd=pumpstop;
    TakPumpSpd = guguStop;
    Air::SendDataUpdate();
}

void Air::StateConvert(AirStateIndex state){
    switch(state){
        case airTakGold://金矿
            airvalvestate[PitchLeft] = false;
            airvalvestate[PitchMiddle] = false;
            airvalvestate[PitchRight] = false;
            MechPumpSpd = fullspeed;
            //ArmPumpSpd = fullspeed;
        break;
        case airTakSilver://银矿
            airvalvestate[PitchLeft] = true;
            airvalvestate[PitchMiddle] = true;
            airvalvestate[PitchRight] = true;
            MechPumpSpd = fullspeed;
        break;
        case airExchange://兑换
        break;
        case airExit://退出
            airvalvestate[PitchLeft] = false;
            airvalvestate[PitchMiddle] = false;
            airvalvestate[PitchRight] = false;
            MechPumpSpd = pumpstop;
            ArmPumpSpd = guguStop;
            TakPumpSpd = guguStop;
        break;
    }

}