#ifndef __FDCAN_H__
#define __FDCAN_H__
#include "board.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);
void MX_FDCAN3_Init(void);

class CanSendMsg
{
public:
    CanSendMsg(FDCAN_HandleTypeDef *hfdcan, bool enableFlag = true, uint32_t IdType = FDCAN_STANDARD_ID);
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    FDCAN_HandleTypeDef *hfdcan;
    bool enableFlag;
	uint32_t IdType;
};

struct MyCanTxStruct
{
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];

    MyCanTxStruct(uint32_t IdType = FDCAN_STANDARD_ID,uint8_t *Data = NULL){
        this->txHeader.IdType = IdType;

        txHeader.TxFrameType = FDCAN_DATA_FRAME;
        txHeader.DataLength = FDCAN_DLC_BYTES_8;
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
        txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        txHeader.MessageMarker = 0;

        if(Data){
            memcpy(txData,Data,8);
        }
        else{
            memset(txData,0,8);
        }
    }
};

typedef struct
{
    uint32_t StdId;
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
} MyCanRxStruct;

typedef struct{
    uint32_t ExtId;
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
}MyCanRxExtStruct;

class Can
{
private:
public:
    MyCanRxStruct can1RxMsg;
    MyCanRxStruct can2RxMsg;
    MyCanRxStruct can3RxMsg;

    MyCanRxExtStruct can1RxExtMsg;

    void init(FDCAN_HandleTypeDef *hfdcan);

    void sendMsg();
    void sendExtMsg();
    void disforceHandle();
};
extern Can can;

uint8_t self_CAN_Transmit(FDCAN_HandleTypeDef *hfdcan, MyCanTxStruct *TxMessage);

enum CanPort
{
	can1 = 0, ///< CAN1
	can2 = 1, ///< CAN2
	can3 = 2  ///< CAN3
};

#ifdef __cplusplus
extern "C"
{
#endif

    void FDCAN1_IT0_IRQHandler(void);
    void FDCAN2_IT0_IRQHandler(void);
    void FDCAN3_IT0_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
