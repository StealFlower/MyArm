#ifndef __VISION_H__
#define __VISION_H__

#include "board.h"
#include "online.h"
#include "devList.h"

#pragma pack(1)
struct VisionSendStruct{
    uint8_t header = 0xA5;
    uint8_t type = 5;
    uint8_t is_start;    
    double time_stamp;
    float joint_position[6];

    uint16_t CRC16CheckSum;
};

#pragma pack()


#pragma pack(1)
struct VisionRecvStruct{
    uint8_t header = 0xA5;
    uint8_t type = 3;
    double time_stamp;
    
    float time_from_start;
    float joint_position[6];
    float joint_velocity[6];
    uint8_t is_end;
    int16_t frame_id;
     uint16_t CRC16CheckSum;
};
#pragma pack()


struct Vision
{
public:
    // 接收缓冲
    uint8_t tmpRecvBuff[sizeof(VisionRecvStruct) + 1];
    //接收大臂数据
    struct VisionRecvStruct vision_recv_struct;
    struct VisionSendStruct vision_send_struct;

    uint8_t ArmReceiveOkFlag;
    //DMA配置发送数据
    LL_DMA_InitTypeDef vision_Tx_DMA_InitStruct;
    //算法通信使能标志
    uint8_t enableFlag;
    void init();
    void sendData(void);
    Online online;
};

extern Vision vision;

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/
    void USART1_IRQHandler(void);
#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif
