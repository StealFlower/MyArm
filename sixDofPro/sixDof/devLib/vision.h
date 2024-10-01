#ifndef __VISION_H__
#define __VISION_H__

#include "board.h"
#include "online.h"

///是否启用应答模式
#define ANSWER_MODE 0
#define MAX_RECV_BUFF_SIZE sizeof(vision_Recv_BigArm_Struct)
//启用大臂调试模式
#if VISION_DEBUG
extern uint8_t DEFORCE_Flag[7];
#endif


enum SendDataType : uint8_t{
    BigArmData,
    visionData,
    AcquireRecognize,
};

enum RecvDataType : uint8_t{
    navigation = 1,
    visionRecv2 = 2,
    visionRecvBigArm = 3,
    visionRecvStation = 8,
};

// #pragma pack(1)
// ///视觉发送结构体（MCU->NUC）.
// struct vision_Send_Struct_t
// {
// 	uint8_t header = 0xA5;
// 	uint8_t type = 5;

// 	float yaw;
// 	float pitch;
// 	uint8_t colorandinvincible ;//空，哨兵无敌，5，4，3，2，1无敌，自己颜色（红1蓝0）
// 	uint8_t BeatSentry : 1 ;
// 	uint8_t islift : 1 ;
// 	uint8_t usegun : 2 ;
// 	uint8_t lockcommand : 1 ;
// 	float nominal_bulletspeed;
// 	float bullet_speed;
// 	uint8_t beat_mode;
// 	uint8_t buff_type;
// 	float time_stamp;
// 	int16_t frame_id;
// 	uint16_t CRC16CheckSum;
// };
// #pragma pack()
///向视觉发送大臂结构体
#pragma pack(1)
struct vision_Send_BigArm_Struct{
    uint8_t header = 0xA5;
    uint8_t type = 5;

    double time_stamp;
    

    int16_t frame_id;
    uint16_t CRC16CheckSum;
};
#pragma pack()
#pragma pack(1)
struct AcquireRecognition_Struct{
    uint8_t header = 0xA5;
    uint8_t type = 4;

    float yaw;
    float time_stamp;
    uint8_t detect_mode;

    int16_t frame_id;
    uint16_t CRC16CheckSum;
};
#pragma pack()
///视觉接收结构体（NUC->MCU）.
// #pragma pack(1)
// struct vision_Recv_Struct_t1
// {
// 	uint8_t header = 0xA5;
// 	uint8_t type = 1; //第二种UsartSender
//     double time_stamp; //视觉时间系下时间戳，用于给电控做时间同步用

// 	float yaw; // yaw角度
// 	float pitch; // pitch角度
//     float distance = 0;

//     uint8_t obj_id : 4;
// 	uint8_t no_obj : 1;
// 	uint8_t beat : 1; //开火指令

//     uint8_t obj_spinnig : 1;
//     uint8_t IsEnergyMode : 1;
//     uint8_t clearInterval : 1;

// 	int16_t frame_id;
// 	uint16_t CRC16CheckSum;

// };
// #pragma pack()
// #pragma pack(1)
// struct vision_Recv_Struct_t2
// {
// 	uint8_t header = 0xA5;
// 	uint8_t type = 2; //第二种UsartSender


// 	float yaw; // yaw角度
// 	float pitch; // pitch角度
//     double time_stamp; //视觉时间系下时间戳，用于给电控做时间同步用

// 	uint8_t no_obj : 1;
// 	uint8_t beat : 1; //开火指令

// 	int16_t frame_id;
// 	uint16_t CRC16CheckSum;

// };
// #pragma pack()
#pragma pack(1)
struct vision_Recv_BigArm_Struct{
    uint8_t header = 0xA5;
    uint8_t type = 7;//记得告诉yegger改
    double time_stamp;//算法提供的时间戳

    float time_from_start;
    float joint_position[7];
    float joint_velocity[7];
    uint8_t TransmitOkFlag;

    int16_t frame_id;
    uint16_t CRC16CheckSum;
};
#pragma pack()
#pragma pack(1)
struct VisionRecognize_RecvStruct//视觉提供的兑换站位姿结构体
{
    uint8_t frameHeader; // 0xA5/
    uint8_t type = 8;
    double time_stamp;

    float yaw = 0;
    float pitch = 0;
    float roll = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    bool finish;


    int16_t frame_id;
    uint16_t CRC16CheckSum;
};
#pragma pack()

#pragma pack(1)
struct SpeedFbData
{
	uint8_t  header = 0xA5;
	uint8_t  type = 5;

	int16_t SpeedFb_X = 0 ;
	int16_t SpeedFb_Y = 0 ;

	float    timeStamp;
	int16_t  frame_id;
	uint16_t CRC16CheckSum;
};
#pragma pack()
#pragma pack(1)
struct CommanderData
{
	uint8_t  header = 0xA5;
	uint8_t  type = 2;

	uint8_t auto_status = 0;
	float  target_x = -1;
	float  target_y = -1;
	uint8_t Destination = 0 ;

	float    timeStamp;
	int16_t  frame_id;
	uint16_t CRC16CheckSum;
};
struct CommandReplyData{
    uint8_t header = 0xA5;
    uint8_t type = 3;
    int16_t reply_frame_id;
    int16_t frame_id;
    uint16_t CRC16CheckSum;
};
#pragma pack()
#pragma pack(1)
struct MatchInfoData
{
	uint8_t header = 0xA5;
	uint8_t type = 4;

	int16_t match_time = -200 ;
	uint16_t robot_hp[14];
    uint8_t self_color : 2;
    uint8_t if_shooting : 2;
    uint8_t if_bullet_empty : 1;
    uint8_t buff_point_status : 2;
    uint16_t virtual_blood = 1500;
    uint16_t base_blood[2];

	float time_stamp;
	int16_t frame_id;
	uint16_t CRC16CheckSum;
};
#pragma pack()
// #pragma pack(1)
// struct NavigationData
// {
//     uint8_t header = 0xA5;
//     uint8_t type = 1;
//     double target_x_speed = 0 ;
//     double target_y_speed = 0 ;
// 	uint8_t spin : 1 ;
// 	uint8_t lift_up : 1 ;
// 	uint8_t on_slope : 1 ;
// 	uint8_t cannotreach : 1 ;
//     double time_stamp;
//     int16_t frame_id;
//     uint16_t CRC16CheckSum;
// };
// #pragma pack()
// #pragma pack(1)
// struct YawData{
//     uint8_t header = 0xA5;
//     uint8_t type = 6;
//     bool fly_mode;
//     float yaw_change;
//     double time_stamp;
//     int16_t frame_id;
//     uint16_t CRC16CheckSum;
// };
// #pragma pack()
// #pragma pack(1)
// struct LAMData{
//     uint8_t header = 0xA5;
//     uint8_t type = 5;
//     uint8_t obj1_x;
//     uint8_t obj1_y;
//     uint8_t obj2_x;
//     uint8_t obj2_y;
//     uint8_t obj3_x;
//     uint8_t obj3_y;
//     uint8_t lam_x;
//     uint8_t lam_y;

//     uint16_t CRC16CheckSum;
// };
// #pragma pack()
// 发送数据
// #pragma pack(1)
// struct VisionSendStruct
// {
//     uint8_t frameHeader; // 0xA5data
//     uint16_t data;
//     uint16_t CRC16CheckSum;
// };
// #pragma pack()

struct Vision
{
public:
    Vision();
    // 接收缓冲
    uint8_t tmpRecvBuff[sizeof(vision_Recv_BigArm_Struct) + 1];
    //接收视觉识别兑换站位姿结构体
    struct VisionRecognize_RecvStruct recvRecognize_RecvStruct;
    //接收大臂数据
    struct vision_Recv_BigArm_Struct BigArmRecvStruct;
    //接收其它数据
    // struct vision_Recv_Struct_t1 recvStruct1;
    // struct vision_Recv_Struct_t2 recvStruct2;
    //接收定位导航部分数据
    // struct CommandReplyData commandReplyData;
    // struct LAMData LAMdata;
    // struct YawData yawData;
    // struct NavigationData navigationData;

    // 发送数据
    // struct vision_Send_BigArm_Struct BigArmSendStruct;
    // struct vision_Send_Struct_t visionStruct;
    struct AcquireRecognition_Struct AcquireInfo;

    uint16_t armDataCnt;
    vision_Recv_BigArm_Struct* tmpArmBuff;
    uint8_t ArmReceiveOkFlag;
    uint16_t armDataLength;
    bool ArmBuffFullFlag;
    //DMA配置发送数据
    LL_DMA_InitTypeDef vision_BigArm_Tx_DMA_InitStruct;
    LL_DMA_InitTypeDef vision_Tx_DMA_InitStruct;
    LL_DMA_InitTypeDef vision_AquireRecognize_Tx_DMA_InitStruct;
    //算法通信使能标志
    uint8_t enableFlag;
    void init();
    void sendData(uint8_t type);

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
