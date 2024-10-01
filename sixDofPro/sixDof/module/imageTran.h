#ifndef _IMAGE_TRAN_H_
#define _IMAGE_TRAN_H_
#include "board.h"
#include "fdcan.h"
#include "filter.h"

//图传链路数据：
//数据传输使用CAN通信，分成多帧收发，每帧上传频率为50Hz。
//自定义控制器数据由4个数据帧组成0x401~0x404，键鼠数据由
//2个数据帧组成0x501~0x502，共计每秒往CAN总线上传300帧
//应保证一个完整数据不被分成两帧，如一个float不被拆成两个

//自定义控制器数据
#pragma pack(1)
typedef union
{
    struct
    {
        //第一帧
        float x;
        float y;
        //第二帧
        float z;
        float pitch;
        //第三帧
        float yaw;
        float roll;
        //第四帧
        uint8_t key1;
        uint8_t key2;
        uint8_t onlineFlag;
        uint8_t ref[5];
    }deltaInfo;
    struct
    {
        //第一帧
        float yaw0;
        float yaw1;
        //第二帧
        float yaw2;
        float pitch1;
        //第三帧
        float roll1;
        float wristPitch;
        //第四帧
        uint8_t key1;
        uint8_t key2;
        uint8_t onlineFlag;
        float wristRoll;
        uint8_t ref;
    }teachInfo;
    uint8_t data[4][8]; //接收数据
}ImageTranRecvCustomCtrlData;
#pragma pack(0)


//键鼠数据
#pragma pack(1)
typedef union
{
    struct 
    {
        //第一帧
        int16_t vx;
        int16_t vy;
        int16_t vz;
        uint8_t leftJump;
        uint8_t rightJump;
        //第二帧
        union
        {
            uint16_t keyValue;
            struct
            {
                uint16_t W : 1;     ///< 0x0001
                uint16_t S : 1;     ///< 0x0002
                uint16_t A : 1;     ///< 0x0004
                uint16_t D : 1;     ///< 0x0008
                uint16_t SHIFT : 1; ///< 0x0010
                uint16_t CTRL : 1;  ///< 0x0020
                uint16_t Q : 1;     ///< 0x0040
                uint16_t E : 1;     ///< 0x0080
                uint16_t R : 1;     ///< 0x0100
                uint16_t F : 1;     ///< 0x0200
                uint16_t G : 1;     ///< 0x0400
                uint16_t Z : 1;     ///< 0x0800
                uint16_t X : 1;     ///< 0x1000
                uint16_t C : 1;     ///< 0x2000
                uint16_t V : 1;     ///< 0x4000
                uint16_t B : 1;     ///< 0x8000
            };
        };
        uint8_t onlineFlag;
        uint16_t ref[5];
    };
    uint8_t data[2][8]; //接收数据
}ImageTranRecvKeyMouseData;
#pragma pack(0)


struct ImageTran
{
public:    
    ImageTranRecvCustomCtrlData customCtrlData; //自定义控制器原始数据
    ImageTranRecvKeyMouseData keyMouseData;     //键鼠原始数据

    //CAN接收消息处理
    void canHandle(MyCanRxStruct *rxMessage);

    //卡尔曼滤波器
    Kf deltaFliter[6];
    //Delta3+3操作臂滤波后数据
    struct
    {
        float x;
        float y;
        float z;
        float pitch;
        float roll;
        float yaw;
        uint8_t key1;
        uint8_t lastKey1;
        uint8_t key1Press;
    }deltaData;

    //示教器滤波器
    Kf teachFliter[7];
    struct
    {
        float yaw0;
        float yaw1;
        float yaw2;
        float pitch1;
        float roll1;
        float wristPitch;
        float wristRoll;
        float wristRollTotal;
    }teachData;
    void CtrlerDataUpdate();
private:
    //Delta数据更新函数
    void deltaDataUpdate();
    //示教器数据更新函数
    void teachDataUpdate();
};

#endif