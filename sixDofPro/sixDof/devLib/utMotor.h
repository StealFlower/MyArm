#ifndef __UTMOTOR_H__
#define __UTMOTOR_H__

#include "fdcan.h"
#include "board.h"
#include "crc.h"
#include "cycle.h"
#include "varCalt.h"
#include "filter.h"

#define REDUCTION_RATIO 9.1f // 电机减速比
#define MaxToleranceLostNum 5 //最大允许的丢包数
#define RAD_TO_DEGREE 57.295779513f // 弧度转化为角度

#define USE_UT_Encoder 1

enum U_Chanel : uint8_t {
    ch1 = 0,
    ch2 = 1,
};

//主控向副控发送读取数据参数
enum acquireParam : uint16_t {
    info_Param=0x101,//包括电机id，模式，错误码，是否使能(0、1)、通道、方向
    TW_Param=0x102,//电机的T，W参数
    LWACC_Param = 0x103,//电机的LW，Acc参数
    TempPos_Param = 0x104,//电机转子位置、温度
    gyro_Param_01 = 0x105,//电机的六轴传感器数据
    gyro_acc_Param = 0x106,
    acc_Param_12 = 0x107,
};
//副控返回指令
enum FeedbackParam : uint16_t {
    info_FB = 0x91,//包括电机id，模式，错误码，是否使能(0、1)、通道、方向
    TW_FB = 0x92,//电机的T，W参数
    LWACC_FB = 0x93,//电机的LW，Acc参数
    TempPos_FB = 0x94,//电机转子位置、温度
    gyro_FB_01 = 0x95,//电机的六轴传感器数据
    gyro_acc_FB = 0x96,
    acc_FB_12 = 0x97,
};
struct MOTOR_send_t{

    //待发送的各项数据
    unsigned short mode; //0 为停转，5 为开环缓慢转动，10 为闭环伺服控制

    //实际给FOC的指令力矩为：K_P*delta_Pos + K_W*delta_W + T
    float T; 	//期望关节的输出力矩（电机本身的力矩）（Nm）
    float W; 	//期望关节速度（电机本身的速度）(rad/s)
    float Pos; 	//期望关节位置（rad）
    float K_P; 	//关节刚度系数
    float K_W; 	//关节速度系数(阻尼)

    uint32_t Res;
};
//电机接受数据
struct MOTOR_recv_t{

    //解读得出的电机数据
    unsigned char motor_id; //电机ID
    unsigned char mode; //0:空闲, 5:开环转动, 10:闭环FOC控制
    int Temp; //温度
    unsigned char MError; //错误码

    float T; 	// 当前实际电机输出力矩
    float W; 	// 当前实际电机速度（高速）
    float LW; 	// 当前实际电机速度（低速）
    int Acc; 	// 电机转子加速度
    float Pos; 	// 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）Rad为单位
    float Pos_Deg; 	// 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）Deg为单位

    float gyro[3]; // 电机驱动板6轴传感器数据
    float acc[3];
};

struct utMotor{
public:
	utMotor(FDCAN_HandleTypeDef *hfdcan,uint8_t Id,uint8_t ch,int8_t direc = 1);
    int float_to_uint(float x, float x_min, float x_max, int bits){
        float span = x_max - x_min;
        float offset = x_min;
        if(x > x_max)
            x = x_max;
        else if(x < x_min)
            x = x_min;
        return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    float uint_to_float(int x_int, float x_min, float x_max, int bits){
        float span = x_max - x_min;
        float offset = x_min;
        return (float) (x_int*((float)span/(float)((1<<bits)-1))+offset);
    }

    float motorPosOffset; //电机位置偏置
    bool onforceFlag; //电机使能标志位
    bool offsetOkFlag; //电机校准完成标志位
    struct MotorInfo {
        FDCAN_HandleTypeDef *hfdcan;
        CanPort canx;
        uint8_t Id;
        uint16_t mode;
        uint8_t ch;
        int8_t direc;
    }motorInfo;

    struct Err {
        bool motorTempErr;//电机温度异常位
        bool motorLostFlag = 1;//电机丢失标志位
    }errInfo;

    inline int8_t acquire_dir(){
        return motorInfo.direc;
    };

	CanSendMsg *motorSendMsg;
    MOTOR_recv_t motorRecvMsg;

	int planNum;
    uint8_t planIndex;
    MitParam **paramList;
	float setPosition;
    float posErr;

public:


	//对象列表
    static utMotor **objectPtrList;
    //对象数量
    static int objectNum;
    //主控接收副控参数回传
	bool canHandle(uint32_t *pRxHeader,uint8_t *pRxData);

    float caledPos;//校准后的点击位置，°为单位
    #if USE_UT_Encoder
    // float bias;//关节轴偏置
    float Encoder_Angle;//编码器角度，°为单位
    #endif

    void mitCtrl(float p_des,float v_des,float kp,float kd,float t_ff);
    void ctrlPosition(float setPosition);
    void motorEnable();
    void motorDisable();

    float getPosition();

    void setPlan(int planIndex,MitParam *paramPtr);
    void setParam(int planIndex,float p_des,float v_des,float kp,float kd,float t_ff);

	Cycle delay;
	VarCale varCale;
	float err, var;
	float delayTime;
	uint8_t hereFlag;
    #if USE_UT_Encoder
    uint8_t caleMotor(float bias,float MechRatio);

    VarCale TorqueVarCale;
    float torqueVar;
    uint8_t lockstate;
    uint16_t lockcnt;
    float getOutTorque();
    void setOutLimit(float );
    #else
    uint8_t ctrlMotorOffset(float resetspeed,float kd,float outTime = 10000);//默认10s
    #endif
	uint8_t exitStatus(float maxErr, float maxVar, float deathRoomTime, float outTime, float tarPos = -1);

private:
    uint8_t reachFlag;
    bool motorReadyFlag;
    uint8_t msgLostTimes;

};

#endif