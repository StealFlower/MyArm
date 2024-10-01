#ifndef __CYBERGEAR_H__
#define __CYBERGEAR_H__

#include "board.h"
#include "cycle.h"
#include "fdcan.h"
#include "varCalt.h"
#include "pid.h"
#include "motor.h"

#define CM_P_MIN -12.5f
#define CM_P_MAX 12.5f
#define CM_V_MIN -30.0f
#define CM_V_MAX 30.0f
#define CM_KP_MIN 0.0f
#define CM_KP_MAX 500.0f
#define CM_KD_MIN 0.0f
#define CM_KD_MAX 5.0f
#define CM_T_MIN -12.0f
#define CM_T_MAX 12.0f

#define RAD_TO_DEGREE 57.295779513f

//typedef struct _MitParam
//{
//    float p_des; float v_des; float kp; float kd; float t_ff;
//}MitParam;

#pragma pack(1)
struct exCanIdInfo{
    uint32_t id:8;
    uint32_t data:16;
    uint32_t mode:5;
    uint32_t res:3;
};
#pragma pack()

extern uint8_t MCUId[8];
///@brief 可读写单个参数列表(7019~701D为最新版本固件可读)
enum paramList : uint16_t{
    run_mode_index = 0X7005,
/// @note 0:运控模式1:位置模式2:速度模式3:电流模式 uint8_t 1byte
    iq_ref_index = 0X7006,
/// @note 电流模式下的Iq指令 float 4byte
    spd_ref_index = 0X700A,
/// @note 速度模式下的速度指令 float 4byte
    limit_torque_index = 0X700B,
/// @note 转矩限制 float 4byte
    cur_kp_index = 0X7010,
/// @note 电流kp float 4byte
    cur_ki_index = 0X7011,
/// @note 电流ki float 4byte
    cur_filt_gain_index = 0X7014,
/// @note 电流滤波系数filt_gain float 4byte
    loc_ref_index = 0X7016,
/// @note 位置模式下的位置指令 float 4byte
    limit_spd_index = 0X7017,
/// @note 位置模式下的速度限制 float 4byte
    limit_cur_index = 0X7018,
/// @note 速度位置模式下的电流限制 float 4byte
    mechPos_index = 0X7019,
/// @note 负载端计圈机械角度 float 4byte
    iqf_index = 0X701A,
/// @note iq滤波值 float 4byte
    mechVel_index = 0X701B,
/// @note 负载端转速 float 4byte
    VBUS_index = 0X701C,
/// @note 电压 float 4byte
    rotation_index = 0X701D,
/// @note 圈数 int16_t 2byte
};

enum modeList : uint8_t {
    mitMode = 0,
    posMode = 1,
    spdMode = 2,
    curMode = 3,
};


struct CyberMotor{
public:
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

    void mitCtrl(float p_des,float v_des,float kp,float kd,float t_ff);
    void pidCtrl(float kp,float ki,float kd);
    void spdCtrl(float spd_ref,float limit_cur);
    void curCtrl(float iq_ref);

    /// Pid速度环
	Pid pidSpd;
	/// Pid位置环
	Pid pidPos;
    float caledPos; //pid模式下校准后的位置 单位 °
	void PosPid(float position);
    void SpdPid(float speed);

    // 设置PID方案
	void setPidPlan(int planIndex, uint8_t spdOrPos, PidParam *paramPtr);
	// 设置PID参数
	void setPidParam(int planIndex, uint8_t spdOrPos, float kp, float ki, float kd, float intLimit, float outLimit);
    void setPidParam(int planIndex, uint8_t spdOrPos, float kp, float ki, float kd, float intLimit, float outLimitLower, float outLimitUpper);

    void posCtrl(float limit_spd,float loc_ref);//以rad/s && rad为单位
    void posCtrl(float limit_spd,float loc_ref,float limit_cur);//以rad/s && rad && A为单位

    void acquireInfo();
    void acquireParam(uint16_t paramIndex);

    void changeMode(uint8_t mode);//先修改运动模式，再进行电机使能
    void changeParam(uint16_t paramIndex,float paramValue);
    void changeParam(uint16_t paramIndex,float paramValue, CanSendMsg *msg);
    void changeParam(uint16_t paramIndex,uint8_t paramValue,MyCanTxStruct *msg);
    void motorEnable();
    void motorDisable();

    void setZero();
    uint8_t ctrlMotorOffset(float reSetSpeed, float maxErr, float outLimit, float outTime = 10000);//电机校准默认溢出时间为10s
    uint8_t mitCalibra(float reSetSpeed,float kd,float spdJudIndex = 2.f,float reachIndex = 2.f,float outTime = 5000);//电机校准默认溢出时间为5s
    void setId(uint8_t id);//先修改Id，再进行电机使能

public:
    //对象列表
    static CyberMotor **objectPtrList;
    //对象数量
    static int objectNum;

    struct MotorInfo{
        FDCAN_HandleTypeDef *hfdcan;
        CanPort canx;
        exCanIdInfo idInfo;
        uint16_t masterId;
    }motorInfo;
    bool offsetOkFlag;

    exCanIdInfo recvIdInfo;


    CanSendMsg *motorSendMsg;

    struct errInfo{
        uint8_t calibraFlag:1;//是否标定
        uint8_t HALLCodeErr:1;//霍尔编码器错误
        uint8_t MagCodeErr:1;//磁编码器错误
        uint8_t OverTemp:1;//过温
        uint8_t OverCur:1;//过流
        uint8_t lackVol:1;//欠压
    };

    struct CanInfo{
        uint8_t masterId;//主机CAN id
        uint8_t canId;//当前电机CAN id
        uint8_t errInfoFlag:6;//故障信息
        errInfo errInfo;
        uint8_t modeState:2;//模式状态:0:Reset模式(复位)1：Cali模式(标定)2:Motor模式(运行)

        float pos_rad;//当前角度(0,65535)->(-4PI,4PI) 2Byte
        float pos_dps;//当前角速度(0,65535)->(-30rad/s,30rad/s) 2Byte,已被转化为度每秒
        float lastPos_rad;//上次角度(0,65535)->(-4PI,4PI) 2Byte

        float pos_deg;//弧度转化后的角度值

        float toq_nm; //电机扭矩(0,65535)->(-12Nm,12Nm) 2Byte
        float currenT;//当前温度Temp(摄氏度)*10
    }canInfo;

    struct{
        float currentLimit;
        float speedLimit;
        float tempLimit;
        float torqueLimit;
        float torqueLimitTemp;

        double offSetPos;
        uint8_t posOffSetFlag;
    }otherInfo;

    struct CaltInfo{

        /*速度*/
        int16_t speed;
        float dps;

        /*位置*/
        // 总位置
		int32_t totalRound;		  ///< 总圈数
		int32_t totalEncoder;	  ///< 总角度(原始单位)
		int32_t lastTotalEncoder; // 上次总机械角度值
		int32_t totalAngle;		  ///< 总角度(度)
		float totalAngle_f;		  ///< 总角度(度)
        /*温度*/
        int16_t temperature; // 电机温度
        /*电流*/
        float trueCurrent; // 实际电流

        /*时间*/
        Cycle cycle;
        float intervalTime; // 信息间隔时间，单位s

        uint32_t msgCnt; // 接受消息计数
    }caltInfo;
    struct ParamInfo{//小米电机参数信息
        modeList run_mode;
        float iq_ref;
        float spd_ref;
        float limit_torque;
        float cur_kp;
        float cur_ki;
        float cur_filt_gain;
        float loc_ref;
        float limit_spd;
        float limit_cur;
        float mechPos;
        float mechPos_deg;
        float iqf;
        float mechVel;
        float VBUS;
        int16_t rotation;
    }paramInfo;

    CyberMotor(FDCAN_HandleTypeDef *hfdcan,uint8_t _CanId,uint16_t _MasterId);
    bool canHandle(uint32_t *pRxHeader,uint8_t *pRxData);

    int planNum;
    uint8_t planIndex;
    MitParam **paramList;
    void setPlan(int planIndex,MitParam *paramPtr);
    void setParam(int planIndex,float p_des,float v_des,float kp,float kd,float t_ff);
    float setPosition;
    float setSpeed;
    float posErr;//可能有问题
    void ctrlPosition(float setPosition);
    void ctrlPosition(float setPosition,float preTorque);
    float getPosition();
    float getPidPos();

    //判断到位
    Cycle delay;
    VarCale varCale;
    float err,var;
    float delayTime;
    uint8_t hereFlag;
    uint8_t reachFlag;
    uint8_t exitStatus(float maxErr,float maxVar,float deathRommTime,float outTime,float tarPos = -1);

private:

    float motorPosOffset; // 电机位置偏置 单位 °
    void fillParam(uint16_t paramIndex,float paramValue, CanSendMsg *msg);
    void fillParam(uint16_t paramIndex,uint8_t paramValue, CanSendMsg *msg);
    void fillParam(uint16_t paramIndex,int16_t paramValue, CanSendMsg *msg);

    void fillParam(uint16_t paramIndex,float paramValue, MyCanTxStruct *msg);
    void fillParam(uint16_t paramIndex,uint8_t paramValue, MyCanTxStruct *msg);
    void fillParam(uint16_t paramIndex,int16_t paramValue, MyCanTxStruct *msg);

    void fillParam(uint16_t paramIndex,float paramValue){
        fillParam(paramIndex,paramValue,motorSendMsg);
    };
    void fillParam(uint16_t paramIndex,uint8_t paramValue){
        fillParam(paramIndex,paramValue,motorSendMsg);
    };
    void fillParam(uint16_t paramIndex,int16_t paramValue){
        fillParam(paramIndex,paramValue,motorSendMsg);
    };

};

#endif
