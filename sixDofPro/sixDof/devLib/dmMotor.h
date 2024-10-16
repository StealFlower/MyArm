#ifndef DM_MOTOR_H
#define DM_MOTOR_H
#include "board.h"
#include "fdcan.h"
#include "cycle.h"
#include "varCalt.h"
#include "online.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

//typedef struct _MitParam
//{
//    float p_des; float v_des; float kp; float kd; float t_ff;
//}MitParam;

struct DmMotor
{
public:
    int float_to_uint(float x, float x_min, float x_max, int bits)
    {
        /// Converts a float to an unsigned int, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    float uint_to_float(int x_int, float x_min, float x_max, int bits)
    {
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }
    
    void mitCtrl(float p_des, float v_des, float kp, float kd, float t_ff);
    void motorEnable();
    void motorDisable();
    void saveZero();
    void clearErr();

public:
    // 对象列表
    static DmMotor **objectPtrList;
    // 对象数量
    static int objectNum;

    struct MotorInfo
    {
        FDCAN_HandleTypeDef *hfdcan;
        CanPort canx;     ///< 挂载CAN总线
        uint8_t canId;
        uint16_t masterId; ///< 调试助手设置(Master ID)
    } motorInfo;

    enum ErrCode
    {
        OverVol = 8,
        UnderVol,
        OverCur,
        MosOverTemp,
        CoiOverTemp, // 线圈过温
        LostCommuni, // 通讯丢失
        OverLoad     // 过载
    };

    CanSendMsg *motorSendMsg;
    struct CanInfo
    {
        uint8_t masterId;
        uint8_t errCode;
        float pos_rad;
        float pos_dps;
        float vel_rads;
        float vel_dps;
        float toq_nm;   // 电机扭矩
        float tMos_c;   // 驱动上MOS的平均温度
        float tRotor_c; // 电机内部线圈的平均温度
    } canInfo;

    DmMotor(FDCAN_HandleTypeDef *hfdcan,uint8_t _CanId,uint16_t _MasterId);
    bool canHandle(uint8_t *pRxData);

    int planNum;
    uint8_t planIndex;
    MitParam **paramList;
    void setPlan(int planIndex, MitParam *paramPtr);
    void setParam(int planIndex, float p_des, float v_des, float kp, float kd, float t_ff);
    float setPosition;
    float posErr;
    void ctrlPosition(float SetPosition,float tff = 0.000001f,float vdes = 0.000001f);
    float getPosition();
    
    /// 离线检测
	Online online;

    // 判断到位
	Cycle delay;
	VarCale varCale;
	float err, var;
	float delayTime;
	uint8_t hereFlag;
	uint8_t exitStatus(float maxErr, float maxVar, float deathRoomTime, float outTime, float tarPos = -1);
};

#endif