/*****************************************************************************
File name: TDT_Device\inc\motor.h
Author: 郑俊元
Version: 1.3.1.191119_alpha
Date: 19.10.15
History:
	——————————————————————————————————————————————————————————————————————————
	参考Readme.md
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "board.h"
#include "online.h"
#include "pid.h"
#include "fdcan.h"
#include "varCalt.h"
#include "filter.h"

/**
 * @addtogroup TDT_Motor
 * @{
 */



/// 电机类型-带G的为云台电机，用于CanInfo根据电机类型进行解析
enum MotorType
{
	M2006,
	M3508,
	M3510,
	GM3510,
	GM6020,
	RMDX4,
};

enum SpdOrPos
{
	spd = 0,
	pos = 1
};

/**
 * @brief CAN信息类，仅提供给Motor的canInfo
 */
struct Motor
{
public:
    // 对象列表
    static Motor **objectPtrList;
    // 对象数量
    static int objectNum;

	/// 电机常规参数
	/// 结构体-电机基本信息-初始化时填充，只允许构造器赋值
	struct MotorInfo
	{
		MotorType type; ///< 电机类型
		CanPort canx;	///< 挂载CAN总线
		uint32_t stdID; ///< 电机CAN总线反馈StdID
	} motorInfo;
	/// 构造函数
	Motor(MotorType motorType, FDCAN_HandleTypeDef *hfdcan, uint32_t _Std_ID);
	/// 对电机进行初始化,包括CAN使能
	void motorInit(void);

	///< 电机使能标志位
	uint8_t enableFlag;

	/// 离线检测
	Online online;

public:
	u8 planIndex; // 当前电机使用的pid方案索引
	/// Pid速度环
	Pid pidSpd;
	/// Pid位置环
	Pid pidPos;
	// 设置PID方案
	void setPlan(int planIndex, uint8_t spdOrPos, PidParam *paramPtr);
	// 设置PID参数
	void setParam(int planIndex, uint8_t spdOrPos, float kp, float ki, float kd, float intLimit, float outLimit);
    void setParam(int planIndex, uint8_t spdOrPos, float kp, float ki, float kd, float intLimit, float outLimitLower, float outLimitUpper);
    // 设置速度限幅
	void setSpdLimit(float spdLimit);
	// 设置输出限幅
	void setOutLimit(float outLimit);

	/*电机控制方法*/
	/// 控制速度
	float setSpeed;
	void ctrlSpeed(float speed);
	/// 控制位置
	float setPosition, lastSetPos;
	/// 零点校准
	uint8_t offsetOkFlag;

	uint8_t ctrlMotorOffset(float reSetSpeed, float maxErr, float speedLimit, float outTime = 10000);
	void ctrlPosition(float position);
    void ctrlSinglePosition(float position);
    void ctrlGraPosition(float position,float gravity);
	/// 控制电流
	float setCurrent;
	float assistKp = 1;
	void ctrlCurrent(float current);
	/// 对温度进行限幅计算
	void overHeatProtect(int16_t temp);
	/// 电机电流发送值
	static CanSendMsg motorSendMsg[3][3];
    static void canMsgInit(void);
	void motorPowerOut(float canResult);

	/*反馈信息*/
	float getMotorSpeedLimit();
	float getMotorCurrentLimit();
	float getPosition();
	int16_t getSpeed();

	// 判断到位
	Cycle delay;
	VarCale varCale;
	float err, var;
	float delayTime;
	uint8_t hereFlag;
	uint8_t exitStatus(float maxErr, float maxVar, float deathRoomTime, float outTime, float tarPos = -1);

    // 判断堵转
    VarCale currentVarCale;
    float curVar;
    uint8_t lockState;
    uint16_t lockCnt;
public:
	/// 电机额外辅助数据
	/// 结构体-电机基本信息-初始化时填充，只允许构造器赋值
	struct
	{
		uint8_t pidOuterCnt;   ///< 外环计次-用于外环
		uint8_t posOffSetFlag; ///< 位置校零标志位
		double offSetPos;	   ///< 位置校零设定值变量
		float overHeatKp;	   ///< 电机过热保护系数，作用于内环输出
		float *powerOutKp;	   ///< 功率控制系数，作用于内环输出 , 功率输出限幅系数
		// 以下变量初始化函数中进行赋默认值
		float currentLimit;		///< 电流限制
		float speedLimit;		///< 速度限制
		float outLimitTemp;		///< 位置校零输出限幅系数临时存储
		float tempLimit;		///< 温度限制
		float criticalTemp;		///< 临界温度
		float maxOverTemp;		///< 最大超出温度
		uint8_t isDjiMotorFlag; ///< 否是DJI电机的标志位
	} otherInfo;

	/// Can信息集合
	struct CanInfo
	{
		/*速度*/
		int16_t speed;			 ///< 原始机械转速
		float dps;				 ///< 转速(度每秒)

		/*位置*/
		// 单圈位置
		int16_t offsetEncoder;			// 机械角度初始值
		int16_t encoder;				///< 原始机械编码器值
		int16_t lastEncoder;			///< 上次原始机械编码器值
		int16_t encoderCalibration;		///< 减去机械角度初值后的角度
		int16_t lastEncoderCalibration; ///< 上次减去机械角度初值后的角度
		float angleF;
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
	} canInfo;
	Kf dpsKf;
	// CAN首次接收信息处理
	void canOffset(uint8_t *pRxData);
	void canHandle(uint8_t *pRxData);
};

/// 结构体：电机列表，用于通过CAN口和ID检索（轮询）所有电机
typedef struct
{
	Motor *motorPoint; ///< 电机对象指针
} MotorList;

/// 电机对象标记
extern MotorList motorList[3][12];
/** @} */

#endif
