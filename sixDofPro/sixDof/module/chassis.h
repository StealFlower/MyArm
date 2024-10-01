#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "board.h"
#include "motor.h"
#include "asr.h"
#include "filter.h"

#define FEED_SOURCE_NUM 4
///TODO:定义底盘参数:底盘长、宽、底盘中心到轮轴心距离
#define CHASSIS_L  350.f//底盘长度
#define CHASSIS_W  382.f//底盘宽度
#define CHASSIS_R  76.f//麦轮半径

enum SetSpeedList
{
    remoteControlSetSpeed = 0,  //遥控器
    keyBoardSetSpeed = 1,       //键盘
    autoSetSpeed = 2,           //取矿自动
    visionSetSpeed = 3,
};

struct Speed
{
    float fb; //前后速度
    float lr; //左右速度
    float ro; //旋转速度
    
    void operator=(Speed speed)
    {
        this->fb=speed.fb;
        this->lr=speed.lr;
        this->ro=speed.ro;
    }
};


struct Chassis
{
private:
    u8 speedMode;                       //速度模式
	float wheelSetSpeed[4] = {0};       //底盘速度设定值
	Speed setSpeed;                     //速度设定值
	Speed lastIndex, tarIndex, setIndex,nowIndex; //速度系数
    
    Asr asr[2];
    float acc;
public:
    //机器人底盘电机的ID按所在象限定义
    //  2|1  0x202|0x201  左前|右前
    //  3|4  0x203|0x204  左后|右后
    //数组元素的定义顺序从左前开始按顺时针方向定义
    //  motor[0]|motor[1]  0x202|0x201  左前|右前
    //  motor[3]|motor[2]  0x203|0x204  左后|右后
    //TODO：数组元素的定义顺序也改成按所在象限定义
    Motor motor[4] = {
	    Motor(M3508,&hfdcan1,0x202),    //左前
    	Motor(M3508,&hfdcan1,0x201),    //右前
		Motor(M3508,&hfdcan1,0x204),    //右后
		Motor(M3508,&hfdcan1,0x203),    //左后
	};
    //构造函数
    Chassis();
    //底盘速度更新
    void update();
    
    u8 planIndex;//PID方案索引
    PidParam spdParam[2];
    //PID参数加载
    void loadParam();
    
    //键值系数
    float* keyIndex;
    //鼠标YAW轴滤波
    Kf yawKf;
    //获取当前速度比例
    float getNowIndex(u8 fbOrLr);

    //各反馈源速度列表
    Speed allFeedbackSourceSetSpeedList[FEED_SOURCE_NUM];
    //遥控器更新
    void remoteControlUpdate(Speed* structPointer);
    //键盘更新
    void keyboardUpdate(Speed* structPointer);
    //取矿自动怼上
    void takeOreUpdate(Speed* structPointer);
    //定位导航
    void visionSpeedUpdate(Speed* structPointer);
    void calcRealWheelSpeed(Speed* structPointer);
};


#endif 
