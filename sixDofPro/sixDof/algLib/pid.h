#ifndef _PID_H
#define _PID_H
#include "board.h"
#include "cycle.h"

/// Pid参数存储结构体
typedef struct
{
    float kp;       ///< 比例系数
    float ki;       ///< 积分系数
    float kd;       ///< 微分系数
    float intLimit; ///< 积分限幅
    float outLimit; ///< 最终结果限幅
    float *fbValuePtr;
    /// 以下参数是更加高级也更加少用的参数
    ///@warning 部分参数及算法未测试
    struct
    {
        float positiveFBValue;      ///< 正反馈数值,会作用在结果上
        float integralThreshold;    ///< 积分分离阈值
        float kpAmpForIntegralSep;  ///< 误差大于积分分离阈值时的放大系数
        uint8_t integralMethod;     ///< 积分方法【正常=0，积分分离=1，变积分=2, 变积分2 =3】
        uint8_t differentialMethod; ///< 微分算法【正常=0，微分先行=1，加低通滤波】
        float VIntegralIndex;       ///< 变积分系数
        float DLPFIndex;            ///< 微分项低通滤波系数
        float S_Buff;               ///< 变积分增益
        float MAXlimit;             ///< 变积分上极限
        float MINlimit;             ///< 变积分下极限
        float alpha;                ///< 不完全微分系数
    } advancedPara;
    float resultMaxSides[2]; // 定义上下限
} PidParam;

/// Pid计算类
struct Pid
{
private:
    /*计算数据*/
    float lastError;  ///< 上次偏差——lastError = [lastError = error]
    float intError;   ///< 偏差积分 += [该次偏差 * 时间]——integralError += [error * T]
    float deltaError; ///< 偏差增量 = [该次偏差 - 上次偏差] / 时间——  deltaError	 = [error - lastError]
    float lastDError; ///< 上次偏差增量 = [上次偏差 - 上上次偏差] / 时间—— deltaError = [error - lastError]

public:
    float *setValuePtr;
    float *fbValuePtr;
    float error;  ///< 该次偏差 = [设定值 - 反馈值]——error = [setValue - feedbackValue]
    float result; ///< 限幅最终输出后结果

    // 构造器
    Pid();
    Pid(int planNum); // 构造器
    // PID算法实现
    float calculate(uint8_t planIndex);
    // 设定参数指针
    void setDataPtr(float *fb, int planIndex);
    // 设置pid方案
    void setPlan(int planIndex, PidParam *paramPtr);
    // 设置pid参数
    void setParam(int index, float kp, float ki, float kd, float intLimit, float outLimit);
    void setParam(int index, float kp, float ki, float kd, float intLimit, float outLimitLower, float outLimitUpper);
    // 检查PID参数
    bool paramCheck(uint8_t planIndex);
    // 清空相关变量
    void clear();

    // 设置正反馈值
    void setPositiveValue(uint8_t planIndex, float positiveValue);
    // 获取反馈值
    float getFbValue(int planIndex = 0);

    PidParam **paramList;

private:
    /*获取周期时间*/
    int planNum;       // 方案数量
    int lastPlanIndex; // 上一次运算的方案号
    Cycle pidCycle;
};

/**
 * @}
 */

#endif
