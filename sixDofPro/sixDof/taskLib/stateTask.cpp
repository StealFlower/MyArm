#include "virtualTask.h"
#include "stateTask.h"
#include "offsetStateMachine.h"
#include "oreMotionStateMachine.h"

#include "dbus.h"
#include "devList.h"
#include "kinematics.h"
#include "vision.h"

class StateTask : public VirtualTask
{
public:
    StateTask(){};
    // 初始化
    void init() override;
    // 主函数
    void run() override;

    // 遥控更新回调
    void rcUpdateCallBack() override;
    // 任务使能回调
    void enableCallBack() override;
    // 任务失能回调
    void disableCallBack() override;
};

/*初始化*/
void StateTask::init()
{
    status = taskStateRun; // 任务运行
    // status = taskStateStop; //任务不运行

    // 电机参数加载
    loadParam();

}

/*遥控更新回调*/
void StateTask::rcUpdateCallBack()
{
}

/*任务使能回调*/
void StateTask::enableCallBack()
{

    // 达妙电机上力回调
    for(uint8_t i=0;i< DmMotor::objectNum;i++)
    {
        DmMotor::objectPtrList[i]->motorEnable();
    }
//	// 小米电机上力回调
//    for(uint8_t i=0;i< CyberMotor::objectNum;i++)
//    {
//        CyberMotor::objectPtrList[i]->motorEnable();
//    }
//	//宇树电机上力回调
//	for(uint8_t i = 0;i< utMotor::objectNum;i++)
//	{
//		utMotor::objectPtrList[i]->motorEnable();
//		utMotor::objectPtrList[i]->onforceFlag = 1;
//	}
}

/*任务失能回调*/
void StateTask::disableCallBack()
{
}
u8 count;
float err,out;
float calibraSpeed,CalibraErr,limit;
u8 cale[5];

//取矿pitch重力补偿计算
float GravityCompste;
float COMIndex;
/*任务主函数*/
void StateTask::run()
{

    //offsetEndFlag = 1; // 跳过校准

    // 状态机开始校准
    if (!offsetEndFlag)
    {
        // 校准状态机运行
        offSetSM.update();
        return;
    }


    // 校准完成重新加载参数
    if (offsetEndFlag && !paramReloadFlag)
    {
        loadParam();         // 重新设置速度限幅，输出限幅
        paramReloadFlag = 1; // 参数重加载完成
    }


    // 状态机校准完成并且已经重新加载完参数
    if (offsetEndFlag && paramReloadFlag)
    {

        for (uint8_t i = 0; i < FSMNum; i++)
        {
            if (FSMList[i]->runFlag && FSMList[i]->enableFlag)
            {
                // 校准完成后不再运行校准状态机
                if (FSMList[i] != &offSetSM)
                {
                    FSMList[i]->update();
                }
            }
        }
    }
}

StateTask stateTask;
