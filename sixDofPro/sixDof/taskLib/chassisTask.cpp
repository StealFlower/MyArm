#include "chassisTask.h"
#include "virtualTask.h"
#include "dbus.h"
#include "devList.h"

/**
 * @brief 底盘实现函数
 * @note 负责底盘速度给定
 */
struct ChassisTask : public VirtualTask
{
private:	
public:
    //初始化
    void init() override;
    //主函数
    void run() override;

    //遥控更新回调
    void rcUpdateCallBack() override;
    //任务使能回调
    void enableCallBack() override;
    //任务失能回调
    void disableCallBack() override;
};

/*初始化*/
void ChassisTask::init()
{
//	status = taskStateRun;  //任务运行	
	status = taskStateStop;  //任务停止运行	
}

/*遥控更新回调*/
void ChassisTask::rcUpdateCallBack()
{
}

/*任务使能回调*/
void ChassisTask::enableCallBack()
{
}

/*任务失能回调*/
void ChassisTask::disableCallBack()
{
}


/*任务主函数*/
void ChassisTask::run()
{
    //底盘更新
    chassis.update();
}


ChassisTask chassisTask;