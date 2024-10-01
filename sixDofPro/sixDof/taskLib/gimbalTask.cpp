#include "virtualTask.h"
#include "devList.h"


class GimbalTask : public VirtualTask
{    
public:
    //初始化
    void init() override;
    //主函数
    void run() override;
};


void GimbalTask::init()
{
     status = taskStateStop; //任务不运行
    
//    status = taskStateRun; //任务运行
//    //任务持续运行
//    setAlwaysRun(true);
    
    gimbal.init();
}


void GimbalTask::run()
{
    gimbal.update();
}

GimbalTask gimbalTask;