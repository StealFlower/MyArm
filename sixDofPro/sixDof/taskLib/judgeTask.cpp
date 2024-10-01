#include "judgeTask.h"
#include "virtualTask.h"
#include "judgement.h"
#include "judge.h"
#include "devList.h"

class JudgeTask : public VirtualTask
{
private:
public:
    //初始化
    void init() override;
    //主函数
    void run() override;
};


void JudgeTask::init()
{
    //任务持续运行
    setAlwaysRun(false);
    //status = taskStateStop; //任务不运行
    //串口初始化
    judgement.init();
}


void JudgeTask::run()
{
    imageTran.CtrlerDataUpdate();
    
    //裁判系统返回单片机信息获取
    judgement.ringQueue();

    //在线检测
    judgement.online.check();
    
    //首次加载
    if(!judge.loadFinish)
    {
        judge.updateFirstLoadUi();
        return;
    }
    //客户端到裁判UI信息更新
    judge.update();
    
    //重新添加UI
    if(rcCtrl.key.CTRL && rcCtrl.key.SHIFT && rcCtrl.keyPress.Z)
    {
        judge.reLoadUi();
    }
}


JudgeTask judgeTask;