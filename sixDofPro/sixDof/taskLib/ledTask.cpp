#include "ledTask.h"
#include "virtualTask.h"
#include "iwdg.h"
#include "led.h"
#include "devList.h"


class LedTask : public VirtualTask
{
public:
    // 初始化
    void init() override;
    // 主函数
    void run() override;
};

void LedTask::init()
{
    // 任务持续运行
    setAlwaysRun(true);
    /*LED初始化*/
    running.init(1000);
   
}

/**
 * @brief LED任务函数
 * @note 负责LED的控制和喂狗
 * @warning 该函数为重写完成
 */
void LedTask::run()
{
    // 趁机喂狗
    iwdg.feed();
    // LED状态灯
    running.show(1);
}

LedTask ledTask;
