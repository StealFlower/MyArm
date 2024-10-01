#ifndef __JUDGE_H__
#define __JUDGE_H__
#include "board.h"
#include "SMUniversal.h"
#include "judgement.h"

enum OperateTpye
{
    add = 1,  //增添
    edt = 2,  //修改
    del = 3   //删除
};  
enum GraphicTpye
{
    line = 0, //直线
    rectangle = 1, //矩形
    circular = 2,   //正圆
    ellipse = 3,    //椭圆
    
    decimal = 5,//小数
    integer = 6, //整形数据
    character = 7  //拉丁字母
};
enum UiColor
{
    rorb=0, //红蓝主色
    yelw,     //黄色
    gren,     //绿色
    orgn,     //橙色
    prip,     //紫色
    pink,     //粉色
    sink,     //青色
    blak,     //黑色
    whit,     //白色

};

struct JudgeCtrl
{
public:
    uint32_t nowTime;
    u8 loadFinish;
public:
    //更新状态值Ui
    void updateStateUi(void);
    //更新辅助Ui
    void updateAssistUi(void);
    //更新ENGINEER
    void updateEngineerUi(void);
    //更新矿石倒计时Ui
    void updateCountDown(void);
    //视觉状态Ui
    void updateVisionUi(void);

    //首次加载
    void updateFirstLoadUi(void);
    //UI常更新
    void update(void);
    //UI重加载
    void reLoadUi(void);
};

extern JudgeCtrl judge;
#endif