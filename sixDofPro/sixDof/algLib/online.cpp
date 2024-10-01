#include "online.h"

//离线检测原理：
//定义变量onlineCnt，每个周期onlineCnt+1，接收到数据反馈时onlineCnt清零
//当onlineCnt大于某个数值时，说明已经一段时间没有收到数据反馈了，即离线了


Online **onlinePtrList = 0;
uint8_t onlineNum = 0;

Online::Online()
{
    // 为对象申请内存
    if (onlinePtrList == 0)
        onlinePtrList = (Online **)malloc(sizeof(Online *));
    else
        onlinePtrList = (Online **)realloc(onlinePtrList, sizeof(Online *) * (onlineNum + 1));

    // 添加对象指针到列表
    onlinePtrList[onlineNum] = this;
    onlineNum += 1;
}


void Online::check()
{
    if (onlineCnt < 1000)
    {
        onlineCnt++;
        onlineFlag = true;
    }
    else
    {
        onlineFlag = false;
		#if UNUSE_RC
		onlineFlag = true;
		#endif
    }
}


void Online::update()
{
    onlineCnt = 0;
    onlineFlag = true;
}
