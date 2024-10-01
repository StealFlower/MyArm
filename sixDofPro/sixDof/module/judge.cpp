#include "judge.h"
#include "SMUniversal.h"
#include "oreMotionStateMachine.h"
#include "devList.h"
#include "vision.h"
//坐标原点在左下角[0,0]
//角度零点在上方，顺时针
//浮点数模式为后32位int32_t值/1000
//整形数模式为后32位int32_t值
extern uint8_t warningFlag;
extern uint8_t OverHeatFlag;
extern uint8_t GoldErrModeFlag;
extern uint8_t SilverErrModeFlag;
uint8_t utf8[] = "测试";

/**
 * @brief 将utf8编码转换为utf16编码
 * 
 * @param utf8 要转化的串
 * @param utf16 转化后的串
 * @param out_len 转化后的字符串长度
 */
void utf8_to_utf16(const uint8_t* utf8, uint16_t* utf16, size_t* out_len){
    size_t i = 0, j = 0;
    while (utf8[i]) {
        uint32_t unicode = 0;
        size_t size = 0;
        if (utf8[i] < 0x80) {
            // 1-byte sequence
            unicode = utf8[i];
            size = 1;
        } else if ((utf8[i] & 0xE0) == 0xC0) {
            // 2-byte sequence
            unicode = (utf8[i] & 0x1F) << 6;
            unicode |= (utf8[i + 1] & 0x3F);
            size = 2;
        } else if ((utf8[i] & 0xF0) == 0xE0) {
            // 3-byte sequence
            unicode = (utf8[i] & 0x0F) << 12;
            unicode |= (utf8[i + 1] & 0x3F) << 6;
            unicode |= (utf8[i + 2] & 0x3F);
            size = 3;
        } else if ((utf8[i] & 0xF8) == 0xF0) {
            // 4-byte sequence, needs surrogate pairs in UTF-16
            unicode = (utf8[i] & 0x07) << 18;
            unicode |= (utf8[i + 1] & 0x3F) << 12;
            unicode |= (utf8[i + 2] & 0x3F) << 6;
            unicode |= (utf8[i + 3] & 0x3F);
            size = 4;
            unicode -= 0x10000;
            utf16[j++] = (unicode >> 10) + 0xD800;
            utf16[j++] = (unicode & 0x3FF) + 0xDC00;
            i += size;
            continue;
        }
        utf16[j++] = unicode;
        i += size;
    }
    utf16[j] = 0; // Null-terminate the UTF-16 string
    if (out_len) {
        *out_len = j; // Optional: Return the length of the UTF-16 string
    }
}


Judgement::GraphicDataStruct assistGraphicUi[][7] = {
    //矩形：|图形名|图形名|图形名|操作|图形类型|图层|颜色|空|空|线宽|起点x|起点y|空|对角顶点x|对角顶点y|
    //直线：|图形名|图形名|图形名|操作|图形类型|图层|颜色|空|空|线宽|起点x|起点y|空|终点|终点y|
    //浮点数：|图形名|图形名|图形名|操作|图形类型|图层|颜色|字体大小|小数位有效个数|线宽|起点x|起点y|乘以1000后,以32位整型数,int32_t|
    //整型数：|图形名|图形名|图形名|操作|图形类型|图层|颜色|字体大小|空|线宽|起点x|起点y|32位整型数,int32_t|
    
    //时刻更新
    {
        //对位
        {0, 0, 1, add, line, 0, pink, 0, 0, 3, 0, 0, 0, 0, 1080},
        {0, 0, 2, add, line, 0, blak, 0, 0, 3, 30, 900, 0, 20, 875},
        //底盘电压
        {0, 0, 3, add, decimal, 0, pink, 25, 1, 3, 10, 850, 0},
        //底盘基准线
        {0, 0, 4, add, line, 0, pink, 0, 0, 3, 750 , 0 , 0, 800 , 300},//底盘基准线左
        {0, 0, 5, add, line, 0, pink, 0, 0, 3, 1350 , 0, 0, 1100, 300},//底盘基准线右
        

    },
    {
        //矿石数量
        {0, 0, 6, add, integer, 0, sink, 25, 0, 3, 50, 900, 0},//Belly
        {0, 0, 7, add, integer, 0, sink, 25, 0, 3, 90, 900, 0},//Roller
        {0, 0, 8, add, integer, 0, sink, 25, 0, 3, 130, 900, 0},//Sucker
        {0, 0, 9, add, decimal, 0, prip, 25, 1, 3, 10, 700, 0 },//视觉发送x方向
        
    },
    
    
    //满足条件时更新//750 700
    {
        //取矿对位辅助    取矿时更新
        {1, 0, 0, del, line, 0, pink, 0, 0, 3, 0, 0, 0, 0, 1080},
        {2, 0, 0, del, rectangle, 0, pink, 0, 0, 3, 1000, 550, 0, 1200, 350},
        //银矿阀门状态    银矿时更新
        {3, 0, 0, del, circular, 0, orgn, 0, 0, 3, 750, 700, 30, 0, 0},//左
        {4, 0, 0, del, circular, 0, orgn, 0, 0, 3, 1020, 700, 30, 0, 0},//中
        {5, 0, 0, del, circular, 0, orgn, 0, 0, 3, 1290, 700, 30, 0, 0},//右
        
        {6, 0, 0, add, rectangle, 0, orgn, 0, 0, 3, 1310, 750, 0, 630, 250},//视觉识别框
    },
};


//字符类
Judgement::GraphicDataStruct assistCharUi[] = {
    
    //字符：|图形名|图形名|图形名|操作|图形类型|图层|颜色|字体大小|字符长度|线宽|起点x|起点y|空|空|空|
	//视觉在线标志
    {1,0,1,add,character,0,gren,25,2,3,1600,850,0,0,0},
    //银矿后退提醒
    {1,0,4,del,character,0,orgn,50,21,6,510,650,0,0,0},
    //电机掉线提醒
    {1,0,5,add,character,0,rorb,40,15,6,600,540,0,0,0},
    //控制器在线提醒
    {1,0,6,add,character,0,gren,25,2,3,1600,750,0,0,0},
    //错误显示
    {1,0,7,add,character,0,prip,15,2,3,1600,350,0,0,0},
    //状态指示
    {1,0,8,add,character,0,rorb,20,20,3,800,800,0,0,0},
};

//数字类
Judgement::NumberDataStruct assistNumUi[][7] = {};


void JudgeCtrl::updateFirstLoadUi(void)
{
    nowTime = (nowTime==0)?sysTickUptime:nowTime;//获取初始时间
    if((sysTickUptime-nowTime)%102==0)
    {
        switch((sysTickUptime-nowTime)/102)
        {
            case 0:
            break;
            case 1:
            break;
            case 2:
                memcpy(&judgement.sendUnionData.graphicDataStruct, assistGraphicUi[0], sizeof(assistGraphicUi[0]));
                judgement.graphicDraw(7);
            break;
            case 3:
                memcpy(&judgement.sendUnionData.graphicDataStruct, assistGraphicUi[1], sizeof(assistGraphicUi[1]));
                judgement.graphicDraw(7);
            break;
            case 4:
                memcpy(&judgement.sendUnionData.graphicDataStruct, assistGraphicUi[2], sizeof(assistGraphicUi[2]));
                judgement.graphicDraw(7);
            break;
            case 5:
                memcpy(&judgement.sendUnionData.clientCustomCharacter.graphic_data_struct, &assistCharUi[4], sizeof(assistCharUi[4]));
                memset(&judgement.sendUnionData.clientCustomCharacter.data,0,sizeof(judgement.sendUnionData.clientCustomCharacter.data));
                memcpy(&judgement.sendUnionData.clientCustomCharacter.data,"Err:None",sizeof("Err:None"));
                judgement.characterDraw();
            break;
            case 6:
                memcpy(&judgement.sendUnionData.clientCustomCharacter.graphic_data_struct, &assistCharUi[5], sizeof(assistCharUi[5]));
                memset(&judgement.sendUnionData.clientCustomCharacter.data,0,sizeof(judgement.sendUnionData.clientCustomCharacter.data));
                memcpy(&judgement.sendUnionData.clientCustomCharacter.data,"NowState:  None",sizeof("NowState:  None"));
                judgement.characterDraw();
            break;
            case 7://修改为可编辑状态
                for(u8 i=0;i<3;i++)
                    for(u8 j=0;j<7;j++)
                        assistGraphicUi[i][j].operateType = edt;
                assistCharUi[4].operateType = edt;
                assistCharUi[5].operateType = edt;
                //结束
                nowTime=0;
                loadFinish=1;
            break;
        }
    }
}


void JudgeCtrl::reLoadUi(void)
{   
    for(u8 i=0;i<3;i++)
        for(u8 j=0;j<7;j++)
            assistGraphicUi[i][j].operateType = add;
    nowTime=0;
    loadFinish=0;
}


int32_t num;
float tranUiPos;
float sustainUiAngle[2];
uint8_t goldUiStatus;
void JudgeCtrl::update(void)
{
    //全部UI绘制不超过30Hz，即帧与帧间隔必须大于33.33333333ms
 
}


JudgeCtrl judge;
