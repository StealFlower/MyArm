#include "varCalt.h"


/**
 * @brief 构造器
 * @param caleNum 方差计算项目数，默认情况下为20
 **/
VarCale::VarCale(u8 caleNum)
{
    this->caleNum = caleNum;
	
    var = 0; //方差值
    avg = 0; //平均值
    dataCnt = 0;
	memset(dataLog,0,sizeof(dataLog));
};



/**
 * @brief 方差计算
 * @param nowValue 当前值
 * @return 当前方差值
 **/
double VarCale::caleVar(float nowValue)
{
    dataCnt++;
    dataLog[dataCnt%caleNum] = nowValue;
    
    if(dataCnt/caleNum > 0)
    {
        avg = 0;
        //计算平均值
        for (u8 i = 0; i < caleNum; i++)
            avg += dataLog[i];
        avg = avg/caleNum;
        var = 0;
        //计算方差
        for (u8 i = 0; i < caleNum; i++)
        {
            float err = avg - dataLog[i];
            var += err * err;
        }
    }
    else
    {
        avg = 0;
        //计算平均值
        for (u8 i = 0; i < dataCnt; i++)
            avg += dataLog[i];
        avg = avg/dataCnt;
        var = 0;
        //计算方差
        for (u8 i = 0; i < dataCnt; i++)
        {
            float err = avg - dataLog[i];
            var += err * err;
        }
    }
    return var;
}


/**
 * @brief 清空函数,用于清空数组
 **/
void VarCale::clear()
{
    memset(dataLog, 0, caleNum * sizeof(float));
	dataCnt=0;
}