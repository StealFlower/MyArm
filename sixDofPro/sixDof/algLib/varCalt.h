#ifndef __VARCALT_H__
#define __VARCALT_H__
#include "board.h"
//方差计算类
class VarCale
{
private:
    uint8_t caleNum;
    double var; //方差值
    double avg; //平均值
    uint32_t dataCnt;
    float dataLog[20];
public:
    VarCale(uint8_t caleNum=20);
    double caleVar(float nowValue);
    void clear();
};



#endif // !__VARCALT_H__
