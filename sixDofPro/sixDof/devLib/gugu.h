#ifndef __GUGU_H__
#define __GUGU_H__
#include "board.h"

enum GuguState
{
    guguStop=410,
    guguOpen=2212,
    guguCheck=1500,
    guguRemind = 1000,

};

class Gugu
{
public:
    uint16_t takGugu, excGugu;
    void init(void);
    void update(void);
};

#endif