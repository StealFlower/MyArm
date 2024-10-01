#ifndef __IWDG_H__
#define __IWDG_H__
#include "board.h"


class Iwdg
{
	private:
	public:
void init();
void feed();
};
extern Iwdg iwdg;

#endif //__IWDG_H__