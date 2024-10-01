#ifndef __ORE_MOTION_H
#define __ORE_MOTION_H
#include "SMUniversal.h"


extern u8 LastSMisExc;
extern uint8_t TwoOrejud;

extern u8 ForceErrMode;
enum FetchPos:uint8_t{
    TwoFirst = 1,
    Single = 2,
    TwoSecond = 3,
    TwoInOneTime = 4,
};

extern StateMachine takeOreSM;

void updateNowPoint(void);
// 初始状态
class TakeInitState : public State
{
public:
	TakeInitState(State **nowStatePointer);
	void update() override;
	void exit() override;
};
extern TakeInitState takeInitState;

// 空接状态
class CatchState : public State
{
public:
	CatchState(State **nowStatePointer);
	void update() override;
	void exit() override;
};
extern CatchState catchState;

// 挡装甲板状态
class HidState : public State
{
public:
	HidState(State **nowStatePointer);
	void update() override;
	void exit() override;
};
extern HidState hidState;

//检录状态
class CheckState : public State
{
public:
	CheckState(State **nowStatePointer);
	void update() override;
	void exit() override;
};
extern CheckState checkstate;
// 金矿状态
class TakeGoldState : public State
{
public:
	TakeGoldState(State **nowStatePointer);
	void update() override;
	void exit() override;
};
extern TakeGoldState takeGoldState;

// 银矿状态
class TakeSilveryState : public State
{
public:
	TakeSilveryState(State **nowStatePointer);
	void update() override;
	void exit() override;
};
extern TakeSilveryState takeSilveryState;

// 捡矿状态
class TakeFloorState : public State
{
public:
	TakeFloorState(State **nowStatePointer);
	void update() override;
	void exit() override;
};
extern TakeFloorState takeFloorState;

// 退出状态
class ExitPerfectState : public State
{
public:
	ExitPerfectState(State **nowStatePointer);
	void update() override;
	void exit() override;
};
extern ExitPerfectState exitPerfectState;

// 兑换状态
class ExchangeState : public State
{
public:
	ExchangeState(State **nowStatePointer);
	void update() override;
	void exit() override;
};
extern ExchangeState exchangeState;

// 手动状态
class HandModeState : public State
{
public:
	HandModeState(State **nowStatePointer);
	void update() override;
	void exit() override;
};
extern HandModeState handModeState;

extern uint8_t remainGoldNum;
extern float AngleGetted,PitchGetted;
#endif