#ifndef OFFSET_STATEMACHINE_H
#define OFFSET_STATEMACHINE_H

#include "SMUniversal.h"
extern StateMachine offSetSM;    //校准状态机

//校准状态
class OffsetState : public State
{
public:
	OffsetState(State **nowStatePointer);
	void update() override;
	void exit() override;
};
extern OffsetState offsetState;

extern uint8_t PRLeftOffseting,PRRightOffseting,Pitch2Offseting,BigRollOffseting,Pitch3Offseting,YawOffseting,BigPitchOffseting;
extern uint8_t LiftOffseting,ClimbOffseting,takPitchOffseting,smallLiftOffseting,smallClimbOffseting;;
void keepNowState(void);
void updateNowPoint(void);
#endif