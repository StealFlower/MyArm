#ifndef __DEV_LIST_H__
#define __DEV_LIST_H__

#include "chassis.h"
#include "gimbal.h"
#include "imageTran.h"
#include "air.h"
#include "motor.h"
#include "doubleMotor.h"
#include "dmMotor.h"
#include "cyberMotor.h"
#include "utMotor.h"
#include "pathPlan.h"
#include "wrist.h"
#include "dbus.h"
#include "arm.h"

extern Chassis chassis;
extern Gimbal gimbal;
extern ImageTran imageTran;
extern Air air;
extern DmMotor Yaw1;
extern DmMotor Pitch1;
extern DmMotor Pitch2;
extern Motor Roll1;
extern Motor LeftMotor;
extern Motor RightMotor;
extern Motor SVCMotor;
extern Wrist wrist;
extern MitParam Yaw1Param[2];
extern MitParam Pitch1Param[2];
extern MitParam Pitch2Param[2];
extern PidParam WristSpdParam[2];
extern PidParam WristPosParam[2];
extern PidParam Roll1SpdParam[2];
extern PidParam Roll1PosParam[2];
extern PathPlan Yaw1Path;
extern PathPlan Pitch1Path;
extern PathPlan Pitch2Path;
extern PathPlan Roll1Path;
extern PathPlan WristPath;

void loadParam();
void DEBUG();

#endif
