#ifndef _ARM_H_
#define _ARM_H_

#include "kinematics.h"
#include "dynamics.h"
#include "cycle.h"

#define RAD_TO_DEGREE 57.295779513f
#define DEGREE_TO_RAD 0.017452007f
//机械结构传动比 实际单位与电机单位之比
#define THETA1_MECH_TO_RPS_RATIO -0.017452007f 
#define THETA2_MECH_TO_RPS_RATIO -0.017452007f
#define THETA3_MECH_TO_RPS_RATIO 0.017452007f
#define THETA4_MECH_TO_RPS_RATIO -0.017452007f
#define THETA5_MECH_TO_RPS_RATIO 0.017452007f
#define THETA6_MECH_TO_RPS_RATIO 0.017452007f

#define THETA1_RPS_TO_MECH_RATIO -57.29578049f 
#define THETA2_RPS_TO_MECH_RATIO -57.29578049f 
#define THETA3_RPS_TO_MECH_RATIO 57.29578049f 
#define THETA4_RPS_TO_MECH_RATIO -57.29578049f 
#define THETA5_RPS_TO_MECH_RATIO 57.29578049f 
#define THETA6_RPS_TO_MECH_RATIO 57.29578049f 

struct MechParam
{
public:
    float l1 ;
    float l2 ;
    MechParam()
    {
        this->l1 = 0.237;
        this->l2 = 0.34;
    }
};

extern MechParam mech;

class RoboticArm
{
public:
    RoboticArm();

    Kinematics kine;
    Dynamics dyna; 

    EndPoint now_end_point;
    JointAngle now_joint_angle;
    EndState now_end_state;
    JointState now_joint_state;

	float delayTime;
    Cycle armcycle;
    float Jacobin[3][3];
    void GetNowParam(void);
    uint8_t exitStatus(EndForce maxForce, float deathRoomTime, float outTime);
    
    JointAngle MechToCalculate(JointAngle mech_angle);
    JointAngle CalculateToMech(JointAngle calculate_angle);
    JointAngle CalculateAngleHandle(JointAngle calculate_angle);
    
    void ctrlPosition(JointState tarstate,EndForce extern_force = EndForce(),u8 mode = 0);
};

extern RoboticArm Arm;
#endif
