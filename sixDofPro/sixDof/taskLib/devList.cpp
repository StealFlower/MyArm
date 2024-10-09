#include "devList.h"
#include "vision.h"
//底盘
Chassis chassis;

//云台
Gimbal gimbal;

//图传链路
ImageTran imageTran;

//气路
Air air;

DmMotor Yaw1(&hfdcan3,0x0A,0x1A);
MitParam Yaw1Param[2];

DmMotor Pitch1(&hfdcan3,0x0B,0x1B);
MitParam Pitch1Param[2];

DmMotor Pitch2(&hfdcan3,0x0C,0x1C);
MitParam Pitch2Param[2];

Motor LeftMotor(M2006,&hfdcan3,0x202);
Motor RightMotor(M2006,&hfdcan3,0x203);
Wrist wrist(&LeftMotor,&RightMotor);
PathPlan WristPath;
PidParam WristSpdParam[2];

PidParam WristPosParam[2];

Motor Roll1{GM6020,&hfdcan3,0x205};
PathPlan Roll1Path;
PidParam Roll1SpdParam[2];
PidParam Roll1PosParam[2];

PathPlan ArmPath;

float kp1,kp2,kd1,kd2;
void loadParam()
{
    Yaw1.setPlan(0,&Yaw1Param[0]);
    Yaw1.setParam(0,0,0,90,1,0);
//    Yaw1.setPlan(1,&Yaw1Param[1]);
//    Yaw1.setParam(0,0,0,200,3.6,0);
    
    Pitch1.setPlan(0,&Pitch1Param[0]);
    Pitch1.setParam(0,0,0,200,7.2,0);
//    Pitch1.setPlan(1,&Pitch1Param[1]);
//    Pitch1.setParam(0,0,0,150,7.5,0);
    
    Pitch2.setPlan(0,&Pitch2Param[0]);
    Pitch2.setParam(0,0,0,150,0.8,0);
//    Pitch2.setPlan(1,&Pitch1Param[1]);
//    Pitch2.setParam(0,0,0,150,7.5,0);
    
//    Roll1.setPlan(0,spd,&Roll1SpdParam[0]);
//    Roll1.setParam(0,spd,2.7,0,0,0,18000);
//    Roll1.setPlan(0,pos,&Roll1PosParam[1]);
//    Roll1.setParam(0,pos,150,0,0,0,6000*6);
//    
//    wrist.setPlan(0,spd,&WristSpdParam[0]);
//    wrist.setParam(0,spd,1,0,0,0,8000);
//    wrist.setPlan(0,pos,&WristPosParam[0]);
//    wrist.setParam(0,pos,20,0,0,0,2000*6);   
//    wrist.setPlan(1,spd,&WristSpdParam[0]);
//    wrist.setParam(1,spd,2.4,0,0,0,16000);
//    wrist.setPlan(1,pos,&WristPosParam[0]);
//    wrist.setParam(1,pos,48,0,0,0,3000*6); 

 
//    Yaw1.setPlan(0,&Yaw1Param[0]);
//    Yaw1.setParam(0,0,0,0,0,0);
////    Yaw1.setPlan(1,&Yaw1Param[1]);
////    Yaw1.setParam(0,0,0,200,3.6,0);
//    
//    Pitch1.setPlan(0,&Pitch1Param[0]);
//    Pitch1.setParam(0,0,0,0,0,0);
////    Pitch1.setPlan(1,&Pitch1Param[1]);
////    Pitch1.setParam(0,0,0,150,7.5,0);
//    
//    Pitch2.setPlan(0,&Pitch2Param[0]);
//    Pitch2.setParam(0,0,0,0,0,0);
////    Pitch2.setPlan(1,&Pitch1Param[1]);
////    Pitch2.setParam(0,0,0,150,7.5,0);
//    Roll1.setPlan(0,spd,&Roll1SpdParam[0]);
//    Roll1.setParam(0,spd,2.7,0,0,0,0);
//    Roll1.setPlan(0,pos,&Roll1PosParam[0]);
//    Roll1.setParam(0,pos,150,0,0,0,6000*6);
//    
//    wrist.setPlan(0,spd,&WristSpdParam[0]);
//    wrist.setParam(0,spd,1,0,0,0,0);
//    wrist.setPlan(0,pos,&WristPosParam[0]);
//    wrist.setParam(0,pos,20,0,0,0,2000*6);    
}

u8 DEBUG_MODE = 0;
EndPoint tarPoint;
JointAngle setJoint;
EndPoint setPoint;
JointTorque jointTorque;
JointTorque tarTorque;
JointTorque errTorque;

JointState tarJointState;
JointState GState;
JointState nowEndState;
JointState NowJointState;


EndPoint nowPoint;
EndForce judge_force(1,0,0);
void DEBUG()
{ 
    Arm.GetNowParam();
    
    if(DEBUG_MODE == 0)
    {
        nowPoint = Arm.now_end_point;
        tarPoint = nowPoint;
        tarPoint.xPos = nowPoint.xPos + 0.2;
    }
    if(DEBUG_MODE == 2)
    {
        tarJointState = pathPlanCalt(&ArmPath,nowPoint,tarPoint,1);
        Arm.ctrlPosition(tarJointState);
//        if(Arm.exitStatus(judge_force,1000,3000)){
//            DEBUG_MODE = 3 ;
//            clearTime(&ArmPath);
//        }           
    }
//    NowJointState.q1 = Yaw1.getPosition()*THETA1_MECH_TO_RPS_RATIO;
//    NowJointState.q2 = (Pitch1.getPosition()-90)*THETA2_MECH_TO_RPS_RATIO;
//    NowJointState.q3 = Pitch2.getPosition()*THETA3_MECH_TO_RPS_RATIO;
//    NowJointState.dq1 = 0;
//    NowJointState.dq2 = 0;
//    NowJointState.dq3 = 0;
//    NowJointState.ddq1 = 0;
//    NowJointState.ddq2 = 0;
//    NowJointState.ddq3 = 0;  
//    GState = NowJointState;
//    
//    nowPoint = kinematics.GetNowEndPoint();
//    nowEndState = kinematics.GetNowEndState();
//    
//    if(DEBUG_MODE == 0)
//    {             
//        loadParam();
//        Yaw1.setPosition = Yaw1.getPosition();
//        Pitch1.setPosition = Pitch1.getPosition();
//        Pitch2.setPosition = Pitch2.getPosition();
//        Roll1.setPosition = Roll1.getPosition();        
//        wrist.setRoll = wrist.getRollAngle();
//        wrist.setPitch = wrist.getPitchAngle();
//       
//        kinematics.nowJoint_Mech.theta1 = Yaw1.getPosition();
//        kinematics.nowJoint_Mech.theta2 = Pitch1.getPosition();
//        kinematics.nowJoint_Mech.theta3 = Pitch2.getPosition();
//        kinematics.nowJoint_Mech.theta4 = Roll1.getPosition();
//        kinematics.nowJoint_Mech.theta5 = wrist.getPitchAngle();
//        kinematics.nowJoint_Mech.theta6 = wrist.getRollAngle();
//        
////        tarPoint = kinematics.GetNowEndPoint();
//        setJoint = kinematics.GetNowJointAngle(nowPoint,1);
//        tarPoint = nowPoint;
//    }  
//    if(DEBUG_MODE == 2){  
//        tarPoint.xPos = nowPoint.xPos + 0.12;
//        tarJointState = pathPlanCalt(&ArmPath,nowPoint,tarPoint,1);

//        setPoint.xPos = tarJointState.q1;
//        setPoint.yPos = tarJointState.q2;
//        setPoint.zPos = tarJointState.q3;
//        
//        setJoint = kinematics.GetNowJointAngle(setPoint,1);
//        
//        jointTorque = dynamics.GetJointTorque(tarJointState);

//        Yaw1.setPosition = setJoint.theta1;
//        Pitch1.setPosition = setJoint.theta2;
//        Pitch2.setPosition = setJoint.theta3;
//        Roll1.setPosition = setJoint.theta4;
//        wrist.setPitch = setJoint.theta5;
//        wrist.setRoll = setJoint.theta6;   

//        Yaw1.ctrlPosition(Yaw1.setPosition,-jointTorque.tao1);
//        Pitch1.ctrlPosition(Pitch1.setPosition,-jointTorque.tao2);
//        Pitch2.ctrlPosition(Pitch2.setPosition,jointTorque.tao3);
//        if(ArmPath.nowTime>1.1){
//            DEBUG_MODE = 4;
//            tarTorque.tao1 = nowEndState.ddq1+1;
//            tarTorque.tao2 = nowEndState.ddq2;
//            tarTorque.tao3 = nowEndState.ddq3;    
//        }            
////        Roll1.ctrlPosition(Roll1.setPosition);
////        wrist.ctrlPosition(wrist.setRoll,wrist.setPitch);        
//    }
//    if(DEBUG_MODE == 3){

//        Yaw1.setPosition = setJoint.theta1;
//        Pitch1.setPosition = setJoint.theta2;
//        Pitch2.setPosition = setJoint.theta3; 
//        Roll1.setPosition = setJoint.theta4;
//        wrist.setPitch = setJoint.theta5;
//        wrist.setRoll = setJoint.theta6;    
//        
//        Yaw1.ctrlPosition(Yaw1.setPosition);
//        Pitch1.ctrlPosition(Pitch1.setPosition);
//        Pitch2.ctrlPosition(Pitch2.setPosition);
//        Roll1.ctrlPosition(Roll1.setPosition);
//        wrist.ctrlPosition(wrist.setRoll,wrist.setPitch);
//    } 
//    if(DEBUG_MODE == 4){
//        clearTime(&ArmPath);
//        
//        errTorque.tao1 = tarTorque.tao1 - nowEndState.ddq1;
//        errTorque.tao2 = tarTorque.tao2 - nowEndState.ddq2;
//        errTorque.tao3 = tarTorque.tao3 - nowEndState.ddq3;
//        
//        if(fabs(errTorque.tao1)<0.075f)
//            DEBUG_MODE = 5;
//        setPoint = setPoint+dynamics.AdmittanceControl(errTorque); 
//        setJoint = kinematics.GetNowJointAngle(setPoint,1);
//        
//        jointTorque = dynamics.GetJointTorque(GState);

//        Yaw1.setPosition = setJoint.theta1;
//        Pitch1.setPosition = setJoint.theta2;
//        Pitch2.setPosition = setJoint.theta3;
//  

//        Yaw1.ctrlPosition(Yaw1.setPosition,-jointTorque.tao1);
//        Pitch1.ctrlPosition(Pitch1.setPosition,-jointTorque.tao2);
//        Pitch2.ctrlPosition(Pitch2.setPosition,jointTorque.tao3);

//        
//    }
    
////-----------------------重力补偿-------------------------////
//    NowJointState.q1 = Yaw1.getPosition()*THETA1_MECH_TO_RPS_RATIO;
//    NowJointState.q2 = (Pitch1.getPosition()-90)*THETA2_MECH_TO_RPS_RATIO;
//    NowJointState.q3 = Pitch2.getPosition()*THETA3_MECH_TO_RPS_RATIO;
//    NowJointState.dq1 = 0;
//    NowJointState.dq2 = 0;
//    NowJointState.dq3 = 0;
//    NowJointState.ddq1 = 0;
//    NowJointState.ddq2 = 0;
//    NowJointState.ddq3 = 0;  
//    tarJointState = NowJointState;
// 
//    if(DEBUG_MODE == 0)
//    {             
//        loadParam();
//        Yaw1.setPosition = Yaw1.getPosition();
//        Pitch1.setPosition = Pitch1.getPosition();
//        Pitch2.setPosition = Pitch2.getPosition();
//        Roll1.setPosition = Roll1.getPosition();        
//        wrist.setRoll = wrist.getRollAngle();
//        wrist.setPitch = wrist.getPitchAngle();
//        
//        kinematics.nowJoint_Mech.theta1 = Yaw1.getPosition();
//        kinematics.nowJoint_Mech.theta2 = Pitch1.getPosition();
//        kinematics.nowJoint_Mech.theta3 = Pitch2.getPosition();
//        kinematics.nowJoint_Mech.theta4 = Roll1.getPosition();
//        kinematics.nowJoint_Mech.theta5 = wrist.getPitchAngle();
//        kinematics.nowJoint_Mech.theta6 = wrist.getRollAngle();
//        tarPoint = kinematics.GetNowEndPoint();
//    }  
//    if(DEBUG_MODE == 2){          
//        setJoint = kinematics.GetNowJointAngle(tarPoint,1);
//        Yaw1.setPosition = setJoint.theta1;
//        Pitch1.setPosition = setJoint.theta2;
//        Pitch2.setPosition = setJoint.theta3;
//        Roll1.setPosition = setJoint.theta4;
//        wrist.setPitch = setJoint.theta5;
//        wrist.setRoll = setJoint.theta6;
//        jointTorque = dynamics.ImpedanceControl(tarJointState);              
//        
//    }    
//    if(DEBUG_MODE == 3){
//        
//        jointTorque = dynamics.ImpedanceControl(tarJointState);              
//      
//        
//        Yaw1.ctrlPosition(Yaw1.setPosition,-jointTorque.tao1);
//        Pitch1.ctrlPosition(Pitch1.setPosition,-jointTorque.tao2);
//        Pitch2.ctrlPosition(Pitch2.setPosition,jointTorque.tao3);

//    } 
}
