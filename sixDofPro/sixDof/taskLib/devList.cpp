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

Motor SVCMotor{M3508,&hfdcan3,0x201};
PathPlan SVCPath;
PidParam SVCSpdParam[2];
PidParam SVCPosParam[2];

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
    
    Roll1.setPlan(0,spd,&Roll1SpdParam[0]);
    Roll1.setParam(0,spd,2.7,0,0,0,18000);
    Roll1.setPlan(0,pos,&Roll1PosParam[1]);
    Roll1.setParam(0,pos,150,0,0,0,6000*6);
    
    wrist.setPlan(0,spd,&WristSpdParam[0]);
    wrist.setParam(0,spd,1,0,0,0,8000);
    wrist.setPlan(0,pos,&WristPosParam[0]);
    wrist.setParam(0,pos,20,0,0,0,2000*6);   
    wrist.setPlan(1,spd,&WristSpdParam[0]);
    wrist.setParam(1,spd,2.4,0,0,0,16000);
    wrist.setPlan(1,pos,&WristPosParam[0]);
    wrist.setParam(1,pos,48,0,0,0,3000*6); 


//    SVCMotor.setPlan(0,spd,&SVCSpdParam[0]);
//    SVCMotor.setParam(0,spd,10,0,0,0,13000);
//    SVCMotor.setPlan(0,pos,&SVCPosParam[0]);
//    SVCMotor.setParam(0,pos,1,0,0,0,12000);    

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
JointState NowJointState;
JointState tarJointState;

void DEBUG()
{   
    if(vision.online.isOnline()){
        setJoint.theta1 = vision.vision_recv_struct.joint_position[0] * 57.2957795;
        setJoint.theta2 = vision.vision_recv_struct.joint_position[1]* 57.2957795;
        setJoint.theta3 = vision.vision_recv_struct.joint_position[2]* 57.2957795;
        setJoint.theta4 = vision.vision_recv_struct.joint_position[3]* 57.2957795;
        setJoint.theta5 = vision.vision_recv_struct.joint_position[4]* 57.2957795;
        setJoint.theta6 = vision.vision_recv_struct.joint_position[5]* 57.2957795;
       tarPoint = kinematics.GetNowEndPoint(setJoint);

    }
    
        
//////    验证正逆解是否正确
//        NowJointState.q1 = Yaw1.getPosition()*THETA1_MECH_TO_RPS_RATIO;
//        NowJointState.q2 = (Pitch1.getPosition()-90)*THETA2_MECH_TO_RPS_RATIO;
//        NowJointState.q3 = Pitch2.getPosition()*THETA3_MECH_TO_RPS_RATIO;
//        NowJointState.dq1 = 0;
//        NowJointState.dq2 = 0;
//        NowJointState.dq3 = 0;
////        NowJointState.dq1 = Yaw1.canInfo.vel_rads;
////        NowJointState.dq2 = Pitch1.canInfo.vel_rads;
////        NowJointState.dq3 = Pitch2.canInfo.vel_rads;
//        NowJointState.ddq1 = 0;
//        NowJointState.ddq2 = 0;
//        NowJointState.ddq3 = 0;
//    dynamics.GetNowJointState();
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
//        tarJointState = NowJointState; 

//    }  
//    if(DEBUG_MODE == 2){  
//        tarJointState.dq1 = 0;     
//        tarJointState.dq2 = 0;        
//        tarJointState.dq3 = 0;        
//        
//        jointTorque = dynamics.ImpedanceControl(tarJointState);              
//        
//    }
//    if(DEBUG_MODE == 3){                
//        tarPoint.xPos += rcCtrl.rc.ch[0]*0.0005;
//        tarPoint.yPos += rcCtrl.rc.ch[1]*0.0005;
//        tarPoint.zPos += rcCtrl.rc.ch[2]*0.0005;
//        setJoint = kinematics.GetNowJointAngle(tarPoint,1);

//        tarJointState.q1 = setJoint.theta1*THETA1_MECH_TO_RPS_RATIO;;
//        tarJointState.q2 = (setJoint.theta2-90)*THETA2_MECH_TO_RPS_RATIO;
//        tarJointState.q3 = setJoint.theta3*THETA3_MECH_TO_RPS_RATIO;
//        
//        jointTorque = dynamics.ImpedanceControl(tarJointState);   
//        
//        Yaw1.ctrlPosition(Yaw1.setPosition,-jointTorque.tao1);
//        Pitch1.ctrlPosition(Pitch1.setPosition,-jointTorque.tao2);
//        Pitch2.ctrlPosition(Pitch2.setPosition,jointTorque.tao3);
////        tarJointState = NowJointState; 

//    }
//    动一动电机，来确定电机的基础特性

//    if(DEBUG_MODE == 0)
//    {
//        loadParam();
//        SVCMotor.setPosition = SVCMotor.getPosition();
//    }
//    if(DEBUG_MODE == 1)
//    {
//        SVCMotor.ctrlSpeed(1200);
//        if(fabs(SVCMotor.setCurrent)>13000){
//            SVCMotor.ctrlSpeed(0);
//            DEBUG_MODE = 0;
//        }       
//    }    
//    if(DEBUG_MODE == 2)
//    {
//        SVCMotor.ctrlSpeed(-1200);
//        if(fabs(SVCMotor.setCurrent)>13000){
//            SVCMotor.ctrlSpeed(0);
//            DEBUG_MODE = 0;
//        }       
//    }    
////----------------------------验证正逆解-----------------------------////    
//    验证正逆解是否正确
//    setJoint = kinematics.GetNowJointAngle(tarPoint,1);
//    setPoint = kinematics.GetNowEndPoint(setJoint);
    
//-------------------------机械臂正逆运动学-------------------------////
    if(DEBUG_MODE == 0)
    {             
        loadParam();
        Yaw1.setPosition = Yaw1.getPosition();
        Pitch1.setPosition = Pitch1.getPosition();
        Pitch2.setPosition = Pitch2.getPosition();
        Roll1.setPosition = Roll1.getPosition();        
        wrist.setRoll = wrist.getRollAngle();
        wrist.setPitch = wrist.getPitchAngle();
       
        kinematics.nowJoint_Mech.theta1 = Yaw1.getPosition();
        kinematics.nowJoint_Mech.theta2 = Pitch1.getPosition();
        kinematics.nowJoint_Mech.theta3 = Pitch2.getPosition();
        kinematics.nowJoint_Mech.theta4 = Roll1.getPosition();
        kinematics.nowJoint_Mech.theta5 = wrist.getPitchAngle();
        kinematics.nowJoint_Mech.theta6 = wrist.getRollAngle();
        tarPoint = kinematics.GetNowEndPoint();
        setJoint = kinematics.GetNowJointAngle(tarPoint,1);
    }  
    if(DEBUG_MODE == 2){          
        setJoint = kinematics.GetNowJointAngle(tarPoint,1);
        Yaw1.setPosition = setJoint.theta1;
        Pitch1.setPosition = setJoint.theta2;
        Pitch2.setPosition = setJoint.theta3;
        Roll1.setPosition = setJoint.theta4+360*Roll1.canInfo.totalRound;
        wrist.setPitch = setJoint.theta5;
        wrist.setRoll = setJoint.theta6;        
    }
    if(DEBUG_MODE == 3){
//        tarPoint.xPos += rcCtrl.rc.ch[0]*0.00005;
//        tarPoint.yPos += rcCtrl.rc.ch[1]*0.00005;
//        tarPoint.zPos += rcCtrl.rc.ch[2]*0.00005;
//        
//        setJoint = kinematics.GetNowJointAngle(tarPoint,1);
        Yaw1.setPosition = setJoint.theta1;
        Pitch1.setPosition = setJoint.theta2;
        Pitch2.setPosition = setJoint.theta3; 
        Roll1.setPosition = setJoint.theta4;
        wrist.setPitch = setJoint.theta5;
        wrist.setRoll = setJoint.theta6;    
        
        Yaw1.ctrlPosition(Yaw1.setPosition);
        Pitch1.ctrlPosition(Pitch1.setPosition);
        Pitch2.ctrlPosition(Pitch2.setPosition);
        Roll1.ctrlPosition(Roll1.setPosition);
        wrist.ctrlPosition(wrist.setRoll,wrist.setPitch);
    } 
    
    
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
