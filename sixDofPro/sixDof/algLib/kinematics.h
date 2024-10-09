#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "board.h"

#define RAD_TO_DEGREE 57.295779513f
#define DEGREE_TO_RAD 0.017452007f
//机械结构传动比 实际单位与电机单位之比
// #define X_MECH_TO_KINE_RATIO 0.01303f     // 483 18583.4609+18482.6074
// #define Y_MECH_TO_KINE_RATIO 0.012151f     // 321-24 25004.3125-562
#define THETA1_MECH_TO_RPS_RATIO -0.017452007f  //90 1154
#define THETA2_MECH_TO_RPS_RATIO -0.017452007f
#define THETA3_MECH_TO_RPS_RATIO 0.017452007f
#define THETA4_MECH_TO_RPS_RATIO -0.017452007f
#define THETA5_MECH_TO_RPS_RATIO 0.017452007f
#define THETA6_MECH_TO_RPS_RATIO 0.017452007f

enum EulerAngle{
    pitch = 0,
    roll,
    yaw,
};

// 机械参数
struct MechParam
{
    float l1;
    float l2;
    float l3;
    float l4;
};
struct JointState
{
public:
    JointState()
    {
        this->q1 = 0;
        this->q2 = 0;
        this->q3 = 0;
        this->dq1 = 0;
        this->dq2 = 0;
        this->dq3 = 0;
        this->ddq1 = 0;
        this->ddq2 = 0;
        this->ddq3 = 0;        
    }
    JointState(float q1, float q2, float q3, float dq1, float dq2,float dq3,float ddq1,float ddq2,float ddq3)
    {
        this->q1 = q1;
        this->q2 = q2;
        this->q3 = q3;
        this->dq1 = dq1;
        this->dq2 = dq2;
        this->dq3 = dq3;
        this->ddq1 = ddq1;
        this->ddq2 = ddq2;
        this->ddq3 = ddq3;     
    } 
    JointState operator-(JointState next)
    {
        JointState result;
        result.q1 = q1 - next.q1;
        result.q2 = q2 - next.q2;
        result.q3 = q3 - next.q3;
        result.dq1 = dq1 - next.dq1;
        result.dq2 = dq2 - next.dq2;
        result.dq3 = dq3 - next.dq3;
        result.ddq1 = ddq1 - next.ddq1;
        result.ddq2 = ddq2 - next.ddq2;
        result.ddq3 = ddq3 - next.ddq3;        
        return result;
    }      
    float q1;
    float q2;
    float q3;
    float dq1;
    float dq2;
    float dq3;
    float ddq1;
    float ddq2;
    float ddq3;
};
//关节电机输出力矩
struct JointTorque
{
public:
    JointTorque()
    {
        this->tao1=0;
        this->tao2=0;
        this->tao3=0;
    }
    JointTorque operator+(JointTorque next)
    {
        JointTorque result;
        result.tao1 = tao1 + next.tao1;
        result.tao2 = tao2 + next.tao2;
        result.tao3 = tao3 + next.tao3;
        return result;
    }
    JointTorque operator-(JointTorque next)
    {
        JointTorque result;
        result.tao1 = tao1 - next.tao1;
        result.tao2 = tao2 - next.tao2;
        result.tao3 = tao3 - next.tao3;
        return result;
    }           
    float tao1;
    float tao2;
    float tao3;
};
//关节角参数
struct JointAngle
{
public:
    JointAngle()
    {
        this->theta1=0;
        this->theta2=0;
        this->theta3=0;
        this->theta4=0;
        this->theta5=0;
        this->theta6=0;
    }
    JointAngle(float theta1, float theta2, float theta3, float theta4, float theta5,float theta6,float theta7)
    {
        this->theta1=theta1;
        this->theta2=theta2;
        this->theta3=theta3;
        this->theta4=theta4;
        this->theta5=theta5;
        this->theta6=theta6;
    }  
    JointAngle operator-(JointAngle next)
    {
        JointAngle result;
        result.theta1 = theta1 - next.theta1;
        result.theta2 = theta2 - next.theta2;
        result.theta3 = theta3 - next.theta3;
        result.theta4 = theta4 - next.theta4;
        result.theta5 = theta5 - next.theta5;
        result.theta6 = theta6 - next.theta6;
        return result;
    }    
    float theta1;
    float theta2;
    float theta3;
    float theta4;
    float theta5;
    float theta6;
};
/// @brief 单位mm,°
/// @brief yaw,pitch,roll的定义方式为Z-Y-X欧拉角：先绕Z转yaw角，再绕Y转roll角，最后绕X转pitch角
struct EndPoint
{
public:
    EndPoint()
    {
        this->xPos= 0;
        this->yPos = 0;
        this->zPos = 0;
        this->yaw = 0;
        this->pitch = 0;
        this->roll = 0;
    }
    EndPoint(float xPos, float yPos, float zPos, float yaw, float pitch, float roll)
    {
        this->xPos = xPos;
        this->yPos = yPos;
        this->zPos = zPos;
        this->yaw = yaw;
        this->pitch = pitch;
        this->roll = roll;
    } 
    EndPoint operator+(EndPoint next)
    {
        EndPoint result;
        result.xPos = xPos + next.xPos;
        result.yPos = yPos + next.yPos;
        result.zPos = zPos + next.zPos;
        result.yaw = yaw + next.yaw;
        result.pitch = pitch + next.pitch;
        result.roll = roll + next.roll;
        return result;
    }        
    float xPos;
    float yPos;
    float zPos;
    float yaw;
    float pitch;
    float roll;

};
// 内容，正逆运动学
// 逆向，根据目标位姿，求出各关节设定值
struct Kinematics
{
public:
    Kinematics();
    JointAngle GetNowJointAngle(EndPoint tarEndPoint,u8 elbm = 1);
    EndPoint GetNowEndPoint(void);
    EndPoint GetNowEndPoint(JointAngle nowJointdps);
    JointState GetNowEndState(void); 

    EndPoint moveAlongAxis(float deltaPos, EndPoint _prePoint); // 求解沿轴线运动的坐标变化

    JointAngle setAngle_Rps;                       // 关节参数
    JointAngle setAngle_Mech;
    JointAngle nowJoint_Rps;
    JointAngle nowJoint_Mech;

    EndPoint nowEndPoint;
    EndPoint tarPoint; // 目标点

    float qua[4];
    float euler[3];
    void mat2qua2euler(float(*m)[3], float* qua,float(*euler));
    
private:
    MechParam mech;
};
struct Dynamics 
{
public:
    Dynamics();
    JointTorque GetJointTorque(JointState jointstate);  
    JointTorque ImpedanceControl(JointState desired_state);
    JointState GetNowJointState(void);
    JointState nowJointState;
    JointState JointToEndPoint(JointState joint_state);
    EndPoint AdmittanceControl(JointTorque desired_torque); 

};
extern Kinematics kinematics;
extern JointAngle nowJointParam;
extern Dynamics dynamics;

#endif /* __KINEMATICS_H__ */


