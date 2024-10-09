#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "board.h"

enum EulerAngle{
    pitch = 0,
    roll,
    yaw,
};

// 机械参数

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

    EndPoint GetEndPoint(JointAngle joint_rps);
    JointAngle GetJointAngle(EndPoint tarEndPoint,u8 elbm = 1);

    void mat2qua2euler(float(*m)[3],float(*euler));

    float euler[3];
    
};

extern Kinematics kinematics;

#endif /* __KINEMATICS_H__ */


