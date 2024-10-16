#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_

#include "board.h"

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

struct EndState
{
public:
    EndState()
    {
        this->x = 0;
        this->y = 0;
        this->z = 0;
        this->vx = 0;
        this->vy = 0;
        this->vz = 0;
        this->ax = 0;
        this->ay = 0;
        this->az = 0;        
    }
    EndState(float x, float y, float z, float vx, float vy,float vz,float ax,float ay,float az)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->vx = vx;
        this->vy = vy;
        this->vz = vz;
        this->ax = ax;
        this->ay = ay;
        this->az = az;     
    } 
    EndState operator-(EndState next)
    {
        EndState result;
        result.x = x - next.x;
        result.y = y - next.y;
        result.z = z - next.z;
        result.vx = vx - next.vx;
        result.vy = vy - next.vy;
        result.vz = vz - next.vz;
        result.ax = ax - next.ax;
        result.ay = ay - next.ay;
        result.az = az - next.az;        
        return result;
    }      
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float ax;
    float ay;
    float az;
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
//关节电机输出力矩
struct EndForce
{
public:
    EndForce()
    {
        this->f1=0;
        this->f2=0;
        this->f3=0;
    }
    EndForce(float f1,float f2,float f3)
    {
        this->f1=f1;
        this->f2=f2;
        this->f3=f3;        
    }
    EndForce operator+(EndForce next)
    {
        EndForce result;
        result.f1 = f1 + next.f1;
        result.f2 = f2 + next.f2;
        result.f3 = f3 + next.f3;
        return result;
    }
    EndForce operator-(EndForce next)
    {
        EndForce result;
        result.f1 = f1 - next.f1;
        result.f2 = f2 - next.f2;
        result.f3 = f3 - next.f3;
        return result;
    }           
    float f1;
    float f2;
    float f3;
};

struct Dynamics 
{
public:
    Dynamics();
    JointState GetNowEndState(void); 
    JointState GetNowJointState(void);

    JointTorque GetJointTorque(JointState joint_state,EndForce end_force = EndForce());  


};
int sgn(float z);
extern Dynamics dynamics;

#endif
