#include "dynamics.h"
#include "SMUniversal.h"
#include "arm.h"

//待填充
Dynamics::Dynamics()
{
}
//获得当前目标点的状态
JointState Dynamics::GetNowEndState(void) 
{
    JointState tempState;
    
    JointAngle nowJoint_Rps;
 
    nowJoint_Rps.theta1 = Yaw1.getPosition()*THETA1_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta2 = (Pitch1.getPosition()-90)*THETA2_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta3 = Pitch2.getPosition()*THETA3_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta4 = (Roll1.getPosition()-180)*THETA4_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta5 = wrist.getPitchAngle()*THETA5_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta6 = wrist.getRollAngle()*THETA6_MECH_TO_RPS_RATIO;
    
    //x,y,z正解
    float c1,s1,c2,s2,c23,s23;
    
    c1 = cosf(nowJoint_Rps.theta1);
    s1 = sinf(nowJoint_Rps.theta1);
    c2 = cosf(nowJoint_Rps.theta2);
    s2 = sinf(nowJoint_Rps.theta2);    
    c23 = cosf(nowJoint_Rps.theta2 + nowJoint_Rps.theta3);
    s23 = sinf(nowJoint_Rps.theta2 + nowJoint_Rps.theta3);
    
    tempState.q1 = c1*(c2*mech.l1 + c23*mech.l2);
    tempState.q2 = s1*(c2*mech.l1 + c23*mech.l2);
    tempState.q3 = mech.l1*s2 + mech.l2*s23;

//  -s1*c2*l1 - s1*c23*l2  -c1*s2*l1-c1*s23*l2  -c1*s23*l2
//  c1*c2*l1 + c1*c23*l2  -s1*s2*l1-s1*s23*l2  -s1*s23*l2
//  0                     c2*l2+c23*l2         c23*l2
    float Jacobin[3][3] = {
                    {-s1*c2*mech.l1 - s1*c23*mech.l2,-c1*s2*mech.l1-c1*s23*mech.l2,-c1*s23*mech.l2},
                    {c1*c2*mech.l1 + c1*c23*mech.l2,-s1*s2*mech.l1-s1*s23*mech.l2,-s1*s23*mech.l2},
                    {0,c2*mech.l2+c23*mech.l2,c23*mech.l2}
                    };
    
    float v[3] = {-Yaw1.canInfo.vel_rads,-Pitch1.canInfo.vel_rads,Pitch2.canInfo.vel_rads};
    
    tempState.dq1 = Jacobin[0][0]*v[0]+Jacobin[0][1]*v[1]+Jacobin[0][2]*v[2];
    tempState.dq2 = Jacobin[1][0]*v[0]+Jacobin[1][1]*v[1]+Jacobin[1][2]*v[2];
    tempState.dq3 = Jacobin[2][0]*v[0]+Jacobin[2][1]*v[1]+Jacobin[2][2]*v[2];
    
    float f[3] = {-Yaw1.canInfo.toq_nm,-Pitch1.canInfo.toq_nm,Pitch2.canInfo.toq_nm};
    
    tempState.ddq1 = Jacobin[0][0]*f[0]+Jacobin[0][1]*f[1]+Jacobin[0][2]*f[2];
    tempState.ddq2 = Jacobin[1][0]*f[0]+Jacobin[1][1]*f[1]+Jacobin[1][2]*f[2];
    tempState.ddq3 = Jacobin[2][0]*f[0]+Jacobin[2][1]*f[1]+Jacobin[2][2]*f[2];    
    
    return tempState;  
}
//获得现在关节角度状态；
JointState Dynamics::GetNowJointState(void)
{
    JointState temp_joint_state;
    temp_joint_state.q1 = Yaw1.getPosition()*THETA1_MECH_TO_RPS_RATIO;
    temp_joint_state.q2 = (Pitch1.getPosition()-90)*THETA2_MECH_TO_RPS_RATIO;
    temp_joint_state.q3 = Pitch2.getPosition()*THETA3_MECH_TO_RPS_RATIO;
    temp_joint_state.dq1 = -Yaw1.canInfo.vel_rads;
    temp_joint_state.dq2 = -Pitch1.canInfo.vel_rads;
    temp_joint_state.dq3 = Pitch2.canInfo.vel_rads;
    temp_joint_state.ddq1 = Yaw1.canInfo.toq_nm;
    temp_joint_state.ddq2 = Pitch1.canInfo.toq_nm;
    temp_joint_state.ddq3 = Pitch2.canInfo.toq_nm;   
    
    return temp_joint_state;
}
////机械臂动力学
////输入关节状态 -- [p1,p2,p3;dp1,dp2,dp3,ddp1,ddp2,ddp3]
////输出关节力矩 -- [t1,t2,t3]
JointTorque Dynamics::GetJointTorque(JointState jointstate,EndForce end_force)
{
    /* 这个形式可以应用计算力矩法，并且惯量矩阵的对角线上的值都可以参与运算*/
    JointTorque Joint_Torque;
    double tao3, tao2, tao1, f1,f2,f3,n11x, n11y, n11z, n22y, n22x, n33x, n33y, n33z;
    double q1, qd1, qdd1, q2, qd2, qdd2, q3, qd3, qdd3;   //关节状态
    double a1, a2, a3, m1, m2, m3,acc1,acc2;
    double Ixx1, Iyy1, Izz1, Ixx2, Izz2, Iyy2, Izz3, Ixx3, Iyy3;
    double M1_Q1, C1_Q1Q3, C1_Q1Q2, G2, C2_Q1Q1, C2_Q2Q2, C2_Q2Q3, C2_Q3Q3, M2_Q3, M2_Q2, G3, C3_Q1Q1, C3_Q2Q2, M3_Q3, M3_Q2;
    q1 = jointstate.q1; qd1 = jointstate.dq1; qdd1 = jointstate.ddq1;
    q2 = jointstate.q2; qd2 = jointstate.dq2; qdd2 = jointstate.ddq2;
    q3 = jointstate.q3; qd3 = jointstate.dq3; qdd3 = jointstate.ddq3;

    a1 = 0; a2 = 0.237; a3 = 0.34,acc1=0.14925,acc2=0.24214;
    m1 = 0; m2 = 1;  m3 = 1.6;
    Izz1 = 0;Ixx2 = 0.0095;Ixx3 = 0.0134;
    Iyy1 = 0; Iyy2 = 0.007;Iyy3 = 0.01711;
    Ixx1 = 0;Izz2 = 0.0074; Izz3 = 0.0052;

    G3 = acc2 * m3 * cosf(q2 + q3) * 9.81;
    C3_Q1Q1 = a3 * m3 * ((0.5 * a2 * sinf(q3) + 0.5 * a3 * sinf(2 * (q2 + q3)) + 0.5 * a2 * sinf(2 * q2 + q3))) + 0.5 * Iyy3 * sinf(2 * (q2 + q3));
    C3_Q2Q2 = a3 * m3 * a2 * sinf(q3);
    M3_Q3 = (a3 * a3 * m3 + Izz3);
    M3_Q2 = (a3 * m3 * (a2 * cosf(q3) + a3) + Izz3);
    
    tao3 = M3_Q2 * qdd2 + M3_Q3 * qdd3 + C3_Q1Q1 * qd1 * qd1 + C3_Q2Q2 * qd2 * qd2 + G3;

    
    G2 = tao3 + acc1 * cosf(q2) * (9.81 * m2 + 9.81 * m3);
    C2_Q1Q1 = a2 * (a2 * sinf(2 * q2) * 0.5 * (m2 + m3) + a3 * m3 * (-0.5 * sinf(q3) + 0.5 * sinf(2 * q2 + q3))) + 0.5 * sinf(2 * q2) * Iyy2;
    C2_Q2Q2 = -a2 * a3 * m3 * sinf(q3);
    C2_Q2Q3 = -2 * a2 * a3 * m3 * sinf(q3);
    C2_Q3Q3 = -a2 * a3 * m3 * sinf(q3);
    M2_Q3 = a2 * a3 * m3 * cosf(q3);
    M2_Q2 = (a2 * (a2 * m2 + a2 * m3 + a3 * m3 * cosf(q3)) + Izz2);
    tao2 = M2_Q2 * qdd2 + M2_Q3 * qdd3 + C2_Q1Q1 * qd1 * qd1 + C2_Q2Q2 * qd2 * qd2 + C2_Q2Q3 * qd2 * qd3 + C2_Q3Q3 * qd3 * qd3 + G2;



    M1_Q1 = 0.500 * (a3 * a3 * m3 + a2 * a2 * (m2 + m3) + a2 * a2 * (m2 + m3) * cosf(2 * q2) + a3 * m3 * (4 * a2 * cosf(q2) * cosf(q2 + q3) + a3 * cosf(2 * (q2 + q3))) + 2 * sinf(q2) * sinf(q2) * Ixx2 + 2 * sinf(q2 + q3) * sinf(q2 + q3) * Ixx3 + 2 * Iyy1 + 2 * cosf(q2) * cosf(q2) * Iyy2 + 2 * cosf(q2 + q3) * cosf(q2 + q3) * Iyy3);
    C1_Q1Q3 = -(2 * sinf(q2 + q3) * (a3 * m3 * (a2 * cosf(q2) + a3 * cosf(q2 + q3)) + cosf(q2 + q3) * (-Ixx3 + Iyy3)));
    C1_Q1Q2 = -(a2 * a2 * (m2 + m3) * sinf(2 * q2) + a3 * m3 * (a3 * sinf(2 * (q2 + q3)) + 2 * a2 * sinf(2 * q2 + q3)) + sinf(2 * q2) * (-Ixx2 + Iyy2) + sinf(2 * (q2 + q3)) * (-Ixx3 + Iyy3));
    tao1 = M1_Q1 * qdd1 + C1_Q1Q3 * qd1 * qd3 + C1_Q1Q2 * qd1 * qd2;

    float c1 = cosf(q1),s1 = sinf(q1),c2 = cosf(q2),s2 = sinf(q2),c23 = cosf(q2+q3),s23 = sinf(q2+q3);

//  -s1*c2*l1 - s1*c23*l2  -c1*s2*l1-c1*s23*l2  -c1*s23*l2
//  c1*c2*l1 + c1*c23*l2  -s1*s2*l1-s1*s23*l2  -s1*s23*l2
//  0                     c2*l2+c23*l2         c23*l2
    f1 = (-s1*c2*a1 - s1*c23*a2)*end_force.f1+(c1*c2*a1 + c1*c23*a2)*end_force.f2;
    f2 = (-c1*s2*a1-c1*s23*a2)*end_force.f1 + (-s1*s2*a1-s1*s23*a2)*end_force.f2 + (c2*a2+c23*a2)*end_force.f3;
    f3 = (-c1*s23*a2)*end_force.f1 + (-s1*s23*a2)*end_force.f2 + (c23*a2)*end_force.f3;
    
    Joint_Torque.tao1 = tao1 + f1;
    Joint_Torque.tao2 = tao2 + f2;
    Joint_Torque.tao3 = tao3 + f3;

    return Joint_Torque;   
}

Dynamics dynamics;
