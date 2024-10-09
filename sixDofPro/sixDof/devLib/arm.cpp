#include "arm.h"
#include "SMUniversal.h"

MechParam mech;

RoboticArm :: RoboticArm()
{
}

//将电机的角度转换成解算模型的角度
JointAngle RoboticArm::MechToCalculate(JointAngle mech_angle)
{
    JointAngle calculate_angle;
    calculate_angle.theta1 = mech_angle.theta1*THETA1_MECH_TO_RPS_RATIO;
    calculate_angle.theta2 = (mech_angle.theta2-90)*THETA2_MECH_TO_RPS_RATIO;
    calculate_angle.theta3 = mech_angle.theta3*THETA3_MECH_TO_RPS_RATIO;
    calculate_angle.theta4 = (mech_angle.theta4-180)*THETA4_MECH_TO_RPS_RATIO;
    calculate_angle.theta5 = mech_angle.theta5*THETA5_MECH_TO_RPS_RATIO;
    calculate_angle.theta6 = mech_angle.theta6*THETA6_MECH_TO_RPS_RATIO;
    return calculate_angle;
}

//将解算角度转换为电机角度
JointAngle RoboticArm::CalculateToMech(JointAngle calculate_angle)
{
    JointAngle mech_angle;
    mech_angle.theta1 = calculate_angle.theta1 * THETA1_RPS_TO_MECH_RATIO;
    mech_angle.theta2 = (calculate_angle.theta2-(PI/2))*THETA2_RPS_TO_MECH_RATIO;
    mech_angle.theta3 = calculate_angle.theta3*THETA3_RPS_TO_MECH_RATIO;
    mech_angle.theta4 = (calculate_angle.theta4-PI)*THETA4_RPS_TO_MECH_RATIO;
    mech_angle.theta5 = calculate_angle.theta5*THETA5_RPS_TO_MECH_RATIO;
    mech_angle.theta6 = calculate_angle.theta6*THETA6_RPS_TO_MECH_RATIO; 
    return mech_angle;    
}

//默认正解
//输出 tempPoint--此刻末端位姿 xyz单位m pyr旋转顺序--yxz
void RoboticArm::GetNowParam(void) 
{ 
    now_joint_angle.theta1 = Yaw1.getPosition()*THETA1_MECH_TO_RPS_RATIO;
    now_joint_angle.theta2 = (Pitch1.getPosition()-90)*THETA2_MECH_TO_RPS_RATIO;
    now_joint_angle.theta3 = Pitch2.getPosition()*THETA3_MECH_TO_RPS_RATIO;
    now_joint_angle.theta4 = (Roll1.getPosition()-180)*THETA4_MECH_TO_RPS_RATIO;
    now_joint_angle.theta5 = wrist.getPitchAngle()*THETA5_MECH_TO_RPS_RATIO;
    now_joint_angle.theta6 = wrist.getRollAngle()*THETA6_MECH_TO_RPS_RATIO;
    
    //x,y,z正解
    float c1,s1,c2,s2,c23,s23;
    
    c1 = cosf(now_joint_angle.theta1);
    s1 = sinf(now_joint_angle.theta1);
    c2 = cosf(now_joint_angle.theta2);
    s2 = sinf(now_joint_angle.theta2);    
    c23 = cosf(now_joint_angle.theta2 + now_joint_angle.theta3);
    s23 = sinf(now_joint_angle.theta2 + now_joint_angle.theta3);
    
    now_end_point.xPos = c1*(c2*mech.l1 + c23*mech.l2);
    now_end_point.yPos = s1*(c2*mech.l1 + c23*mech.l2);
    now_end_point.zPos = mech.l1*s2 + mech.l2*s23;
    
    float c4,s4,c5,s5,c6,s6;
    
    c4 = cosf(now_joint_angle.theta4);
    s4 = sinf(now_joint_angle.theta4);
    c5 = cosf(now_joint_angle.theta5);
    s5 = sinf(now_joint_angle.theta5);
    c6 = cosf(now_joint_angle.theta6);
    s6 = sinf(now_joint_angle.theta6);    


    float rmat[3][3]={
        {c1*c5*c23 - s5*(s1*s4 + c1*c4*s23), s6*(c5*(s1*s4 + c1*c4*s23) + c1*c23*s5) - c6*(c4*s1 - c1*s4*s23),s6*(c4*s1 - c1*s4*s23) + c6*(c5*(s1*s4 + c1*c4*s23) + c1*c23*s5)},
        {s5*(c1*s4 - c4*s1*s23) + c5*c23*s1, c6*(c1*c4 + s1*s4*s23) - s6*(c5*(c1*s4 - c4*s1*s23) - c23*s1*s5),- s6*(c1*c4 + s1*s4*s23) - c6*(c5*(c1*s4 - c4*s1*s23) - c23*s1*s5)},
        {            - c5*s23 - c4*c23*s5  ,                              c6*c23*s4 - s6*(s5*s23 - c4*c5*c23),   - c6*(s5*s23 - c4*c5*c23) - c23*s4*s6                          }
    };
    
    float Euler[3];

    kine.mat2qua2euler(rmat,Euler);

    now_end_point.yaw = Euler[yaw] * 0.017452007f;           
    now_end_point.pitch = Euler[pitch] * 0.017452007f;
    now_end_point.roll = Euler[roll] * 0.017452007f;
    
    now_joint_state.q1 = now_joint_angle.theta1;
    now_joint_state.q2 = now_joint_angle.theta2;
    now_joint_state.q3 = now_joint_angle.theta3;
    
    now_joint_state.dq1 = -Yaw1.canInfo.vel_rads;
    now_joint_state.dq2 = -Pitch1.canInfo.vel_rads;
    now_joint_state.dq3 = Pitch2.canInfo.vel_rads;
    
//关节加速度无法测得，所以默认使用关节电机的力矩代替，下面的末端状态的加速度同理
    now_joint_state.ddq1 = -Yaw1.canInfo.toq_nm;
    now_joint_state.ddq2 = -Pitch1.canInfo.toq_nm;
    now_joint_state.ddq3 = Pitch2.canInfo.toq_nm;

//雅可比矩阵
//  -s1*c2*l1 - s1*c23*l2  -c1*s2*l1-c1*s23*l2  -c1*s23*l2
//  c1*c2*l1 + c1*c23*l2  -s1*s2*l1-s1*s23*l2  -s1*s23*l2
//  0                     c2*l2+c23*l2         c23*l2
    float mat[3][3] = {{(-s1*c2*mech.l1 - s1*c23*mech.l2),(-c1*s2*mech.l1-c1*s23*mech.l2),(-c1*s23*mech.l2)},
                            {(c1*c2*mech.l1 + c1*c23*mech.l2),(-s1*s2*mech.l1-s1*s23*mech.l2),( -s1*s23*mech.l2)},
                            {0,(c2*mech.l2+c23*mech.l2),(c23*mech.l2)}};

    memcpy(Jacobin,mat,sizeof(mat));
        
    now_end_state.x = now_end_point.xPos;
    now_end_state.y = now_end_point.yPos;
    now_end_state.z = now_end_point.zPos;
                            
    now_end_state.vx = Jacobin[0][0]*now_joint_state.dq1+Jacobin[0][1]*now_joint_state.dq2+Jacobin[0][2]*now_joint_state.dq3;
    now_end_state.vy = Jacobin[1][0]*now_joint_state.dq1+Jacobin[1][1]*now_joint_state.dq2+Jacobin[1][2]*now_joint_state.dq3;
    now_end_state.vz = Jacobin[2][0]*now_joint_state.dq1+Jacobin[2][1]*now_joint_state.dq2+Jacobin[2][2]*now_joint_state.dq3;
                            
    now_end_state.ax = Jacobin[0][0]*now_joint_state.ddq1+Jacobin[0][1]*now_joint_state.ddq2+Jacobin[0][2]*now_joint_state.ddq3;
    now_end_state.ay = Jacobin[1][0]*now_joint_state.ddq1+Jacobin[1][1]*now_joint_state.ddq2+Jacobin[1][2]*now_joint_state.ddq3;
    now_end_state.az = Jacobin[2][0]*now_joint_state.ddq1+Jacobin[2][1]*now_joint_state.ddq2+Jacobin[2][2]*now_joint_state.ddq3;    
}

void RoboticArm::ctrlPosition(JointState tarstate,EndForce extern_force,u8 mode)
{
    EndPoint tar_point;
    JointState now_state;
    
    tar_point.xPos = tarstate.q1;
    tar_point.yPos = tarstate.q2;
    tar_point.zPos = tarstate.q3;
    tar_point.pitch = now_end_point.pitch;
    tar_point.roll = now_end_point.roll;
    tar_point.yaw = now_end_point.yaw;
    
    //用目标加速度来估计当前加速度
    now_state.q1 = now_joint_state.q1;
    now_state.q2 = now_joint_state.q2;
    now_state.q3 = now_joint_state.q3;
    now_state.dq1 = now_joint_state.dq1;
    now_state.dq2 = now_joint_state.dq2;
    now_state.dq3 = now_joint_state.dq3;    
    now_state.ddq1 = tarstate.ddq1;
    now_state.ddq2 = tarstate.ddq2;
    now_state.ddq3 = tarstate.ddq3;
    
    JointAngle set_joint = CalculateToMech(CalculateAngleHandle(kine.GetJointAngle(tar_point)));
    
    JointTorque set_torque = dyna.GetJointTorque(now_state);
    
    Yaw1.ctrlPosition(set_joint.theta1,-set_torque.tao1);
    Pitch1.ctrlPosition(set_joint.theta2,-set_torque.tao2);
    Pitch2.ctrlPosition(set_joint.theta3,set_torque.tao3);
    Roll1.ctrlPosition(set_joint.theta4);
    wrist.ctrlPosition(set_joint.theta6,set_joint.theta5);
    
}

uint8_t RoboticArm::exitStatus(EndForce maxForce, float deathRoomTime, float outTime)
{   
    //默认err为pid位置环的误差，有输入值时err为设定位置与当前值的误差
    EndForce temp_force;
    JointTorque temp_torque;
    
    //静态下关节扭矩
    temp_torque.tao3 =  0.24214 * 1.6 * cosf(now_joint_angle.theta2+now_joint_angle.theta3)*9.81 ;
    temp_torque.tao2 =  temp_torque.tao3 + 0.14925 * (1.6+1) * cosf(now_joint_angle.theta2)*9.81 ;
    temp_torque.tao1 =  0 ;
    //对应静态下的末端力
    temp_force.f1 = Jacobin[0][1]*temp_torque.tao2+Jacobin[0][2]*temp_torque.tao3 + maxForce.f1;
    temp_force.f2 = Jacobin[1][1]*temp_torque.tao2+Jacobin[1][2]*temp_torque.tao3 + maxForce.f2;
    temp_force.f3 = Jacobin[2][1]*temp_torque.tao2+Jacobin[2][2]*temp_torque.tao3 + maxForce.f3;
    
    EndForce err_force;
    err_force.f1 = now_end_state.ax-temp_force.f1;
    err_force.f2 = now_end_state.ay-temp_force.f2;
    err_force.f3 = now_end_state.az-temp_force.f3;
    
    if (fabs(delayTime) <0.0001f) // 首次开始
    {
    }
    delayTime += 2; //若任务运行频率为500Hz则默认为2ms
    if (fabs(err_force.f1) < 0.1f && fabs(err_force.f2) < 0.1f && fabs(err_force.f2) < 0.1f && delayTime > deathRoomTime)
    {
        delayTime = 0;
        return 1;
    }
    if (delayTime > outTime)
    {
        delayTime = 0;
        return 1;
    }
    return 0;    
}    

JointAngle RoboticArm::CalculateAngleHandle(JointAngle calculate_angle)
{
    JointAngle set_angle;
    set_angle = calculate_angle;
    if(fabs(calculate_angle.theta1)>PI/2)
        set_angle.theta1 = Yaw1.getPosition()*THETA1_MECH_TO_RPS_RATIO;
    if(calculate_angle.theta2<PI/6 || calculate_angle.theta2>PI*5/6)
        set_angle.theta2 = (Pitch1.getPosition()-90)*THETA2_MECH_TO_RPS_RATIO;
    if(calculate_angle.theta3 >-PI/6 || calculate_angle.theta3<-PI*2/3)
        set_angle.theta3 = Pitch2.getPosition()*THETA3_MECH_TO_RPS_RATIO;    
    if(fabs(calculate_angle.theta5)>PI/2)
        set_angle.theta5 = wrist.getPitchAngle()*THETA5_MECH_TO_RPS_RATIO;
    if(calculate_angle.theta6>PI)
        set_angle.theta6 = wrist.getRollAngle()*THETA6_MECH_TO_RPS_RATIO;

    static int totalRound = 0;
    static float last_theta4 = 0;    
    
    if(armcycle.getCycleT()*1000 > 6){
        totalRound = Roll1.canInfo.totalRound;
        last_theta4 = 0;
    }
      
    if(calculate_angle.theta4 - last_theta4 > PI)
        totalRound --;
    else if(calculate_angle.theta4 - last_theta4 < -PI)
        totalRound ++;
    set_angle.theta4 = totalRound*2*PI+calculate_angle.theta4;
    last_theta4 = calculate_angle.theta4;

    return set_angle;
}

RoboticArm Arm;
