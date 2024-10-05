#include "kinematics.h"
#include "SMUniversal.h"
#include "math.h"
#include "filter.h"

Kinematics::Kinematics()
{
    // 机械参数加载
    mech.l1 = 237;
    mech.l2 = 340;
}
//注意输入输出
//输入：nowJoint_Mech 读出的电机的原始数据 --需要先给nowJoint_Mech赋值
//输出：nowEndPoint 现在末端点的位姿x y z mm p y r dps
EndPoint Kinematics::GetNowEndPoint(void) 
{
    EndPoint tempPoint;
    nowJoint_Rps.theta1 = nowJoint_Mech.theta1*THETA1_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta2 = (nowJoint_Mech.theta2-90)*THETA2_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta3 = nowJoint_Mech.theta3*THETA3_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta4 = (nowJoint_Mech.theta4-180)*THETA4_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta5 = nowJoint_Mech.theta5*THETA5_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta6 = nowJoint_Mech.theta6*THETA6_MECH_TO_RPS_RATIO;
    
    //x,y,z正解
    float c1,s1,c2,s2,c23,s23;
    
    c1 = cosf(nowJoint_Rps.theta1);
    s1 = sinf(nowJoint_Rps.theta1);
    c2 = cosf(nowJoint_Rps.theta2);
    s2 = sinf(nowJoint_Rps.theta2);    
    c23 = cosf(nowJoint_Rps.theta2 + nowJoint_Rps.theta3);
    s23 = sinf(nowJoint_Rps.theta2 + nowJoint_Rps.theta3);
    
    tempPoint.xPos = c1*(c2*mech.l1 + c23*mech.l2);
    tempPoint.yPos = s1*(c2*mech.l1 + c23*mech.l2);
    tempPoint.zPos = mech.l1*s2 + mech.l2*s23;
    
    float c4,s4,c5,s5,c6,s6;
    
    c4 = cosf(nowJoint_Rps.theta4);
    s4 = sinf(nowJoint_Rps.theta4);
    c5 = cosf(nowJoint_Rps.theta5);
    s5 = sinf(nowJoint_Rps.theta5);
    c6 = cosf(nowJoint_Rps.theta6);
    s6 = sinf(nowJoint_Rps.theta6);    


    float rmat[3][3]={
        {c1*c5*c23 - s5*(s1*s4 + c1*c4*s23), s6*(c5*(s1*s4 + c1*c4*s23) + c1*c23*s5) - c6*(c4*s1 - c1*s4*s23),s6*(c4*s1 - c1*s4*s23) + c6*(c5*(s1*s4 + c1*c4*s23) + c1*c23*s5)},
        {s5*(c1*s4 - c4*s1*s23) + c5*c23*s1, c6*(c1*c4 + s1*s4*s23) - s6*(c5*(c1*s4 - c4*s1*s23) - c23*s1*s5),- s6*(c1*c4 + s1*s4*s23) - c6*(c5*(c1*s4 - c4*s1*s23) - c23*s1*s5)},
        {            - c5*s23 - c4*c23*s5  ,                              c6*c23*s4 - s6*(s5*s23 - c4*c5*c23),   - c6*(s5*s23 - c4*c5*c23) - c23*s4*s6                          }
    };
    
    mat2qua2euler(rmat,qua,euler);

    tempPoint.yaw = euler[yaw] * 0.017452007f;           
    tempPoint.pitch = euler[pitch] * 0.017452007f;
    tempPoint.roll = euler[roll] * 0.017452007f;
    
    return tempPoint;  
}

//机械臂正运动学
//输入--jointAngle关节角度
//输出--对应末端点
EndPoint Kinematics::GetNowEndPoint(JointAngle jointAngle) 
{
    EndPoint tempPoint;
    nowJoint_Rps.theta1 = jointAngle.theta1*THETA1_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta2 = (jointAngle.theta2-90)*THETA2_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta3 = jointAngle.theta3*THETA3_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta4 = (jointAngle.theta4-180)*THETA4_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta5 = jointAngle.theta5*THETA5_MECH_TO_RPS_RATIO;
    nowJoint_Rps.theta6 = jointAngle.theta6*THETA6_MECH_TO_RPS_RATIO;
    
    //x,y,z正解
    float c1,s1,c2,s2,c23,s23;
    
    c1 = cosf(nowJoint_Rps.theta1);
    s1 = sinf(nowJoint_Rps.theta1);
    c2 = cosf(nowJoint_Rps.theta2);
    s2 = sinf(nowJoint_Rps.theta2);    
    c23 = cosf(nowJoint_Rps.theta2 + nowJoint_Rps.theta3);
    s23 = sinf(nowJoint_Rps.theta2 + nowJoint_Rps.theta3);
    
    tempPoint.xPos = c1*(c2*mech.l1 + c23*mech.l2);
    tempPoint.yPos = s1*(c2*mech.l1 + c23*mech.l2);
    tempPoint.zPos = mech.l1*s2 + mech.l2*s23;
    
    float c4,s4,c5,s5,c6,s6;
    
    c4 = cosf(nowJoint_Rps.theta4);
    s4 = sinf(nowJoint_Rps.theta4);
    c5 = cosf(nowJoint_Rps.theta5);
    s5 = sinf(nowJoint_Rps.theta5);
    c6 = cosf(nowJoint_Rps.theta6);
    s6 = sinf(nowJoint_Rps.theta6);    


    float rmat[3][3]={
        {c1*c5*c23 - s5*(s1*s4 + c1*c4*s23), s6*(c5*(s1*s4 + c1*c4*s23) + c1*c23*s5) - c6*(c4*s1 - c1*s4*s23),s6*(c4*s1 - c1*s4*s23) + c6*(c5*(s1*s4 + c1*c4*s23) + c1*c23*s5)},
        {s5*(c1*s4 - c4*s1*s23) + c5*c23*s1, c6*(c1*c4 + s1*s4*s23) - s6*(c5*(c1*s4 - c4*s1*s23) - c23*s1*s5),- s6*(c1*c4 + s1*s4*s23) - c6*(c5*(c1*s4 - c4*s1*s23) - c23*s1*s5)},
        {            - c5*s23 - c4*c23*s5  ,                              c6*c23*s4 - s6*(s5*s23 - c4*c5*c23),   - c6*(s5*s23 - c4*c5*c23) - c23*s4*s6                          }
    };
    
    mat2qua2euler(rmat,qua,euler);

    tempPoint.yaw = euler[yaw] * 0.017452007f;           
    tempPoint.pitch = euler[pitch] * 0.017452007f;
    tempPoint.roll = euler[roll] * 0.017452007f;
    
    return tempPoint;  
}

//机械臂逆解
//输入目标点 ----肘角默认肘向上
//输出对应的关节角
JointAngle Kinematics::GetNowJointAngle(EndPoint tarEndPoint,u8 elbm)
{
    //theta1的逆解
    //唯一解    
    if(fabs(tarEndPoint.xPos)<0.0001f && tarEndPoint.yPos>0)
        setAngle_Rps.theta1 = PI/2;
    else if(fabs(tarEndPoint.xPos)<0.0001f && tarEndPoint.yPos<0)
        setAngle_Rps.theta1 = -PI/2;
    else
        setAngle_Rps.theta1 = atan2f(tarEndPoint.yPos,tarEndPoint.xPos);
    
    //theta3的逆解
    //theta3[-150,150]--考虑是否有解
    if(fabs((powf(tarEndPoint.xPos,2)+powf(tarEndPoint.yPos,2)+powf(tarEndPoint.zPos,2)-powf(mech.l1,2)-powf(mech.l2,2))/(2*mech.l1*mech.l2))>0.999f)
        setAngle_Rps.theta3 = Pitch2.getPosition()*THETA3_MECH_TO_RPS_RATIO;
    else{
        if(elbm)
            setAngle_Rps.theta3 = -acosf( (powf(tarEndPoint.xPos,2)+powf(tarEndPoint.yPos,2)+powf(tarEndPoint.zPos,2)-powf(mech.l1,2)-powf(mech.l2,2))/(2*mech.l1*mech.l2));
        else
            setAngle_Rps.theta3 = acosf( (powf(tarEndPoint.xPos,2)+powf(tarEndPoint.yPos,2)+powf(tarEndPoint.zPos,2)-powf(mech.l1,2)-powf(mech.l2,2))/(2*mech.l1*mech.l2));
    }
    
    //theta2的逆解
    //theta2[0,180]--唯一解
    float c3 = cosf(setAngle_Rps.theta3),s3 = sinf(setAngle_Rps.theta3);
    float x_c;//x_c = tarEndPoint.xpos/cos(theta1) = =tarEndPoint.ypos/sin(theta1)
    if(fabs(cosf(setAngle_Rps.theta1))<0.1f)
        x_c = tarEndPoint.yPos/sinf(setAngle_Rps.theta1);
    else
        x_c = tarEndPoint.xPos/cosf(setAngle_Rps.theta1);
    
    if(fabs((x_c*(mech.l1+c3*mech.l2)+tarEndPoint.zPos*s3*mech.l2)/(powf(mech.l1,2)+powf(mech.l2,2)+2*c3*mech.l1*mech.l2))>0.999f)
         setAngle_Rps.theta2 = (Pitch1.getPosition()-90)*THETA2_MECH_TO_RPS_RATIO;
    else
        setAngle_Rps.theta2 = acosf( (x_c*(mech.l1+c3*mech.l2)+tarEndPoint.zPos*s3*mech.l2)/(powf(mech.l1,2)+powf(mech.l2,2)+2*c3*mech.l1*mech.l2));
        
    float c1 = cosf(setAngle_Rps.theta1),s1 = sinf(setAngle_Rps.theta1),c23 = cosf(setAngle_Rps.theta2+setAngle_Rps.theta3),s23 = sinf(setAngle_Rps.theta2+setAngle_Rps.theta3),cz = cosf(tarEndPoint.yaw),sz = sinf(tarEndPoint.yaw),cy = cosf(tarEndPoint.pitch),sy = sinf(tarEndPoint.pitch),cx = cosf(tarEndPoint.roll),sx = sinf(tarEndPoint.roll);
    float rmat[3][3] = {
        {s23*sy + c1*c23*cy*cz + c23*cy*s1*sz, c23*s1*(cx*cz + sx*sy*sz) - c1*c23*(cx*sz - cz*sx*sy) - cy*s23*sx, c1*c23*(sx*sz + cx*cz*sy) - cx*cy*s23 - c23*s1*(cz*sx - cx*sy*sz)},
        {c1*cy*sz - cy*cz*s1,                  c1*(cx*cz + sx*sy*sz) + s1*(cx*sz - cz*sx*sy),                      - c1*(cz*sx - cx*sy*sz) - s1*(sx*sz + cx*cz*sy)},
        {c1*cy*cz*s23 - c23*sy + cy*s1*s23*sz, s1*s23*(cx*cz + sx*sy*sz) - c1*s23*(cx*sz - cz*sx*sy) + c23*cy*sx, c1*s23*(sx*sz + cx*cz*sy) - s1*s23*(cz*sx - cx*sy*sz) + c23*cx*cy}     
    };
    //求解后三角
    float solve_theta4,solve_theta5,solve_theta6;
    //处理theta5 = 0
    if(fabs(rmat[0][0])>0.999f){
        if(wrist.getPitchAngle()>= -60 && wrist.getPitchAngle()<= 60)
            setAngle_Rps.theta5 = 0;
        if(wrist.getPitchAngle()<-60)
            setAngle_Rps.theta5 = -PI/2;
        if(wrist.getPitchAngle()>60)
            setAngle_Rps.theta5 = PI/2;
        setAngle_Rps.theta4 = (Roll1.getPosition()-180)*THETA4_MECH_TO_RPS_RATIO;
        setAngle_Rps.theta6 = wrist.getRollAngle()*THETA6_MECH_TO_RPS_RATIO;
    }
    else{    
        setAngle_Rps.theta5 = acosf(rmat[0][0]) ;
        //处理theta4 = +-90
        if(fabs(rmat[2][0])<0.01f){
            if(rmat[1][0]*setAngle_Rps.theta5>0)
                setAngle_Rps.theta4 = PI/2;
            else
                setAngle_Rps.theta4 = -PI/2;
        }
        else
            setAngle_Rps.theta4 = atan2f(rmat[1][0],-rmat[2][0]);

        //处理theta5 = +-90
        if(fabs(rmat[0][2])<0.01f){
            if(rmat[0][1]*setAngle_Rps.theta5>0)
                setAngle_Rps.theta6 = PI/2;
            else
                setAngle_Rps.theta6 = -PI/2;
        }
        else
            setAngle_Rps.theta6 = atan2f(rmat[0][1],rmat[0][2]);         
    }
    
    JointAngle setJointAngle;
    setJointAngle.theta1 = setAngle_Rps.theta1 / THETA1_MECH_TO_RPS_RATIO;
    setJointAngle.theta2 = (setAngle_Rps.theta2-(PI/2))/THETA2_MECH_TO_RPS_RATIO;
    setJointAngle.theta3 = setAngle_Rps.theta3/THETA3_MECH_TO_RPS_RATIO;
    setJointAngle.theta4 = (setAngle_Rps.theta4-PI)/THETA4_MECH_TO_RPS_RATIO;
    setJointAngle.theta5 = setAngle_Rps.theta5/THETA5_MECH_TO_RPS_RATIO;
    setJointAngle.theta6 = setAngle_Rps.theta6/THETA6_MECH_TO_RPS_RATIO;   
    
    return setJointAngle;
}

//四元数转换成欧拉角--输入旋转矩阵m[3][3],
//输出欧拉角 ---对应旋转顺序 --pitch roll yaw 
void Kinematics::mat2qua2euler(float(*m)[3], float* qua,float(*euler))
{
	float q1 = sqrt(m[0][0] + m[1][1] + m[2][2] + 1) / 2;
	float q2, q3, q4, tr, s;
	if (fabs(q1)>0.0001f) {
		q2 = (m[2][1] - m[1][2]) / 4 / q1;
		q3 = (m[0][2] - m[2][0]) / 4 / q1;
		q4 = (m[1][0] - m[0][1]) / 4 / q1;
	}
	else {
		tr = m[0][0] + m[1][1] + m[2][2];
		if (tr > 0) {
			s = sqrt(tr + 1.0) * 2;
			q1 = 0.25 * s;
			q2 = (m[2][1] - m[1][2]) / s;
			q3 = (m[0][2] - m[2][0]) / s;
			q4 = (m[1][0] - m[0][1]) / s;
		}
		else if ((m[0][0] > m[1][1]) && (m[0][0] > m[2][2])) {
			s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2;
			q1 = (m[2][1] - m[1][2]) / s;
			q2 = 0.25 * s;
			q3 = (m[0][1] + m[1][0]) / s;
			q4 = (m[0][2] + m[2][0]) / s;
		}
		else if(m[1][1] > m[2][2])
		{
			s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
			q1 = (m[0][2] - m[2][0]) / s;
			q2 = (m[0][1] + m[1][0]) / s;
			q3 = 0.25 * s;
			q4 = (m[1][2] + m[2][1]) / s;
		}
		else {
			s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
			q1 = (m[1][0] - m[0][1]) / s;
			q2 = (m[0][2] + m[2][0]) / s;
			q3 = (m[1][2] + m[2][1]) / s;
			q4 = 0.25 * s;
		}
	}
	qua[0] = q1;
	qua[1] = q2;
	qua[2] = q3;
	qua[3] = q4;

    euler[pitch] = asinf(-2 * qua[1] * qua[3] + 2 * qua[0]* qua[2])* RAD_TO_DEGREE;
    euler[roll] = atan2f(2 * qua[2] * qua[3] + 2 * qua[0] * qua[1], -2 * powf(qua[1],2) - 2 * powf(qua[2],2) + 1)* RAD_TO_DEGREE;
    euler[yaw] = atan2f(2 * (qua[1] *qua[2] + qua[0]*qua[3]),powf(qua[0],2)+powf(qua[1],2)-powf(qua[2],2)-powf(qua[3],2)) * RAD_TO_DEGREE;
}

//待填充
Dynamics::Dynamics()
{
}

////机械臂动力学
////输入关节状态 -- [p1,p2,p3;dp1,dp2,dp3,ddp1,ddp2,ddp3]
////输出关节力矩 -- [t1,t2,t3]
JointTorque Dynamics::GetJointTorque(JointState jointstate)
{
    /* 这个形式可以应用计算力矩法，并且惯量矩阵的对角线上的值都可以参与运算*/
    JointTorque Joint_Torque;
    double tao3, tao2, tao1, n11x, n11y, n11z, n22y, n22x, n33x, n33y, n33z;
    double q1, qd1, qdd1, q2, qd2, qdd2, q3, qd3, qdd3;   //关节状态
    double a1, a2, a3, m1, m2, m3,acc1,acc2;
    double Ixx1, Iyy1, Izz1, Ixx2, Izz2, Iyy2, Izz3, Ixx3, Iyy3;
    double M1_Q1, C1_Q1Q3, C1_Q1Q2, G2, C2_Q1Q1, C2_Q2Q2, C2_Q2Q3, C2_Q3Q3, M2_Q3, M2_Q2, G3, C3_Q1Q1, C3_Q2Q2, M3_Q3, M3_Q2;
    q1 = jointstate.q1; qd1 = jointstate.dq1; qdd1 = jointstate.ddq1;
    q2 = jointstate.q2; qd2 = jointstate.dq2; qdd2 = jointstate.ddq2;
    q3 = jointstate.q3; qd3 = jointstate.dq3; qdd3 = jointstate.ddq3;

    a1 = 0; a2 = 0.237; a3 = 0.33,acc1=0.14925,acc2=0.24214;
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

    Joint_Torque.tao1 = tao1;
    Joint_Torque.tao2 = tao2;
    Joint_Torque.tao3 = tao3;

    return Joint_Torque;
}

//获得现在关节角度状态；
JointState Dynamics::GetNowJointState(void)
{
    nowJointState.q1 = Yaw1.getPosition()*THETA1_MECH_TO_RPS_RATIO;
    nowJointState.q2 = (Pitch1.getPosition()-90)*THETA2_MECH_TO_RPS_RATIO;
    nowJointState.q3 = Pitch2.getPosition()*THETA3_MECH_TO_RPS_RATIO;
    nowJointState.dq1 = -Yaw1.canInfo.vel_rads;
    nowJointState.dq2 = -Pitch1.canInfo.vel_rads;
    nowJointState.dq3 = Pitch2.canInfo.vel_rads;
    nowJointState.ddq1 = Yaw1.canInfo.toq_nm;
    nowJointState.ddq2 = Pitch1.canInfo.toq_nm;
    nowJointState.ddq3 = Pitch2.canInfo.toq_nm;   
    
    return nowJointState;
}

//关节状态到末端点状态的转变
//速度雅可比矩阵：
//  -s1*c2*l1 - s1*c23*l2  -c1*s2*l1-c1*s23*l2  -c1*s23*l2
//  c1*c2*l1 + c1*c23*l2  -s1*s2*l1-s1*s23*l2  -s1*s23*l2
//  0                     c2*l2+c23*l2         c23*l2
JointState Dynamics::JointToEndPoint(JointState joint_state)
{
    JointState tempState;
    
    float c1 = cosf(joint_state.q1),s1 = sinf(joint_state.q1),c2 = cosf(joint_state.q2),s2 = sinf(joint_state.q2),c23 = cosf(joint_state.q2+joint_state.q3),s23 = sinf(joint_state.q2+joint_state.q3);
 
    float l1 = 0.237, l2 = 0.330;
    
    tempState.q1 = c1*(c2*l1 + c23*l2);
    tempState.q2 = s1*(c2*l1 + c23*l2);
    tempState.q3 = l1*s2 + l2*s23;     

    tempState.dq1 = (-s1*(c2*l1 + c23*l2)) * joint_state.dq1 + (-c1*s2*l1-c1*s23*l2) * joint_state.dq2 + (-c1*s23*l2) * joint_state.dq3;
    tempState.dq2 = (c1*(c2*l1 + c23*l2)) * joint_state.dq1 + (-s1*s2*l1-s1*s23*l2) * joint_state.dq2 + (-s1*s23*l2) * joint_state.dq3;
    tempState.dq3 = (c2*l1+c23*l2) * joint_state.ddq2 + (c23*l2) * joint_state.dq3;
    
    tempState.ddq1 = 0;
    tempState.ddq2 = 0;
    tempState.ddq3 = 0;
    
    return tempState;
}

//阻抗控制：F = M x a+ C * v + K * x;M惯性参数，C阻尼系数，K刚度系数
float K[3]={0,0,0},C[3]={0,0,0},M[3]={0,0,0};//0.3 0.4 0.3
Kf Dkf1,Dkf2,Dkf3;
Kf kf1,kf2,kf3;

JointState Estate;
JointTorque testTor;

JointState testState1,testState2;
//输入期望关节状态
//感觉效果不是特别好，阻尼系数给大后就开始抖动
JointTorque Dynamics::ImpedanceControl(JointState desired_state)
{
    JointTorque tempTao;      
    JointTorque estimateT;
 
    float dq[3],q[3];
    
    nowJointState = GetNowJointState();
    JointState Estate = JointToEndPoint(desired_state) - JointToEndPoint(nowJointState);
    
    testState1 = JointToEndPoint(desired_state);
    testState2 = JointToEndPoint(nowJointState);
    
    q[0]  = Estate.q1;
    q[1]  = Estate.q2;
    q[2]  = Estate.q3;
    
//    q[0] = kf1.KalmanFilter(Estate.q1,1,100,0);
//    q[1] = kf1.KalmanFilter(Estate.q2,1,100,0);
//    q[2] = kf1.KalmanFilter(Estate.q3,1,100,0);
    
    float F[3];//-3 -40 10
    
    dq[0] = Dkf1.KalmanFilter(Estate.dq1,0.3,100,0);
    dq[1] = Dkf2.KalmanFilter(Estate.dq2,0.3,100,0);
    dq[2] = Dkf3.KalmanFilter(Estate.dq3,0.3,100,0);
    
    F[0] = C[0]*dq[0]+K[0]*q[0];
    F[1] = C[1]*dq[1]+K[1]*q[1];
    F[2] = C[2]*dq[2]+K[2]*q[2];  
    
    testTor.tao1 = F[0];
    testTor.tao2 = F[1];
    testTor.tao3 = F[2];
   
//  -s1*c2*l1 - s1*c23*l2  -c1*s2*l1-c1*s23*l2  -c1*s23*l2
//  c1*c2*l1 + c1*c23*l2  -s1*s2*l1-s1*s23*l2  -s1*s23*l2
//  0                     c2*l1+c23*l2         c23*l2
    float c1 = cosf(nowJointState.q1),s1 = sinf(nowJointState.q1),c2 = cosf(nowJointState.q2),s2 = sinf(nowJointState.q2),c23 = cosf(nowJointState.q2 + nowJointState.q3),s23 = sinf(nowJointState.q2 + nowJointState.q3);
    float l1 = 0.237,l2 = 0.340;
    
    tempTao.tao1 = (-s1*c2*l1 - s1*c23*l2)*F[0] + (c1*c2*l1 + c1*c23*l2)*F[1] ;
    tempTao.tao2 = (-c1*s2*l1-c1*s23*l2)*F[0] + (-s1*s2*l1-s1*s23*l2)*F[1] + (c2*l1+c23*l2)*F[2];
    tempTao.tao3 = (-c1*s23*l2)*F[0] + (-s1*s23*l2)*F[1] + (c23*l2)*F[2];
    
    JointState tempState;
    tempState.q1 = nowJointState.q1;//q1_b
    tempState.q2 = nowJointState.q2;//q2_b
    tempState.q3 = nowJointState.q3;//q3_b
    tempState.dq1 = nowJointState.dq1;//qd1_b
    tempState.dq2 = nowJointState.dq2;//qd2_b
    tempState.dq3 = nowJointState.dq3;//qd3_b
    tempState.ddq1 = 0;//v1
    tempState.ddq2 = 0;//v2
    tempState.ddq3 = 0;//v3
    
    estimateT = GetJointTorque(tempState) + tempTao;
   
    return estimateT;
} 

Kinematics kinematics;
Dynamics dynamics;
