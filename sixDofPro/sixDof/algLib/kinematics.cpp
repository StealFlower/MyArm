#include "kinematics.h"
#include "SMUniversal.h"
#include "arm.h"

Kinematics::Kinematics()
{
}
//��е�����˶�ѧ--������
//����-- nowJoint_rps �����Ӧ�ؽڽ� ��λrps ����Ĺؽڽ��ǽ����Ĺؽڽ�,�͵���������ת˳����޹�ϵ
//���-- tempPoint ��Ӧ��ĩ�˵�
EndPoint Kinematics::GetEndPoint(JointAngle joint_rps)
{
    EndPoint tempPoint;  
    //x,y,z����
    float c1,s1,c2,s2,c23,s23;
    
    c1 = cosf(joint_rps.theta1);
    s1 = sinf(joint_rps.theta1);
    c2 = cosf(joint_rps.theta2);
    s2 = sinf(joint_rps.theta2);    
    c23 = cosf(joint_rps.theta2 + joint_rps.theta3);
    s23 = sinf(joint_rps.theta2 + joint_rps.theta3);
    
    tempPoint.xPos = c1*(c2*mech.l1 + c23*mech.l2);
    tempPoint.yPos = s1*(c2*mech.l1 + c23*mech.l2);
    tempPoint.zPos = mech.l1*s2 + mech.l2*s23;
    
    
    float c4,s4,c5,s5,c6,s6;
    
    c4 = cosf(joint_rps.theta4);
    s4 = sinf(joint_rps.theta4);
    c5 = cosf(joint_rps.theta5);
    s5 = sinf(joint_rps.theta5);
    c6 = cosf(joint_rps.theta6);
    s6 = sinf(joint_rps.theta6);    

    float rmat[3][3]={
        {c1*c5*c23 - s5*(s1*s4 + c1*c4*s23), s6*(c5*(s1*s4 + c1*c4*s23) + c1*c23*s5) - c6*(c4*s1 - c1*s4*s23),s6*(c4*s1 - c1*s4*s23) + c6*(c5*(s1*s4 + c1*c4*s23) + c1*c23*s5)},
        {s5*(c1*s4 - c4*s1*s23) + c5*c23*s1, c6*(c1*c4 + s1*s4*s23) - s6*(c5*(c1*s4 - c4*s1*s23) - c23*s1*s5),- s6*(c1*c4 + s1*s4*s23) - c6*(c5*(c1*s4 - c4*s1*s23) - c23*s1*s5)},
        {            - c5*s23 - c4*c23*s5  ,                              c6*c23*s4 - s6*(s5*s23 - c4*c5*c23),   - c6*(s5*s23 - c4*c5*c23) - c23*s4*s6                          }
    };
    
    mat2qua2euler(rmat,euler);

    tempPoint.yaw = euler[yaw] * 0.017452007f;           
    tempPoint.pitch = euler[pitch] * 0.017452007f;
    tempPoint.roll = euler[roll] * 0.017452007f;
    
    return tempPoint;  
}

//��е�����
//����Ŀ��� tarEndPoint --xyz ��λm λ�˵���ת˳�� yzx elbm -- ��� -- ���Ĭ��������
//��� setAngle_rps -- �����Ķ�Ӧ�Ľ����Ƕ� ��λ rps
JointAngle Kinematics::GetJointAngle(EndPoint tarEndPoint,u8 elbm)
{    
    JointAngle setAngle_rps;                       // �ؽڲ���    
    //theta1�����
    //Ψһ��    
    if(fabs(tarEndPoint.xPos)<0.0001f && tarEndPoint.yPos>0)
        setAngle_rps.theta1 = PI/2;
    else if(fabs(tarEndPoint.xPos)<0.0001f && tarEndPoint.yPos<0)
        setAngle_rps.theta1 = -PI/2;
    else
        setAngle_rps.theta1 = atan2f(tarEndPoint.yPos,tarEndPoint.xPos);
    
    //theta3�����
    //theta3[-150,150]--�����Ƿ��н�
    if(fabs((powf(tarEndPoint.xPos,2)+powf(tarEndPoint.yPos,2)+powf(tarEndPoint.zPos,2)-powf(mech.l1,2)-powf(mech.l2,2))/(2*mech.l1*mech.l2))>0.999f)
        setAngle_rps.theta3 = Pitch2.getPosition()*THETA3_MECH_TO_RPS_RATIO;
    else{
        if(elbm)
            setAngle_rps.theta3 = -acosf( (powf(tarEndPoint.xPos,2)+powf(tarEndPoint.yPos,2)+powf(tarEndPoint.zPos,2)-powf(mech.l1,2)-powf(mech.l2,2))/(2*mech.l1*mech.l2));
        else
            setAngle_rps.theta3 = acosf( (powf(tarEndPoint.xPos,2)+powf(tarEndPoint.yPos,2)+powf(tarEndPoint.zPos,2)-powf(mech.l1,2)-powf(mech.l2,2))/(2*mech.l1*mech.l2));
    }
    
    //theta2�����
    //theta2[0,180]--Ψһ��
    float c3 = cosf(setAngle_rps.theta3),s3 = sinf(setAngle_rps.theta3);
    float x_c;//x_c = tarEndPoint.xpos/cos(theta1) = =tarEndPoint.ypos/sin(theta1)
    if(fabs(cosf(setAngle_rps.theta1))<0.1f)
        x_c = tarEndPoint.yPos/sinf(setAngle_rps.theta1);
    else
        x_c = tarEndPoint.xPos/cosf(setAngle_rps.theta1);
    
    if(fabs((x_c*(mech.l1+c3*mech.l2)+tarEndPoint.zPos*s3*mech.l2)/(powf(mech.l1,2)+powf(mech.l2,2)+2*c3*mech.l1*mech.l2))>0.999f)
         setAngle_rps.theta2 = (Pitch1.getPosition()-90)*THETA2_MECH_TO_RPS_RATIO;
    else
        setAngle_rps.theta2 = acosf( (x_c*(mech.l1+c3*mech.l2)+tarEndPoint.zPos*s3*mech.l2)/(powf(mech.l1,2)+powf(mech.l2,2)+2*c3*mech.l1*mech.l2));
        
    float c1 = cosf(setAngle_rps.theta1),s1 = sinf(setAngle_rps.theta1),c23 = cosf(setAngle_rps.theta2+setAngle_rps.theta3),s23 = sinf(setAngle_rps.theta2+setAngle_rps.theta3),cz = cosf(tarEndPoint.yaw),sz = sinf(tarEndPoint.yaw),cy = cosf(tarEndPoint.pitch),sy = sinf(tarEndPoint.pitch),cx = cosf(tarEndPoint.roll),sx = sinf(tarEndPoint.roll);
    float rmat[3][3] = {
        {s23*sy + c1*c23*cy*cz + c23*cy*s1*sz, c23*s1*(cx*cz + sx*sy*sz) - c1*c23*(cx*sz - cz*sx*sy) - cy*s23*sx, c1*c23*(sx*sz + cx*cz*sy) - cx*cy*s23 - c23*s1*(cz*sx - cx*sy*sz)},
        {c1*cy*sz - cy*cz*s1,                  c1*(cx*cz + sx*sy*sz) + s1*(cx*sz - cz*sx*sy),                      - c1*(cz*sx - cx*sy*sz) - s1*(sx*sz + cx*cz*sy)},
        {c1*cy*cz*s23 - c23*sy + cy*s1*s23*sz, s1*s23*(cx*cz + sx*sy*sz) - c1*s23*(cx*sz - cz*sx*sy) + c23*cy*sx, c1*s23*(sx*sz + cx*cz*sy) - s1*s23*(cz*sx - cx*sy*sz) + c23*cx*cy}     
    };
    //��������
    float solve_theta4,solve_theta5,solve_theta6;
    //����theta5 = 0
    if(fabs(rmat[0][0])>0.999f){
        if(wrist.getPitchAngle()>= -60 && wrist.getPitchAngle()<= 60)
            setAngle_rps.theta5 = 0;
        if(wrist.getPitchAngle()<-60)
            setAngle_rps.theta5 = -PI/2;
        if(wrist.getPitchAngle()>60)
            setAngle_rps.theta5 = PI/2;
        setAngle_rps.theta4 = (Roll1.getPosition()-180)*THETA4_MECH_TO_RPS_RATIO;
        setAngle_rps.theta6 = wrist.getRollAngle()*THETA6_MECH_TO_RPS_RATIO;
    }
    else{    
        setAngle_rps.theta5 = acosf(rmat[0][0]) ;
        //����theta4 = +-90
        if(fabs(rmat[2][0])<0.01f){
            if(rmat[1][0]*setAngle_rps.theta5>0)
                setAngle_rps.theta4 = PI/2;
            else
                setAngle_rps.theta4 = -PI/2;
        }
        else
            setAngle_rps.theta4 = atan2f(rmat[1][0],-rmat[2][0]);

        //����theta5 = +-90
        if(fabs(rmat[0][2])<0.01f){
            if(rmat[0][1]*setAngle_rps.theta5>0)
                setAngle_rps.theta6 = PI/2;
            else
                setAngle_rps.theta6 = -PI/2;
        }
        else
            setAngle_rps.theta6 = atan2f(rmat[0][1],rmat[0][2]);         
    }        
    return setAngle_rps;
}

//��Ԫ��ת����ŷ����--������ת����m[3][3],
//���ŷ���� ---��Ӧ��ת˳�� --pitch roll yaw 
void Kinematics::mat2qua2euler(float(*m)[3],float(*euler))
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

    euler[pitch] = asinf(-2 * q2 * q4 + 2 * q1* q3)* RAD_TO_DEGREE;
    euler[roll] = atan2f(2 * q3 * q4 + 2 * q1 * q2, -2 * powf(q2,2) - 2 * powf(q3,2) + 1)* RAD_TO_DEGREE;
    euler[yaw] = atan2f(2 * (q2 * q3 + q1*q4),powf(q1,2)+powf(q2,2)-powf(q3,2)-powf(q4,2)) * RAD_TO_DEGREE;
}

Kinematics kinematics;
