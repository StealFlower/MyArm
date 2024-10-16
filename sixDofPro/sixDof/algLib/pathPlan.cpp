#include "pathPlan.h"

/*初末位置平滑曲线*/
float pathPlanCalt(PathPlan *path, float x0, float x1, float t1)
{
    if (path->nowTime == 0)  
    {
        float v0 = 0, v1 = 0, a0 = 0, a1 = 0;
        // 计算五次六项式系数
        path->linearParams[0] = x0;                                                                                                  // 0次项
        path->linearParams[1] = v0;                                                                                                  // 1次项
        path->linearParams[2] = a0 / 2;                                                                                              // 2次项
        path->linearParams[3] = (20 * x1 - 20 * x0 - (8 * v1 + 12 * v0) * t1 - (3 * a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 3));      // 3次项
        path->linearParams[4] = (30 * x0 - 30 * x1 + (14 * v1 + 16 * v0) * t1 + (3 * a0 - 2 * a1) * powf(t1, 2)) / (2 * pow(t1, 4)); // 4次项
        path->linearParams[5] = (12 * x1 - 12 * x0 - (6 * v1 + 6 * v0) * t1 - (a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 5));           // 5次项
    }
    path->nowTime += path->cycle.getCycleT(); // 获取时间
    float x = 0;

    // 六项式加和计算
    if (path->nowTime <= t1)
    {
        for (u8 j = 0; j < 6; j++)
            x += path->linearParams[j] * powf(path->nowTime, j);
    }
    else
    {
        x = x1;
    }
    return x;
}

JointState pathPlanCalt(PathPlan *path, EndPoint pos0, EndPoint pos1, float t1)
{
    if (path->nowTime == 0)
    {
        float v0 = 0, v1 = 0, a0 = 0, a1 = 0;
        // 计算五次六项式系数
        path->matrixParams[0][0] = pos0.xPos;                                                                                                  // 0次项
        path->matrixParams[0][1] = v0;                                                                                                  // 1次项
        path->matrixParams[0][2] = a0 / 2;                                                                                              // 2次项
        path->matrixParams[0][3] = (20 * pos1.xPos - 20 * pos0.xPos - (8 * v1 + 12 * v0) * t1 - (3 * a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 3));      // 3次项
        path->matrixParams[0][4] = (30 * pos0.xPos - 30 * pos1.xPos + (14 * v1 + 16 * v0) * t1 + (3 * a0 - 2 * a1) * powf(t1, 2)) / (2 * pow(t1, 4)); // 4次项
        path->matrixParams[0][5] = (12 * pos1.xPos - 12 * pos0.xPos - (6 * v1 + 6 * v0) * t1 - (a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 5));           // 5次项
        path->matrixParams[1][0] = pos0.yPos;                                                                                                  // 0次项
        path->matrixParams[1][1] = v0;                                                                                                  // 1次项
        path->matrixParams[1][2] = a0 / 2;                                                                                              // 2次项
        path->matrixParams[1][3] = (20 * pos1.yPos - 20 * pos0.yPos - (8 * v1 + 12 * v0) * t1 - (3 * a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 3));      // 3次项
        path->matrixParams[1][4] = (30 * pos0.yPos - 30 * pos1.yPos + (14 * v1 + 16 * v0) * t1 + (3 * a0 - 2 * a1) * powf(t1, 2)) / (2 * pow(t1, 4)); // 4次项
        path->matrixParams[1][5] = (12 * pos1.yPos - 12 * pos0.yPos - (6 * v1 + 6 * v0) * t1 - (a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 5));           // 5次项
        path->matrixParams[2][0] = pos0.zPos;                                                                                                  // 0次项
        path->matrixParams[2][1] = v0;                                                                                                  // 1次项
        path->matrixParams[2][2] = a0 / 2;                                                                                              // 2次项
        path->matrixParams[2][3] = (20 * pos1.zPos - 20 * pos0.zPos - (8 * v1 + 12 * v0) * t1 - (3 * a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 3));      // 3次项
        path->matrixParams[2][4] = (30 * pos0.zPos - 30 * pos1.zPos + (14 * v1 + 16 * v0) * t1 + (3 * a0 - 2 * a1) * powf(t1, 2)) / (2 * pow(t1, 4)); // 4次项
        path->matrixParams[2][5] = (12 * pos1.zPos - 12 * pos0.zPos - (6 * v1 + 6 * v0) * t1 - (a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 5));           // 5次项        
    }
    path->nowTime += path->cycle.getCycleT(); // 获取时间
 
    JointState tempJointState;

    // 六项式加和计算
    if (path->nowTime <= t1)
    {
        for (u8 j = 0; j < 6; j++)
            tempJointState.q1 += path->matrixParams[0][j] * powf(path->nowTime, j);
        for (u8 j = 0; j < 6; j++)
            tempJointState.q2 += path->matrixParams[1][j] * powf(path->nowTime, j);
        for (u8 j = 0; j < 6; j++)
            tempJointState.q3 += path->matrixParams[2][j] * powf(path->nowTime, j);   
        
        for (u8 j = 1; j < 6; j++)
            tempJointState.dq1 += j*path->matrixParams[0][j] * powf(path->nowTime, j-1);
        for (u8 j = 1; j < 6; j++)
            tempJointState.dq2 += j*path->matrixParams[1][j] * powf(path->nowTime, j-1);
        for (u8 j = 1; j < 6; j++)
            tempJointState.dq3 += j*path->matrixParams[2][j] * powf(path->nowTime, j-1);

        for (u8 j = 2; j < 6; j++)
            tempJointState.ddq1 += (j-1)*j*path->matrixParams[0][j] * powf(path->nowTime, j-2);
        for (u8 j = 2; j < 6; j++)
            tempJointState.ddq2 += (j-1)*j*path->matrixParams[1][j] * powf(path->nowTime, j-2);
        for (u8 j = 2; j < 6; j++)
            tempJointState.ddq3 += (j-1)*j*path->matrixParams[2][j] * powf(path->nowTime, j-2);        
    }
    else
    {
        tempJointState.q1 = pos1.xPos;
        tempJointState.q2 = pos1.yPos;
        tempJointState.q3 = pos1.zPos;
        tempJointState.dq1 = 0;
        tempJointState.dq2 = 0;
        tempJointState.dq3 = 0;        
        tempJointState.ddq1 = 0;
        tempJointState.ddq2 = 0;
        tempJointState.ddq3 = 0;        
    }
    
    return tempJointState;
}


JointState pathPlanCalt(PathPlan *path, EndPoint pos0, EndPoint pos1,float velocity, float t1)
{
    if (path->nowTime == 0)
    {
        float v0 = 0, v1 = 0, a0 = 0, a1 = 0;
        // 计算五次六项式系数
        path->matrixParams[0][0] = pos0.xPos;                                                                                                  // 0次项
        path->matrixParams[0][1] = v0;                                                                                                  // 1次项
        path->matrixParams[0][2] = a0 / 2;                                                                                              // 2次项
        path->matrixParams[0][3] = (20 * pos1.xPos - 20 * pos0.xPos - (8 * velocity + 12 * v0) * t1 - (3 * a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 3));      // 3次项
        path->matrixParams[0][4] = (30 * pos0.xPos - 30 * pos1.xPos + (14 * velocity + 16 * v0) * t1 + (3 * a0 - 2 * a1) * powf(t1, 2)) / (2 * pow(t1, 4)); // 4次项
        path->matrixParams[0][5] = (12 * pos1.xPos - 12 * pos0.xPos - (6 * velocity + 6 * v0) * t1 - (a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 5));           // 5次项
        path->matrixParams[1][0] = pos0.yPos;                                                                                                  // 0次项
        path->matrixParams[1][1] = v0;                                                                                                  // 1次项
        path->matrixParams[1][2] = a0 / 2;                                                                                              // 2次项
        path->matrixParams[1][3] = (20 * pos1.yPos - 20 * pos0.yPos - (8 * v1 + 12 * v0) * t1 - (3 * a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 3));      // 3次项
        path->matrixParams[1][4] = (30 * pos0.yPos - 30 * pos1.yPos + (14 * v1 + 16 * v0) * t1 + (3 * a0 - 2 * a1) * powf(t1, 2)) / (2 * pow(t1, 4)); // 4次项
        path->matrixParams[1][5] = (12 * pos1.yPos - 12 * pos0.yPos - (6 * v1 + 6 * v0) * t1 - (a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 5));           // 5次项
        path->matrixParams[2][0] = pos0.zPos;                                                                                                  // 0次项
        path->matrixParams[2][1] = v0;                                                                                                  // 1次项
        path->matrixParams[2][2] = a0 / 2;                                                                                              // 2次项
        path->matrixParams[2][3] = (20 * pos1.zPos - 20 * pos0.zPos - (8 * v1 + 12 * v0) * t1 - (3 * a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 3));      // 3次项
        path->matrixParams[2][4] = (30 * pos0.zPos - 30 * pos1.zPos + (14 * v1 + 16 * v0) * t1 + (3 * a0 - 2 * a1) * powf(t1, 2)) / (2 * pow(t1, 4)); // 4次项
        path->matrixParams[2][5] = (12 * pos1.zPos - 12 * pos0.zPos - (6 * v1 + 6 * v0) * t1 - (a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 5));           // 5次项        
    }
    path->nowTime += path->cycle.getCycleT(); // 获取时间
 
    JointState tempJointState;

    float lentgh = sqrt(powf((pos1.xPos - pos0.xPos),2) + powf((pos1.yPos - pos0.yPos),2) + powf((pos1.zPos - pos0.zPos),2) );
    float x_k = (pos1.xPos - pos0.xPos)/lentgh ,y_k = (pos1.yPos - pos0.yPos)/lentgh,z_k = (pos1.zPos - pos0.zPos)/lentgh;
    // 六项式加和计算
    if (path->nowTime <= t1)
    {
        for (u8 j = 0; j < 6; j++)
            tempJointState.q1 += path->matrixParams[0][j] * powf(path->nowTime, j);  
        for (u8 j = 0; j < 6; j++)
            tempJointState.q2 += path->matrixParams[1][j] * powf(path->nowTime, j);
        for (u8 j = 0; j < 6; j++)
            tempJointState.q3 += path->matrixParams[2][j] * powf(path->nowTime, j);           
        for (u8 j = 1; j < 6; j++)
            tempJointState.dq1 += j*path->matrixParams[0][j] * powf(path->nowTime, j-1);
        for (u8 j = 1; j < 6; j++)
            tempJointState.dq2 += j*path->matrixParams[1][j] * powf(path->nowTime, j-1);
        for (u8 j = 1; j < 6; j++)
            tempJointState.dq3 += j*path->matrixParams[2][j] * powf(path->nowTime, j-1);

        for (u8 j = 2; j < 6; j++)
            tempJointState.ddq1 += (j-1)*j*path->matrixParams[0][j] * powf(path->nowTime, j-2);
        for (u8 j = 2; j < 6; j++)
            tempJointState.ddq2 += (j-1)*j*path->matrixParams[1][j] * powf(path->nowTime, j-2);
        for (u8 j = 2; j < 6; j++)
            tempJointState.ddq3 += (j-1)*j*path->matrixParams[2][j] * powf(path->nowTime, j-2);        
    }
    else
    {
        tempJointState.q1  = pos1.xPos + x_k*velocity*(path->nowTime-t1);
        tempJointState.q2  = pos1.yPos + y_k*velocity*(path->nowTime-t1);
        tempJointState.q3  = pos1.zPos + z_k*velocity*(path->nowTime-t1);
        tempJointState.dq1 = x_k*velocity; 
        tempJointState.dq2 = y_k*velocity; 
        tempJointState.dq3 = z_k*velocity; 
        tempJointState.ddq1 = 0;
        tempJointState.ddq2 = 0;
        tempJointState.ddq3 = 0;        
    }
    
    return tempJointState;
}

void clearTime(PathPlan *path1, PathPlan *path2, PathPlan *path3, PathPlan *path4,
    PathPlan *path5, PathPlan *path6, PathPlan *path7, PathPlan *path8,PathPlan *path9)
{
    memset(path1, 0, sizeof(*path1));
    if (path2 != 0)
        memset(path2, 0, sizeof(*path2));
    if (path3 != 0)
        memset(path3, 0, sizeof(*path3));
    if (path4 != 0)
        memset(path4, 0, sizeof(*path4));
    if (path5 != 0)
        memset(path5, 0, sizeof(*path5));
    if (path6 != 0)
        memset(path6, 0, sizeof(*path6));
    if (path7 != 0)
        memset(path7, 0, sizeof(*path7));
    if (path8 != 0)
        memset(path8, 0, sizeof(*path8));
    if (path9 != 0)
        memset(path9, 0, sizeof(*path9));
}

