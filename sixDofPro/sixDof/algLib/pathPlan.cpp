#include "pathPlan.h"

/*初末位置平滑曲线*/
float pathPlanCalt(PathPlan *path, float x0, float x1, float t1)
{
    if (path->nowTime == 0)
    {
        float v0 = 0, v1 = 0, a0 = 0, a1 = 0;
        // 计算五次六项式系数
        path->param[0] = x0;                                                                                                  // 0次项
        path->param[1] = v0;                                                                                                  // 1次项
        path->param[2] = a0 / 2;                                                                                              // 2次项
        path->param[3] = (20 * x1 - 20 * x0 - (8 * v1 + 12 * v0) * t1 - (3 * a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 3));      // 3次项
        path->param[4] = (30 * x0 - 30 * x1 + (14 * v1 + 16 * v0) * t1 + (3 * a0 - 2 * a1) * powf(t1, 2)) / (2 * pow(t1, 4)); // 4次项
        path->param[5] = (12 * x1 - 12 * x0 - (6 * v1 + 6 * v0) * t1 - (a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 5));           // 5次项
    }
    path->nowTime += path->cycle.getCycleT(); // 获取时间
    float x = 0;

    // 六项式加和计算
    if (path->nowTime <= t1)
    {
        for (u8 j = 0; j < 6; j++)
            x += path->param[j] * powf(path->nowTime, j);
    }
    else
    {
        x = x1;
    }
    return x;
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

