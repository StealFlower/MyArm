#ifndef __PATHPLAN_H
#define __PATHPLAN_H
#include "board.h"
#include "cycle.h"

// 五次六项式曲线轨迹规划
typedef struct 
{
    float nowTime;
    float param[6];
    Cycle cycle;
}PathPlan;

float pathPlanCalt(PathPlan *path, float x0, float x1, float t1);
void clearTime(PathPlan *path1, PathPlan *path2 = 0, PathPlan *path3 = 0, PathPlan *path4 = 0, 
    PathPlan *path5 = 0,PathPlan *path6 = 0, PathPlan *path7 = 0, PathPlan *path8 = 0,PathPlan *path9 = 0);

#endif