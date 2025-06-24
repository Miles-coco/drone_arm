#ifndef __KINEMATIC_SOLVER_H_
#define __KINEMATIC_SOLVER_H_

#include <stdbool.h>
typedef struct 
{
    float x,y;//末端坐标
} point2D;

typedef struct 
{
    float theta1;//电机1角度
    float theta2;//电机2角度
} JointAngles;

typedef struct 
{
    float min_theta1;
    float max_theta1;
    float min_theta2;
    float max_theta2;
} JointLimits; 


//逆运动学求解接口
int IK_Solve(point2D target,float L1,float L2,const JointLimits* limits,JointAngles* angles);
static float normalize_angle(float angle);
static bool within_limits(float angle, float min, float max);

#endif // 1
