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
    bool valid;//解是否有效
} JointAngles;

typedef struct 
{
    float min_theta1;
    float max_theta1;
    float min_theta2;
    float max_theta2;
} JointLimits; 


//逆运动学求解接口
JointAngles IK_Solve(point2D target,float L1,float L2,const JointLimits* limits);

#endif // 1
