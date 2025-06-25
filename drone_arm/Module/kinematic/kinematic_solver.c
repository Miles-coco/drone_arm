#include "kinematic_solver.h"
#include <math.h>
#include <stdbool.h>

// 确保定义π常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define PI M_PI

// 定义π常量
#define DEG_TO_RAD(x) ((x) * PI / 180.0f)
#define RAD_TO_DEG(x) ((x) * 180.0f / PI)

// 逆运动学求解函数
JointAngles IK_Solve(point2D target, float L1, float L2, const JointLimits* limits) 
{
    JointAngles result = {.theta1 = 0.0f, .theta2 = 0.0f, .valid = false};
    
    // 检查有效输入
    if (L1 <= 0.001f || L2 <= 0.001f) {
        return result; // 无效连杆长度
    }
    
    float x = target.x;
    float y = target.y;
    
    // 计算目标到原点的距离
    float R;
    R = sqrtf(x*x + y*y);
    
    // 检查工作空间边界
    float minR = fabsf(L1 - L2);
    float maxR = L1 + L2;
    
    if (R < minR - 0.001f || R > maxR + 0.001f) {
        return result; // 超出工作空间
    }
    
    // 计算基础方向角度
    float alpha = atan2f(y, x);
    
    // 使用余弦定律计算中间变量
    float cos_beta = (L1 * L1 + R * R - L2 * L2) / (2.0f * L1 * R);
    
    // 数值稳定性处理
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    
    // 计算beta角度
    float beta = acosf(cos_beta);
    
    // 计算θ₁
    float theta1 = alpha + beta;
    
    // 使用CMSIS-DSP优化三角函数计算
    float sin_theta1 = sinf(theta1);
    float cos_theta1 = cosf(theta1);
    
    // 根据新方程计算sinθ2和cosθ2
    float sin_theta2 = (L1 * sin_theta1 - x) / L2;
    float cos_theta2 = (L1 * cos_theta1 + y) / L2;
    
    // 计算θ2
    float theta2 = atan2f(sin_theta2, cos_theta2);
    
    // 验证θ₁>θ₂约束（使用容差）
    const float angle_tolerance = 0.001f;
    if (theta1 < theta2 - angle_tolerance) {
        return result; // 违反约束
    }
    
    // 检查是否在限位范围内
    bool valid = (theta1 >= limits->min_theta1 && theta1 <= limits->max_theta1) &&
                 (theta2 >= limits->min_theta2 && theta2 <= limits->max_theta2);
    
    if (valid) {
        result.theta1 = theta1;
        result.theta2 = theta2;
        result.valid = true;
    }
    
    return result;
}