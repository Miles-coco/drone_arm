#include "kinematic_solver.h"
#include <math.h>
#include <stdbool.h>
#include "arm_math.h"

// 定义π常量
#define PI 3.14159265358979323846f
#define PI_2 (PI / 2.0f)     // π/2
#define DEG_TO_RAD(x) ((x) * PI / 180.0f)
#define RAD_TO_DEG(x) ((x) * 180.0f / PI)

// 角度归一化到[0, 2π)
static float normalize_angle(float angle) {
    angle = fmodf(angle, 2.0f * PI);
    if (angle < 0.0f) angle += 2.0f * PI;
    return angle;
}

// 检查角度是否在限位范围内
static bool within_limits(float angle, float min, float max) {
    angle = normalize_angle(angle);
    
    // 处理角度范围跨越0点的情况
    if (min > max) {
        return (angle >= min) || (angle <= max);
    }
    return (angle >= min) && (angle <= max);
}

// 简化的逆运动学求解函数 - 利用θ₁>θ₂约束
int IK_Solve(point2D target, float L1, float L2, const JointLimits* limits, JointAngles* angles) {
    // 检查有效输入
    if (L1 <= 0.001f || L2 <= 0.001f) return 0;
    
    float x = target.x;
    float y = target.y;
    
    // 计算目标到原点的距离（使用CMSIS-DSP优化平方根）
    float32_t R;
    arm_sqrt_f32(x*x + y*y, &R);
    
    // 检查工作空间边界
    float minR = fabsf(L1 - L2);
    float maxR = L1 + L2;
    
    if (R < minR - 0.001f || R > maxR + 0.001f) {
        return 0; // 超出工作空间
    }
    
    // 计算基础方向角度
    float alpha = atan2f(y, x);
    
    // 使用余弦定律计算中间变量（简化公式）
    float cos_beta = (L1 * L1 + R * R - L2 * L2) / (2.0f * L1 * R);
    
    // 数值稳定性处理
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    
    // 计算beta角度
    float beta = acosf(cos_beta);
    
    // 基于θ₁>θ₂约束计算唯一解
    float theta1 = normalize_angle(alpha + beta);
    
    // 使用CMSIS-DSP优化三角函数计算
    float32_t sin_theta1, cos_theta1;
    arm_sin_cos_f32(RAD_TO_DEG(theta1), &sin_theta1, &cos_theta1);
    
    // 根据新方程计算sinθ2和cosθ2
    float sin_theta2 = (L1 * sin_theta1 - x) / L2;
    float cos_theta2 = (L1 * cos_theta1 + y) / L2;
    
    // 计算θ2
    float theta2 = atan2f(sin_theta2, cos_theta2);
    theta2 = normalize_angle(theta2);
    
    // 验证θ₁>θ₂约束
    const float angle_tolerance = 0.001f; // 允许的误差范围
    if (theta1 < theta2 - angle_tolerance) {
        // 如果违反约束，应用修正
        theta1 += 2.0f * PI;
        theta1 = normalize_angle(theta1);
    }
    
    // 检查是否在限位范围内
    bool valid = within_limits(theta1, limits->min_theta1, limits->max_theta1) &&
                 within_limits(theta2, limits->min_theta2, limits->max_theta2);
    
    if (valid) {
        angles->theta1 = theta1;
        angles->theta2 = theta2;
        return 1; // 唯一解
    }
    
    return 0; // 不在限位范围内
}
