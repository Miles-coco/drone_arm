#include "kinematic_solver.h"
#include <math.h>
#include <stdbool.h>
#include "arm_math.h"

// 定义π常量
#define PI_2 (PI / 2.0f)     // π/2
#define PI_4 (PI / 4.0f)     // π/4
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

// 逆运动学求解函数 - 使用CMSIS-DSP优化
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
    
    // 计算目标方向角度
    float beta = atan2f(y, x); // 使用标准atan2保证精度
    
    // 使用余弦定律计算中间角度
    float L1_sq = L1 * L1;
    float R_sq = R * R;
    float L2_sq = L2 * L2;
    float cos_alpha = (L1_sq + R_sq - L2_sq) / (2.0f * L1 * R);
    
    // 数值稳定性处理
    if (cos_alpha > 1.0f) cos_alpha = 1.0f;
    if (cos_alpha < -1.0f) cos_alpha = -1.0f;
    
    // 计算alpha角度（使用标准函数保证精度）
    float alpha = acosf(cos_alpha);
    
    // 计算两种可能的配置
    float theta1_options[2] = {
        normalize_angle(beta + alpha),
        normalize_angle(beta - alpha)
    };
    
    float theta2_options[2];
    int valid_solution_count = 0;
    float best_difference = 1e6f; // 初始化为一个大的值
    JointAngles best_solution;
    
    // 计算两种可能的theta2
    for (int i = 0; i < 2; i++) {
        // 使用CMSIS-DSP优化三角函数计算
        float32_t sin_theta1, cos_theta1;
        arm_sin_cos_f32(RAD_TO_DEG(theta1_options[i]), &sin_theta1, &cos_theta1);
        
        // 计算从末端到第二个主动臂的向量
        float x2 = x - L1 * cos_theta1;
        float y2 = y - L1 * sin_theta1;
        
        // 计算theta2
        theta2_options[i] = atan2f(y2, x2); // 使用标准函数保证精度
        
        // 归一化角度
        theta1_options[i] = normalize_angle(theta1_options[i]);
        theta2_options[i] = normalize_angle(theta2_options[i]);
        
        // 检查是否在限位范围内
        bool valid_theta1 = within_limits(theta1_options[i], limits->min_theta1, limits->max_theta1);
        bool valid_theta2 = within_limits(theta2_options[i], limits->min_theta2, limits->max_theta2);
        
        if (valid_theta1 && valid_theta2) {
            // 计算与当前角度的差异
            float difference = fabsf(theta1_options[i] - angles->theta1) + 
                              fabsf(theta2_options[i] - angles->theta2);
            
            // 如果这是更优的解，或者第一个有效解
            if (difference < best_difference) {
                best_difference = difference;
                best_solution.theta1 = theta1_options[i];
                best_solution.theta2 = theta2_options[i];
            }
            valid_solution_count++;
        }
    }
    
    // 如果有有效解，更新输出
    if (valid_solution_count > 0) {
        angles->theta1 = best_solution.theta1;
        angles->theta2 = best_solution.theta2;
    }
    
    return valid_solution_count;
}