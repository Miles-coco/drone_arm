#include "coordinate_mapping.h"
#include <math.h>
#include "Remote_Control.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 默认摇杆通道配置
static StickChannelConfig stick_config =
    {
        .x_channel = 2, // 默认通道2控制X轴
        .y_channel = 1  // 默认通道1控制Y轴
};

// 默认工作空间配置
static WorkspaceConfig workspace_config =
    {
        .min_theta1 = (30.0f / 180.0f) * M_PI,   // 默认电机1最小角度 -180°
        .max_theta1 = 0.0f,                      // 默认电机1最大角度 180°
        .min_theta2 = -(110.0f / 180.0f) * M_PI, // 默认电机2最小角度 -135°
        .max_theta2 = -(60.0f / 180.0f) * M_PI   // 默认电机2最大角度 -45°
};

// 辅助函数：将值从一个范围映射到另一个范围
static float map_value(float input, float in_min, float in_max, float out_min, float out_max)
{
    // 处理除零错误
    if (fabsf(in_max - in_min) < 0.0001f)
    {
        return (out_min + out_max) * 0.5f; // 返回中间值
    }

    // 线性映射
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 用于配置映射到x,y的遥控器通道
void Set_Stick_Channels(uint8_t x_ch, uint8_t y_ch)
{
    stick_config.x_channel = x_ch;
    stick_config.y_channel = y_ch;
}

// 设置关节角度范围
void Set_Joint_Ranges(float min_t1, float max_t1, float min_t2, float max_t2)
{
    workspace_config.min_theta1 = min_t1;
    workspace_config.max_theta1 = max_t1;
    workspace_config.min_theta2 = min_t2;
    workspace_config.max_theta2 = max_t2;
}

// 将遥控器通道值直接映射到关节角度
JointAngles Map_To_JointSpace(void)
{
    // 默认返回值
    JointAngles angles = {
        .theta1 = 0.0f,
        .theta2 = -M_PI, // 默认-90°的安全位置
        .valid = false};

    // 获取摇杆原始值
    int16_t ch_x = get_channel_raw_value(stick_config.x_channel);
    int16_t ch_y = get_channel_raw_value(stick_config.y_channel);

// 确定原始值范围（根据遥控器类型）
#if defined(RC_PROTOCOL_DJI)
    const int16_t RAW_MIN = -660;
    const int16_t RAW_MAX = 660;
#elif defined(RC_PROTOCOL_TDF)
    const int16_t RAW_MIN = -1024;
    const int16_t RAW_MAX = 1024;
#else
    const int16_t RAW_MIN = -1000;
    const int16_t RAW_MAX = 1000;
#endif

    // 应用死区处理（消除摇杆中心的微小偏移）
    const int16_t deadzone = 50; // 原始值死区
    if (abs(ch_x) < deadzone)
        ch_x = 0;
    if (abs(ch_y) < deadzone)
        ch_y = 0;

    // 映射到电机1的角度范围（θ₁）
    angles.theta1 = map_value(
        (float)ch_x,
        (float)RAW_MIN,
        (float)RAW_MAX,
        workspace_config.min_theta1,
        workspace_config.max_theta1);

    // 映射到电机2的角度范围（θ₂）
    angles.theta2 = map_value(
        (float)ch_y,
        (float)RAW_MIN,
        (float)RAW_MAX,
        workspace_config.min_theta2,
        workspace_config.max_theta2);

    // 应用安全边界（确保角度在限位范围内）
    angles.theta1 = fmaxf(fminf(angles.theta1, workspace_config.max_theta1), workspace_config.min_theta1);
    angles.theta2 = fmaxf(fminf(angles.theta2, workspace_config.max_theta2), workspace_config.min_theta2);

    // 验证θ₁ > θ₂约束（使用容差）
    const float angle_tolerance = 0.001f;
    if (angles.theta1 < angles.theta2 - angle_tolerance)
    {
        // 违反约束时调整为安全位置
        angles.theta1 = angles.theta2 - angle_tolerance;
    }

    angles.valid = true;
    return angles;
}