#include "coordinate_mapping.h"
#include "math.h"
#include "Remote_Control.h"

// 默认工作空间配置
static WorkspaceConfig workspace_config = {
    .x_min = -0.3f,
    .x_max = 0.3f,
    .y_min = 0.1f,
    .y_max = 0.5f,
    .safety_margin = 0.02f
};

// 默认摇杆通道配置
static StickChannelConfig stick_config = {
    .x_channel = 0, // 默认通道0控制X轴
    .y_channel = 1  // 默认通道1控制Y轴
};

// 映射曲线平滑函数
static float smooth_curve(float value) {
    // 使用Sigmoid函数实现平滑过渡
    const float k = 3.0f; // 调整曲线陡峭程度
    return (2.0f / (1.0f + expf(-k * value))) - 1.0f;
}

void Mapping_Init(WorkspaceConfig* config) {
    if (config != NULL) {
        workspace_config = *config;
    }
}

void Set_Stick_Channels(uint8_t x_ch, uint8_t y_ch) {
    stick_config.x_channel = x_ch;
    stick_config.y_channel = y_ch;
}

point2D Map_To_Cartesian(void) {
    point2D result = {0, 0};
    
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
    
    // 归一化到[-1, 1]范围
    float norm_x = (float)(ch_x - RAW_MIN) / (RAW_MAX - RAW_MIN) * 2.0f - 1.0f;
    float norm_y = (float)(ch_y - RAW_MIN) / (RAW_MAX - RAW_MIN) * 2.0f - 1.0f;
    
    // 应用死区处理（避免摇杆微动）
    const float deadzone = 0.05f; // 5%死区
    if (fabsf(norm_x) < deadzone) norm_x = 0.0f;
    if (fabsf(norm_y) < deadzone) norm_y = 0.0f;
    
    // 应用平滑曲线（可选）
    // norm_x = smooth_curve(norm_x);
    // norm_y = smooth_curve(norm_y);
    
    // 计算范围比例
    float range_x = workspace_config.x_max - workspace_config.x_min;
    float range_y = workspace_config.y_max - workspace_config.y_min;
    
    // 映射到实际工作空间
    result.x = workspace_config.x_min + (norm_x + 1.0f) * 0.5f * range_x;
    result.y = workspace_config.y_min + (norm_y + 1.0f) * 0.5f * range_y;
    
    // 应用安全边界（确保不超出工作空间）
    result.x = fmaxf(fminf(result.x, workspace_config.x_max - workspace_config.safety_margin), 
                     workspace_config.x_min + workspace_config.safety_margin);
    result.y = fmaxf(fminf(result.y, workspace_config.y_max - workspace_config.safety_margin), 
                     workspace_config.y_min + workspace_config.safety_margin);
    
    return result;
}