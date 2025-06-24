#ifndef __COORDINATE_MAPPING_H__
#define __COORDINATE_MAPPING_H__

#include "remote_control.h"
#include "kinematic_solver.h"

// 工作空间配置
typedef struct {
    float x_min;     // X轴最小值
    float x_max;     // X轴最大值
    float y_min;     // Y轴最小值
    float y_max;     // Y轴最大值
    float safety_margin; // 安全边距
} WorkspaceConfig;

// 摇杆通道配置
typedef struct {
    uint8_t x_channel; // X轴对应通道号
    uint8_t y_channel; // Y轴对应通道号
} StickChannelConfig;

/**
 * 初始化工作空间配置
 * @param config 工作空间配置结构体指针
 */
void Mapping_Init(WorkspaceConfig* config);

/**
 * 设置摇杆通道映射
 * @param x_ch X轴通道号
 * @param y_ch Y轴通道号
 */
void Set_Stick_Channels(uint8_t x_ch, uint8_t y_ch);

/**
 * 将摇杆通道值映射到笛卡尔坐标系
 * @return 映射后的坐标点
 */
point2D Map_To_Cartesian(void);

#endif // __COORDINATE_MAPPING_H__