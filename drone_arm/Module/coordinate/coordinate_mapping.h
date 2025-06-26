#ifndef __COORDINATE_MAPPING_H__
#define __COORDINATE_MAPPING_H__

#include <stdint.h>
#include <stdbool.h>
#include "remote_control.h"

// 关节角度结构体
typedef struct {
    float theta1;     // 电机1角度 (弧度)
    float theta2;     // 电机2角度 (弧度)
    bool valid;       // 是否有效解
} JointAngles;

// 摇杆通道配置
typedef struct {
    uint8_t x_channel; // X轴对应通道号
    uint8_t y_channel; // Y轴对应通道号
} StickChannelConfig;

// 工作空间配置
typedef struct {
    float min_theta1; // 关节1最小角度 (弧度)
    float max_theta1; // 关节1最大角度 (弧度)
    float min_theta2; // 关节2最小角度 (弧度)
    float max_theta2; // 关节2最大角度 (弧度)
} WorkspaceConfig;

/**
 * 设置摇杆通道映射
 * @param x_ch X轴通道号
 * @param y_ch Y轴通道号
 */
void Set_Stick_Channels(uint8_t x_ch, uint8_t y_ch);

/**
 * 设置关节角度范围
 * @param min_t1 关节1最小角度 (弧度)
 * @param max_t1 关节1最大角度 (弧度)
 * @param min_t2 关节2最小角度 (弧度)
 * @param max_t2 关节2最大角度 (弧度)
 */
void Set_Joint_Ranges(float min_t1, float max_t1, float min_t2, float max_t2);

/**
 * 将摇杆通道值直接映射到关节角度空间
 * @return 映射后的关节角度
 */
JointAngles Map_To_JointSpace(void);

#endif // __COORDINATE_MAPPING_H__