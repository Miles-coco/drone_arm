#ifndef __ARM_CONTROL_H_
#define __ARM_CONTROL_H_

#include <stdbool.h>
#include "kinematic_solver.h"
#include "coordinate_mapping.h"  // 包含坐标映射头文件

// 机械臂状态枚举
typedef enum {
    ARM_IDLE,    // 空闲状态
    ARM_MOVING,  // 运动中
    ARM_ERROR    // 错误状态
} ArmState;

// 机械臂配置结构
typedef struct {
    float L1;            // 第一臂长度
    float L2;            // 第二臂长度
    JointLimits limits;  // 关节限位
} ArmConfig;

// 初始化函数
void ArmControl_Init(void);

// 设置目标位置
void Set_Target_Position(point2D target);

// 更新机械臂控制
void ArmControl_Update(void);

// 获取当前机械臂状态
ArmState Get_Arm_State(void);

// 获取当前末端位置
point2D Get_Current_Position(void);

#endif // __ARM_CONTROL_H_