#ifndef __ARM_CONTROL_H__
#define __ARM_CONTROL_H__

#include "coordinate_mapping.h"
#include "kinematic_solver.h"
#include "dm_ctrl.h"

// 定义数学常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0f)
#endif

// 定义 ArmConfig 结构体
typedef struct {
    float L1;            // 主动臂1长度
    float L2;            // 主动臂2长度
    JointLimits limits;  // 关节限位
} ArmConfig;

// 机械臂控制状态
typedef enum {
    ARM_IDLE,        // 空闲状态
    ARM_MOVING,      // 运动中
    ARM_ERROR        // 错误状态
} ArmState;

// 初始化机械臂控制
void ArmControl_Init(void);

// 更新机械臂控制（应在控制循环中调用）
void ArmControl_Update(void);

// 获取当前机械臂状态
ArmState Get_Arm_State(void);

// 设置目标位置（外部接口）
void Set_Target_Position(point2D target);

#endif // __ARM_CONTROL_H__