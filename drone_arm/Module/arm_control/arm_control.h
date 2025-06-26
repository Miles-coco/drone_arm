#ifndef __ARM_CONTROL_H_
#define __ARM_CONTROL_H_

#include <stdbool.h>
#include "coordinate_mapping.h"  // 包含坐标映射头文件

// 初始化函数
void ArmControl_Init(void);

// 更新机械臂控制
void ArmControl_Update(void);

#endif // __ARM_CONTROL_H_