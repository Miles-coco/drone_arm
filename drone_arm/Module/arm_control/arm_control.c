#include "arm_control.h"
#include "main.h"
#include "math.h"
#include "kinematic_solver.h"
#include "remote_control.h"  // 包含remote_control_active声明
#include "dm_ctrl.h"
#include "coordinate_mapping.h"  // 包含坐标映射头文件

// 定义π常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 角度转换宏
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0f)

// 机械臂参数配置
static const ArmConfig arm_config = {
    .L1 = 0.17f,
    .L2 = 0.231f,
    .limits = {
        .min_theta1 = -110.0f/180.0f*M_PI,
        .max_theta1 = -20.0f/180.0f*M_PI,
        .min_theta2 = -130.0f/180.0f*M_PI,
        .max_theta2 = -30.0f/180.0f*M_PI
    }
};

// 声明外部工作空间配置（在coordinate_mapping.c中定义）
extern WorkspaceConfig workspace_config;

// 当前状态
static ArmState arm_state = ARM_IDLE;
static point2D current_target = {0, 0.2f}; // 默认位置
static JointAngles current_angles = {.theta1 = M_PI/4, .theta2 = M_PI/2, .valid = true};
static JointAngles target_angles = {.theta1 = M_PI/4, .theta2 = M_PI/2, .valid = true};

// 初始化函数
void ArmControl_Init(void) {
    // 初始化工作空间映射（使用默认配置）（在coordinate_mapping.c中定义）
    Mapping_Init(NULL);
    
    // 设置摇杆通道（使用默认配置）
    Set_Stick_Channels(0, 1);
    
    // 初始化电机
    dm4310_motor_init();
    
    // 启用电机控制
    ctrl_enable(ALL_MOTORS);
    
    // 设置初始状态
    arm_state = ARM_IDLE;
}

// 设置目标位置
void Set_Target_Position(point2D target) 
{
    // 使用坐标映射模块验证目标点
    point2D safe_target = target;
    
    // 应用安全边界
    safe_target.x = fmaxf(fminf(safe_target.x, 
                                workspace_config.x_max - workspace_config.safety_margin), 
                          workspace_config.x_min + workspace_config.safety_margin);
    
    safe_target.y = fmaxf(fminf(safe_target.y, 
                                workspace_config.y_max - workspace_config.safety_margin), 
                          workspace_config.y_min + workspace_config.safety_margin);
    
    // 更新目标点
    current_target = safe_target;
    arm_state = ARM_MOVING;
}

// 平滑角度过渡
static void Smooth_Angle_Transition(void) 
{
    const float MAX_ANGLE_STEP = DEG_TO_RAD(5.0f);
    
    // 检查θ1步进
    float delta1 = target_angles.theta1 - current_angles.theta1;
    if (fabsf(delta1) > MAX_ANGLE_STEP) 
    {
        current_angles.theta1 += copysignf(MAX_ANGLE_STEP, delta1);
    } 
    else 
    {
        current_angles.theta1 = target_angles.theta1;
    }
    
    // 检查θ2步进
    float delta2 = target_angles.theta2 - current_angles.theta2;
    if (fabsf(delta2) > MAX_ANGLE_STEP) 
    {
        current_angles.theta2 += copysignf(MAX_ANGLE_STEP, delta2);
    } 
    else 
    {
        current_angles.theta2 = target_angles.theta2;
    }
    
    // 检查是否到达目标
    if (fabsf(delta1) < 0.001f && fabsf(delta2) < 0.001f) 
    {
        arm_state = ARM_IDLE;
    }
}

// 更新机械臂控制
void ArmControl_Update(void) 
{
    // 空闲状态等待新目标
    if (arm_state == ARM_IDLE) 
    {
        // 检查是否有遥控器输入
        if (remote_control_active()) 
        {
            // 从遥控器获取新目标点
            point2D new_target = Map_To_Cartesian();
            Set_Target_Position(new_target);
        }
        return;
    }
    
    // 1. 运动学求解
    JointAngles solution = IK_Solve(current_target,  // 添加缺失的参数
                                    arm_config.L1, 
                                    arm_config.L2, 
                                    &arm_config.limits);
    
    if (!solution.valid) {
        // 无解情况处理
        arm_state = ARM_ERROR;
        return;
    }
    
    // 更新目标角度
    target_angles = solution;
    
    // 2. 平滑角度过渡
    Smooth_Angle_Transition();
    
    // 3. 设置电机角度
    // 假设Motor1对应theta1，Motor2对应theta2
    extern motor_t motor[num]; // 假设外部定义了motor数组
    
    motor[Motor1].cmd.pos_set = current_angles.theta1;
    motor[Motor2].cmd.pos_set = current_angles.theta2;
    
    // 应用设置
    dm4310_set(&motor[Motor1]);
    dm4310_set(&motor[Motor2]);
    
    // 发送控制命令
    ctrl_send(ALL_MOTORS);
    
    // 检查是否到达目标
    if (arm_state == ARM_MOVING) {
        // 获取当前位置
        point2D current_position = Get_Current_Position();
        
        // 计算当前位置与目标位置的距离
        float dx = current_target.x - current_position.x;
        float dy = current_target.y - current_position.y;
        float distance = sqrtf(dx*dx + dy*dy);
        
        // 如果接近目标，切换到空闲状态
        if (distance < 0.005f) { // 5mm容差
            arm_state = ARM_IDLE;
        }
    }
}

// 获取当前机械臂状态
ArmState Get_Arm_State(void) 
{
    return arm_state;
}

// 获取当前末端位置（前向运动学）
point2D Get_Current_Position(void) 
{
    // 使用当前角度计算末端位置
    float theta1 = current_angles.theta1;
    float theta2 = current_angles.theta2;
    
    point2D position;
    position.x = arm_config.L1 * sinf(theta1) - arm_config.L2 * sinf(theta2);
    position.y = -arm_config.L1 * cosf(theta1) + arm_config.L2 * cosf(theta2);
    
    return position;
}