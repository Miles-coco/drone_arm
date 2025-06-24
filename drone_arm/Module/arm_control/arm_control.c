#include "arm_control.h"
#include "main.h"
#include "math.h"

// 机械臂参数配置
static const ArmConfig arm_config = {
    .L1 = 0.15f,        // 主动臂1长度 (m)
    .L2 = 0.15f,        // 主动臂2长度 (m)
    .limits = {
        .min_theta1 = 0.0f,       // 最小角度 (弧度)
        .max_theta1 = M_PI,       // 最大角度 (弧度)
        .min_theta2 = 0.0f,
        .max_theta2 = M_PI
    }
};

// 工作空间配置
static WorkspaceConfig workspace_config = {
    .x_min = -0.3f,
    .x_max = 0.3f,
    .y_min = 0.1f,
    .y_max = 0.5f,
    .safety_margin = 0.02f
};

// 当前状态
static ArmState arm_state = ARM_IDLE;
static point2D current_target = {0, 0.2f}; // 默认位置
static JointAngles current_angles = {M_PI/4, M_PI/2}; // 当前角度
static JointAngles target_angles = {M_PI/4, M_PI/2}; // 目标角度

// 初始化函数
void ArmControl_Init(void) {
    // 初始化工作空间映射
    Mapping_Init(&workspace_config);
    Set_Stick_Channels(0, 1); // 通道0控制X轴，通道1控制Y轴
    
    // 初始化电机
    dm4310_motor_init();
    
    // 启用电机控制
    ctrl_enable();
    
    // 设置初始状态
    arm_state = ARM_IDLE;
}

// 设置目标位置
void Set_Target_Position(point2D target) {
    current_target = target;
    arm_state = ARM_MOVING;
}

// 平滑角度过渡
static void Smooth_Angle_Transition(void) {
    const float MAX_ANGLE_STEP = DEG_TO_RAD(5.0f); // 每步最大5度
    
    // 检查θ1步进
    float delta1 = target_angles.theta1 - current_angles.theta1;
    if (fabsf(delta1) > MAX_ANGLE_STEP) {
        current_angles.theta1 += copysignf(MAX_ANGLE_STEP, delta1);
    } else {
        current_angles.theta1 = target_angles.theta1;
    }
    
    // 检查θ2步进
    float delta2 = target_angles.theta2 - current_angles.theta2;
    if (fabsf(delta2) > MAX_ANGLE_STEP) {
        current_angles.theta2 += copysignf(MAX_ANGLE_STEP, delta2);
    } else {
        current_angles.theta2 = target_angles.theta2;
    }
    
    // 检查是否到达目标
    if (fabsf(delta1) < 0.001f && fabsf(delta2) < 0.001f) {
        arm_state = ARM_IDLE;
    }
}

// 更新机械臂控制
void ArmControl_Update(void) {
    // 空闲状态等待新目标
    if (arm_state == ARM_IDLE) {
        return;
    }
    
    // 1. 运动学求解
    JointAngles solutions[2];
    int num_solutions = IK_Solve(current_target, 
                                 arm_config.L1, 
                                 arm_config.L2, 
                                 &arm_config.limits, 
                                 &current_angles); // 传递当前角度
    
    if (num_solutions == 0) {
        // 无解情况处理
        arm_state = ARM_ERROR;
        return;
    }
    
    // 选择最优解（IK_Solve已将结果写入current_angles）
    // target_angles = current_angles; // 不再需要
    
    // 2. 平滑角度过渡
    Smooth_Angle_Transition();
    
    // 3. 设置电机角度
    // 假设Motor1对应theta1，Motor2对应theta2
    motor[Motor1].cmd.pos_set = current_angles.theta1;
    motor[Motor2].cmd.pos_set = current_angles.theta2;
    
    // 应用设置
    dm4310_set(&motor[Motor1]);
    dm4310_set(&motor[Motor2]);
    
    // 发送控制命令
    ctrl_send();
}

// 获取当前机械臂状态
ArmState Get_Arm_State(void) {
    return arm_state;
}