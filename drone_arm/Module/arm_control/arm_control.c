#include "arm_control.h"
#include "main.h"
#include "math.h"
#include "remote_control.h"  // 包含remote_control_active声明
#include "dm_ctrl.h"
#include "coordinate_mapping.h"

// 定义π常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 角度转换宏
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0f)

// 当前目标角度
static JointAngles target_angles;
// 当前执行角度
static JointAngles current_angles;

// 最大角度变化率 (约5度/控制周期)
static const float MAX_ANGLE_STEP = DEG_TO_RAD(5.0f);

// 初始化函数
void ArmControl_Init(void) 
{
    // 设置工作空间范围（使用默认值）
    Set_Joint_Ranges(
        -M_PI,          // 关节1最小角度 -180°
        M_PI,           // 关节1最大角度 180°
        -3*M_PI/4,      // 关节2最小角度 -135°
        -M_PI/4         // 关节2最大角度 -45°
    );
    
    // 设置摇杆通道（使用默认配置）
    Set_Stick_Channels(1, 2);
    
    // 初始化电机
    dm4310_motor_init();
    
    // 启用电机控制
    ctrl_enable(ALL_MOTORS);
    
    // 初始化角度为安全位置
    target_angles.theta1 = 0.0f;
    target_angles.theta2 = -M_PI/2;  // -90度
    target_angles.valid = true;
    current_angles = target_angles;  // 初始当前角度等于目标角度
}

// 应用平滑处理
static void Apply_Smoothing(void)
{
    // 关节1平滑
    float delta1 = target_angles.theta1 - current_angles.theta1;
    if(fabsf(delta1) > MAX_ANGLE_STEP) {
        current_angles.theta1 += copysignf(MAX_ANGLE_STEP, delta1);
    } else {
        current_angles.theta1 = target_angles.theta1;
    }
    
    // 关节2平滑
    float delta2 = target_angles.theta2 - current_angles.theta2;
    if(fabsf(delta2) > MAX_ANGLE_STEP) {
        current_angles.theta2 += copysignf(MAX_ANGLE_STEP, delta2);
    } else {
        current_angles.theta2 = target_angles.theta2;
    }
}

// 更新机械臂控制
void ArmControl_Update(void) 
{
    // 检查遥控器是否激活
    if (remote_control_active()) 
    {
        // 获取遥控器映射的关节角度
        JointAngles new_angles = Map_To_JointSpace();
        
        // 如果映射有效，则更新目标角度
        if (new_angles.valid) {
            target_angles = new_angles;
        }
    }
    
    // 应用平滑处理
    Apply_Smoothing();
    
    // 直接应用平滑后的角度到电机
    // 应用关节1角度
    motor[Motor1].cmd.pos_set = current_angles.theta1;
    dm4310_set(&motor[Motor1]);
    
    // 应用关节2角度
    motor[Motor2].cmd.pos_set = current_angles.theta2;
    dm4310_set(&motor[Motor2]);
    
    // 发送控制命令
    ctrl_send(ALL_MOTORS);
}