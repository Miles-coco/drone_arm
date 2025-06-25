#include "task.h"
#include "scheduler.h"
#include "driver.h"
#include "dm_ctrl.h"
#include "dm_drv.h"
#include "arm_control.h"

void task_init(void)
{
	//初始化调度器
	scheduler_init();

	//创建各周期任务
    create_task(task_1ms, 1, "1ms Fast Loop");
    create_task(task_5ms, 5, "5ms Control Loop");
    create_task(task_10ms, 10, "10ms Arm Update");
    create_task(task_100ms, 100, "100ms Debug Output");
}

void task_1ms(void)
{

}

void task_2ms(void)
{

}

void task_5ms(void)
{
	ctrl_send(ALL_MOTORS);
}

void task_10ms(void)
{
	ArmControl_Update();
}

void task_100ms(void)
{
	// 调试信息输出
    static uint8_t count = 0;
    
    if(count++ >= 10) 
	{ // 每秒输出一次
        count = 0;
        
        // 获取当前位置
        point2D current_pos = Get_Current_Position();
        
        // 获取关节角度
        JointAngles angles = {0};
        angles.theta1 = motor[Motor1].para.pos; // 假设存储的是弧度
        angles.theta2 = motor[Motor2].para.pos;
        
        // 实际系统中通过串口输出
        /*
        printf("Arm State: %d\n", Get_Arm_State());
        printf("Position: X=%.3f, Y=%.3f\n", current_pos.x, current_pos.y);
        printf("Angles: Theta1=%.1f°, Theta2=%.1f°\n", 
               angles.theta1 * 180/M_PI, angles.theta2 * 180/M_PI);
        */
    }
}

void task_500ms(void)
{

}
