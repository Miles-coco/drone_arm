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

}

void task_500ms(void)
{
      float real_pos1 = motor[Motor1].para.pos;
      float real_pos2 = motor[Motor2].para.pos;
  
}
