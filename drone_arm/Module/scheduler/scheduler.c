//任务调度器文件
#include "scheduler.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"

volatile uint32_t sys_time = 0; //系统时间
run_queue_t run_queue[MAX_TASKS]; //任务队列
uint8_t task_num = 0;  //当前任务数量

//任务表初始化
int scheduler_init(void)
{
	//配置SysTick为1ms中断
	if(HAL_SYSTICK_Config(SystemCoreClock/1000) != HAL_OK )
	{
		return -1; //初始化失败		
	}

	//初始化任务队列
	for(int i = 0; i < MAX_TASKS ; i++)
	{
		run_queue[i].fun = NULL;
		run_queue[i].active = false;
	}
	
	return 0;
}

//创建新任务
int create_task(void (*fun)(void),uint32_t period,const char* task_name)
{
	//临界区开始(防止在任务创建时被中断)
	__disable_irq();

	if(task_num >= MAX_TASKS)
	{
		__enable_irq();
		return -1;//指示超过最大任务数
	}

	if(fun == NULL || period == 0)
	{
		__enable_irq();
		return -2; //指示为无效参数
	}

	//查找空闲任务槽
	uint8_t i;
	for(i = 0; i < MAX_TASKS; i++)
	{
		if(run_queue[i].fun == NULL)
		{
			break;
		}
	} 

	if(i >= MAX_TASKS)
	{
		__enable_irq();
		return -3 ;//没有空闲槽位
	}

	//填充任务结构
	run_queue[i].fun = fun;
	run_queue[i].period = period;
	run_queue[i].last_run = sys_time;
	run_queue[i].active	= true ;
	run_queue[i].exec_count = 0;
	run_queue[i].task_name = task_name;

	task_num++;

	__enable_irq();
	return 0;
}

int delete_task(uint8_t task_id)
{
	if(task_id >= MAX_TASKS || run_queue[task_id].fun == NULL)
	{
		return -1; //无效任务id
	}

	__disable_irq(); //临界区开始(防止中断干扰)
	run_queue[task_id].fun = NULL;
    run_queue[task_id].active = false;
    task_num--;

	__enable_irq();//临界区结束

	return 0;
}

//任务表执行函数
int scheduler_run(void)
{
	static uint32_t last_run = 0;

	//防止过度频繁调用(最小时间间隔检查)
	if((sys_time-last_run)<SCHEDULER_TIME_MS)
	{
		return 0;
	}

	last_run = sys_time;

	//遍历所有任务
	for(uint8_t i = 0; i< MAX_TASKS; i++)
	{
		if(run_queue[i].fun != NULL && run_queue[i].active)
		{
			//检查是否到达执行时间
			uint32_t elapsed = sys_time - run_queue[i].last_run;
			if(elapsed >= run_queue[i].period)
			{
				//更新执行时间并执行任务
				run_queue[i].last_run = sys_time;
				run_queue[i].exec_count++;
				run_queue[i].fun(); //执行任务
			}
		}
	}
	return 0;
}

void scheduler_dump_info(void) {
    printf("\n=== Task Scheduler Info ===\n");
    printf("System Time: %lums\n", (unsigned long)sys_time);
    printf("Active Tasks: %d/%d\n", task_num, MAX_TASKS);
    
    for(uint8_t i = 0; i < MAX_TASKS; i++) {
        if(run_queue[i].fun != NULL && run_queue[i].active) {
            uint32_t next_run = run_queue[i].last_run + run_queue[i].period;
            uint32_t remain = (next_run > sys_time) ? (next_run - sys_time) : 0;
            
            printf("Task[%d]: %s\n", i, run_queue[i].task_name);
            printf("  Period: %lums, Executions: %lu\n", (unsigned long)run_queue[i].period, (unsigned long)run_queue[i].exec_count);
            printf("  Next run in: %lums\n", (unsigned long)remain);
        }
    }
    printf("===========================\n");
}