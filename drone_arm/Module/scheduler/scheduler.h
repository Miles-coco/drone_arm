#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>


#define MAX_TASKS 32   // 更合理的任务数量（节省RAM）
#define SCHEDULER_TIME_MS 1  // 时间单位=1ms

typedef struct
{
	void (*fun)(void);        //任务函数指针
	uint32_t period;          //执行周期
	uint32_t last_run;        //上次执行的时间戳
	bool active;              //任务激活标志
	uint32_t exec_count;      //执行任务计数器(调试用)
	const char* task_name;    //任务名称(调试用)
}run_queue_t;

extern volatile uint32_t sys_time; //系统时间(ms)

//函数声明
int scheduler_init(void);
int create_task(void (*fun)(void), uint32_t period, const char* task_name);
int delete_task(uint8_t task_id);
int scheduler_run(void);
void scheduler_dump_info(void);



#endif /* __SCHEDULER_H__ */


