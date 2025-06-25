#ifndef __DM_CTRL_H__
#define __DM_CTRL_H__
#include "main.h"
#include "dm_drv.h"

// 定义位掩码用于选择多个电机
#define MOTOR1_BIT (1 << Motor1)
#define MOTOR2_BIT (1 << Motor2)
// #define MOTOR3_BIT (1 << Motor3)

// 宏定义同时控制两个电机
#define ALL_MOTORS (MOTOR1_BIT | MOTOR2_BIT)


typedef enum
{
	Motor1, //值为0
	Motor2, //值为1
	//Motor3,
	num
} motor_num;

extern motor_t motor[num];

void dm4310_motor_init(void);
void ctrl_enable(uint32_t motor_mask);
void ctrl_disable(uint32_t motor_mask);
void ctrl_set(uint32_t motor_mask);
void ctrl_clear_para(uint32_t motor_mask);
void ctrl_clear_err(uint32_t motor_mask);
void ctrl_send(uint32_t motor_mask);

#endif /* __DM4310_CTRL_H__ */

