#include "dm_drv.h"
#include "dm_ctrl.h"
#include "can_driver.h"
#include "string.h"

motor_t motor[num];
int8_t motor_id = 1;

/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310电机初始化函数
* @param:      	void
* @retval:     	void
* @details:    	初始化三个DM4310型号的电机，设置默认参数和控制模式。
*               分别初始化Motor1和Motor2，设置ID、控制模式和命令模式等信息。
************************************************************************
**/
void dm4310_motor_init(void)
{
	// 初始化Motor1和Motor2的电机结构
	memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
	memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
	memset(&motor[Motor3], 0, sizeof(motor[Motor2]));

	// 设置Motor1的电机信息
	motor[Motor1].id = 1;
	motor[Motor1].ctrl.mode = 2;		// 0: MIT模式   1: 位置速度模式   2: 速度模式
	motor[Motor1].cmd.mode = 2;

	// 设置Motor2的电机信息
	motor[Motor2].id = 1;
	motor[Motor2].ctrl.mode = 2;
	motor[Motor2].cmd.mode = 2;
	
	// 设置Motor3的电机信息
	motor[Motor3].id = 2;
	motor[Motor3].ctrl.mode = 2;
	motor[Motor3].cmd.mode = 2;
}

/**
************************************************************************
* @brief:      	ctrl_enable: 启用电机控制函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），启用对应的电机控制。
*               设置指定电机的启动标志，并调用dm4310_enable函数启用电机。
************************************************************************
**/
void ctrl_enable(void)
{
	switch(motor_id)
	{
		case 1:
			// 启用Motor1的电机控制
			motor[Motor1].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// 启用Motor2的电机控制
			motor[Motor2].start_flag = 1;
			dm4310_enable(&hcan2, &motor[Motor2]);
			break;
		case 3:
			// 启用Motor3的电机控制
			motor[Motor3].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor3]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_disable: 禁用电机控制函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），禁用对应的电机控制。
*               设置指定电机的启动标志为0，并调用dm4310_disable函数禁用电机。
************************************************************************
**/
void ctrl_disable(void)
{
	switch(motor_id)
	{
		case 1:
			// 禁用Motor1的电机控制
			motor[Motor1].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// 禁用Motor2的电机控制
			motor[Motor2].start_flag = 0;
			dm4310_disable(&hcan2, &motor[Motor2]);
			break;
		case 3:
			// 禁用Motor2的电机控制
			motor[Motor3].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor3]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_set: 设置电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），设置对应电机的参数。
*               调用dm4310_set函数设置指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_set(void)
{
	switch(motor_id)
	{
		case 1:
			// 设置Motor1的电机参数
			dm4310_set(&motor[Motor1]);
			break;
		case 2:
			// 设置Motor2的电机参数
			dm4310_set(&motor[Motor2]);
			break;
		case 3:
			// 设置Motor3的电机参数
			dm4310_set(&motor[Motor3]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_clear_para: 清除电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），清除对应电机的参数。
*               调用dm4310_clear函数清除指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_clear_para(void)
{
	switch(motor_id)
	{
		case 1:
			// 清除Motor1的电机参数
			dm4310_clear_para(&motor[Motor1]);
			break;
		case 2:
			// 清除Motor2的电机参数
			dm4310_clear_para(&motor[Motor2]);
			break;
		case 3:
			// 清除Motor2的电机参数
			dm4310_clear_para(&motor[Motor3]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_clear_err: 清除电机错误信息
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），清除对应电机的参数。
*               调用dm4310_clear函数清除指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_clear_err(void)
{
	switch(motor_id)
	{
		case 1:
			// 清除Motor1的电机错误参数
			dm4310_clear_err(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// 清除Motor2的电机错误参数
			dm4310_clear_err(&hcan2, &motor[Motor2]);
			break;
		case 3:
			// 清除Motor3的电机错误参数
			dm4310_clear_err(&hcan1, &motor[Motor3]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_send: 发送电机控制命令函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），向对应电机发送控制命令。
*               调用dm4310_ctrl_send函数向指定电机发送控制命令，以响应外部命令。
************************************************************************
**/
void ctrl_send(void)
{
	switch(motor_id)
	{
		case 1:
			 // 向Motor1发送控制命令
			dm4310_ctrl_send(&hcan1, &motor[Motor1]);
			break;
		case 2:
			 // 向Motor2发送控制命令
			dm4310_ctrl_send(&hcan2, &motor[Motor2]);
			break;
		case 3:
			 // 向Motor3发送控制命令
			dm4310_ctrl_send(&hcan1, &motor[Motor3]);
			break;
	}
}
/**
************************************************************************
* @brief:      	can1_rx_callback: CAN1接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
void can1_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	canx_receive_data(&hcan1, &rec_id, rx_data);
	switch (rec_id)
	{
 		case 0: 
			{
				switch ((rx_data[0])&0x0F)
				{
					case 11: dm4310_fbdata(&motor[Motor1], rx_data); break;
					case 12: dm4310_fbdata(&motor[Motor3], rx_data); break;
				}
				
			} break;
	}
}
/**
************************************************************************
* @brief:      	can1_rx_callback: CAN2接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
uint16_t rec_id;
void can2_rx_callback(void)
{
	
	uint8_t rx_data[8] = {0};
	canx_receive_data(&hcan2, &rec_id, rx_data);
	switch (rec_id)
	{
		case 0: dm4310_fbdata(&motor[Motor2], rx_data); break;
	}
}

