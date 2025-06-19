#include "dm_drv.h"
#include "can_driver.h"
#include "can.h"


/**
************************************************************************
* @brief:      	dm4310_enable: ����DM4310�������ģʽ����
* @param[in]:   hcan:    ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	���ݵ������ģʽ������Ӧ��ģʽ��ͨ��CAN���߷�����������
*               ֧�ֵĿ���ģʽ����λ��ģʽ��λ���ٶȿ���ģʽ���ٶȿ���ģʽ
************************************************************************
**/
void dm4310_enable(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			enable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			enable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
	}	
}
/**
************************************************************************
* @brief:      	dm4310_disable: ����DM4310�������ģʽ����
* @param[in]:   hcan:    ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	���ݵ������ģʽ������Ӧ��ģʽ��ͨ��CAN���߷��ͽ�������
*               ֧�ֵĿ���ģʽ����λ��ģʽ��λ���ٶȿ���ģʽ���ٶȿ���ģʽ
************************************************************************
**/
void dm4310_disable(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			disable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			disable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			disable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
	}	
	dm4310_clear_para(motor);
}
/**
************************************************************************
* @brief:      	dm4310_ctrl_send: ����DM4310������������
* @param[in]:   hcan:    ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	���ݵ������ģʽ������Ӧ�����DM4310���
*               ֧�ֵĿ���ģʽ����λ��ģʽ��λ���ٶȿ���ģʽ���ٶȿ���ģʽ
************************************************************************
**/
void dm4310_ctrl_send(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			mit_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case 1:
			pos_speed_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case 2:
			speed_ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
	}	
}
/**
************************************************************************
* @brief:      	dm4310_set: ����DM4310������Ʋ�������
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	���������������DM4310����Ŀ��Ʋ���������λ�á��ٶȡ�
*               ��������(KP)��΢������(KD)��Ť��
************************************************************************
**/
void dm4310_set(motor_t *motor)
{
	motor->ctrl.kd_set 	= motor->cmd.kd_set;
	motor->ctrl.kp_set	= motor->cmd.kp_set;
	motor->ctrl.pos_set	= motor->cmd.pos_set;
	motor->ctrl.vel_set	= motor->cmd.vel_set;
	motor->ctrl.tor_set	= motor->cmd.tor_set;

}
/**
************************************************************************
* @brief:      	dm4310_clear: ���DM4310������Ʋ�������
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	��DM4310�������������Ϳ��Ʋ������㣬����λ�á��ٶȡ�
*               ��������(KP)��΢������(KD)��Ť��
************************************************************************
**/
void dm4310_clear_para(motor_t *motor)
{
	motor->cmd.kd_set 	= 0;
	motor->cmd.kp_set	 	= 0;
	motor->cmd.pos_set 	= 0;
	motor->cmd.vel_set 	= 0;
	motor->cmd.tor_set 	= 0;
	
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.kp_set	= 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
}
/**
************************************************************************
* @brief:      	dm4310_clear_err: ���DM4310���������
* @param[in]:   hcan: 	 ָ��CAN���ƽṹ���ָ��
* @param[in]:  	motor:   ָ�����ṹ���ָ��
* @retval:     	void
* @details:    	���ݵ���Ŀ���ģʽ�����ö�Ӧģʽ�����������
************************************************************************
**/
void dm4310_clear_err(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			clear_err(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			clear_err(hcan, motor->id, POS_MODE);
			break;
		case 2:
			clear_err(hcan, motor->id, SPEED_MODE);
			break;
	}	
}
/**
************************************************************************
* @brief:      	dm4310_fbdata: ��ȡDM4310����������ݺ���
* @param[in]:   motor:    ָ��motor_t�ṹ��ָ�룬������������Ϣ�ͷ�������
* @param[in]:   rx_data:  ָ������������ݵ�����ָ��
* @retval:     	void
* @details:    	�ӽ��յ�����������ȡDM4310����ķ�����Ϣ���������ID��
*               ״̬��λ�á��ٶȡ�Ť���Լ�����¶Ȳ���
************************************************************************
**/
void dm4310_fbdata(motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}

/**
************************************************************************
* @brief:      	float_to_uint: ������ת��Ϊ�޷�����������
* @param[in]:   x_float:	��ת���ĸ�����
* @param[in]:   x_min:		��Χ��Сֵ
* @param[in]:   x_max:		��Χ���ֵ
* @param[in]:   bits: 		Ŀ���޷���������λ��
* @retval:     	�޷����������
* @details:    	�������ĸ����� x ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��ָ��λ�����޷�������
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: �޷�������ת��Ϊ����������
* @param[in]:   x_int: ��ת�����޷�������
* @param[in]:   x_min: ��Χ��Сֵ
* @param[in]:   x_max: ��Χ���ֵ
* @param[in]:   bits:  �޷���������λ��
* @retval:     	���������
* @details:    	���������޷������� x_int ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��������
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: ���õ��ģʽ����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ������ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������������ض�ģʽ������
************************************************************************
**/
void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: ���õ��ģʽ����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ���õ�ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������ͽ����ض�ģʽ������
************************************************************************
**/
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	save_pos_zero: ����λ����㺯��
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ����λ������ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������ͱ���λ����������
************************************************************************
**/
void save_pos_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	clear_err: ������������
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ��������ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������������������
************************************************************************
**/
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	mit_ctrl: MITģʽ�µĵ�����ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   pos:			λ�ø���ֵ
* @param[in]:   vel:			�ٶȸ���ֵ
* @param[in]:   kp:				λ�ñ���ϵ��
* @param[in]:   kd:				λ��΢��ϵ��
* @param[in]:   torq:			ת�ظ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������MITģʽ�µĿ���֡��
************************************************************************
**/
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl: λ���ٶȿ��ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   vel:			�ٶȸ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������λ���ٶȿ�������
************************************************************************
**/
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	speed_ctrl: �ٶȿ��ƺ���
* @param[in]:   hcan: 		ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   vel: 			�ٶȸ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN�������������ٶȿ�������
************************************************************************
**/
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 4);
}
