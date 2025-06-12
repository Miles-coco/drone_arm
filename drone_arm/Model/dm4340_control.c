#include "dm4340_control.h"
#include "stm32f4xx_hal.h"
#include "bsp_can.h"

Motor_Inf mtr;//电机信息结构体
extern CAN_HandleTypeDef hcan1;//CAN总线接口声明
extern CAN_HandleTypeDef hcan2;

/**
 * 浮点数据归一化成无符号整数，常用于嵌入式系统的电机控制场景，特别是要将物理量转化位固定位数数字信号的应用中
 * x：特定物理量
 * x_min，x_max：x的范围
 * bits：x需要转化的位范围
 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * 将归一化无符号整数还原成原始浮点数据
 * x：特定物理量
 * x_min，x_max：x的范围
 * bits：x需要转化的位范围
 */
    
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

//控制电机函数
void ctrl_motor(CAN_HandleTypeDef* hcan,uint16_t id, Motor_MIT_Data_t* _dm43_mit_t) //MIT模式
{
    uint8_t dm43_can_send_data[8];//can报文数据域
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;//临时存储转换后的整数值
    
    pos_tmp = float_to_uint(_dm43_mit_t->p_int, P_MIN, P_MAX, 16);//目标位置
    vel_tmp = float_to_uint(_dm43_mit_t->v_int, V_MIN, V_MAX, 12);//目标速度
    kp_tmp  = float_to_uint(_dm43_mit_t->kp_int, KP_MIN, KP_MAX, 12);//位置增益
    kd_tmp  = float_to_uint(_dm43_mit_t->kd_int, KD_MIN, KD_MAX, 12);//微分增益
    tor_tmp = float_to_uint(_dm43_mit_t->t_int, T_MIN, T_MAX, 12);//前馈转矩
	
	dm43_can_send_data[0] = (pos_tmp >> 8); //位置高8位
	dm43_can_send_data[1] = pos_tmp; //位置低八位
	dm43_can_send_data[2] = (vel_tmp >> 4); //速度高八位
	dm43_can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8); //速度低4位+kp高4位
	dm43_can_send_data[4] = kp_tmp; //kp低八位
	dm43_can_send_data[5] = (kd_tmp >> 4); //kd高8位
	dm43_can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8); //kd低4位 + 转矩高4位
	dm43_can_send_data[7] = tor_tmp; //转矩低8位
	
	//Tx_Header.StdId=id;设置目标电机ID
	//1: Tx_Header.IDE=CAN_ID_STD; 标准CAN_ID  0: Tx_Header.IDE=CAN_ID_EXT; 拓展CAN_ID 
	//1: Tx_Header.RTR=CAN_RTR_REMOTE; 数据帧    0: Tx_Header.RTR=CAN_RTR_REMOTE; 数据帧
	//Tx_Header.DLC=0x08; 8字节数据长度
	if(CANx_SendStdData(hcan,id,dm43_can_send_data,8,1,1)!=0)
	{
		//暂时没有调试，后面加上来
	}
}	

void ctrl_motor2(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel) //位置模式
{
//	uint8_t *pbuf,*vbuf;
//	pbuf=(uint8_t*)&_pos;
//	vbuf=(uint8_t*)&_vel;
//	
//	hcan->pTxMsg->StdId = id;
//	hcan->pTxMsg->IDE = CAN_ID_STD;
//	hcan->pTxMsg->RTR = CAN_RTR_DATA;
//	hcan->pTxMsg->DLC = 0x08;
//	hcan->pTxMsg->Data[0] = *pbuf;
//	hcan->pTxMsg->Data[1] = *(pbuf+1);
//	hcan->pTxMsg->Data[2] = *(pbuf+2);
//	hcan->pTxMsg->Data[3] = *(pbuf+3);
//	hcan->pTxMsg->Data[4] = *vbuf;
//	hcan->pTxMsg->Data[5] = *(vbuf+1);
//	hcan->pTxMsg->Data[6] = *(vbuf+2);
//	hcan->pTxMsg->Data[7] = *(vbuf+3);
//	
//	HAL_CAN_Transmit(hcan, 100);
}	

void ctrl_motor3(CAN_HandleTypeDef* hcan,uint16_t id, float _vel) //速度模式
{
//uint8_t *vbuf;
//	vbuf=(uint8_t*)&_vel;
//	
//	hcan->pTxMsg->StdId = id;
//	hcan->pTxMsg->IDE = CAN_ID_STD;
//	hcan->pTxMsg->RTR = CAN_RTR_DATA;
//	hcan->pTxMsg->DLC = 0x04;
//	hcan->pTxMsg->Data[0] = *vbuf;
//	hcan->pTxMsg->Data[1] = *(vbuf+1);
//	hcan->pTxMsg->Data[2] = *(vbuf+2);
//	hcan->pTxMsg->Data[3] = *(vbuf+3);
//	
//	HAL_CAN_Transmit(hcan, 100);
}	

void start_motor(CAN_HandleTypeDef* hcan,uint16_t id)
{
    uint8_t dm43_can_send_data[8];

	dm43_can_send_data[0] = 0xFF;
	dm43_can_send_data[1] = 0xFF;
	dm43_can_send_data[2] = 0xFF;
	dm43_can_send_data[3] = 0xFF;
	dm43_can_send_data[4] = 0xFF;
	dm43_can_send_data[5] = 0xFF;
	dm43_can_send_data[6] = 0xFF;
	dm43_can_send_data[7] = 0xFC;

	//Tx_Header.StdId=id;设置目标电机ID
	//1: Tx_Header.IDE=CAN_ID_STD; 标准CAN_ID  0: Tx_Header.IDE=CAN_ID_EXT; 拓展CAN_ID 
	//1: Tx_Header.RTR=CAN_RTR_REMOTE; 数据帧    0: Tx_Header.RTR=CAN_RTR_REMOTE; 数据帧
	//Tx_Header.DLC=0x08; 8字节数据长度

	if(CANx_SendStdData(hcan, id, dm43_can_send_data, 8, 1, 1) != 0)
    {
        // 错误处理
    }

}	

void lock_motor(CAN_HandleTypeDef* hcan,uint16_t id)
{
    uint8_t dm43_can_send_data[8];

	dm43_can_send_data[0] = 0xFF;
	dm43_can_send_data[1] = 0xFF;
	dm43_can_send_data[2] = 0xFF;
	dm43_can_send_data[3] = 0xFF;
	dm43_can_send_data[4] = 0xFF;
	dm43_can_send_data[5] = 0xFF;
	dm43_can_send_data[6] = 0xFF;
	dm43_can_send_data[7] = 0xFD;
        
	if(CANx_SendStdData(hcan, id, dm43_can_send_data, 8, 1, 1) != 0)
    {
        // 错误处理
    }
}	

