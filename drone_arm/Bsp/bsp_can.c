#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "can.h"
#include <stdint.h>

extern CAN_HandleTypeDef hcan1;//CAN总线接口声明
extern CAN_HandleTypeDef hcan2;

/**
 * 初始化STM32的CAN过滤器系统，目前接收所有CAN_ID发来的消息
 * 配置CAN过滤器，硬件级数据筛选，用于高效过滤掉不需要的CAN消息，减少CPU负载
 * 启动CAN外设
 * 使能接收中断
 * 设置消息接收FIFO
 */
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;//声明CAN过滤器结构体
    can_filter_st.FilterActivation = ENABLE; //启用此过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK; //掩码模式(另一种是ID列表模式)
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; //32位宽过滤器
    can_filter_st.FilterIdHigh = 0x0000; //要接收的ID高位
    can_filter_st.FilterIdLow = 0x0000; //要接收的ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000; //掩码高位(决定哪些位需匹配)
    can_filter_st.FilterMaskIdLow = 0x0000;  //掩码低位
    can_filter_st.FilterBank = 0; //使用的过滤器编号
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0; //将接收到的消息存到FIFO0
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st); //使用的过滤器组编号
    HAL_CAN_Start(&hcan1); //
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14; //
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len, uint8_t isStandard, uint8_t isDataFrame)
{
  CAN_TxHeaderTypeDef   Tx_Header;
  uint32_t mailbox;
	
	Tx_Header.StdId=ID;
	Tx_Header.ExtId=0;
	Tx_Header.IDE=isStandard ? CAN_ID_STD : CAN_ID_EXT;
	Tx_Header.RTR=isDataFrame ? CAN_RTR_DATA : CAN_RTR_REMOTE;
	Tx_Header.DLC=Len > 8 ? 8 : Len;

  for(uint8_t attempt = 0;attempt < 3 ; attempt ++)
  {
    if (attempt == 0) mailbox = CAN_TX_MAILBOX0;
    else if(attempt == 1) mailbox = CAN_TX_MAILBOX1;
    else mailbox = CAN_TX_MAILBOX2;
  
    if(HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,&mailbox)==HAL_OK)
    {
      return 0;
    }
  }
  return 1;//指连试三次之后，均不行则返回一


}
