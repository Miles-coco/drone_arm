#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"

void can_filter_init(void);
uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len, uint8_t isStandard, uint8_t isDataFrame);

#endif