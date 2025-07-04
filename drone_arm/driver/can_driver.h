#ifndef __CAN_DRIVER_H__
#define __CAN_DRIVER_H__

#include "main.h"
#include "bsp_can.h"

#define CAN_SEND_ID 0X500
#define CAN_RECEIVE_ID 0X580
#define CAN_BUFFER_SIZE   (128)

typedef struct
{
	uint8_t *data;
	uint16_t read_index, write_index;
	uint16_t size;
} can_fifo_buffer_t;

extern uint8_t can_buff[CAN_BUFFER_SIZE];

void can_driver_init(void);
void canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t canx_receive_data(CAN_HandleTypeDef *hcan, uint16_t *rec_id, uint8_t *buf);

uint16_t can_serial_available(void);
uint8_t can_serial_read_char(void);
uint16_t can_serial_write(uint8_t *buffer, uint16_t length); //����
uint16_t can_serial_read(uint8_t *buffer, uint16_t length);  //����

void can_para_init(void);
void can_transmit(void);



#endif /* __CAN_DRIVER_H__ */


