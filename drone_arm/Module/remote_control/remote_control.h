#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

//#define RC_PROTOCOL_DJI    // 大疆协议
#define RC_PROTOCOL_TDF     // 天地飞协议

#include "usart.h"
#include <stdint.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


#define SBUS_RX_BUF_NUM 50u

#define RC_FRAME_LENGTH 25

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
typedef  struct __packed
{
        struct __packed
        {
                int16_t ch[5];
                uint8_t s[2];
        } rc;
        struct __packed
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        struct __packed
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

typedef struct __packed
{
    /* rocker channel information */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    
    /* Vr channel information */
    int16_t Vra;
    int16_t Vrb;

    /* left and right lever information */
    uint8_t sw1;
    uint8_t sw2;
    uint8_t sw3;
    uint8_t sw4;
} rc_info_t;

extern void MX_USART3_UART_Init(void);

extern uint16_t channels[18];
extern RC_ctrl_t rc_ctrl;//用于存储大疆遥控器的控制数据结构体
extern rc_info_t rc_ctrl1;//用于存储天地飞遥控器的数据结构体
void Sbus_Data_Count(rc_info_t *rc, uint8_t *sbusData);//天地飞遥控解析函数
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void remote_control_init(void);
int16_t get_channel_raw_value(uint8_t ch_num);
int16_t get_channel_mapped_value(uint8_t ch_num,int16_t min,int16_t max);
RC_ctrl_t get_remote_control_data(void);
__weak void USART3_IRQHandler(void); 

#endif
