#include "string.h"
#include "stdlib.h"
#include "Remote_Control.h"
#include "main.h"
#include "stm32f4xx.h"
#include "usart.h"

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];//双缓冲区初始化，16*2=32
uint16_t channels[18] = {0};  //初始化各通道
RC_ctrl_t rc_ctrl;//用于存储大疆遥控器的控制数据结构体
rc_info_t rc_ctrl1;//用于存储天地飞遥控器的数据结构体

//遥控初始化函数
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //设置USART3控制寄存器的DMAR位，使串口在接收到数据自动触发DMA请求
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //启用串口空闲中断：数据流：字节1 → 字节2 → ... → 字节N → 总线空闲
    //事件链：DMA连续搬运 → 总线空闲 → 触发中断 → 处理数据
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //安全禁用DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲地址1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲地址2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //启用双缓冲模式
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}

// 遥控初始化函数,建立双缓冲机制，启用USART3空闲终端，配置DMA为循环双缓冲模式
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

//数据获取接口
RC_ctrl_t get_remote_control_point(void)
{
    RC_ctrl_t copy;
    __disable_irq();//防止拷贝期间数据被修改
    memcpy(&copy,&rc_ctrl,sizeof(rc_ctrl));
    __enable_irq();
    return copy;
}

/**
  * 字节0: 0x0F (帧头)
  * 字节1-5: 通道0-3
  * 字节6-7: 鼠标X
  * 字节8-9: 鼠标Y
  * 字节10-11: 鼠标滚轮
  * 字节12: 鼠标左键
  * 字节13: 鼠标右键
  * 字节14-15: 键盘值
  * 字节16-17: 通道4
  * 字节18-22: 保留 (可能用于扩展通道)
  * 字节23: 状态字节 (0x08: failsafe, 0x04: frame lost)
  * 字节24: 0x00 (帧尾)
  */
//协议解析流程，适配大疆遥控器
void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    //防止空指针导致的程序崩溃
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left 左开关
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right 右开关
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis 鼠标x
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis 鼠标y
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis 鼠标z
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ? 
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    //所有通道减去中位偏移量
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
//适配天地飞遥控器
void Sbus_Data_Count(rc_info_t *rc, uint8_t *sbusData)
{
    //帧头帧尾校验
    if((sbusData[0]) == 0x0F && (sbusData[24] == 0x00))
    {
        //摇杆通道解析
        rc->ch1 = ((sbusData[1]|sbusData[2]<< 8) & 0x07FF) - 1024;
        rc->ch2 = ((sbusData[2]>>3|sbusData[3]<<5) & 0x07FF) - 1024;
        rc->ch3 = ((sbusData[3]>>6|sbusData[4]<<2|sbusData[5]<<10) & 0x07FF) - 1024;
        rc->ch4 = ((sbusData[5]>>1|sbusData[6]<<7) & 0x07FF) - 1024;
        //附加通道解析(旋钮)
        rc->Vra = ((sbusData[6]>>4|sbusData[7]<<4) & 0x07FF) - 1024;
        rc->Vrb = ((sbusData[7]>>7|sbusData[8]<<1|sbusData[9]<<9) & 0x07FF) - 1024;
        //拨杆开关解析
        rc->sw1 = ((sbusData[9]>>2|sbusData[10]<<6) & 0x07FF);
        switch(rc->sw1)
        {
            case 240 : rc->sw1 = 1;break;     //上
            case 0   : rc->sw1 = 2;break;     //中
            case 15  : rc->sw1 = 3;break;     //下
            default  : rc->sw1 = 1;           //上(默认)
        }
        //另一个拨杆开关
        rc->sw2 = ((sbusData[10]>>5|sbusData[11]<<3) & 0x07FF); // & the other 8 + 2 channels if you need them
        switch(rc->sw2)
        {
            case 240 : rc->sw2 = 1;break;
            case 0   : rc->sw2 = 2;break;
            case 15  : rc->sw2 = 3;break;
            default  : rc->sw2 = 1;
        }
        //又一个拨杆开关
        rc->sw3 = ((sbusData[12]|sbusData[13]<< 8) & 0x07FF);
        switch(rc->sw3)
        {
            case 240 : rc->sw3 = 1;break;
            case 0   : rc->sw3 = 2;break;
            case 15  : rc->sw3 = 3;break;
            default  : rc->sw3 = 1;
        }
        //又一个拨杆开关
        rc->sw4 = ((sbusData[13]>>3|sbusData[14]<<5) & 0x07FF);
        switch(rc->sw4)
        {
            case 240 : rc->sw4 = 1;break;
            case 0   : rc->sw4 = 2;break;
            case 15  : rc->sw4 = 3;break;
            default  : rc->sw4 = 1;
        }
        //#ifdef ALL_CHANNELS
        channels[10] = ((sbusData[14]>>6|sbusData[15]<<2|sbusData[16]<<10) & 0x07FF);
        channels[11] = ((sbusData[16]>>1|sbusData[17]<<7) & 0x07FF);
        channels[12] = ((sbusData[17]>>4|sbusData[18]<<4) & 0x07FF);
        channels[13] = ((sbusData[18]>>7|sbusData[19]<<1|sbusData[20]<<9) & 0x07FF);
        channels[14] = ((sbusData[20]>>2|sbusData[21]<<6) & 0x07FF);
        channels[15] = ((sbusData[21]>>5|sbusData[22]<<3) & 0x07FF);
    }

}


//中断处理逻辑
void USART3_IRQHandler(void)
{
    
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收数据寄存器非空,即缓冲区内接收到了数据还没被读取
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);//清除数据未被读取标志，不需做其他处理，实际数据接收由DMA完成
    }
    else if(USART3->SR & UART_FLAG_IDLE)//如果数据接收器为空，即监测到串口总线空闲(IDLE)(表示串口总线连续11位实践没有数据传输),表示一帧数据已经接收完成
    {
        //静态变量初始化(用于保存接收数据的长度)
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3); //清除空闲标志

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)//使用缓冲区0
        {
            //停止DMA传输，防止配置期间发生数据传输
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //计算接受数据长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //重置DMA计数器
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //切换到缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //重启DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            //数据处理
            if(this_time_rx_len == RC_FRAME_LENGTH)//仅在接收到的数据个数正好等于遥控协议标准数据个数时解析数据
            {
                //sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                //Sbus_Data_Count(&rc_ctrl1,sbus_rx_buf[0]);//处理DMA0的数据
                //协议选择
                #if defined(RC_PROTOCOL_DJI)
                    sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                #elif defined(RC_PROTOCOL_TDF)
                    Sbus_Data_Count(&rc_ctrl1, sbus_rx_buf[0]);
                #else 
                    #error "No remote control protocol defined! Please add #define RC_PROTOCOL_DJI or RC_PROTOCOL_TDF"
                #endif


            }
        }
        else//使用缓冲区1
        {
            //停止DMA运输，防止运输期间发生数据传输
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length，获得数据传输长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght重置DMA计数器
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0 切换到DMA0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            //数据处理
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                //Sbus_Data_Count(&rc_ctrl1,sbus_rx_buf[1]);
                //协议选择
                #if defined(RC_PROTOCOL_DJI)
                    sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                #elif defined(RC_PROTOCOL_TDF)
                    Sbus_Data_Count(&rc_ctrl1, sbus_rx_buf[1]);
                #else 
                    #error "No remote control protocol defined! Please add #define RC_PROTOCOL_DJI or RC_PROTOCOL_TDF"
                #endif
            }
        }
    }
}
