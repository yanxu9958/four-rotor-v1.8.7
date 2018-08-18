#ifndef _usart2_h_
#define _usart2_h_


#include "stm32f10x.h"
#include "stdio.h"

typedef struct
{
    uint8_t  send_period_cnt;
    uint32_t Head;
    float    databuf[8];
    uint32_t End;
}_DMA_usart2_wave;


void usart2_init(u32 bound);

void USART2_DMA_SEND_DATA(u32 SendBuff,u16 len);


void ANO_DMA_SEND_DATA(void);




#endif

