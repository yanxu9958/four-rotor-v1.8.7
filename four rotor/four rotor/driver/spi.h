#ifndef _spi_h_
#define _spi_h_

#include "stm32f10x.h"


void Spi_Init(void);			 

u8 Spi_RW_Byte(u8 TxData);



#endif

