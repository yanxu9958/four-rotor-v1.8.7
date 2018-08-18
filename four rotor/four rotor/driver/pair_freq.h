#ifndef _pair_freq_h_
#define _pair_freq_h_



#include "stm32f10x.h"


typedef struct
{
    uint8_t flag ;
    uint8_t addr[5];
    uint8_t freq_channel[4];
}_PAIR;

extern _PAIR pair;

void wait_pairing(void);


#endif

