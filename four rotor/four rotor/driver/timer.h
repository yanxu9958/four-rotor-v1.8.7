#ifndef _timer_h_
#define _timer_h_

#include "stm32f10x.h"

typedef struct
{
    float       last_time_us;
    float       now_time_us;
    float       delta_time_us;
    float       delta_time_ms;
}_Time_test;    


void timer_init(void);

void time_check(_Time_test *running);



#endif

