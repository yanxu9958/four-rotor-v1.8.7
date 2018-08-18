#ifndef _pwm_h_
#define _pwm_h_

#include "stm32f10x.h"


void pwm_init(void);

void pwm_out(uint16_t pwm1,uint16_t pwm2,uint16_t pwm3,uint16_t pwm4);

#endif

