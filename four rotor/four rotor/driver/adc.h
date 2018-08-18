#ifndef _adc_h_
#define _adc_h_


#include "stm32f10x.h"

typedef struct
{
    uint16_t adc;
    float voltage;
    uint8_t danger_flag;
}_ADC_VALUE;

#include "stm32f10x.h"


void adc_init(void);
void voltage_detection(void);


extern _ADC_VALUE bat;


#endif


