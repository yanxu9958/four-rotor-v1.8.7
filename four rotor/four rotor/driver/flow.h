#ifndef _flow_h_
#define _flow_h_

#include "stm32f10x.h"


typedef struct
{
	float x;
	float y;
}_XY_F;


typedef struct
{
    uint8_t packet[10];
    uint8_t dat;
    uint8_t dat_cnt;
    uint8_t step;
    int16_t x;
    int16_t y;
	
	_XY_F pos;
	_XY_F last_pos;    
    _XY_F fix_pos;
	_XY_F nav_pos;
	
	_XY_F vel;
    _XY_F fix_vel;
    _XY_F nav_vel;
    
	_XY_F att;

    
    float pos_lpf_factor;
    float vel_lpf_factor;
    
    float att_pos_lpf_factor;
    
}_FLOW;

void flow_handle(void);

extern _FLOW flow;


#endif


