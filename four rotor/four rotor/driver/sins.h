#ifndef _sins_h_
#define _sins_h_


#include "stm32f10x.h"
#include "mpu6050.h"

#define pos_delay_num  50

typedef struct
{
	SI_F_XYZ state_delta;
	
	SI_F_XYZ last_acc;
	SI_F_XYZ acc;
	SI_F_XYZ acc_cor;
	
	SI_F_XYZ vel;
	SI_F_XYZ vel_org;
	SI_F_XYZ vel_cor;
	SI_F_XYZ vel_delta;
	
	SI_F_XYZ pos;
	SI_F_XYZ pos_org;
	SI_F_XYZ pos_cor;
	SI_F_XYZ pos_history[pos_delay_num];
    
    U16_XYZ pos_delay_cnt;
    
    
    uint16_t high_start_cnt;
}_SINS;


void sins_high(void);

extern _SINS sins;


#endif

