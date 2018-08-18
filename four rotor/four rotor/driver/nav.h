#ifndef _nav_h_
#define _nav_h_



#include "stm32f10x.h"
#include "mpu6050.h"

typedef struct
{
    float acc[3];
	float acc_lenth;
    float integral_vel[3];
    
}_NAV_DATA;

void integral_vel(void);
void get_nav_acc(SI_F_XYZ acc_lpf);

extern _NAV_DATA nav;


#endif

