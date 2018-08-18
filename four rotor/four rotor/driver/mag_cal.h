#ifndef _mag_cal_h_
#define _mag_cal_h_



#include "mpu6050.h"
#include "stm32f10x.h"


typedef struct
{
    uint16_t cnt;
    uint16_t i;
    uint8_t  flag;
    uint8_t  start_flag;    
    uint32_t sum;
    
    SI_F_XYZ offset;
    SI_F_XYZ offset_flash_read;
    SI_F_XYZ offset_flash_write;
    
    float flash_finish_flag;
}_MAG_CAL;







extern _MAG_CAL mag_cal;


#endif

