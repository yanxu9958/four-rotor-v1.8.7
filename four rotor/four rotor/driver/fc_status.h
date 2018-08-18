#ifndef _fc_status_h_
#define _fc_status_h_



#include "stm32f10x.h"


#define fc_unlock  1
#define fc_lock    0

typedef struct 
{
    uint8_t  state :        1;
    uint8_t  unlock_flag :  1;
    uint8_t  lock_flag :    1;
    
    uint16_t unlock_cnt ;
    uint16_t lock_cnt ;
}_FC_STATUS;




void FC_status_polling(void);
void get_chip_id(void);
extern _FC_STATUS fc;




#endif

