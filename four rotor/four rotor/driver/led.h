#ifndef _led_h_
#define _led_h_

#include "stm32f10x.h"

void led_init(void);

//#define led1 1
//#define led2 2
//#define led3 3
//#define led4 4

/*   LED2          LED1   */ 
    /** *   /|\   * * *
         *   |   *
          *  |  *
           * | *
            * *
             *
            * *
           *   *
          *     *
         *       *
    * * *         * * */
/*  LED3            LED4   */ 

void led_off_all(void);

void blink_polling(void);

void led_on(uint8_t led_num);
void led_off(uint8_t led_num);
void led_toggle(uint8_t led_num);


#endif

