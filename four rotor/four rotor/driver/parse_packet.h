#ifndef _parse_packet_h_
#define _parse_packet_h_

#include "nrf24l01.h"
#include "stm32f10x.h"

typedef struct
{
	uint16_t thr;
    uint16_t thr_zone;
	float pit;
	float rol;
	float yaw;
    uint8_t key_l;
    uint8_t key_r;
    
    uint32_t key_l_cnt;
    uint32_t key_r_cnt;
    uint32_t high_cnt;
	uint32_t fix_cnt;
    
    uint8_t key_l_flag  : 1;
    uint8_t key_r_flag  : 1;
	uint8_t mode        : 2;
	uint8_t high_flag   : 1;
	uint8_t high_led_flag   : 2;
    uint8_t fix_flag    : 1;
    uint8_t fix_led_flag    : 2;
    
    uint8_t signal_lost_flag : 1;
	uint8_t signal_cnt;
    
}_RC_DATA;


extern _RC_DATA rc;
extern u8 Rx_packet[RX_PLOAD_WIDTH];

void nrf_parse_packet(void);
void parse_key_info(void);

#endif

