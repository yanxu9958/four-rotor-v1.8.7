#include "parse_packet.h"
#include "nrf24l01.h"
#include "led.h"
#include "imath.h"
#include "mpu6050.h"
#include "controller.h"
#include "pair_freq.h"

u8 Rx_packet[RX_PLOAD_WIDTH] = {0};

_RC_DATA rc = {0};

//数据包解析
void nrf_parse_packet(void)
{
	if(NRF24L01_RxPacket(Rx_packet)==0)						//成功接收到数据
	{
        if((Rx_packet[RX_PLOAD_WIDTH-1]!=0x8B))
            return;
        
		rc.signal_cnt = 0;
        
		rc.thr = Rx_packet[2]<<8|Rx_packet[1];
		rc.pit = -(float)(((int8_t)Rx_packet[3]) - 50);
        rc.rol = -(float)(((int8_t)Rx_packet[4]) - 50);
        rc.yaw =  (float)(((int8_t)Rx_packet[5]) - 50);
        rc.key_l = Rx_packet[6];
        rc.key_r = Rx_packet[7];
        
		rc.thr = throttle_limit(rc.thr,0,1000);
        rc.pit = direction_to_zero(rc.pit,-5,5);
        rc.rol = direction_to_zero(rc.rol,-5,5);
        rc.yaw = direction_to_zero(rc.yaw,-5,5);   

        rc.thr_zone = rc.thr;
	}
	else
	{
		if(rc.signal_cnt<200)
        {
			rc.signal_cnt++;  
            rc.signal_lost_flag = 0;            
        }

		if(rc.signal_cnt==200)                  //信号丢失减速处理         
		{
            rc.signal_lost_flag = 1;
            if(rc.thr>10)
            {
                rc.thr -= 1;		                
            }
		}
		rc.thr = throttle_limit(rc.thr,0,1000);
	}
}

#define high_blink_period 50000

#define fix_blink_period 50000

//按键信息解析
void parse_key_info(void)
{
    if(rc.key_l==0xE1) 
    {
        rc.mode = 1;   
		    
		//定高开启且指示灯闪烁作提示，定高标志置位
		if(rc.high_flag==0&&rc.pit<=35&&rc.rol>=-35)
		{
			if(rc.high_cnt < high_blink_period)
				rc.high_cnt++;
			if(rc.high_cnt >= high_blink_period)
			{
				rc.high_cnt = 0;
				rc.high_flag = 1;				//定高标志位置位
				rc.high_led_flag = 1;
			}
		}
		else if(rc.high_flag==1&&rc.pit<=35&&rc.rol>=-35)
		{
			if(rc.high_cnt < high_blink_period)
				rc.high_cnt++;
			if(rc.high_cnt >= high_blink_period)
			{
				rc.high_cnt = 0;
				rc.high_flag = 0;				//清楚定高标志位
                high_mark_flag = 0;             //标记油门标志清除
				rc.high_led_flag = 2;
			}			
		}
    }        

    if(rc.key_r==0xC8)
    {
        rc.mode = 2;  
		//定点开启且指示灯闪烁作提示，定点标志置位
		if(rc.fix_flag==0&&rc.yaw>=-35)
		{
			if(rc.fix_cnt < fix_blink_period)
				rc.fix_cnt++;
			if(rc.fix_cnt >= fix_blink_period)
			{
				rc.fix_cnt = 0;
				rc.fix_flag = 1;				//定点标志位置位
				rc.fix_led_flag = 1;
				
				flow_fix_flag = 0;  
			}
		}
		else if(rc.fix_flag==1&&rc.yaw>=-35)
		{
			if(rc.fix_cnt < fix_blink_period)
				rc.fix_cnt++;
			if(rc.fix_cnt >= fix_blink_period)
			{
				rc.fix_cnt = 0;
				rc.fix_flag = 0;				//清楚定点标志位
                fix_mark_flag = 0;              //清楚定点标志位
				rc.fix_led_flag = 2;
			}			
		}       
    }

    if((rc.key_l!=0xE1)&&(rc.key_r!=0xC8))
    {
        rc.mode = 0;        

    }

}




