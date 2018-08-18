#include "timer.h"
#include "led.h"
#include "systick.h"
#include "iic.h"
#include "mpu6050.h"
#include "imu.h"
#include "usart2.h"
#include "ist8310.h"
#include "spl06.h"
#include "nrf24l01.h"
#include "pwm.h"
#include "adc.h"
#include "controller.h"
#include "parse_packet.h"
#include "fc_status.h"
#include "gyro_cal.h"
#include "acc_cal.h"
#include "flow.h"


//定时器初始化 5ms
void timer_init(void)
{ 
	TIM_TimeBaseInitTypeDef TIM_timeBaseStucture;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_timeBaseStucture.TIM_ClockDivision = TIM_CKD_DIV1;      	//分频因子，输出给定时器的ETRP数字滤波器提供时钟
	TIM_timeBaseStucture.TIM_Prescaler = 72-1;                		//预分频，给TIMx_CNT驱动的时钟，注意：实际的预分频值是0+1
	TIM_timeBaseStucture.TIM_Period = 5000-1;	
	TIM_timeBaseStucture.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM3,&TIM_timeBaseStucture);
	
	TIM_Cmd(TIM3,ENABLE);
    
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);							//先清除定时器更新标志位，防止一开启中断就进入中断处理函数中
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
}


uint32_t running_tim_cnt = 0;

//测试系统运行时间
void time_check(_Time_test *running)
{
    running->last_time_us = running->now_time_us;
    running->now_time_us = running_tim_cnt * 5000 + TIM3->CNT;
    
    running->delta_time_us = running->now_time_us - running->last_time_us;
    running->delta_time_ms = running->delta_time_us * 0.001f;
}

_Time_test run_start;
_Time_test run_stop;

void TIM3_IRQHandler(void)
{      
   	if(TIM3->SR&0X0001)
	{        
        running_tim_cnt++ ;
        
        time_check(&run_start);
        
		//无线数据解析
		nrf_parse_packet();

        //陀螺仪数据
        get_gyro_raw();
        get_deg_s(&gyro_raw_f,&Mpu.deg_s);
        get_rad_s(&gyro_raw_f,&Mpu.rad_s);
        
        //加速度数据
        get_acc_raw();
        
        //姿态解算时加速度低通滤波 
        acc_iir_lpf(&acc_raw_f,&acc_att_lpf,Mpu.att_acc_factor);
        get_acc_g(&acc_att_lpf,&Mpu.acc_g);  
       
        //高度融合加速度低通滤波 z
        acc_iir_lpf(&acc_raw_f,&acc_fix_lpf,Mpu.fix_acc_factor);
                
        //气压计数据
        get_spl_status();
        
        //姿态解算
        mahony_update(Mpu.rad_s.x,Mpu.rad_s.y,Mpu.rad_s.z,Mpu.acc_g.x,Mpu.acc_g.y,Mpu.acc_g.z);
        
        Matrix_ready();
		
        //光流处理
        flow_handle();
		
        //pid控制器
        _controller_detect();
        _controller_perform();
        
        _controller_output();
                
        //传感器校准轮询
        gyro_cal_polling();
        acc_cal_polling();        
        FC_status_polling();                //锁定状态轮询
        
        blink_polling();
        
        //匿名地面站波形显示
        ANO_DMA_SEND_DATA();      
        
        time_check(&run_stop);
	}				   
	TIM3->SR&=~(1<<0);  
}


