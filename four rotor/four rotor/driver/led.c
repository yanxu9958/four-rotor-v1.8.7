#include "led.h"
#include "acc_cal.h"
#include "gyro_cal.h"
#include "fc_status.h"
#include "adc.h"
#include "parse_packet.h"
void led_on_all(void)
{
    led_on(1);
    led_on(2);
    led_on(3);
    led_on(4);    
}

void led_off_all(void)
{
    led_off(1);
    led_off(2);
    led_off(3);
    led_off(4);     
}


/* led端口初始化 */
void led_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC,ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_Init(GPIOC, &GPIO_InitStructure);  

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
    
    led_on_all();
}


uint16_t blink_speed_cnt = 0;
#define  blink_speed  15                //闪烁速度设置

uint16_t blink_period_cnt = 0;
#define  blink_period 50                //闪烁周期设置

//指示灯闪烁
uint8_t blink(uint8_t led1,uint8_t led2,uint8_t led3,uint8_t led4)
{
    uint8_t blink_stop_flag = 0;
    
    if(blink_speed_cnt<blink_speed)
        blink_speed_cnt++;
    if(blink_speed_cnt==blink_speed) 
    { 
        if(blink_period_cnt<blink_period)
            blink_period_cnt++;
        if(blink_period_cnt==blink_period)
        {
            blink_period_cnt = 0;
            blink_stop_flag = 1;
        }
        blink_speed_cnt = 0;
    
        if(led1==1)       
            led_toggle(1);
        if(led2==1)
            led_toggle(2);
        if(led3==1)
            led_toggle(3);
        if(led4==1)
            led_toggle(4);
    }   
    return blink_stop_flag;    
}

//led闪烁
void blink_polling(void)
{    
    uint8_t blink_flag = 0;

    //上锁解锁指示灯闪烁
    if(fc.unlock_flag==1)               //解锁标志置位
    {
        blink_flag = blink(1,1,1,1);    //全部指示灯闪烁 
        if(blink_flag==1)
        {
            fc.unlock_flag = 0;         //清除标志位
        }
    }
    else if(fc.lock_flag==1)            //上锁标志置位
    {                                   
        blink_flag = blink(1,1,1,1);    //全部指示灯闪烁
        if(blink_flag==1)               
        {                               
            fc.lock_flag = 0;           //清除标志位
        }
    }    
    //进入加速度计校准模式标志置位且指示灯闪烁
    else if(cal_acc.start_flag==1)      
    {
        blink_flag = blink(0,1,0,1);    //2+4号电机闪烁
        if(blink_flag==1)
        {
            cal_acc.start_flag = 0;     //清除标志
        }
    }
    else if(cal_acc.single==1)          //加速度计第1面校准，该面校准完成后自动清除标志
    {
        blink_flag = blink(0,0,0,1);    //4号电机闪烁     
    }
    else if(cal_acc.single==2)          //加速度计第2面校准，该面校准完成后自动清除标志
    {
        blink_flag = blink(0,1,1,0);    //2+3号电机闪烁     
    }
    else if(cal_acc.single==3)          //加速度计第3面校准，该面校准完成后自动清除标志
    {
        blink_flag = blink(1,0,0,1);    //1+4号电机闪烁     
    }
    else if(cal_acc.single==4)          //加速度计第4面校准，该面校准完成后自动清除标志
    {
        blink_flag = blink(1,1,0,0);    //1+2号电机闪烁     
    }
    else if(cal_acc.single==5)          //加速度计第5面校准，该面校准完成后自动清除标志
    {
        blink_flag = blink(0,0,1,1);    //3+4号电机闪烁     
    }
    else if(cal_acc.single==6)          //加速度计第6面校准，该面校准完成后自动清除标志
    {
        blink_flag = blink(0,0,1,0);    //3号电机闪烁     
    }
    
    else if(cal_acc.all_finish_flag==1) //加速度计所有面完成校准
    {
        blink_flag = blink(1,1,1,1);    //1+2+3+4号电机闪烁
        if(blink_flag==1)
        {
            cal_acc.all_finish_flag = 0;//清除标志
        }        
    }
    //陀螺仪零偏校准
    else if(cal_gyro.start_flag==1)     
    {
        blink_flag = blink(1,0,1,0);    //1+3号电机闪烁 
        if(blink_flag==1)
        {
            cal_gyro.start_flag = 0;    //清除标志
        }        
    }
    //无线信号丢失交替闪烁
    else if(rc.signal_lost_flag==1)     
    {
        static uint8_t exchange_cnt = 0;
        
        exchange_cnt++;
        if(exchange_cnt<125)
            blink(1,0,1,0);
        else
            blink(0,1,0,1);
    }    
    //低压灯光闪烁
    else if(bat.danger_flag==1)         
    {
        blink(1,1,1,1);                 //1+2+3+4号电机闪烁
    }

    //进入定高模式指示灯闪烁（只有在解锁后拨定高才有效）
	else if(rc.high_led_flag==1&&fc.state==fc_unlock)		
	{
        blink_flag = blink(1,1,0,0);    //1+2号电机闪烁 
        if(blink_flag==1)
        {
            rc.high_led_flag = 0;    	//清除标志
        }        		
	}
    //退出定高模式指示灯闪烁（只有在解锁后拨定高才有效）
	else if(rc.high_led_flag==2&&fc.state==fc_unlock)		
	{
        blink_flag = blink(0,0,1,1);    //3+4号电机闪烁 
        if(blink_flag==1)
        {
            rc.high_led_flag = 0;    	//清除标志
        }       		
	}
    
    //进入定点模式指示灯闪烁
	else if(rc.fix_led_flag==1)		
	{
        blink_flag = blink(0,1,1,0);    //3+2号电机闪烁 
        if(blink_flag==1)
        {
            rc.fix_led_flag = 0;    	//清除标志
        }        		
	}
    //退出定点模式指示灯闪烁
	else if(rc.fix_led_flag==2)		
	{
        blink_flag = blink(1,0,0,1);    //1+4号电机闪烁 
        if(blink_flag==1)
        {
            rc.fix_led_flag = 0;    	//清除标志
        }       		
	}
    //关闭所以指示灯
    else 
    {
        led_off_all();
    }
}

//点亮led
void led_on(uint8_t led_num)
{
    if(led_num==1)
    {
        GPIOC->BRR  = GPIO_Pin_15;
    }
    if(led_num==2)
    {
        GPIOC->BRR  = GPIO_Pin_14;
    }
    if(led_num==3)
    {
        GPIOB->BRR  = GPIO_Pin_15;
    }
    if(led_num==4)
    {
        GPIOC->BRR  = GPIO_Pin_13;
    }
}

//熄灭led
void led_off(uint8_t led_num)
{
    if(led_num==1)
    {
        GPIOC->BSRR  = GPIO_Pin_15;
    }
    if(led_num==2)
    {
        GPIOC->BSRR  = GPIO_Pin_14;
    }
    if(led_num==3)
    {
        GPIOB->BSRR  = GPIO_Pin_15;
    }
    if(led_num==4)
    {
        GPIOC->BSRR  = GPIO_Pin_13;
    }

}

//led灯跳转
void led_toggle(uint8_t led_num)
{
    if(led_num==1)
    {
        GPIOC->ODR ^= GPIO_Pin_15;
    }
    if(led_num==2)
    {
        GPIOC->ODR ^= GPIO_Pin_14;
    }
    if(led_num==3)
    {
        GPIOB->ODR ^= GPIO_Pin_15;
    }
    if(led_num==4)
    {
        GPIOC->ODR ^= GPIO_Pin_13;
    }       
}



