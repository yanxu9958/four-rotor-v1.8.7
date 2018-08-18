#include "pwm.h"
#include "imath.h"


void pwm_init(void)
{
	GPIO_InitTypeDef GPIO_initStructure;
	TIM_TimeBaseInitTypeDef TIM_timeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);	
    
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_initStructure);

	//定时器周期 t = 1000*8/72 (us) = 1/9000(s)     故频率为 9khz
	
	//配置时基
	TIM_timeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //不分频
	TIM_timeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_timeBaseInitStructure.TIM_Period = 1000-1;						//设置ARR值
	TIM_timeBaseInitStructure.TIM_Prescaler = 8-1;						//时钟预分频值
	TIM_TimeBaseInit(TIM1,&TIM_timeBaseInitStructure);
	
	//配置OC输出通道
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;					//采用PWM模式1输出波形
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//设置CH通道的有效电平
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		//设置CH通道的空闲状态的电平
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//使能CH通道
	TIM_OCInitStructure.TIM_Pulse = 0;									//设置TIM1的CCR值
	
    TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);                  
	
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);                  
	
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);                  
	
	TIM_OC4Init(TIM1,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);                  
	
	//使能TIM的ARR和CRR，以及使能TIM定时器,开启pwm输出
	TIM_ARRPreloadConfig(TIM1,ENABLE);									
         
	TIM_Cmd(TIM1,ENABLE);
	
	//开始启动定时器输出pwm,这句是高级定时器才有的，输出pwm必须打开
	TIM_CtrlPWMOutputs(TIM1,ENABLE);  
}


void pwm_out(uint16_t pwm1,uint16_t pwm2,uint16_t pwm3,uint16_t pwm4)
{
	TIM1->CCR1 = throttle_limit(pwm1,0,1000);
	TIM1->CCR2 = throttle_limit(pwm2,0,1000);
	TIM1->CCR3 = throttle_limit(pwm3,0,1000);
	TIM1->CCR4 = throttle_limit(pwm4,0,1000);
}




