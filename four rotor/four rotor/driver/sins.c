#include "sins.h"
#include "timer.h"
#include "spl06.h"
#include "nav.h"

_Time_test sins_high_time;

#define high_delay  2
_SINS sins = {0};

float TIME_CONTANST_ZER=3.0f;
#define K_ACC_ZER 	        (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))						
#define K_POS_ZER           (3.0f / TIME_CONTANST_ZER)

//竖直方向惯导融合
void sins_high(void)
{
    if(sins.high_start_cnt<500)                         //上电后初始加速度数据不稳定，等待2秒在进行融合
        sins.high_start_cnt++;
    if(sins.high_start_cnt>=500)
    {
        //融合周期检测
        time_check(&sins_high_time);
        //高度状态差 ，气压计与历史惯导高度差值（cm）,采用历史惯导数据原因：气压观测数据的延时造成与惯导不一致，故采用历史惯导值进行作差
        sins.state_delta.z = spl.high - sins.pos_history[high_delay].z;       
            
                                                        //三路积分反馈量纠正惯导
        sins.acc_cor.z +=  sins.state_delta.z * K_ACC_ZER * sins_high_time.delta_time_ms * 0.001f;
        sins.vel_cor.z +=  sins.state_delta.z * K_VEL_ZER * sins_high_time.delta_time_ms * 0.001f;
        sins.pos_cor.z +=  sins.state_delta.z * K_POS_ZER * sins_high_time.delta_time_ms * 0.001f;
        
        sins.last_acc.z = sins.acc.z ;                  //存储本次加速度作为上次加速度
        sins.acc.z = nav.acc[2] + sins.acc_cor.z;       //惯导加速度(本次加速度)
        
        //v = a*t                                       //速度增量
        sins.vel_delta.z = (sins.last_acc.z + sins.acc.z)*0.5f*sins_high_time.delta_time_ms*0.001f;  
        
        //s = v0*t + a*t^2/2  =  v0*t + v*t/2
                                                        //原始位置
        sins.pos_org.z += (sins.vel.z + 0.5f*sins.vel_delta.z)*sins_high_time.delta_time_ms*0.001f;
        sins.pos.z = sins.pos_org.z + sins.pos_cor.z;   //矫正后的位置
        
        sins.vel_org.z += sins.vel_delta.z;             //原始速度
        sins.vel.z = sins.vel_org.z + sins.vel_cor.z;   //矫正后的速度
        
                                                        //历史高度数据存储
        for(sins.pos_delay_cnt.z=pos_delay_num-1;sins.pos_delay_cnt.z>0;sins.pos_delay_cnt.z--)
        {
            sins.pos_history[sins.pos_delay_cnt.z].z = sins.pos_history[sins.pos_delay_cnt.z-1].z;
        }
        sins.pos_history[0].z = sins.pos.z;        
    }
    

}











