#include "controller.h"
#include "pid.h"
#include "imu.h"
#include "nrf24l01.h"
#include "pwm.h"
#include "mpu6050.h"
#include "parse_packet.h"
#include "fc_status.h"
#include "sins.h"
#include "flow.h"


_CONTROLLER_CNT controller_cnt = {0};
_THROTTLE_TYPE  throttle_type = {0};
_CONTROLLER_MODE _control = {0};

#define throttle_flying  10

_OUT_Motor Motor1 = {0};
_OUT_Motor Motor2 = {0};
_OUT_Motor Motor3 = {0};
_OUT_Motor Motor4 = {0};

//外环角度控制器
void angle_controller(void)
{
    static uint16_t yaw_init_cnt = 0;

    all.rol_angle.feedback = att.rol;               //姿态解算的rol角作为反馈量
    pid_controller(&all.rol_angle);                 //外环pid控制器      
    all.pit_angle.feedback = att.pit;               //姿态解算的pit角作为反馈量
    pid_controller(&all.pit_angle);                 //外环pid控制器    
    
    if(yaw_init_cnt<300)
        yaw_init_cnt++;
    else
    {
        if(rc.yaw==0)                               //偏航杆位于中位时进行双环控制
        {
            if(all.yaw_angle.expect==0)
            {
                all.yaw_angle.expect = att.yaw;     //最初时和偏航打舵之后，当前偏航角作为目标角度       
            }
            all.yaw_angle.feedback = att.yaw;
            pid_controller(&all.yaw_angle);         //外环pid控制器
            
            all.yaw_gyro.expect = all.yaw_angle.out; 
        }
        else                                        //偏航进行打舵时，遥感直接作为速度环期望    
        {
            all.yaw_angle.expect = 0;
            all.yaw_gyro.expect = rc.yaw*5;      
        }        
    }          
}
//内环角速度控制器
void gyro_controller(void)
{
    all.pit_gyro.expect = all.pit_angle.out;         //外环输出作为内环期望值
    all.pit_gyro.feedback = Mpu.deg_s.x;             //角速度值作为反馈量 
    pid_controller(&all.pit_gyro);                   //内环pid控制器     
                                                    
    all.rol_gyro.expect = all.rol_angle.out;         //外环输出作为内环期望值
    all.rol_gyro.feedback = Mpu.deg_s.y;             //角速度值作为反馈量
    pid_controller(&all.rol_gyro);                   //内环pid控制器 
                                                    
    all.yaw_gyro.feedback = Mpu.deg_s.z;             //角速度值作为反馈量   
    pid_controller(&all.yaw_gyro);                   //内环pid控制器    
}

#define thr_deadzone_min 350
#define thr_deadzone_max 650
#define vel_up     200
#define vel_down   200
#define throttle_max     1000
#define throttle_min     0

uint8_t high_mark_flag = 0;                     //进入定高时油门记录标志位
uint8_t fix_mark_flag = 0;
//控制模式选择
void _controller_detect(void)
{
    //自稳模式
    if(rc.high_flag==0)
    {
        _control.mode = 1;         
        
        all.rol_angle.expect = rc.rol;                  //遥控器数据作为rol的期望角
        all.pit_angle.expect = rc.pit;                  //遥控器数据作为pit的期望角         
        throttle_type.final_out = rc.thr;               //最终油门输出来至于遥控器油门
    }    
    //定高模式
    if(rc.high_flag==1)
    {
        if(high_mark_flag==0)                                                           //定高开启，开启当时保存一次当时高度值                                   
        {                                                                               
            throttle_type.mark_high = rc.thr;                                           //保存当前高度油门    
            all.pos_high.expect = sins.pos.z;                                           //设置一次期望高度    
            high_mark_flag = 1;                                                         
        }                                                                               
        if(high_mark_flag==1)                                                           
        {                                                                               
            _control.mode = 2;                                                          //定高模式   
            all.rol_angle.expect = rc.rol;                  //遥控器数据作为rol的期望角
            all.pit_angle.expect = rc.pit;                  //遥控器数据作为pit的期望角              
        }
    }
    //定点模式(在定高模式基础上才能开启定点)
    if(rc.fix_flag==1&&rc.high_flag==1)    
    {
        if(fix_mark_flag==0)
        {
            all.pos_fix_x.expect = flow.fix_pos.x;                                      //保存当前x轴方向位置
            all.pos_fix_y.expect = flow.fix_pos.y;                                      //保存当前y轴方向位置
            
            fix_mark_flag = 1;
        }
        if(fix_mark_flag==1)
        {
            _control.mode = 3;
        }
    }
}

//控制器
void _controller_perform(void)
{
    switch(_control.mode)
    {
        case 1: 
                //自稳模式    
                angle_controller();  
                gyro_controller();
                break;
        
        case 2: 
                //定高模式
                high_controller();
                angle_controller();  
                gyro_controller();
                break;
        
        case 3: //定点模式     
                high_controller();
                fix_controller();
                angle_controller();  
                gyro_controller();
                break;
        
        default :                           
                break;
    }
}

//油门遥感中位时保持当前高度，进行高度外环控制
void keep_alt(void)
{
    if(all.pos_high.expect == 0)
    {
        all.pos_high.expect = sins.pos.z;                                       //设置一次期望高度
    }
    controller_cnt.high++;
    if(controller_cnt.high>=2)                                                  //高度10ms控制周期
    {
        controller_cnt.high = 0;
        all.pos_high.feedback = sins.pos.z;                                     //当前惯导高度作为反馈量
        pid_controller(&all.pos_high);                                          //高度控制
        all.vel_high.expect = all.pos_high.out ;                                //高度外环输出作为内环速度期望
    }
}
//以期望速度向上，油门遥感超过中位，只有速度控制
void rise_alt(void)
{
    //上升速度    最大2m/s
    all.vel_high.expect = vel_up * (rc.thr_zone - thr_deadzone_max)/( throttle_max - thr_deadzone_max);
    all.pos_high.expect = 0;    
}

//以期望速度向下，油门遥感低于中位，只有速度控制
void drop_alt(void)
{
    //下降速度        
    all.vel_high.expect = vel_down * (rc.thr_zone - thr_deadzone_min)/( thr_deadzone_min - throttle_min);
    all.pos_high.expect = 0;       
}
    
//高度控制器
void high_controller(void)
{
    /** 竖直高度控制 **/
    
    
    //油门置于中位
    if(rc.thr_zone>=thr_deadzone_min&&rc.thr_zone<=thr_deadzone_max)                
    {
        _control.high_mode = 2;
    }
    //油门大于中位
    else if(rc.thr_zone>thr_deadzone_max)
    {  
        _control.high_mode = 3;
    }
    //油门小于中位
    else if(rc.thr_zone<thr_deadzone_min)
    {
        _control.high_mode = 1;
    }     

    switch(_control.high_mode)
    {
        case 1: //向下减速
                drop_alt(); 
                break;
        
        case 2: //保持当前高度
                keep_alt();
                break;
        
        case 3: //向上加速
                rise_alt();
                break;
        
        default:
                break;
    }
    
    /** 竖直速度控制 **/
    all.vel_high.feedback = sins.vel.z;                                             //惯导速度作为速度反馈量
    pid_controller(&all.vel_high);                                                  //速度5ms控制周期
    
    //最终油门输出 = 进入定高时油门(当时遥控器油门) + 高度控制器速度环输出
    throttle_type.final_out = throttle_type.mark_high + all.vel_high.out;          
}
uint8_t flow_fix_flag = 0;
//定点控制
void fix_controller(void)
{
	//无方向控制，保存方向杆中位
	if(rc.pit==0&&rc.rol==0)
	{
		//方向杆回中时保存当前位置为期望点位
		if((all.pos_fix_x.expect==0&&all.pos_fix_y.expect==0)||flow_fix_flag==0)
		{
			flow_fix_flag = 1;
            all.pos_fix_x.expect = -flow.fix_pos.x;                  //保存当前x轴方向位置
            all.pos_fix_y.expect = -flow.fix_pos.y;                  //保存当前y轴方向位置			
		}
		//光流当前点位作为反馈点位
		all.pos_fix_x.feedback =  -flow.fix_pos.x;  
		all.pos_fix_y.feedback =  -flow.fix_pos.y;  
		//位置控制器
		pid_controller(&all.pos_fix_x);
		pid_controller(&all.pos_fix_y);
		
		//位置输出作为速度期望
		all.vel_fix_x.expect = all.pos_fix_x.out;
		all.vel_fix_y.expect = all.pos_fix_y.out;
		
		//光流速度作为反馈速度
		all.vel_fix_x.feedback = -flow.fix_vel.x;
		all.vel_fix_y.feedback = -flow.fix_vel.y;
		
		pid_controller(&all.vel_fix_x);
		pid_controller(&all.vel_fix_y);
		
		all.rol_angle.expect = all.vel_fix_x.out;
		all.pit_angle.expect = all.vel_fix_y.out;

//		//位置输出作为速度期望
//		all.rol_angle.expect = all.pos_fix_x.out;
//		all.pit_angle.expect = all.pos_fix_y.out;        
	}
	//有方向打舵控制
	else
	{
		//方向杆直接控制速度期望
//		all.vel_fix_x.expect = 0;
//		all.vel_fix_y.expect = 0;		
		all.vel_fix_x.expect = rc.rol*20;
		all.vel_fix_y.expect = rc.pit*20;
		
		//光流速度作为反馈速度
		all.vel_fix_x.feedback = -flow.fix_vel.x;
		all.vel_fix_y.feedback = -flow.fix_vel.y;
		
		pid_controller(&all.vel_fix_x);
		pid_controller(&all.vel_fix_y);
		
		all.rol_angle.expect = all.vel_fix_x.out;
		all.pit_angle.expect = all.vel_fix_y.out;	
		
		//方向控速时，位置期望清零
		all.pos_fix_x.expect = 0;                 
		all.pos_fix_y.expect = 0;                 			
	}
}

//控制器pwm输出
void _controller_output(void)
{
    if(fc.state==fc_unlock)                         //解锁才输出            
    {
        if(rc.thr>throttle_flying)                  //大于起飞油门
        {
            Motor1.out = throttle_type.final_out + all.pit_gyro.out - all.rol_gyro.out + all.yaw_gyro.out;
            Motor2.out = throttle_type.final_out + all.pit_gyro.out + all.rol_gyro.out - all.yaw_gyro.out;   
            Motor3.out = throttle_type.final_out - all.pit_gyro.out + all.rol_gyro.out + all.yaw_gyro.out;
            Motor4.out = throttle_type.final_out - all.pit_gyro.out - all.rol_gyro.out - all.yaw_gyro.out;        
        }
        else                                        //小于起飞油门
        {
            Motor1.out = rc.thr;  
            Motor2.out = rc.thr;
            Motor3.out = rc.thr;
            Motor4.out = rc.thr;
            
            clear_integral(&all.pit_angle);         //清除积分
            clear_integral(&all.pit_gyro);          //清除积分        
            clear_integral(&all.rol_angle);         //清除积分    
            clear_integral(&all.rol_gyro);          //清除积分
            clear_integral(&all.yaw_angle);         //清除积分
            clear_integral(&all.yaw_gyro);          //清除积分
        }          
    }
    else                                        //未解锁
    {
        Motor1.out = 0;
        Motor2.out = 0;
        Motor3.out = 0;
        Motor4.out = 0;
        
        clear_integral(&all.pit_angle);         //清除积分
        clear_integral(&all.pit_gyro);          //清除积分        
        clear_integral(&all.rol_angle);         //清除积分    
        clear_integral(&all.rol_gyro);          //清除积分
        clear_integral(&all.yaw_angle);         //清除积分
        clear_integral(&all.yaw_gyro);          //清除积分        
    }
  
    pwm_out(Motor1.out,Motor2.out,Motor3.out,Motor4.out);
}


