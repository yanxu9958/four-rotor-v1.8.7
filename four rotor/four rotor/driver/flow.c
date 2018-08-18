#include "flow.h"
#include "imu.h"
#include "imath.h"
#include "math.h"


_FLOW flow = {0};

//光流数据包接收
void flow_parse(u8 data_packet)  
{  
    u8 sum = 0;  
      
    switch(flow.step)  
    {  
        case 0:  
                //帧头
                if(data_packet==0xFE) 
                {  
                    flow.step = 1;  
                    flow.packet[flow.dat_cnt++] = data_packet;  
                }
                else 
                    flow.step = 0;  
        break;  
              
        case 1:  
                //数据长度
                if(data_packet==0x04)
                {  
                    flow.step = 2;  
                    flow.packet[flow.dat_cnt++] = data_packet;  
                }
                else 
                    flow.step = 0;  
        break;  
              
        case 2:  
                //数据接收
                flow.packet[flow.dat_cnt++] = data_packet;  
              
                if(flow.dat_cnt==9)  
                {  
                    flow.step = 0;  
                    flow.dat_cnt = 0;  
                   
                    //有效数据累加和
                    sum =  (flow.packet[2] + flow.packet[3] + flow.packet[4] + flow.packet[5]);  
                      
                    if((0xAA == data_packet) && (sum == flow.packet[6])&&(0x30 == flow.packet[7]))    
                    {   
                        flow.x = ( (int16_t)(*(flow.packet+3)<<8)|*(flow.packet+2) );  
                        flow.y = ( (int16_t)(*(flow.packet+5)<<8)|*(flow.packet+4) );  
                    }  
                }  
        break;  
              
        default:      
                flow.step = 0;  
                flow.dat_cnt = 0;  
        break;  
    }  
}  


//光流串口接收中断函数
void USART3_IRQHandler(void)
{  
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        flow.dat = USART_ReceiveData(USART3);
        
        flow_parse(flow.dat);
    }
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
}

void flow_iir_lpf(_XY_F *_flow_in,_XY_F *_flow_out,float lpf_factor)
{
	_flow_out->x = _flow_out->x + lpf_factor*(_flow_in->x - _flow_out->x); 
	_flow_out->y = _flow_out->y + lpf_factor*(_flow_in->y - _flow_out->y);   
}

//光流数据处理
void flow_handle(void)
{
	//光流数据积分得出位移
	flow.pos.x += flow.x;
	flow.pos.y += flow.y;
	
    //倾角位移补偿
    flow.att.x = 2450.0f*tan(att.rol*deg_to_rad);
    flow.att.y = 2450.0f*tan(att.pit*deg_to_rad);
    
    //倾角补偿后的位移输出
	flow.fix_pos.x = (flow.pos.x - flow.att.x)*0.08f;  
	flow.fix_pos.y = (flow.pos.y - flow.att.y)*0.08f;	
	
	//求微分速度  
	flow.vel.x = (flow.fix_pos.x - flow.last_pos.x)*30.0f;   
	flow.vel.y = (flow.fix_pos.y - flow.last_pos.y)*30.0f;  
//    flow.nav_pos.y = -flow.pos.x*sin_yaw + flow.pos.y*cos_yaw;
//	flow.nav_pos.x =  flow.pos.x*cos_yaw + flow.pos.y*sin_yaw;
	
////	flow.vel.x = (flow.pos.x - flow.last_pos.x)/0.005f;   
////	flow.vel.y = (flow.pos.y - flow.last_pos.y)/0.005f; 	
////	
////    flow.last_pos.x = flow.pos.x; 
////	flow.last_pos.y = flow.pos.y;  	
	
    flow.last_pos.x = flow.fix_pos.x; 
	flow.last_pos.y = flow.fix_pos.y;  
    
	//光流速度低通滤波  
    flow_iir_lpf(&flow.vel,&flow.fix_vel,flow.vel_lpf_factor);
}




