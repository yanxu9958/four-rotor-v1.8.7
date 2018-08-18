#include "usart2.h"
#include "mpu6050.h"
#include "imu.h"
#include "spl06.h"
#include "ist8310.h"
#include "adc.h"
#include "timer.h"
#include "fc_status.h"
#include "parse_packet.h"
#include "acc_cal.h"
#include "flash.h"
#include "nav.h"
#include "sins.h"
#include "flow.h"

//#pragma import(__use_no_semihosting)             
//                
//struct __FILE 
//{ 
//	int handle; 
//}; 

//FILE __stdout;       
//    
//void _sys_exit(int x) 
//{ 
//	x = x; 
//}
//    
int fputc(int ch, FILE *f)
{      
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET)
    {;}
		USART_SendData(USART2,(uint8_t)ch);
	return ch;
}

//上外上位机串口初始化
void usart2_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    //  TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    //  RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound; 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //8bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //stop bit is 1
	USART_InitStructure.USART_Parity = USART_Parity_No;                             //no parity
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //no Hardware Flow Control
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 //enable tx and rx
	USART_Init(USART2, &USART_InitStructure);
    
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);                                    //rx interrupt is enable
    
	USART_Cmd(USART2, ENABLE);    
}


/* 串口3的DMA通道配置                */
/* 存储器到外设传输方向              */
/* DMA_CHx:         DMA传输通道x     */
/* peripheral_addr: 外设地址         */
/* memory_addr:     内存地址         */
/* data_length:     传输的数据长度   */  
void USART2_DMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 peripheral_addr,u32 memory_addr,u16 data_length)
{
    DMA_InitTypeDef DMA_InitStructure;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	                        //时钟使能                     
                                                                                
    DMA_DeInit(DMA_CHx);                                                        //复位
                                                                                
    DMA_InitStructure.DMA_PeripheralBaseAddr = peripheral_addr;                 //外设地址     
    DMA_InitStructure.DMA_MemoryBaseAddr =memory_addr;                          //内存地址  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                          //外设作为传输的目的地
    DMA_InitStructure.DMA_BufferSize = data_length;                             //数据缓存大小                       
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //外设地址不自增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //内存地址自增   
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //外设数据宽度8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             //内存数据宽度8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               //正常模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                     //高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //无内存到内存传输                    
    DMA_Init(DMA_CHx, &DMA_InitStructure);                                 
}

/* 串口DMA数据发送 */
void USART2_DMA_SEND_DATA(u32 SendBuff,u16 len) 
{
	USART2_DMA_Config(DMA1_Channel7,(u32)&USART2->DR,(u32)SendBuff,len);
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);                              //使能串口DMA发送
    DMA_Cmd(DMA1_Channel7, ENABLE);                                             //使能DMA传输
}


/* 将大于一个字节的数据拆分成多个字节发送 */
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

uint8_t data_to_send[100];

/* 向匿名上位机发送姿态角，锁定状态 */
void ANO_DT_Send_Status(void)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;

	_temp = (int)(att.rol*100);                     //横滚角
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
    
	_temp = (int)(att.pit*100);                     //俯仰角    
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
    
	_temp = (int)(att.yaw*100);                     //偏航角
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp2 = (int32_t)(100*spl.high);       		//高度 
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

    data_to_send[_cnt++]=0x01;  					//飞行模式    01：姿态  02：定高  03：定点
    data_to_send[_cnt++]= fc.state;                 //锁定状态

	data_to_send[3] = _cnt-4;
	sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
    
    USART2_DMA_SEND_DATA((u32)(data_to_send),_cnt); //发送           
}

 void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
    u8 _cnt=0;
    vs16 _temp;
    u8 sum = 0;
    u8 i=0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;

    _temp = a_x;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = g_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = m_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    
    USART2_DMA_SEND_DATA((u32)(data_to_send),_cnt);
}

/* 遥控器通道数据 */
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
    u8 _cnt=0;
    u8 i=0;
    u8 sum = 0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x03;
    data_to_send[_cnt++]=0;
    
    data_to_send[_cnt++]=BYTE1(thr);
    data_to_send[_cnt++]=BYTE0(thr);
    
    data_to_send[_cnt++]=BYTE1(yaw);
    data_to_send[_cnt++]=BYTE0(yaw);
    
    data_to_send[_cnt++]=BYTE1(rol);
    data_to_send[_cnt++]=BYTE0(rol);
    
    data_to_send[_cnt++]=BYTE1(pit);
    data_to_send[_cnt++]=BYTE0(pit);
    
    data_to_send[_cnt++]=BYTE1(aux1);
    data_to_send[_cnt++]=BYTE0(aux1);
    
    data_to_send[_cnt++]=BYTE1(aux2);
    data_to_send[_cnt++]=BYTE0(aux2);
    
    data_to_send[_cnt++]=BYTE1(aux3);
    data_to_send[_cnt++]=BYTE0(aux3);
    
    data_to_send[_cnt++]=BYTE1(aux4);
    data_to_send[_cnt++]=BYTE0(aux4);
    
    data_to_send[_cnt++]=BYTE1(aux5);
    data_to_send[_cnt++]=BYTE0(aux5);
    
    data_to_send[_cnt++]=BYTE1(aux6);
    data_to_send[_cnt++]=BYTE0(aux6);

    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++]=sum;
    
    USART2_DMA_SEND_DATA((u32)(data_to_send),_cnt);
}

void ANO_DT_Send_Power(float votage, float current)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = (uint16_t)100*votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = (uint16_t)100*current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	USART2_DMA_SEND_DATA((u32)(data_to_send), _cnt);
}
//用户自定义数据发送：1-5：int16t类型数据     6-10：float类型数据
 void ANO_DT_Send_User( s16 user1,s16 user2,s16 user3,s16 user4,s16 user5,
                        float user6,float user7,float user8,float user9,float user10,
                        float user11,float user12,float user13,float user14,float user15)
{
    u8 _cnt=0;
    vs16 _temp;
    float _temp_f;
    
    u8 sum = 0;
    u8 i=0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xF1;
    data_to_send[_cnt++]=0;
    
    //1-5  int16t类型数据
    _temp = user1;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = user2;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = user3;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = user4;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = user5;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    //6-10 ：float类型数据
    _temp_f = user6;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user7;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user8;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user9;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user10;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);


    _temp_f = user11;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user12;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user13;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user14;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user15;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);

    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    
    USART2_DMA_SEND_DATA((u32)(data_to_send),_cnt);
}

void ANO_DMA_SEND_DATA(void)
{
    static uint8_t ANO_debug_cnt = 0;
    ANO_debug_cnt++;
    if(ANO_debug_cnt==1)
    {
        ANO_DT_Send_Status();
    }
    else if(ANO_debug_cnt==2)
    { 
        ANO_DT_Send_Senser((int16_t)acc_raw.x,(int16_t)acc_raw.y,(int16_t)acc_raw.z, 
                           (int16_t)gyro_raw.x,(int16_t)gyro_raw.y,(int16_t)gyro_raw.z,
                           (int16_t)nav.acc[2],(int16_t)sins.acc.z,(int16_t)sins.vel.z);
    }
    else if(ANO_debug_cnt==3)
    {
        ANO_DT_Send_RCData(rc.thr+1000,rc.yaw+1500,rc.rol+1500,rc.pit+1500,rc.key_l+1500,rc.key_r+1500,0,0,0,0);
    }
    else if(ANO_debug_cnt==4)
    {
        ANO_DT_Send_Power((bat.voltage),0);
    }
    else if(ANO_debug_cnt==5)
    {
        //前五个数据为int16 ，后10个float
        ANO_DT_Send_User(15,20,25,30,35,
                        /*acc_raw_f.z,acc_1_lpf.z,acc_butter_lpf.z,0,0,*/
                         nav.acc[2],sins.acc.z,sins.vel.z,spl.high,sins.pos.z,
                         flow.fix_pos.x,flow.att.x,flow.pos.x,flow.fix_vel.x,flow.vel.x);
    }
    else if(ANO_debug_cnt==6)
    {
        ANO_debug_cnt = 0;
    }
}





