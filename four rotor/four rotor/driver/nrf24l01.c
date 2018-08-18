#include "nrf24l01.h"
#include "spi.h"
#include "systick.h"
#include "led.h"
#include "imath.h"
#include "pair_freq.h"


const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x1F,0x2E,0x3D,0x4C,0x5B}; 	
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0x1F,0x2E,0x3D,0x4C,0x5B};
 
void NRF24L01_Init(void)
{ 	
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);		

	//CE
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//IRQ
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	NRF_CE_L; 			
	SPI_CSN_H;			 	 
}

//无线是否在位检测
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0X18,0X18,0X18,0X18,0X18};
	u8 i;
   	 
	SPI_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);
	SPI_Read_Buf(TX_ADDR,buf,5); 
	for(i=0;i<5;i++)
        if(buf[i]!=0X18)
            break;	 							   
	if(i!=5)
        return 1;
	return 0;		 
}	 	 
//向寄存器写入值
u8 SPI_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
    
   	SPI_CSN_L;        
	
  	status = Spi_RW_Byte(reg);
  	Spi_RW_Byte(value);  
    
  	SPI_CSN_H;    
    
  	return(status);       			
}

//读取寄存器值
u8 SPI_Read_Reg(u8 reg)
{
	u8 reg_val;	  
    
 	SPI_CSN_L;  
	
  	Spi_RW_Byte(reg);   
  	reg_val = Spi_RW_Byte(0XFF);
	
  	SPI_CSN_H;   
    
  	return(reg_val);        
}	
//读出寄存器中连续len个字节长度的值
u8 SPI_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	
    
  	SPI_CSN_L;         
	
  	status = Spi_RW_Byte(reg);	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
        pBuf[u8_ctr]=Spi_RW_Byte(0XFF);
	
  	SPI_CSN_H; 
    
  	return status;        
}
//向寄存器写入连续len个字节的值
u8 SPI_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;
    
 	SPI_CSN_L;    
	
  	status = Spi_RW_Byte(reg);
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
		Spi_RW_Byte(*pBuf++);
	
  	SPI_CSN_H;   
    
  	return status;         
}			

//接收模式	   
void NRF24L01_RX_Mode(void)
{
	NRF_CE_L;	

    SPI_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)pair.addr,RX_ADR_WIDTH);
  	SPI_Write_Reg(NRF_WRITE_REG+EN_AA,0x00);      
  	SPI_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);	 
  	SPI_Write_Reg(NRF_WRITE_REG+RF_CH,pair.freq_channel[0]);	       
  	SPI_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
  	SPI_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07);
  	SPI_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);
    
  	NRF_CE_H; 
}						 
//发送模式
//void NRF24L01_TX_Mode(void)
//{														 
//	NRF_CE_L;	    
//	
//  	SPI_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);
//  	SPI_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); 
//	
//  	SPI_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);                  
//  	SPI_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); 
//  	SPI_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);
//  	SPI_Write_Reg(NRF_WRITE_REG+RF_CH,4);       
//  	SPI_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07);  
//  	SPI_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);   
//	
//	NRF_CE_H;
//}


//无线发送数据包
//u8 NRF24L01_TxPacket(u8 *txbuf)
//{
//	u8 sta;
//  
//	NRF_CE_L;
//  	SPI_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);
// 	NRF_CE_H;	   
//	
//	while(NRF_IRQ!=0);
//	
//	sta=SPI_Read_Reg(STATUS);  	   
//	SPI_Write_Reg(NRF_WRITE_REG+STATUS,sta); 
//	if(sta&MAX_TX)
//	{
//		SPI_Write_Reg(FLUSH_TX,0xff); 
//		return MAX_TX; 
//	}
//	if(sta&TX_OK)
//	{
//		return TX_OK;
//	}
//	return 0xff;
//}

//接收数据包
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    				

	sta = SPI_Read_Reg(STATUS); 	 					    //状态标志位
	
	SPI_Write_Reg(NRF_WRITE_REG+STATUS,sta); 	
	if(sta&RX_OK)											//接收成功
	{
		SPI_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		SPI_Write_Reg(FLUSH_RX,0xff);
		return 0; 
	}	   
	return 1;
}					    











