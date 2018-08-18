#include "ist8310.h"
#include "iic.h"
#include "imu.h"
#include "math.h"

_S16_UNIT_XYZ ist_raw = {0};

_IST8310 ist = {0};

void IST8310_init(void)
{
    IIC_Write_One_Byte(IST8310_SLAVE_ADDRESS,0x41,0x24);   //开启16x内部平均
    IIC_Write_One_Byte(IST8310_SLAVE_ADDRESS,0x42,0xC0);   //Set/Reset内部平均    
}


/* 读取两个字节数据 */
/* 返回值：16位数据 */
static int16_t get_data(uint8_t REG_Address)
{
	uint8_t Hd,Ld;
    
	Ld = IIC_Read_One_Byte(IST8310_SLAVE_ADDRESS,REG_Address);
	Hd = IIC_Read_One_Byte(IST8310_SLAVE_ADDRESS,REG_Address+1);
    
	return (Hd<<8) + Ld;
}    

void get_ist_raw(void)
{
    ist_raw.x = get_data(0x03);
    ist_raw.y = get_data(0x05);
    ist_raw.z = get_data(0x07);    
}

void get_ist_data(void)
{
    static uint8_t ist_cnt = 0;

    ist_cnt++;
    if(ist_cnt==1)
    {
        IIC_Write_One_Byte(IST8310_SLAVE_ADDRESS,IST8310_REG_CNTRL1,0x01);      //Single Measurement Mode
    }
    else if(ist_cnt==3)      
    {
        get_ist_raw();
        
        ist_cnt=0;
    }

    att.mx =  ist_raw.x;
    att.my = -ist_raw.y;
    att.mz =  ist_raw.z;
    
    //倾角补偿
    ist.thx = att.mx * cos_rol + att.mz * sin_rol;
    ist.thy = att.mx * sin_pit * sin_rol + att.my * cos_pit - att.mz * cos_rol * sin_pit;

    ist.angle = atan2(ist.thx,ist.thy)*57.296f;    
}

