#include "spl06.h"
#include "iic.h"
#include "math.h"


static struct SPL06_t spl06;
static struct SPL06_t *p_spl06;

_SPL_DATA spl = {0};
_SPL_FLAG spl_flag = {0};
_SPL_CNT  spl_cnt = {0};


void spl_write(unsigned char hw_addr, unsigned char reg_addr, unsigned char val)
{
	IIC_Write_One_Byte(hw_addr,reg_addr,val);
}


uint8_t spl_read(unsigned char hw_addr, unsigned char reg_addr)
{
	uint8_t reg_data;

	reg_data = IIC_Read_One_Byte(hw_addr,reg_addr);
	return reg_data;
}

void spl_init(void)
{
    p_spl06 = &spl06;        
    
    p_spl06->raw_pressure = 0;  
    p_spl06->raw_temperature = 0;
    p_spl06->chip_id = 0x34;					/* read Chip Id */
    
    spl_get_calibration_param();     			//获取校准参数                     

    spl_set_rate(PRESSURE_SENSOR,128, 32);      //速度设置     
    spl_set_rate(TEMPERATURE_SENSOR,32, 8);  	//速度设置
	
    spl_start_continuous(CONTINUOUS_P_AND_T);       
} 


void spl_set_rate(uint8_t sensor, uint8_t sample_rate, uint8_t over_sample)
{ 
    uint8_t reg = 0;
    int32_t kPkT = 0;
	
    switch(sample_rate)
    { 
        case 2:
            reg |= (1<<5);
            break;
        case 4:
            reg |= (2<<5);
            break;
        case 8:
            reg |= (3<<5);
            break;
        case 16:
            reg |= (4<<5);
            break;
        case 32:
            reg |= (5<<5);
            break;
        case 64:
            reg |= (6<<5);
            break;
        case 128:
            reg |= (7<<5);
            break;
        case 1:
        default:
            break;
    }
    switch(over_sample)
    {
        case 2:
            reg |= 1;
            kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            kPkT = 7864320;
            break;
        case 16:
            kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            kPkT = 524288;
            break;
    }
    if(sensor == 0)
    {
        p_spl06->kP = kPkT;
        spl_write(HW_ADR, 0x06, reg);
        if(over_sample > 8)
        {
            reg = spl_read(HW_ADR, 0x09);
            spl_write(HW_ADR, 0x09, reg | 0x04);
        }
    }
    if(sensor == 1)
    {
        p_spl06->kT = kPkT;
        spl_write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature
        if(over_sample > 8)
        {
            reg = spl_read(HW_ADR, 0x09);
            spl_write(HW_ADR, 0x09, reg | 0x08);
        }
    }
}

void spl_get_calibration_param(void)
{
    uint32_t h;
    uint32_t m;
    uint32_t l;
    //读取校准参数c0
    h = spl_read(HW_ADR, 0x10);
    l = spl_read(HW_ADR, 0x11);
    p_spl06->calib_param.c0 = (int16_t)h<<4 | l>>4;
    p_spl06->calib_param.c0 = (p_spl06->calib_param.c0&0x0800)?(0xF000|p_spl06->calib_param.c0):p_spl06->calib_param.c0;
    //读取校准参数c1
    h = spl_read(HW_ADR, 0x11);
    l = spl_read(HW_ADR, 0x12);
    p_spl06->calib_param.c1 = (int16_t)(h&0x0F)<<8 | l;
    p_spl06->calib_param.c1 = (p_spl06->calib_param.c1&0x0800)?(0xF000|p_spl06->calib_param.c1):p_spl06->calib_param.c1;
    //读取校准参数c00
    h = spl_read(HW_ADR, 0x13);
    m = spl_read(HW_ADR, 0x14);
    l = spl_read(HW_ADR, 0x15);
    p_spl06->calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    p_spl06->calib_param.c00 = (p_spl06->calib_param.c00&0x080000)?(0xFFF00000|p_spl06->calib_param.c00):p_spl06->calib_param.c00;
    //读取校准参数c10
    h = spl_read(HW_ADR, 0x15);
    m = spl_read(HW_ADR, 0x16);
    l = spl_read(HW_ADR, 0x17);
    p_spl06->calib_param.c10 = (int32_t)h<<16 | (int32_t)m<<8 | l;
    p_spl06->calib_param.c10 = (p_spl06->calib_param.c10&0x080000)?(0xFFF00000|p_spl06->calib_param.c10):p_spl06->calib_param.c10;
    //读取校准参数c01
    h = spl_read(HW_ADR, 0x18);
    l = spl_read(HW_ADR, 0x19);
    p_spl06->calib_param.c01 = (int16_t)h<<8 | l;
    //读取校准参数c11
    h = spl_read(HW_ADR, 0x1A);
    l = spl_read(HW_ADR, 0x1B);
    p_spl06->calib_param.c11 = (int16_t)h<<8 | l;
    //读取校准参数c20
    h = spl_read(HW_ADR, 0x1C);
    l = spl_read(HW_ADR, 0x1D);
    p_spl06->calib_param.c20 = (int16_t)h<<8 | l;
    //读取校准参数c21
    h = spl_read(HW_ADR, 0x1E);
    l = spl_read(HW_ADR, 0x1F);
    p_spl06->calib_param.c21 = (int16_t)h<<8 | l;
    //读取校准参数c30
    h = spl_read(HW_ADR, 0x20);
    l = spl_read(HW_ADR, 0x21);
    p_spl06->calib_param.c30 = (int16_t)h<<8 | l;
}

//启动温度采集
void spl_start_temperature(void)
{
    spl_write(HW_ADR, 0x08, 0x02);
}
//启动气压采集
void spl_start_pressure(void)
{
    spl_write(HW_ADR, 0x08, 0x01);
}

/*
   Select node for the continuously measurement
  1: pressure; 2: temperature; 3: pressure and temperature
*/
void spl_start_continuous(uint8_t mode)
{
    spl_write(HW_ADR, 0x08, mode+4);
}

//获取原始温度值
void spl_get_raw_temp(void)
{
    uint8_t h[3] = {0};
		
    h[0] = spl_read(HW_ADR, 0x03);
    h[1] = spl_read(HW_ADR, 0x04);
    h[2] = spl_read(HW_ADR, 0x05);
    		
    p_spl06->raw_temperature = (int32_t)h[0]<<16 | (int32_t)h[1]<<8 | (int32_t)h[2];
    p_spl06->raw_temperature = (p_spl06->raw_temperature&0x800000) ? (0xFF000000|p_spl06->raw_temperature) : p_spl06->raw_temperature;
}
//获取原始气压值
void spl_get_raw_pressure(void)
{
    uint8_t h[3];
	
    h[0] = spl_read(HW_ADR, 0x00);
    h[1] = spl_read(HW_ADR, 0x01);
    h[2] = spl_read(HW_ADR, 0x02);
	
    p_spl06->raw_pressure = (int32_t)h[0]<<16 | (int32_t)h[1]<<8 | (int32_t)h[2];
    p_spl06->raw_pressure= (p_spl06->raw_pressure&0x800000) ? (0xFF000000|p_spl06->raw_pressure) : p_spl06->raw_pressure;
}
//获取温度校准值
float spl_get_temperature(void)
{
    float compensate_temp;
    float fTsc;

    fTsc = p_spl06->raw_temperature / (float)p_spl06->kT;                               //Calculate scaled measurement results.
    compensate_temp =  p_spl06->calib_param.c0 * 0.5 + p_spl06->calib_param.c1 * fTsc;  //Calculate compensated measurement results
    
    return compensate_temp;
}

//获取气压校准值
float spl_get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;           
    float compensate_press;

    //Calculate scaled measurement results.
    fTsc = p_spl06->raw_temperature / (float)p_spl06->kT;
    fPsc = p_spl06->raw_pressure / (float)p_spl06->kP;
    
    //intermediate variable 
    qua2 = p_spl06->calib_param.c10 + fPsc * (p_spl06->calib_param.c20 + fPsc* p_spl06->calib_param.c30);
    qua3 = fTsc * fPsc * (p_spl06->calib_param.c11 + fPsc * p_spl06->calib_param.c21);

    //Calculate compensated measurement results.
    compensate_press = p_spl06->calib_param.c00 + fPsc * qua2 + fTsc * p_spl06->calib_param.c01 + qua3;
    
    return compensate_press;
}

void get_spl_data(void)
{   
    spl_cnt.dat++;
    if(spl_cnt.dat==1)
    { 
        spl_get_raw_temp();                   
        spl.temperature = spl_get_temperature();  
    }
    else
    {
        spl_get_raw_pressure();                
        spl.pressure = spl_get_pressure(); 
        
        spl_cnt.dat = 0;
    }
}

void get_spl_high(float baro_pressure)       
{
    float Tempbaro = (float)(baro_pressure / spl.pressure_offset) * 1.0f;
    
    spl.high = 4433000.0f * (1 - powf((float)(Tempbaro),0.190295f));
}


//气压计数据状态更新
void get_spl_status(void)
{
    get_spl_data();                                     //获取温度、气压原始数据及其校准后的数据                            
    
    if(spl_cnt.offset<=100)                             //等待100*5ms
        spl_cnt.offset++;
    
    if(spl_cnt.offset==99)                              //读取一次初始零偏值
    {
        spl_flag.offset_ok = 1;
        
        spl.pressure_offset = spl.pressure;
        
        get_spl_high(spl.pressure);
    }
    if(spl_flag.offset_ok==1)                        
    {
        get_spl_high(spl.pressure);                     //高度更新
    }
}








