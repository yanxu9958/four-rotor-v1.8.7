#include "mpu6050.h"
#include "iic.h"
#include "systick.h"
#include "filter.h"
#include "flash.h"
#include "parse_packet.h"
#include "acc_cal.h"

S16_XYZ acc_raw = {0};                  //加速度计原始数据存储
S16_XYZ gyro_raw = {0};                 //陀螺仪原始数据存储
SI_F_XYZ acc_raw_f = {0};
SI_F_XYZ gyro_raw_f = {0};


SI_F_XYZ acc_att_lpf = {0};
SI_F_XYZ acc_fix_lpf = {0};

SI_F_XYZ acc_1_lpf = {0};
SI_F_XYZ acc_butter_lpf = {0};

SI_F_XYZ gyro_lpf = {0};
SI_F_XYZ gyro_offset = {0,0,0} ;         //陀螺仪零偏数据存储

_Mpu6050_data Mpu = {0};

//mpu初始化
void mpu6050_init(void)
{
	IIC_Write_One_Byte(0xD0,PWR_MGMT_1, 0x80);		
	delay_ms(100);													
	IIC_Write_One_Byte(0xD0,PWR_MGMT_1, 0x00);              //唤醒mpu		
 
    /* when DLPF is disabled( DLPF_CFG=0 or 7),陀螺仪输出频率 = 8kHz; 
       when DLPFis enabled,陀螺仪输出频率 = 1KHz 
       fs(采样频率) = 陀螺仪输出频率 / (1 + SMPLRT_DIV)*/	
    
	IIC_Write_One_Byte(0xD0,SMPLRT_DIV, 0x00);		        //sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz	
	IIC_Write_One_Byte(0xD0,MPU_CONFIG, 0x03);              //内部低通  acc:44hz	gyro:42hz
	IIC_Write_One_Byte(0xD0,GYRO_CONFIG, 0x18);			    // gyro scale  ：+-2000deg/s
	IIC_Write_One_Byte(0xD0,ACCEL_CONFIG, 0x10);			// Accel scale ：+-8g (65536/16=4096 LSB/g)    			   			
}

//两字节数据合成
static int get_data(unsigned char REG_Address)
{
	unsigned char H,L;
	H = IIC_Read_One_Byte(0xD0,REG_Address);
	L = IIC_Read_One_Byte(0xD0,REG_Address+1);

	return ((H<<8)+L);   
}
//get id
uint8_t get_mpu_id(void)
{
    u8 mpu_id;
    mpu_id = IIC_Read_One_Byte(0xD0,WHO_AM_I);
    
    return mpu_id;
}

//读取加速度计原始三轴数据
void get_acc_raw(void)
{
    acc_raw.x = get_data(ACCEL_XOUT_H);
    acc_raw.y = get_data(ACCEL_YOUT_H);
    acc_raw.z = get_data(ACCEL_ZOUT_H); 

	//椭球校准后的三轴加速度量
	acc_raw_f.x = (float)(cal_acc.K[0]*((float)acc_raw.x) - cal_acc.B[0]*one_g_to_acc);
	acc_raw_f.y = (float)(cal_acc.K[1]*((float)acc_raw.y) - cal_acc.B[1]*one_g_to_acc);
	acc_raw_f.z = (float)(cal_acc.K[2]*((float)acc_raw.z) - cal_acc.B[2]*one_g_to_acc);
}

_Butterworth_parameter gyro_30hz_parameter =
{
    //200hz---30hz
    1,  -0.7477891782585,    0.272214937925,
    0.1311064399166,   0.2622128798333,   0.1311064399166 
}; 

_Butterworth_data   gyro_butter_data[3];

//读取陀螺仪三轴数据量
void get_gyro_raw(void)
{
    gyro_raw.x = get_data(GYRO_XOUT_H) - gyro_offset.x;
    gyro_raw.y = get_data(GYRO_YOUT_H) - gyro_offset.y;
    gyro_raw.z = get_data(GYRO_ZOUT_H) - gyro_offset.z;        
    
    gyro_raw_f.x = (float)butterworth_lpf(((float)gyro_raw.x),&gyro_butter_data[0],&gyro_30hz_parameter);
    gyro_raw_f.y = (float)butterworth_lpf(((float)gyro_raw.y),&gyro_butter_data[1],&gyro_30hz_parameter);
    gyro_raw_f.z = (float)butterworth_lpf(((float)gyro_raw.z),&gyro_butter_data[2],&gyro_30hz_parameter);
}


//求取IIR滤波因子
void get_iir_factor(float *out_factor,float Time, float Cut_Off)
{
	*out_factor = Time /( Time + 1/(2.0f * PI * Cut_Off) );
}

//IIR低通滤波器(加速度)
void acc_iir_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out,float lpf_factor)
{
	acc_out->x = acc_out->x + lpf_factor*(acc_in->x - acc_out->x); 
	acc_out->y = acc_out->y + lpf_factor*(acc_in->y - acc_out->y); 
	acc_out->z = acc_out->z + lpf_factor*(acc_in->z - acc_out->z); 
}

//加速度计滤波参数  
_Butterworth_parameter acc_5hz_parameter =
{
  //200hz---1hz
//  1,   -1.955578240315,   0.9565436765112,
//  0.000241359049042, 0.000482718098084, 0.000241359049042
  //200hz---2hz
//  1,   -1.911197067426,   0.9149758348014,
//  0.0009446918438402,  0.00188938368768,0.0009446918438402
    //200hz---5hz
    1,                  -1.778631777825,    0.8008026466657,
    0.005542717210281,   0.01108543442056,  0.005542717210281
    //200hz---10hz
//    1,   -1.561018075801,   0.6413515380576,
//    0.02008336556421,  0.04016673112842,  0.02008336556421
    //200hz---15hz
//    1,   -1.348967745253,   0.5139818942197,
//    0.04125353724172,  0.08250707448344,  0.04125353724172
    //200hz---20hz
//    1,    -1.14298050254,   0.4128015980962,
//    0.06745527388907,   0.1349105477781,  0.06745527388907
    //200hz---30hz
//    1,  -0.7477891782585,    0.272214937925,
//    0.1311064399166,   0.2622128798333,   0.1311064399166 
}; 

_Butterworth_data   acc_butter_data[3];

//加速度计巴特沃斯低通
void acc_butterworth_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out)
{
    acc_out->x = butterworth_lpf(acc_in->x,&acc_butter_data[0],&acc_5hz_parameter);
    acc_out->y = butterworth_lpf(acc_in->y,&acc_butter_data[1],&acc_5hz_parameter);
    acc_out->z = butterworth_lpf(acc_in->z,&acc_butter_data[2],&acc_5hz_parameter);    
}

//原始加速度量转为 g
void get_acc_g(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out)
{
	acc_out->x = (float)(acc_in->x * acc_raw_to_g);
	acc_out->y = (float)(acc_in->y * acc_raw_to_g);
	acc_out->z = (float)(acc_in->z * acc_raw_to_g);
}

//#define filter_num 8

////滑动窗口滤波(对陀螺仪)
//void gyro_slid_filter(S16_XYZ *gyro_in,S16_XYZ *gyro_out)
//{
//    static int16_t filter_x[filter_num];
//    static int16_t filter_y[filter_num];
//    static int16_t filter_z[filter_num];

//	static int8_t filter_count;
//	int filter_sum_x = 0;
//    int filter_sum_y = 0;
//    int filter_sum_z = 0;

//	unsigned char i = 0;
//	
//	filter_x[filter_count] = gyro_in->x;
//	filter_y[filter_count] = gyro_in->y;
//	filter_z[filter_count] = gyro_in->z;

//	for(i=0;i<filter_num;i++)
//	{
//		filter_sum_x += filter_x[i];
//		filter_sum_y += filter_y[i];
//		filter_sum_z += filter_z[i];
//	}		
//	gyro_out->x = filter_sum_x / filter_num ;
//	gyro_out->y = filter_sum_y / filter_num ;
//	gyro_out->z = filter_sum_z / filter_num ;
//	
//	filter_count++;
//	if(filter_count == filter_num)
//		filter_count=0;
//}


//raw -> rad/s
void get_rad_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_out)
{
	gyro_out->x = (float)(gyro_in->x * gyro_raw_to_radian_s);
	gyro_out->y = (float)(gyro_in->y * gyro_raw_to_radian_s);
	gyro_out->z = (float)(gyro_in->z * gyro_raw_to_radian_s);
}

//raw -> deg/s
void get_deg_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_deg_out)
{
	gyro_deg_out->x = (float)(gyro_in->x * gyro_raw_to_deg_s);
	gyro_deg_out->y = (float)(gyro_in->y * gyro_raw_to_deg_s);
	gyro_deg_out->z = (float)(gyro_in->z * gyro_raw_to_deg_s);    
}

