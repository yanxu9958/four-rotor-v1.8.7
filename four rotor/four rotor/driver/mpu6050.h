#ifndef _mpu6050_h_
#define _mpu6050_h_

#include "stm32f10x.h"

typedef struct
{
	signed short x;
	signed short y;
	signed short z;
}S16_XYZ;

typedef struct
{
	uint16_t x;
	uint16_t y;
	uint16_t z;
}U16_XYZ;

typedef struct
{
	signed int x;
	signed int y;
	signed int z;
}S32_XYZ;

typedef struct
{
	float x;
	float y;
	float z;
}SI_F_XYZ;


typedef struct 
{
    SI_F_XYZ deg_s;
    SI_F_XYZ rad_s;
    SI_F_XYZ acc_g;

	float att_acc_factor;
    float fix_acc_factor;
    
}_Mpu6050_data;


#define	mpu_address	    0x68

#define	SMPLRT_DIV		0x19
#define	MPU_CONFIG		0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define FIFO_EN			0x23
#define INT_PIN_CFG		0x37
#define INT_ENABLE		0x38
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define USER_CTRL		0x6A
#define	PWR_MGMT_1		0x6B
#define PWR_MGMT_2		0x6C
#define	WHO_AM_I		0x75

#define PI                      3.1415926535898f
#define gyro_raw_to_deg_s       0.0609756097561f   //+-250°/s:131LSB/°/s   +-500°/s:65.5LSB/°/s   +-1000°/s:32.8LSB/°/s    +-2000°/s:16.4LSB/°/s(本次所以)
#define acc_raw_to_g            0.000244140625f    //+-2g : 16384LSB/g     +-4g : 8192LSB/g   +-8g : 4096LSB/g(本次所用)   +-16g : 2048LSB/g  

#define deg_to_rad              (PI / 180.0f)
#define rad_to_angle            (180.0f / PI)                    
#define gyro_raw_to_radian_s	(gyro_raw_to_deg_s * deg_to_rad)

#define accmax_1g      4096
#define gravity_mss    9.80665f                    // acceleration due to gravity in m/s/s

#define acc_to_1g      gravity_mss / accmax_1g
#define one_g_to_acc   accmax_1g / gravity_mss

void mpu6050_init(void);
uint8_t get_mpu_id(void);
void get_acc_raw(void);
void get_gyro_raw(void);
void get_gyro_offset(void);

void get_rad_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_out);
void get_deg_s(SI_F_XYZ *gyro_raw_filter,SI_F_XYZ *gyro_deg_out);
void get_acc_g(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out);

void get_iir_factor(float *out_factor,float Time, float Cut_Off);
void acc_iir_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out,float lpf_factor);
void acc_butterworth_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out);

extern _Mpu6050_data Mpu;
extern S16_XYZ  gyro_raw;
extern S16_XYZ  acc_raw;
extern SI_F_XYZ acc_1_lpf;
extern SI_F_XYZ acc_bt_lpf;
extern SI_F_XYZ gyro_lpf;
extern SI_F_XYZ gyro_offset;

extern SI_F_XYZ acc_raw_f;
extern SI_F_XYZ gyro_raw_f;

extern SI_F_XYZ acc_butter_lpf;

extern SI_F_XYZ acc_att_lpf;
extern SI_F_XYZ acc_fix_lpf;

#endif
