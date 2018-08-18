#ifndef _spl06_h_
#define _spl06_h_


#include "stm32f10x.h"



//#define HW_ADR 0x77     //SDO HIGH OR NC

#define HW_ADR (0x76<<1)  //SDO LOW

#define PRESSURE_REG     0X00               // Pressure Data (PRS_Bn) £º 3bytes
#define TEMP_REG         0X03               // Temperature Data (TMP_Tn) £º 3bytes
#define PRS_CFG          0x06               // Pressure Configuration (PRS_CFG) 
#define TMP_CFG          0x07               // Temperature Configuration (TMP_CFG)
#define MEAS_CFG         0x08               // Sensor Operating Mode and Status (MEAS_CFG)
#define SPL06_REST_VALUE 0x09               // Interrupt and FIFO configuration (CFG_REG)
#define SPL06_REST_REG   0x0C               // Soft Reset and FIFO flush (RESET)
#define PRODUCT_ID       0X0D               // Product and Revision ID (ID)

#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2           
#define CONTINUOUS_P_AND_T      3

#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1

struct SPL_calib_param_t                    
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
};

struct SPL06_t 
{
    struct  SPL_calib_param_t calib_param;      /**<calibration data*/
    uint8_t chip_id;                             /**<chip id*/
    int32_t raw_pressure;                     
    int32_t raw_temperature;                  
    int32_t kP;                              
    int32_t kT;                              
};

typedef struct 
{
    float pressure;
    float pressure_offset;
    float temperature;
    float high;    
}_SPL_DATA;

typedef struct 
{
    uint8_t offset_ok   : 1;
    uint8_t finish      : 1;
    uint8_t             : 7;
}_SPL_FLAG;

typedef struct 
{
    uint8_t dat         : 2;
    uint8_t offset      : 8;
}_SPL_CNT;

extern _SPL_DATA spl;


void spl_start_temperature(void);
void spl_start_pressure(void);

void spl_get_raw_temp(void);
void spl_get_raw_pressure(void);

float spl_get_temperature(void);
float spl_get_pressure(void);

void spl_start_continuous(uint8_t mode); 
void spl_set_rate(uint8_t sensor, uint8_t sample_rate, uint8_t over_sample);
void spl_get_calibration_param(void);
void spl_init(void);
void get_spl_data(void);
void get_spl_status(void);












#endif

