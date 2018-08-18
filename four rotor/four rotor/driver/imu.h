#ifndef _imu_h_
#define _imu_h_

typedef struct 
{
    float ax;
    float ay;
    float az;
    
    float gx;
    float gy;
    float gz;
    
    float mx;
    float my;
    float mz;
    
    float pit;
    float rol;
    float yaw;
}_Attitude;


typedef struct
{
    float DCM[3][3];        //机体坐标系 -> 地理坐标系
    float DCM_T[3][3];      //地理坐标系 -> 机体坐标系
}_Matrix;

extern _Matrix Mat;

extern _Attitude att;
extern float sin_pit,sin_rol,sin_yaw;
extern float cos_pit,cos_rol,cos_yaw;

void mahony_update(float gx, float gy, float gz, float ax, float ay, float az) ;

void rotation_matrix(void);
void rotation_matrix_T(void);

void Matrix_ready(void);




#endif

