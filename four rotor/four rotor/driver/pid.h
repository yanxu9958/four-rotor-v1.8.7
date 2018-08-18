#ifndef _pid_h_
#define _pid_h_

#include "stm32f10x.h"

typedef struct
{
    float err;
    float err_last;

    float expect;
    float feedback;

    float kp;
    float ki;
    float kd;

    float integral;
    float integral_max;

    float out;
    float out_max;
}_PID;


typedef struct
{
    //姿态外环
    _PID pit_angle;
    _PID rol_angle;
    _PID yaw_angle;
    //姿态内环
    _PID pit_gyro;      
    _PID rol_gyro;
    _PID yaw_gyro;
    //竖直定高
    _PID acc_high;
    _PID vel_high;
    _PID pos_high;
    
    //定点
    _PID acc_fix_x;
    _PID vel_fix_x;
    _PID pos_fix_x;
  
    //定点
    _PID acc_fix_y;
    _PID vel_fix_y;
    _PID pos_fix_y;
           
}_ALL_PID;


extern _ALL_PID all;

float pid_controller(_PID *controller);
void all_pid_init(void);
void clear_integral(_PID *controller);


#endif
