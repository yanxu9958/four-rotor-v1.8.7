#include "imu.h"
#include "imath.h"
#include "math.h"
#include "mpu6050.h"
#include "timer.h"
#include "ist8310.h"
#include "nav.h"
#include "sins.h"


_Matrix Mat = {0};

_Attitude att = {0};

#define kp 	    0.5f        //proportional gain governs rate of convergence to accelerometer/magnetometer 
#define ki 	    0.0001f     //integral gain governs rate of convergenceof gyroscope biases

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     //quaternion elements representing theestimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    //scaled integral error  

_Time_test att_time;

//Gyroscope units are radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
 
    if(ax*ay*az==0)
        return;
    
    //姿态解算时间检测
    time_check(&att_time);
    
    //[ax,ay,az]是机体坐标系下加速度计测得的重力向量(竖直向下)
	norm = invSqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	//VectorA = MatrixC * VectorB
	//VectorA ：参考重力向量转到在机体下的值
	//MatrixC ：地理坐标系转机体坐标系的旋转矩阵  
	//VectorB ：参考重力向量（0,0,1）      
    //[vx,vy,vz]是地理坐标系重力分向量[0,0,1]经过DCM旋转矩阵(C(n->b))计算得到的机体坐标系中的重力向量(竖直向下)    

    vx = Mat.DCM_T[0][2];
    vy = Mat.DCM_T[1][2];
    vz = Mat.DCM_T[2][2];
    
    //机体坐标系下向量叉乘得到误差向量，误差e就是测量得到的vˉ和预测得到的 v^之间的相对旋转。这里的vˉ就是[ax,ay,az]’,v^就是[vx,vy,vz]’
    //利用这个误差来修正DCM方向余弦矩阵(修正DCM矩阵中的四元素)，这个矩阵的作用就是将b系和n正确的转化直到重合。
    //实际上这种修正方法只把b系和n系的XOY平面重合起来，对于z轴旋转的偏航，加速度计无可奈何，
    //但是，由于加速度计无法感知z轴上的旋转运动，所以还需要用地磁计来进一步补偿。
    //两个向量的叉积得到的结果是两个向量的模与他们之间夹角正弦的乘积a×v=|a||v|sinθ,
    //加速度计测量得到的重力向量和预测得到的机体重力向量已经经过单位化，因而他们的模是1，
    //也就是说它们向量的叉积结果仅与sinθ有关，当角度很小时，叉积结果可以近似于角度成正比。

    ex = ay*vz - az*vy;
	ey = az*vx - ax*vz;
	ez = ax*vy - ay*vx;
 
    //对误差向量进行积分
	exInt = exInt + ex*ki;
	eyInt = eyInt + ey*ki;
	ezInt = ezInt + ez*ki;

    //通过调节Kp、Ki两个参数，可以控制加速度计修正陀螺仪积分姿态的速度。
	gx = gx + kp*ex + exInt;
	gy = gy + kp*ey + eyInt;
	gz = gz + kp*ez + ezInt;

    //一阶龙格库塔法更新四元数 
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)* att_time.delta_time_ms*0.0005f;
	q1 = q1 + ( q0*gx + q2*gz - q3*gy)* att_time.delta_time_ms*0.0005f;
	q2 = q2 + ( q0*gy - q1*gz + q3*gx)* att_time.delta_time_ms*0.0005f;
	q3 = q3 + ( q0*gz + q1*gy - q2*gx)* att_time.delta_time_ms*0.0005f; 

    //把上述运算后的四元数进行归一化处理。得到了物体经过旋转后的新的四元数。
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
    
	att.pit =  atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3) * rad_to_angle;
	att.rol =  asin(2.0f*(q0*q2 - q1*q3)) * rad_to_angle;       
 
    //z轴角速度积分的偏航角
    att.yaw += Mpu.deg_s.z  * att_time.delta_time_ms*0.001f;   

//    if((att.yaw<-90&&ist.angle>90)||(att.yaw>90&&ist.angle<-90))
//        att.yaw = - att.yaw*0.98f + ist.angle*0.02f;
//    else
//        att.yaw = att.yaw*0.98f + ist.angle*0.02f;
}

//旋转矩阵：机体坐标系 -> 地理坐标系
void rotation_matrix(void)
{
    Mat.DCM[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
    Mat.DCM[0][1] = 2.0f * (q1*q2 -q0*q3);
    Mat.DCM[0][2] = 2.0f * (q1*q3 +q0*q2);

    Mat.DCM[1][0] = 2.0f * (q1*q2 +q0*q3);
    Mat.DCM[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
    Mat.DCM[1][2] = 2.0f * (q2*q3 -q0*q1);
     
    Mat.DCM[2][0] = 2.0f * (q1*q3 -q0*q2);
    Mat.DCM[2][1] = 2.0f * (q2*q3 +q0*q1);
    Mat.DCM[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;   
}


//旋转矩阵的转置矩阵：地理坐标系 -> 机体坐标系
void rotation_matrix_T(void)
{
    Mat.DCM_T[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
    Mat.DCM_T[0][1] = 2.0f * (q1*q2 +q0*q3);    
    Mat.DCM_T[0][2] = 2.0f * (q1*q3 -q0*q2); 
    
    Mat.DCM_T[1][0] = 2.0f * (q1*q2 -q0*q3);
    Mat.DCM_T[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;  
    Mat.DCM_T[1][2] = 2.0f * (q2*q3 +q0*q1);    
    
    Mat.DCM_T[2][0] = 2.0f * (q1*q3 +q0*q2);
    Mat.DCM_T[2][1] = 2.0f * (q2*q3 -q0*q1);
    Mat.DCM_T[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;   
}

float sin_pit = 0,sin_rol = 0,sin_yaw = 0;
float cos_pit = 0,cos_rol = 0,cos_yaw = 0;

void Matrix_ready(void)
{
    rotation_matrix();                      //旋转矩阵更新
    rotation_matrix_T();                    //旋转矩阵的逆矩阵更新
    
    sin_pit = sin(att.pit * deg_to_rad);
    cos_pit = cos(att.pit * deg_to_rad);
    sin_rol = sin(att.rol * deg_to_rad);
    cos_rol = cos(att.rol * deg_to_rad);
    sin_yaw = sin(att.yaw * deg_to_rad);
    cos_yaw = cos(att.yaw * deg_to_rad);     

    //此处输入参数为三轴加速度数据，以便进行数据转换
	get_nav_acc(acc_fix_lpf);                          //导航系下三轴加速度  cm/s^2
    sins_high();                            //竖直方向惯导融合
}
