/*
 * mpu_angle.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "mpu_angle.h"
#include "math.h"
#include "zf_device_icm20602.h"
#include "zf_driver_delay.h"
/***************************************************
*  姿态解算文件
*  author: USTH_斜杠青年
***************************************************/
float temp_gyrox,temp_gyroy,temp_gyroz;     //加速度传感器原始数据
float gyrox,gyroy,gyroz;    //陀螺仪原始数据
uint8 flag_offset=0;     //角速度调0完成标志位

extern float Accel_x;        //X轴加速度
extern float Accel_y;       //Y轴加速度
extern float Accel_z;      //Z轴加速度
extern float Gyro_x;      //X轴陀螺仪
extern float Gyro_y;     //Y轴陀螺仪
extern float Gyro_z;    //Z轴陀螺仪

float Angle_x_temp=0;       //由加速度计算的x倾斜角度
float Angle_y_temp=0;       //由加速度计算的y倾斜角度
float Angle_X_Final=0;      //X最终倾斜角度//俯仰角
float Angle_Y_Final=0;      //Y最终倾斜角度//横滚角
float Turn_speed=0;       //角速度,用做轮速差使用，舵机控制
float  Gyro_Y_Offset = 0, Gyro_X_Offset = 0, Gyro_Z_Offset=0;    //姿态零位值,陀螺仪具有累计误差,需要开机时修正
//角度计算
void Angle_Calcu(void)
{
    //float accx,accy,accz;    //三方向角加速度值
    static float ratio = 0.048f;    //拟合系数
    static float temp_turn_speed[3] = {0.0f, 0.0f, 0.0f};

    icm20602_get_acc();    //获取三轴加速值
    icm20602_get_gyro();   //获取三轴陀螺仪值
//    accx=icm20602_acc_transition(Accel_x);
//    accy=icm20602_acc_transition(Accel_y);
//    accz=icm20602_acc_transition(Accel_z);
//    //加速度反正切公式计算三个轴和水平面坐标系之间的夹角
//    Angle_x_temp=(atan(accy/accz))*57.32;//*180/3.14
//    Angle_y_temp=(atan(accx/accz))*57.32;//*180/3.14
    if(flag_offset==1){    //角速度调0完成
    temp_gyrox=icm20602_gyro_transition(Gyro_x)-Gyro_X_Offset;
    temp_gyroy=icm20602_gyro_transition(Gyro_y)-Gyro_Y_Offset;
    temp_gyroz=icm20602_gyro_transition(Gyro_z)-Gyro_Z_Offset;
    }else{
    temp_gyrox=icm20602_gyro_transition(Gyro_x);
    temp_gyroy=icm20602_gyro_transition(Gyro_y);
    temp_gyroz=icm20602_gyro_transition(Gyro_z);
    }
    gyrox=Mpu_Filter_GyroX(temp_gyrox);
    gyroy=Mpu_Filter_GyroY(temp_gyroy);
    gyroz=Mpu_Filter_GyroZ(temp_gyroz);
    Turn_speed = (gyroz) * ratio;    //Gryo_Y拟合公式
    //对转向角速度进行中值滤波
    temp_turn_speed[2] = temp_turn_speed[1];
    temp_turn_speed[1] = temp_turn_speed[0];
    temp_turn_speed[0] = Turn_speed;
    if (temp_turn_speed[0] > temp_turn_speed[1])
    {
        if (temp_turn_speed[1] > temp_turn_speed[2])
            Turn_speed = temp_turn_speed[1];
        else if (temp_turn_speed[1] < temp_turn_speed[2] && temp_turn_speed[2] < temp_turn_speed[0])
            Turn_speed = temp_turn_speed[2];
        else
            Turn_speed = temp_turn_speed[0];
    }
    else
    {
        if (temp_turn_speed[2] > temp_turn_speed[1])
            Turn_speed = temp_turn_speed[1];
        else if (temp_turn_speed[1] > temp_turn_speed[2] && temp_turn_speed[2] > temp_turn_speed[0])
            Turn_speed = temp_turn_speed[2];
        else
            Turn_speed = temp_turn_speed[0];
    }
  //Kalman_Filter_X(Angle_x_temp,gyrox);  //卡尔曼滤波计算X倾角,俯仰角
  //Kalman_Filter_Y(Angle_y_temp,gyroy);  //卡尔曼滤波计算Y倾角,横滚角
}
/***************************************************
*                 姿态滤波器 - Gyro_Z
*  类型：私有
*  返回值：int16_滤波后的Gyro_X值
*  功能：滤波
***************************************************/
static float Mpu_Filter_GyroZ(float para)
{
    int32 para_filitered = 0;
    static float Filter_GyroZ[3] = {0, 0, 0};

    Filter_GyroZ[2] = Filter_GyroZ[1];
    Filter_GyroZ[1] = Filter_GyroZ[0];
    Filter_GyroZ[0] = para;

    para_filitered = (Filter_GyroZ[0] * 6 + Filter_GyroZ[1] * 3 + Filter_GyroZ[2] * 1) / 10;

    return para_filitered;
}

/***************************************************
*                 姿态滤波器 - Gyro_X
*  类型：私有
*  返回值：int16_滤波后的Gyro_X值
*  功能：滤波
***************************************************/
static float Mpu_Filter_GyroX(float para)
{
    float para_filitered = 0;
    static float Filter_GyroX[3] = {0, 0, 0};

    Filter_GyroX[2] = Filter_GyroX[1];
    Filter_GyroX[1] = Filter_GyroX[0];
    Filter_GyroX[0] = para;

    para_filitered = (Filter_GyroX[0] * 6 + Filter_GyroX[1] * 3 + Filter_GyroX[2] * 1) / 10;

    return para_filitered;
}

/***************************************************
*                 姿态滤波器 - Gyro_Y
*  类型：私有
*  返回值：int16_滤波后的Gyro_X值
*  功能：滤波
***************************************************/
static float Mpu_Filter_GyroY(float para)
{
    float para_filitered = 0;
    static float Filter_GyroY[3] = {0, 0, 0};

    Filter_GyroY[2] = Filter_GyroY[1];
    Filter_GyroY[1] = Filter_GyroY[0];
    Filter_GyroY[0] = para;

    para_filitered = (Filter_GyroY[0] * 6 + Filter_GyroY[1] * 3 + Filter_GyroY[2] * 1) / 10;

    return para_filitered;
}
/***************************************************
*                修正传感器初始值
*  类型：公有
*  返回值：无
*  功能：发车前调零，调零完成则可以发车，即一开始如果有0漂，通过这个采集一段时间0漂再减去即可
*  使用：初始化时进行调用
***************************************************/
void gyroOffset_init()
{
    float gyro_y_offset_sum = 0, gyro_x_offset_sum = 0,gyro_z_offset_sum=0;
    for (uint16_t i = 0; i < 100; ++i) {
        icm20602_get_gyro();
        gyro_x_offset_sum  +=icm20602_gyro_transition(Gyro_x);
        gyro_y_offset_sum  +=icm20602_gyro_transition(Gyro_y);
        gyro_z_offset_sum  +=icm20602_gyro_transition(Gyro_z);
        system_delay_ms(10);
    }
    Gyro_Z_Offset = (float)(gyro_z_offset_sum / 100.0);
    Gyro_Y_Offset = (float)(gyro_y_offset_sum / 100.0);
    Gyro_X_Offset = (float)(gyro_x_offset_sum / 100.0);
    flag_offset   = 1;//完成调零
}
/***************************************************
*               计算偏航角度
*  类型：公有
*  入口参数：角速度
*  返回值：偏航角
*  功能：计算偏航角度
***************************************************/
float Calculate_Turn_Angle(float turn_speed)
{
    static float Angle_Car = 0.0f;
    Angle_Car += turn_speed * 0.02; //20ms的采集时间
    return Angle_Car;
}

//卡尔曼参数
float Q_angle = 0.001;      //角度数据置信度，角度噪声的协方差
float Q_gyro  = 0.003;      //角速度数据置信度，角速度噪声的协方差
float R_angle = 0.5;        //加速度计测量噪声的协方差
float dt      = 0.02;       //滤波算法计算周期，由定时器定时20ms
char  C_0     = 1;          //H矩阵值
float Q_bias, Angle_err;    //Q_bias:陀螺仪的偏差  Angle_err:角度偏量
float PCt_0, PCt_1, E;      //计算的过程量
float K_0, K_1, t_0, t_1;   //卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
float P[4] ={0,0,0,0};  //过程协方差矩阵的微分矩阵，中间变量
float PP[2][2] = { { 1, 0 },{ 0, 1 } };//过程协方差矩阵P

void Kalman_Filter_X(float Accel,float Gyro) //卡尔曼函数
{
    //步骤一，先验估计
    //公式：X(k|k-1) = AX(k-1|k-1) + BU(k)
    //X = (Angle,Q_bias)
    //A(1,1) = 1,A(1,2) = -dt
    //A(2,1) = 0,A(2,2) = 1
    Angle_X_Final += (Gyro - Q_bias) * dt; //状态方程,角度值等于上次最优角度加角速度减零漂后积分

    //步骤二，计算过程协方差矩阵的微分矩阵
    //公式：P(k|k-1)=AP(k-1|k-1)A^T + Q
    //Q(1,1) = cov(Angle,Angle) Q(1,2) = cov(Q_bias,Angle)
    //Q(2,1) = cov(Angle,Q_bias)    Q(2,2) = cov(Q_bias,Q_bias)
    P[0]= Q_angle - PP[0][1] - PP[1][0];
    P[1]= -PP[1][1];// 先验估计误差协方差
    P[2]= -PP[1][1];
    P[3]= Q_gyro;


    PP[0][0] += P[0] * dt;
    PP[0][1] += P[1] * dt;
    PP[1][0] += P[2] * dt;
    PP[1][1] += P[3] * dt;
    Angle_err = Accel - Angle_X_Final;  //Z(k)先验估计 计算角度偏差

    //步骤三，计算卡尔曼增益
    //公式：Kg(k)= P(k|k-1)H^T/(HP(k|k-1)H^T+R)
    //Kg = (K_0,K_1) 对应Angle,Q_bias增益
    //H = (1,0) 可由z=HX+v求出z:Accel
    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];
    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    //步骤四，后验估计误差协方差
    //公式：P(k|k)=(I-Kg(k)H)P(k|k-1)
    //也可写为：P(k|k)=P(k|k-1)-Kg(k)HP(k|k-1)
    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    //步骤五，计算最优角速度值
    //公式：X(k|k)= X(k|k-1)+Kg(k)(Z(k)-X(k|k-1))
    Angle_X_Final += K_0 * Angle_err;    //后验估计，给出最优估计值
    Q_bias        += K_1 * Angle_err;    //后验估计，跟新最优估计值偏差
    Gyro_x         = Gyro - Q_bias;
}

void Kalman_Filter_Y(float Accel,float Gyro)
{
    Angle_Y_Final += (Gyro - Q_bias) * dt;
    P[0]=Q_angle - PP[0][1] - PP[1][0];
    P[1]=-PP[1][1];
    P[2]=-PP[1][1];
    P[3]=Q_gyro;
    PP[0][0] += P[0] * dt;
    PP[0][1] += P[1] * dt;
    PP[1][0] += P[2] * dt;
    PP[1][1] += P[3] * dt;
    Angle_err = Accel - Angle_Y_Final;
    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];
    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;
    Angle_Y_Final   += K_0 * Angle_err;
    Q_bias  += K_1 * Angle_err;
    Gyro_y   = Gyro - Q_bias;
}



