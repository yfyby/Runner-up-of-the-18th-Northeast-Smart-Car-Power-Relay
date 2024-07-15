/*
 * mpu_angle.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef MPU_ANGLE_H_
#define MPU_ANGLE_H_

#include "zf_common_typedef.h"
//卡尔曼解算法库
extern float Angle_X_Final;         //解算后俯仰角
extern float Angle_Y_Final;         //解算后横滚角
extern float temperature;           //陀螺仪温度数据
extern float aacx,aacy,aacz;        //加速度传感器原始数据
extern float gyrox,gyroy,gyroz;     //陀螺仪原始数据
void Angle_Calcu(void);
static float Mpu_Filter_GyroZ(float para);
static float Mpu_Filter_GyroX(float para);
static float Mpu_Filter_GyroY(float para);
float Calculate_Turn_Angle(float turn_speed);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);
void gyroOffset_init();
#endif /* MPU_ANGLE_H_ */
