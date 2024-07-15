/*
 * mpu_angle.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef MPU_ANGLE_H_
#define MPU_ANGLE_H_

#include "zf_common_typedef.h"
//���������㷨��
extern float Angle_X_Final;         //���������
extern float Angle_Y_Final;         //���������
extern float temperature;           //�������¶�����
extern float aacx,aacy,aacz;        //���ٶȴ�����ԭʼ����
extern float gyrox,gyroy,gyroz;     //������ԭʼ����
void Angle_Calcu(void);
static float Mpu_Filter_GyroZ(float para);
static float Mpu_Filter_GyroX(float para);
static float Mpu_Filter_GyroY(float para);
float Calculate_Turn_Angle(float turn_speed);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);
void gyroOffset_init();
#endif /* MPU_ANGLE_H_ */
