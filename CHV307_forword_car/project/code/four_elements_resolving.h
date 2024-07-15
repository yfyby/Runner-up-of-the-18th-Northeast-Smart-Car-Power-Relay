/*
 * four_elements_resolving.h
 *
 *  Created on: 2023��6��12��
 *      Author: linjias
 */

#ifndef FOUR_ELEMENTS_RESOLVING_H_
#define FOUR_ELEMENTS_RESOLVING_H_
#include "zf_common_headfile.h"

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} icm_param_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;

typedef struct {
    float pitch;    //������
    float roll;     //ƫ����
    float yaw;       //������
} euler_param_t;


typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_param_t;

extern euler_param_t eulerAngle;
extern icm_param_t icm_data;

void GyroOffset_Init(void);

float fast_sqrt(float x);

void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);

void ICM_getValues();

void ICM_getEulerianAngles(void);

#endif /* FOUR_ELEMENTS_RESOLVING_H_ */
