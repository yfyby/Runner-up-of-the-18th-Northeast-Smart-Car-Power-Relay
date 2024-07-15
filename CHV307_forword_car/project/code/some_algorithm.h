/*
 * some_algorithm.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef SOME_ALGORITHM_H_
#define SOME_ALGORITHM_H_
#include "zf_common_typedef.h"
//一阶卡尔曼滤波参数类型枚举
typedef enum para_type {
    Left_Speed = 0,
    Right_Speed,
    Inductor_Hori_Left,
    Inductor_Vert_Left,
    Inductor_Middle,
    Inductor_Vert_Right,
    Inductor_Hori_Right,
} Para_Type;

int16 First_Order_KalmanFilter(int16 para, Para_Type type, float kalman_Q, float kalman_R);
float Arry_Sum(const float ar[], uint8 n);
float LIMIT(float value,float pos,float neg);
float Slope_Calculate(uint8 begin,uint8 end,float a[]);
float Streeing_damping(float duty,float distance,float car_speed,float dt);


#endif /* SOME_ALGORITHM_H_ */
