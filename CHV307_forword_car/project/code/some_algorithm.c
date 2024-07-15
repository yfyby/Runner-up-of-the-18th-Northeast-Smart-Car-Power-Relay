/*
 * some_algorithm.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "some_algorithm.h"
#include "math.h"
/***************************************************
*                   һ��Kalman�˲���
*  ���ͣ�����     ���ϣ��ϵͳ�Ƚ��ȶ���Q��������һ��С��ֵ����������Ƚ�׼ȷ��RҲ��������һ��С��ֵ��
*  ����ֵ����     ��һ���棬���ϵͳԤ���Ǹ߶ȶ�̬�ģ����������������������ϴ��Q��ֵR���ܸ����ʡ�
*  ���ܣ��˲�
*  author: Kalman
һ�׿���������
Q:�����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
R:�����������R���󣬶�̬��Ӧ�����������ȶ��Ա��                  //���紫�������ȸߣ���R����С�㣬���ι۲�ֵ
***************************************************/
int16 First_Order_KalmanFilter(int16 para, Para_Type type, float kalman_Q, float kalman_R)
{
    static int16 X_Last[7] = {0, 0, 0, 0, 0, 0, 0};
    static int16 P_Last[7] = {0, 0, 0, 0, 0, 0, 0};
    float x_mid, x_now, p_mid, p_now, kg;

    x_mid = (float)X_Last[type];
    p_mid = (float)P_Last[type] + kalman_Q;

    kg = p_mid / (p_mid + kalman_R);    //����������
    x_now = x_mid + kg * ((float)para - x_mid);
    p_now = (1 - kg) * p_mid;           //�������ֵ
    P_Last[type] = (int16)p_now;
    X_Last[type] = (int16)x_now;
    return (int16)x_now;
}

/***************************************************
*                  ��С���˷����б��
*  ���ͣ�����
*  ����ֵ��б��
*  ���ܣ���С���˷����б��
***************************************************/
float Slope_Calculate(uint8 begin,uint8 end,float a[])
{
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    uint8 i = 0;
    static float resultlast=0;
    float result;
    for (i = begin; i < end; i++) {
    sum_x += i;
    sum_y += a[i];
    sum_xy += i * a[i];
    sum_xx += i * i;
   }
 if((end - begin) * sum_xx - sum_x * sum_x) //�жϳ����Ƿ�Ϊ��
 {
   result=((end - begin) * sum_xy - sum_x * sum_y) / ((end - begin) * sum_xx - sum_x * sum_x);
      }
 else
 {
  result=resultlast;
 }
   return result;
}
/***************************************************
*                   �������
*  ���ͣ�����
*  ��ڲ������������һ�����������С
*  ����ֵ�������
*  ���ܣ����
***************************************************/
float Arry_Sum(const float ar[], uint8 n)
{
    uint8 i;
    float sum = 0;

    for (i = 0; i < n; i++)
        sum += ar[i];

    return sum;
}

//==============================================================================
//  @brief      �޷�����
//  @param      ����ֵ  ����  ����
//==============================================================================
float LIMIT(float value,float pos,float neg)
{
    float mTemp = value;
    mTemp = (mTemp>pos)? pos:mTemp;
    mTemp = (mTemp<neg)? neg:mTemp;
    return mTemp;
}
//==============================================================================
//  @brief      ���ռ�ձ�˥��
//  @param      ��ʼռ�ձ�    ˥������    ��ǰ����   �ۼ�����
//==============================================================================
float Streeing_damping(float duty,float distance,float car_speed,float dt)
{
  static float ration=0;
    static float add_distance=0;
    float re_duty=0;
    add_distance+=car_speed*dt;
    ration=-(duty/distance);
    re_duty=duty+ration*add_distance;
    if(add_distance>=distance)
    {
        add_distance=0;
    }
    return re_duty;
}

