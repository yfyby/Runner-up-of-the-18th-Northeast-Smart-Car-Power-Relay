/*
 * some_algorithm.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "some_algorithm.h"
#include "math.h"
/***************************************************
*                   一阶Kalman滤波器
*  类型：公有     如果希望系统比较稳定，Q可以设置一个小的值，如果测量比较准确，R也可以设置一个小的值。
*  返回值：无     另一方面，如果系统预计是高度动态的，或者如果测量有噪声，则较大的Q和值R可能更合适。
*  功能：滤波
*  author: Kalman
一阶卡尔曼参数
Q:过程噪声方差，Q增大，动态响应变快，收敛稳定性变坏
R:测量噪声方差，R增大，动态响应变慢，收敛稳定性变好                  //假如传感器精度高，则R可以小点，信任观测值
***************************************************/
int16 First_Order_KalmanFilter(int16 para, Para_Type type, float kalman_Q, float kalman_R)
{
    static int16 X_Last[7] = {0, 0, 0, 0, 0, 0, 0};
    static int16 P_Last[7] = {0, 0, 0, 0, 0, 0, 0};
    float x_mid, x_now, p_mid, p_now, kg;

    x_mid = (float)X_Last[type];
    p_mid = (float)P_Last[type] + kalman_Q;

    kg = p_mid / (p_mid + kalman_R);    //卡尔曼增益
    x_now = x_mid + kg * ((float)para - x_mid);
    p_now = (1 - kg) * p_mid;           //先验估计值
    P_Last[type] = (int16)p_now;
    X_Last[type] = (int16)x_now;
    return (int16)x_now;
}

/***************************************************
*                  最小二乘法拟合斜率
*  类型：公有
*  返回值：斜率
*  功能：最小二乘法拟合斜率
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
 if((end - begin) * sum_xx - sum_x * sum_x) //判断除数是否为零
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
*                   数组求和
*  类型：公有
*  入口参数：被计算的一组数，数组大小
*  返回值：数组和
*  功能：求和
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
//  @brief      限幅函数
//  @param      输入值  上限  下限
//==============================================================================
float LIMIT(float value,float pos,float neg)
{
    float mTemp = value;
    mTemp = (mTemp>pos)? pos:mTemp;
    mTemp = (mTemp<neg)? neg:mTemp;
    return mTemp;
}
//==============================================================================
//  @brief      舵机占空比衰减
//  @param      初始占空比    衰减距离    当前车速   累加周期
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

