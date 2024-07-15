/*
 * mypid.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "mypid.h"
#include "system.h"
#include "math.h"
#include "zf_device_imu660ra.h"
#include "some_algorithm.h"
float SU400_duty=120;
PID steer_pid,L_car_stop_pid,R_car_stop_pid;
ERR L_car_stop_err,R_car_stop_err;
//分段pid
float serve_normal_pd[2]={2.28,1.25};     //二次函数pd基数2.28,1.25  (决赛不需要修改)
float serve_pd[2]={4.9,5.5};              //摄像头循迹直道舵机不抖动的PD参数(决赛不需要修改)
float use_inductor_pd[2]={52,55.5};      //(断路)电感pid,动态p基数
float use_slope_pd[2]= {-2,-2.1};     //(左圆环)斜率打角pid(决赛不需要修改)
float use_slope_pd_r[2]= {-2.25,-2.15}; //(右圆环)斜率打角pid(决赛不需要修改)
float use_ring_pd[2]=  {5.3,4.8};    //左圆环pid (5.45，4.95,在圆环中间参数)(5.3,4.8,切内环参数)(决赛不需要修改)
float use_ring_pd_r[2]= {5.5,5.49};   //右圆环pid (5.65，5.539,在圆环中间参数)(5.5,5.49,切内环参数)(决赛不需要修改)
float use_slope_cross[2]= {5.8,3.05};  //斜入十字5.8,3.9
float use_slope_cross_l[2]= {3.8,3.25};  //左斜入十字
void Mypid_Init()
{
    steer_pid.Kp = 8;
    steer_pid.Ki = 0;
    steer_pid.Kd = 1.8;
    steer_pid.out= 0;
    steer_pid.min=-SU400_duty;
    steer_pid.max=SU400_duty;

    L_car_stop_pid.Kp = 3050;
    L_car_stop_pid.Ki = 58.5;
    L_car_stop_pid.Kd = 0;
    L_car_stop_pid.out= 0;
    L_car_stop_pid.min=-4500;
    L_car_stop_pid.max= 4500;

    R_car_stop_pid.Kp = 3050;
    R_car_stop_pid.Ki = 58.5;
    R_car_stop_pid.Kd = 0;
    R_car_stop_pid.out= 0;
    R_car_stop_pid.min=-4500;
    R_car_stop_pid.max=4500;

    L_car_stop_err.dError=0;
    L_car_stop_err.Error=0;
    L_car_stop_err.LastError=0;
    L_car_stop_err.SumError=0;

    R_car_stop_err.dError=0;
    R_car_stop_err.Error=0;
    R_car_stop_err.LastError=0;
    R_car_stop_err.SumError=0;

}
//摄像头pid
float Steering_gear_PID(float Error)
{
    static float DeviationTemp[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static float last_duty=0;
    float D_err=0;
    DeviationTemp[5] = DeviationTemp[4];
    DeviationTemp[4] = DeviationTemp[3];
    DeviationTemp[3] = DeviationTemp[2];
    DeviationTemp[2] = DeviationTemp[1];
    DeviationTemp[1] = DeviationTemp[0];
    DeviationTemp[0] = Error;

    if(L_CirCle_State==5 || R_CirCle_State==5 || L_CirCle_State==8 || R_CirCle_State==8) //圆环状态5切换到状态6之间微分清0，防抖
    {                                                                 //圆环状态8切换到状态9之间微分清0，防抖
        DeviationTemp[5] = 0;
        DeviationTemp[4] = 0;
        DeviationTemp[3] = 0;
        DeviationTemp[2] = 0;
        DeviationTemp[1] = 0;
        DeviationTemp[0] = 0;
    }
    if(L_CirCle_State==6 && R_CirCle_State==6)     //圆环内斜率代替微分
    D_err=Slope_Calculate(0,6,DeviationTemp);
    else
    D_err = DeviationTemp[0]-DeviationTemp[5];

    //pd参数设置
    if(L_CirCle_State==6)      //圆环内不使用平方误差，避免对小误差不灵敏
    {
        steer_pid.Kp=use_ring_pd[0];
        steer_pid.Kd=use_ring_pd[1];
    }
    else if(R_CirCle_State==6)  //左右重量不一样，左右圆环参数不一样
    {
        steer_pid.Kp=use_ring_pd_r[0];
        steer_pid.Kd=use_ring_pd_r[1];
    }
    else if(R_slope_cross_state)   //右斜入十字，需要较大的占空比来矫正姿态
    {
        steer_pid.Kp=use_slope_cross[0];
        steer_pid.Kd=use_slope_cross[1];
    }
    else if(L_slope_cross_state)   //左斜入十字，需要较大的占空比来矫正姿态
    {
        steer_pid.Kp=use_slope_cross_l[0];
        steer_pid.Kd=use_slope_cross_l[1];
    }
    else if(L_CirCle_State==9 || R_CirCle_State==9)   //圆环9,削弱P
    {
        steer_pid.Kp=(Error*Error*0.8) /SU400_duty+serve_normal_pd[0];
        steer_pid.Kd=0;
    }
    else        //正常赛道
    {
       steer_pid.Kp=(Error*Error) /SU400_duty+serve_normal_pd[0];      //二次函数P
       steer_pid.Kd=serve_normal_pd[1];
       if(s_state[0]==1)
       {
        steer_pid.Kd=serve_normal_pd[1]+1.5;
       }
    }
    //输出计算
    if(L_slope_cross_state || R_slope_cross_state)
    {
    steer_pid.out= (steer_pid.Kp* DeviationTemp[0] +  steer_pid.Kd* D_err)*0.7+last_duty*0.3;
    }
    else if(L_CirCle_State==6 || R_CirCle_State==6)    //圆环内进行平滑处理
    {
    steer_pid.out= (steer_pid.Kp* DeviationTemp[0] +  steer_pid.Kd* D_err)*0.9+last_duty*0.1;
    }
    else                                          //其他地方不进行平滑
    {
    steer_pid.out= (steer_pid.Kp* DeviationTemp[0] +  steer_pid.Kd* D_err);
    }

    if(steer_pid.out >steer_pid.max) steer_pid.out = steer_pid.max;
    if(steer_pid.out < steer_pid.min) steer_pid.out = steer_pid.min;

    last_duty= steer_pid.out;
    return steer_pid.out;
}
//电感pid
float Inductor_user_PID(float Error)
{
    float d_err=0;
    float out=0;
    static float Inductor_err_record[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    Inductor_err_record[4] = Inductor_err_record[3];
    Inductor_err_record[3] = Inductor_err_record[2];
    Inductor_err_record[2] = Inductor_err_record[1];
    Inductor_err_record[1] = Inductor_err_record[0];
    Inductor_err_record[0] = Error;
    d_err=Error-Inductor_err_record[4];
    out=(Error*Error*8+use_inductor_pd[0])*Error+use_inductor_pd[1]*d_err;
    if(out >steer_pid.max) out = steer_pid.max;
    if(out < steer_pid.min) out = steer_pid.min;
    return out;
}
//圆环斜率转化角度pid
float slopetoangle_user_PID(float Error)
{
    float d_err=0;
    static float slope_last_err=0;
    d_err=Error-slope_last_err;
    if(L_CirCle_State==4 || L_CirCle_State==5)
    {
        steer_pid.out=(use_slope_pd[0]*Error+use_slope_pd[1]*d_err);
    }
    else if(R_CirCle_State==4 || R_CirCle_State==5)
    {
        steer_pid.out=(use_slope_pd_r[0]*Error+use_slope_pd_r[1]*d_err);
    }
    if(steer_pid.out >steer_pid.max) steer_pid.out = steer_pid.max;
    if(steer_pid.out < steer_pid.min) steer_pid.out = steer_pid.min;
    slope_last_err=Error;
    return steer_pid.out;
}
//停车控制
float Car_Stop_PID(PID *pid,ERR *err,float current)
{
    err->Error =(-current);

    pid->out += (pid->Kp * (err->Error - err->LastError)
            +  pid->Ki * err->Error
            );
    err->LastError =err->Error;
    if(pid->out > pid->max) pid->out = pid->max;
    if(pid->out < pid->min) pid->out = pid->min;
    return pid->out;
}





