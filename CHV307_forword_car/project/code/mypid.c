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
//�ֶ�pid
float serve_normal_pd[2]={2.28,1.25};     //���κ���pd����2.28,1.25  (��������Ҫ�޸�)
float serve_pd[2]={4.9,5.5};              //����ͷѭ��ֱ�������������PD����(��������Ҫ�޸�)
float use_inductor_pd[2]={52,55.5};      //(��·)���pid,��̬p����
float use_slope_pd[2]= {-2,-2.1};     //(��Բ��)б�ʴ��pid(��������Ҫ�޸�)
float use_slope_pd_r[2]= {-2.25,-2.15}; //(��Բ��)б�ʴ��pid(��������Ҫ�޸�)
float use_ring_pd[2]=  {5.3,4.8};    //��Բ��pid (5.45��4.95,��Բ���м����)(5.3,4.8,���ڻ�����)(��������Ҫ�޸�)
float use_ring_pd_r[2]= {5.5,5.49};   //��Բ��pid (5.65��5.539,��Բ���м����)(5.5,5.49,���ڻ�����)(��������Ҫ�޸�)
float use_slope_cross[2]= {5.8,3.05};  //б��ʮ��5.8,3.9
float use_slope_cross_l[2]= {3.8,3.25};  //��б��ʮ��
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
//����ͷpid
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

    if(L_CirCle_State==5 || R_CirCle_State==5 || L_CirCle_State==8 || R_CirCle_State==8) //Բ��״̬5�л���״̬6֮��΢����0������
    {                                                                 //Բ��״̬8�л���״̬9֮��΢����0������
        DeviationTemp[5] = 0;
        DeviationTemp[4] = 0;
        DeviationTemp[3] = 0;
        DeviationTemp[2] = 0;
        DeviationTemp[1] = 0;
        DeviationTemp[0] = 0;
    }
    if(L_CirCle_State==6 && R_CirCle_State==6)     //Բ����б�ʴ���΢��
    D_err=Slope_Calculate(0,6,DeviationTemp);
    else
    D_err = DeviationTemp[0]-DeviationTemp[5];

    //pd��������
    if(L_CirCle_State==6)      //Բ���ڲ�ʹ��ƽ���������С������
    {
        steer_pid.Kp=use_ring_pd[0];
        steer_pid.Kd=use_ring_pd[1];
    }
    else if(R_CirCle_State==6)  //����������һ��������Բ��������һ��
    {
        steer_pid.Kp=use_ring_pd_r[0];
        steer_pid.Kd=use_ring_pd_r[1];
    }
    else if(R_slope_cross_state)   //��б��ʮ�֣���Ҫ�ϴ��ռ�ձ���������̬
    {
        steer_pid.Kp=use_slope_cross[0];
        steer_pid.Kd=use_slope_cross[1];
    }
    else if(L_slope_cross_state)   //��б��ʮ�֣���Ҫ�ϴ��ռ�ձ���������̬
    {
        steer_pid.Kp=use_slope_cross_l[0];
        steer_pid.Kd=use_slope_cross_l[1];
    }
    else if(L_CirCle_State==9 || R_CirCle_State==9)   //Բ��9,����P
    {
        steer_pid.Kp=(Error*Error*0.8) /SU400_duty+serve_normal_pd[0];
        steer_pid.Kd=0;
    }
    else        //��������
    {
       steer_pid.Kp=(Error*Error) /SU400_duty+serve_normal_pd[0];      //���κ���P
       steer_pid.Kd=serve_normal_pd[1];
       if(s_state[0]==1)
       {
        steer_pid.Kd=serve_normal_pd[1]+1.5;
       }
    }
    //�������
    if(L_slope_cross_state || R_slope_cross_state)
    {
    steer_pid.out= (steer_pid.Kp* DeviationTemp[0] +  steer_pid.Kd* D_err)*0.7+last_duty*0.3;
    }
    else if(L_CirCle_State==6 || R_CirCle_State==6)    //Բ���ڽ���ƽ������
    {
    steer_pid.out= (steer_pid.Kp* DeviationTemp[0] +  steer_pid.Kd* D_err)*0.9+last_duty*0.1;
    }
    else                                          //�����ط�������ƽ��
    {
    steer_pid.out= (steer_pid.Kp* DeviationTemp[0] +  steer_pid.Kd* D_err);
    }

    if(steer_pid.out >steer_pid.max) steer_pid.out = steer_pid.max;
    if(steer_pid.out < steer_pid.min) steer_pid.out = steer_pid.min;

    last_duty= steer_pid.out;
    return steer_pid.out;
}
//���pid
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
//Բ��б��ת���Ƕ�pid
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
//ͣ������
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





