/*
 * wheel.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "wheel.h"
#include "zf_driver_pwm.h"
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ֵ����ʼ��
//  @param      freq        PWMƵ��(10Hz-3MHz)
//  @param      Vl          �����ٶȣ�0-10000��
//  @param      Vr          �����ٶȣ�0-10000��
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void wheel_init(uint32 freq, int16 Vl, int16 Vr)
{
    int16 PWM8_duty;//L1ռ�ձ�
    int16 PWM7_duty;//L2ռ�ձ�
    int16 PWM6_duty;//R1ռ�ձ�
    int16 PWM5_duty;//R2ռ�ձ�

    if(Vl >= 0)
    {
        PWM8_duty = 0;
        PWM7_duty = Vl;
    }
    else
    {
        PWM8_duty = -Vl;
        PWM7_duty = 0;
    }

    if(Vr >= 0)
    {
        PWM6_duty = 0;
        PWM5_duty = Vr;
    }
    else
    {
        PWM6_duty = -Vr;
        PWM5_duty = 0;
    }

    pwm_init(TIM5_PWM_MAP0_CH1_A0,freq,PWM8_duty);   //L1��ʼ��
    pwm_init(TIM5_PWM_MAP0_CH2_A1,freq,PWM7_duty);   //L2��ʼ��
    pwm_init(TIM5_PWM_MAP0_CH3_A2,freq,PWM6_duty);   //R1��ʼ��
    pwm_init(TIM5_PWM_MAP0_CH4_A3,freq,PWM5_duty);   //R2��ʼ��
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ֵ���ٶ�����
//  @param      Vl          �����ٶȣ�0-10000��
//  @param      Vr          �����ٶȣ�0-10000��
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void wheel_speed(int16 Vl, int16 Vr)
{
    int16 PWM8_duty;//L1ռ�ձ�
    int16 PWM7_duty;//L2ռ�ձ�
    int16 PWM6_duty;//R1ռ�ձ�
    int16 PWM5_duty;//R2ռ�ձ�
    if(Vl >= 0)
    {
        PWM8_duty = 0;
        PWM7_duty = Vl;
    }
    else
    {
        PWM8_duty = -Vl;
        PWM7_duty = 0;
    }
    if(Vr >= 0)
    {
        PWM6_duty = 0;
        PWM5_duty = Vr;
    }
    else
    {
        PWM6_duty = -Vr;
        PWM5_duty = 0;
    }

    pwm_set_duty(TIM5_PWM_MAP0_CH1_A0,PWM8_duty);    //L1�ٶ�����TIM5_PWM_MAP0_CH1_A0
    pwm_set_duty(TIM5_PWM_MAP0_CH2_A1,PWM7_duty);    //L2�ٶ�����TIM5_PWM_MAP0_CH2_A1
    pwm_set_duty(TIM5_PWM_MAP0_CH3_A2,PWM6_duty);    //R1�ٶ�����TIM5_PWM_MAP0_CH3_A2
    pwm_set_duty(TIM5_PWM_MAP0_CH4_A3,PWM5_duty);    //R2�ٶ�����TIM5_PWM_MAP0_CH4_A3
}

