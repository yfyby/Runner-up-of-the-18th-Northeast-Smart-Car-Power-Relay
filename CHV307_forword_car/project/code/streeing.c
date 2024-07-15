/*
 * streeing.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "zf_driver_pwm.h"
#include "streeing.h"
int16 MID_duty=1420;
int16 L_R_angle_duty=120;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      �����ʼ���������ֵԽ�󣬶����ƫ��
//  @param      freq        PWMƵ��(10Hz-3MHz)
//  @param      angl        ����Ƕȣ�������ת  ������ת  -150 <= angl <= 150��
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void steering_init(uint32 freq, int16 angl)
{
    uint16 PWM1_duty;

    if(angl > L_R_angle_duty)angl = L_R_angle_duty;                               //�޷�
    if(angl < -L_R_angle_duty)angl = -L_R_angle_duty;
    PWM1_duty = MID_duty - angl;

    pwm_init(TIM2_PWM_MAP1_CH1_A15,freq,PWM1_duty);  //PWMA��ʼ��
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����Ƕ�����
//  @param      angl        ����Ƕȣ�������ת  ������ת  -150 <= angl <= 150��
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void steering_angl(int16 angl)
{
    uint16 PWM1_duty;

    if(angl > L_R_angle_duty)angl = L_R_angle_duty;                       //�޷�
    if(angl < -L_R_angle_duty)angl = -L_R_angle_duty;
    PWM1_duty =MID_duty  - angl;

    pwm_set_duty(TIM2_PWM_MAP1_CH1_A15,PWM1_duty);   //PWMA����ռ�ձ�
}

