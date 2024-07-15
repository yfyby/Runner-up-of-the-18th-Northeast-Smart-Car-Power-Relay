/*
 * Been.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "Been.h"
#include "zf_driver_gpio.h"
//==============================================================================
//  @brief      ��ʼ��������
//==============================================================================
void Beep_Init(){
     gpio_init(C13, GPO, 0, GPO_PUSH_PULL);
     Beep_set(0);
}
//==============================================================================
//  @brief      ����������
//==============================================================================
void Beep_set(uint8 Value){
    gpio_set_level (C13,Value );
}

/***************************************************
*                 ������������ʼ��
*  ���ͣ�����
*  ����ֵ����
*  ���ܣ�������
***************************************************/
void Car_checkinit()
{
    gpio_init(E5, GPI, GPIO_HIGH, GPI_PULL_UP);                               // ��ʼ�� F5 ���� Ĭ�ϸߵ�ƽ ��������
}
