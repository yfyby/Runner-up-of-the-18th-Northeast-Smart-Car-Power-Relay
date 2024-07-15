/*
 * speed_mode.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef SPEED_MODE_H_
#define SPEED_MODE_H_
#include "zf_common_typedef.h"
#include "motor.h"
//Ԫ�ؼ���������
typedef struct time_count_element {
    uint8 cross_ms;            //ʮ�ּ�ʱ��
    uint8 l_slope_cross_ms;            //��б��ʮ�ּ�ʱ��
    uint8 r_slope_cross_ms;            //��б��ʮ�ּ�ʱ��
    uint8 ring_ms;            //��Բ��������
    uint8 R_ring_ms;            //��Բ��������
    uint8 garage_ms;         //���������
    uint8 barrier_ms;       //�ϰ��������
    uint8 break_road_ms;   //��·������
    uint8 ramp_ms;        //�µ�������
    uint8 stop_car_ms;      //ɲ������
    uint8 out_road_ms;     //����������
} Time_Count_Type_Def_element;
void control_mode(Time_Count_Type_Def *time_count);
void Car_Communication(void);
void ELEMENT_INIT(void);
void ELEMENT_COUNT(void);
#endif /* SPEED_MODE_H_ */
