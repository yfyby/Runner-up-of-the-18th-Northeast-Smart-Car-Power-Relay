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
//元素计数器类型
typedef struct time_count_element {
    uint8 cross_ms;            //十字计时器
    uint8 l_slope_cross_ms;            //左斜入十字计时器
    uint8 r_slope_cross_ms;            //右斜入十字计时器
    uint8 ring_ms;            //左圆环计数器
    uint8 R_ring_ms;            //右圆环计数器
    uint8 garage_ms;         //车库计数器
    uint8 barrier_ms;       //障碍物计数器
    uint8 break_road_ms;   //断路计数器
    uint8 ramp_ms;        //坡道计数器
    uint8 stop_car_ms;      //刹车计数
    uint8 out_road_ms;     //出赛道计数
} Time_Count_Type_Def_element;
void control_mode(Time_Count_Type_Def *time_count);
void Car_Communication(void);
void ELEMENT_INIT(void);
void ELEMENT_COUNT(void);
#endif /* SPEED_MODE_H_ */
