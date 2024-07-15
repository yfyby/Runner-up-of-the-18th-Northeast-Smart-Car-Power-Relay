/*
 * image.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef IMAGE_H_
#define IMAGE_H_
#include "zf_common_typedef.h"
#define image_w 94
void Reair_Mid_Line(uint8 begin,uint8 end,uint8 type);
void Ramp_Deal(float gyro);
void Add_line(uint8 type,uint8 line[], uint8 x1, uint8 y1,uint8 x2, uint8 y2);
void Add_line_other(uint8 type,uint8 line[], uint8 begin, uint8 end,uint8 begin_y,uint8 end_y);
void Add_line_constant(uint8 type,uint8 line[],uint8 begin_y,uint8 end_y,uint8 end_x,uint8 end_Y,float slope);
void Add_line_other_plan(uint8 type,uint8 line[],uint8 out_line[], uint8 begin, uint8 end,uint8 begin_y,uint8 end_y);
void Add_line_user_twopoint_slope(uint8 type,uint8 line[], uint8 begin_x, uint8 begin_y,uint8 end_x, uint8 end_y,uint8 num);
void cross_deal(uint8(*image)[image_w]);
void  slope_cross_deal(uint8(*image)[image_w]);
uint8 Straight_line_judgment(uint8 arr[],uint8 type,int8 st,int8 en);

void Charge_Resue(uint8 dir_type);

void Check_Small_S(void);
void Ring_Deal(uint8(*image)[image_w]);
float turn_error(float inductor_error);
void Out_door_deal(uint8 type);
void In_door_deal(uint8 type,uint8(*image)[image_w]);
void Break_Road_Deal(uint8(*image)[image_w],float tof_distance);
void  Barrier_Deal(uint8(*image)[image_w],float tof_distance );
uint8 Find_guaidian(uint8 begin ,uint8 end,uint8 Line[],uint8 flag,uint8 UD);
uint8 Fine_baihang(uint8 type,uint8 begin ,uint8 end,uint8(*image)[image_w],uint8 Hang);
uint8 Fine_heihang(uint8 type,uint8 begin ,uint8 end,uint8(*image)[image_w],uint8 hang);
uint8 Fine_width_baihang(uint8 type,uint8 begin ,uint8 end,uint8(*image)[image_w],uint8 Hang);
void Final_deal(uint8(*image)[image_w]);
void steer_deal(void);
void Fine_element_angle_point(uint8(*image)[image_w]);
void Find_angle_point_grow_dir(void);
void Test_angle_point(uint8(*image)[image_w]);
//ÄæÍ¸ÊÓ²âÊÔ
void Fine_perspective_angle_point(uint8(*image)[image_w]);
void Calculate_perspective_angle(void);
#endif /* IMAGE_H_ */
