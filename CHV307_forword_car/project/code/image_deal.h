/*
 * image_deal.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef IMAGE_DEAL_H_
#define IMAGE_DEAL_H_
#include "zf_common_typedef.h"
//图像大小定义
#define Width 188
#define Hight  120
#define GrayScale 256              //定义256个灰度级
#define pi 3.14159
#define IMG_BLACK 0
#define IMG_WHITE 255
#define H 60
#define W 94
typedef struct
{
    int x, y;
} Point;
void morph_open(uint8(*image)[W]);
void dilate(uint8(*image)[W]);
void Copy_image(uint8(*in_IMG)[W], uint8(*out_IMG)[W]);
void Get_Use_Image(uint8(*in_IMG)[Width], uint8(*out_IMG)[W]);
void Get_cut_image(uint8** image, uint8** out_image,uint8 height, uint8 width);
void sobelAutoThreshold (uint8(*image)[W],uint8(*out_image)[W],uint8 Height_t, uint8 Width_t);
void sobelAutoThreshold_test (uint8(*image)[W],uint8(*out_image)[W],uint8 Height_t, uint8 Width_t);
float Slope_calculate(uint8 begin,uint8 end,uint8 *p);
float getAngle(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3);
void Image_Deal(uint8(*image)[Width],uint8(*out_image)[W],uint8 type,uint8 clip_value);
uint8 my_adapt_part_threshold(uint8(*tmImage)[W], uint16 col, uint16 row,uint16 begin_col, uint16 begin_row);   //注意计算阈值的一定要是原图像
void adaptive_threshold(uint8(*img0)[W],uint8(*img1)[W], uint16 col, uint16 row, int block_size, int down_value);
uint8 Gray_Search_Line(uint8(*img)[W],uint8 i1,uint8 j1,uint8 i2,uint8 j2,uint8 thres);
uint8 adaptive_thres(uint8 *img0, uint8 x,uint8 y,int16 block_size, int8 down_value);
float process_curvity(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3);
float Intercept(int startline, int endline,uint8 *Line,float k);
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2);
void regression(int type, int startline, int endline);
#endif /* IMAGE_DEAL_H_ */
