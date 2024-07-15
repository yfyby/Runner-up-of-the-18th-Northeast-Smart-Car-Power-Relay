/*
 * imgproc.h
 *
 *  Created on: 2023年4月26日
 *      Author: linjias
 */

#ifndef IMGPROC_H_
#define IMGPROC_H_
#include "zf_common_typedef.h"
#define IMGPROC_W 94
#define IMGPROC_H 60
#define MY_PI                       (3.141593f     )
#define ANGLE2RAD(x)                ((x)*0.017453f ) // 角度转弧度
#define RAD2ANGLE(x)                ((x)*57.295780f)// 弧度转角度
#define MIN(a,b)                    ((a) < (b) ? (a) : (b))                     // 将 a 限定在 a <= b
#define MAX(a,b)                    ((a) > (b) ? (a) : (b))                     // 将 a 限定在 a >= b
#define MINMAX(x,low,upper)         ((x)>(upper)?(upper):(x)<(low)?(low):(x))   // 将 x 限定在 low <= x <= upper
#define CLIP(x,low,upper)           ((x)>(upper)?(upper):(x)<(low)?(low):(x))   // 等同 MINMAX
#define ABS(x)                      ((x)<0?-(x):(x))                            // 绝对值

#define TRANSFORMATION_MATRIX   {{1.0018, -0.8642, 2.1631 },\
                                 {0.0000,-0.0142, 109.4118 },\
                                 {0.0000, -0.0093,1.0244}}\

////八领域扫线
//#define image_h 60//图像高度
//#define image_w 94//图像宽度
//#define white_pixel 255
//#define black_pixel 0
//#define border_max  image_w-2 //边界最大值
//#define border_min  1   //边界最小值
//#define uesr_RED     0XF800    //红色
//#define uesr_GREEN   0X07E0    //绿色
//#define uesr_BLUE    0X001F    //蓝色
// 快速开方
float Q_sqrt(float num);
// 快速e^x
float Q_exp(float num);
// 低通
float low_pass_filter1(float newdata, float fc, float Ts);
// 反变换
void map_xy(float X, float Y, int* x, int* y);
void draw_line(uint8(*image)[IMGPROC_W]);
void draw_seed_line(uint8(*image)[IMGPROC_W],uint8 begin);
void draw_point_red(uint8 x,uint8 y);
void draw_point_blue(uint8 x,uint8 y);
void draw_line_differ_colour(uint8(*image)[IMGPROC_W]);
int clip(int x, int low, int up);
float fclip(float x, float low, float up);
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel);
float Three_angle_points(float x1,float y1, float x2,float y2,float x3,float y3);
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist);
void nms_angle(float angle_in[], int num, float angle_out[], int kernel);
void image_process(uint8(*image)[IMGPROC_W]);
void findline_lefthand_adaptive(uint8(*img)[IMGPROC_W],uint8 width,uint8 height, uint8 block_size, uint8 clip_value, uint8 x, uint8 y, uint8 (*pts)[2], int *num);
void findline_righthand_adaptive(uint8(*img)[IMGPROC_W],uint8 width,uint8 height, uint8 block_size, uint8 clip_value, uint8 x, uint8 y, uint8 (*pts)[2], int *num);
void gray_image_process(uint8(*image)[IMGPROC_W],uint8 w ,uint8 h,uint8 block_size,uint8 clip_value);
uint8 labyrinth_get_start_point(uint8 start_row,uint8(*image)[image_w]);
void labyrinth_image_process(uint8(*image)[image_w],uint8 w ,uint8 h);
void findline_lefthand_binaryzation(uint8(*img)[94], uint8 x, uint8 y, uint8 (*pts)[2], int *num);
void findline_righthand_binaryzation(uint8(*img)[94], uint8 x, uint8 y, uint8 (*pts)[2], int *num);
void findline_lefthand_binaryzation_angle_user(uint8(*img)[94], uint8 x, uint8 y, uint8 (*pts)[2], int *num);
void findline_righthand_binaryzation_angle_user(uint8(*img)[94], uint8 x, uint8 y, uint8 (*pts)[2], int *num);
void Get_new_Line(uint8 Line[],uint8 start_x,uint8 start_y,uint8 (*pts)[2],uint8 num);
#endif /* IMGPROC_H_ */
