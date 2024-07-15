/*
 * imgproc.c
 *
 *  Created on: 2023年4月26日
 *      Author: linjias
 */
#include "zf_common_headfile.h"
#define ONE_PI  (3.14159265f)
//牛顿迭代法快速计算 1.0/sqrt(x)逆平方根
float InvSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = * (int* ) & x;
    i = 0x5f375a86- (i>>1);
    x = * (float* )&i;
    x = x * (1.5f - xhalf * x * x);
    return x;
}

int My_pow(int x, int y)
{
    int i;
    int z=1;

    for(i=1;i<=y;i++) z=z*x;
    return z;
}


float mx_sin(float rad)
{
    float sine;
    if (rad < 0) {
        sine = rad*(1.27323954f + 0.405284735f * rad);
    } else {
        sine = rad * (1.27323954f - 0.405284735f * rad);
    }
    if (sine < 0) {
        sine = sine*(-0.225f * (sine + 1) + 1);
    } else {
        sine = sine * (0.225f *( sine - 1) + 1);
    }
    return sine;
}

float My_sin(float rad)
{
    int8 flag = 1;
    if (rad >= ONE_PI) {
        rad -= ONE_PI;
        flag = -1;
    }
    return mx_sin(rad) * flag;
}
float My_cos(float rad)
{
    int8 flag = 1;
    rad += ONE_PI/2.0;
    if (rad >= ONE_PI){
        flag = -1;
        rad -= ONE_PI;
    }
    return My_sin(rad)*flag;
}

void My_swap(int *x, int *y) {
    int t = 0;
    t = *x;
    *x = *y;
    *y = t;
}

float Q_sqrt(float num)
{
    //储存长整形
    long i;
    //变为float
    float number = (float)num;
    float x, y;
    const float f = 1.5F;
    x = number * 0.5F;
    y = number;
    i = *(long*)&y;//将浮点数表示的内存转变为整数
    i = 0x5f3759df - (i >> 1); //注意这一行
    y = *(float*)&i;
    //牛顿迭代
    y = y * (f - (x * y * y));
    y = y * (f - (x * y * y));
    //返回平方根
    return number * y;
}
//e^x
float Q_exp(float num)
{
    num = 1.f + num / 256.f;
    num *= num; num *= num; num *= num; num *= num;
    num *= num; num *= num; num *= num; num *= num;
    return num;
}
// 低通
float low_pass_filter1(float newdata, float fc, float Ts)
{
    static float output;
    float alpha;

    alpha = (2 * MY_PI * fc * Ts) / ((2 * MY_PI * fc * Ts) + 1);
    output = alpha * newdata + (1.f - alpha) * output;

    return output;
}

//反变换，透视坐标转换成原图坐标
static float INV_M[][3] = TRANSFORMATION_MATRIX;
void map_xy(float X, float Y, int* x, int* y)
{
    float z = INV_M[2][0] * X + INV_M[2][1] * Y + INV_M[2][2];
    *x = (int)((INV_M[0][0] * X + INV_M[0][1] * Y + INV_M[0][2]) / z);
    *y = (int)((INV_M[1][0] * X + INV_M[1][1] * Y + INV_M[1][2]) / z);
}
//整型限幅
int clip(int x, int low, int up) {
    return x > up ? up : x < low ? low : x;
}
//浮点限幅
float fclip(float x, float low, float up) {
    return x > up ? up : x < low ? low : x;
}
// 点集等距采样,使采样后点与点的距离为`dist`
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist){
    int remain = 0, len = 0;
    for(int i=0; i<num1-1 && len < *num2; i++){
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float dx = pts_in[i+1][0] - x0;
        float dy = pts_in[i+1][1] - y0;
        float dn = sqrt(dx*dx+dy*dy);
        dx /= dn;
        dy /= dn;

        while(remain < dn && len < *num2){
            x0 += dx * remain;
            pts_out[len][0] = x0;
            y0 += dy * remain;
            pts_out[len][1] = y0;

            len++;
            dn -= remain;
            remain = dist;
        }
        remain -= dn;
    }
    *num2 = len;
}
/*******建议在逆透视完之后对边界等距采样完成使用**********/
//点集三角滤波
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel){
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++) {
            pts_out[i][0] += pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
            pts_out[i][1] += pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
        }
        pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
        pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
    }
}
/*******建议在逆透视完之后对边界等距采样完成使用**********/
//3点集角度
float Three_angle_points(float x1,float y1, float x2,float y2,float x3,float y3)
{
        float angle=0;
        float dx1 = x2 - x1;
        float dy1 = y2 - y1;
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = x3 - x2;
        float dy2 = y3 - y2;
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);    //转换为弧度
        return angle;
}
// 点集局部角度变化率
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist)
{
    int numcut1 = num - 1;

    for (int i = 0; i < num; i++)
    {
        if (i <= 0 || i >= numcut1)
        {
            angle_out[i] = 0;
            continue;
        }
        int icutdist = i - dist;
        int iadddist = i + dist;

        float dx1 = pts_in[i][0] - pts_in[CLIP(icutdist, 0, numcut1)][0];
        float dy1 = pts_in[i][1] - pts_in[CLIP(icutdist, 0, numcut1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[CLIP(iadddist, 0, numcut1)][0] - pts_in[i][0];
        float dy2 = pts_in[CLIP(iadddist, 0, numcut1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;

        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);    //转换为弧度
    }
}
// 非极大抑制,获取局部最大值
void nms_angle(float angle_in[], int num, float angle_out[], int kernel)
{
    int half = kernel >> 1;
    int numcut1 = num - 1;
    for (int i = 0; i < num; i++)
    {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++)
        {
            if (ABS(angle_in[CLIP(i + j, 0, numcut1)]) > ABS(angle_out[i]))
            {
                angle_out[i] = 0;
                break;
            }
        }
    }
}
/*************************以下变量只在修补边界中使用******************************/
uint8 Repair_Flag=0;                                                              //需要修补边界标志位
uint8 Left_Line_New[63]={0}, Right_Line_New[63]={0};                             //左右边界修补后的坐标
uint8 Left_Add[60]={0}, Right_Add[60]={0},Mid_Add[60]={0};                      //左右边界修补标志位
uint8 Left_Add_Start=0, Right_Add_Start=0, Mid_Add_Start=0;                    //记录修补的开始位置
uint8 Mid_Count =0;                                                          //能够识别的最远处中点行数
float Left_Last_Slope = 0;                                                   //左中右边界斜率
float Right_Last_Slope = 0;
float Mid_Last_Slope=0;
uint8 L_repire=0;                                                          //修补点行坐标
uint8 R_repire=0;
uint8 M_repire=0;
/*************************以上变量只在修补边界中使用******************************/
/*---------------------------------------------------------------
 【功    能】二值化图像八领域找边界找边界以及中线
 【参    数】二值化图像
 【返 回 值】无
 ----------------------------------------------------------------*/
int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}
int16 limit_a_b(int16 x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}
int16 limit1(int16 x, int16 y)
{
    if (x > y)          return y;
    else if (x < -y)    return -y;
    else                return x;
}
/*
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数,图像数组,是否直接灰度找边界，注意，
 */
#define image_w  94
#define IMG_BLACK 0
#define IMG_WHITE 255
#define border_min 1
#define border_max 92
#define image_h 60
#define PI 3.14159265358979f
uint8 gray_thers=10;            //差比和阈值
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 labyrinth_start_point_l[2] = { 0 };//迷宫左边起点的x，y值
uint8 labyrinth_start_point_r[2] = { 0 };//迷宫右边起点的x，y值
//灰度巡线找边界专用
void  gray_level_dot_find(uint8(*image)[image_w],uint8 witch_l,uint8 witch_r,uint8 hight)
  {
      static uint8 mid_begin=47;
      static uint8 gray_mid_begin=47;
      for(uint8 i=59;i>hight;i--)
      {
          for(uint8 j=mid_begin;j<witch_r-1;j++)
          {
              if(Gray_Search_Line(image,i,j,i,j+2,(uint8)gray_thers))
               {
                   Right_Line[i]=j;
                   Right_Line_flag[i]=1;
                   break;
               }else
               {
                   Right_Line[i]=witch_r;
                   Right_Line_flag[i]=0;
               }
           }

          for(uint8 j=mid_begin;j>witch_l+1;j--)
            {
                  if (Gray_Search_Line(image,i,j,i,j-2,(uint8)gray_thers))
                   {
                      Left_Line[i]=j;
                      Left_Line_flag[i]=1;
                       break;
                   }else
                   {
                       Left_Line[i]=witch_l;
                       Left_Line_flag[i]=0;
                   }
            }
      }
      for(uint8 j=gray_mid_begin;j>witch_l+1;j--)
        {
              if (Gray_Search_Line(image,hight,j,hight,j-2,gray_thers))
               {
                  start_point_l[0]=j;
                  start_point_l[1]=hight;
                  Left_Line[hight]=j-2;
                  Left_Line_flag[hight]=1;
                   break;
               }else
               {
                   start_point_l[0]=witch_l;
                   start_point_l[1]=hight;
                   Left_Line[hight]=witch_l;
                   Left_Line_flag[hight]=0;
               }
         }
            for(uint8 j=gray_mid_begin;j<witch_r-1;j++)
            {
                if(Gray_Search_Line(image,hight,j,hight,j+2,gray_thers))
                 {
                    start_point_r[0]=j;
                    start_point_r[1]=hight;
                    Right_Line[hight]=j-2;
                    Right_Line_flag[hight]=1;
                    break;
                 }else
                 {
                     start_point_r[0]=witch_r;
                     start_point_r[1]=hight;
                     Right_Line[hight]=witch_r;
                     Right_Line_flag[hight]=0;
                 }
              }

            mid_begin=(Left_Line[58]+Right_Line[58])/2;
            gray_mid_begin=(start_point_r[0]+start_point_l[0])/2;
 }
//二值化巡线找边界专用
uint8 get_start_point(uint8 start_row,uint8(*image)[image_w])
{
    int8 i = 0,j=0,l_found = 0,r_found = 0;
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

    static uint8 mid_begin=47;                //初始扫描屏幕中点
    static uint8 strat_row_mid=47;           //八邻域起始中点
    //先找最底下一行中心线
    for(j=59;j>start_row;j--)
    {
        for(i=mid_begin;i>2;i--)                   //从中间向左找上升沿,第二帧图像起始点使用前一张图像的第二行中点开始找
         {
               if(image[j][i-3] == IMG_BLACK
                     &&  image[j][i-2] == IMG_BLACK
                          && image[j][i-1] == IMG_WHITE
                          && image[j][i] == IMG_WHITE)
                 {
                              Left_Line[j] = i-2;
                              Left_Line_flag[j]=1;           //找到左线
                              if(i-2<=2)
                              {
                                  Left_Line[j]=0;                                  //如果没有找到边界,人为确定
                                  Left_Line_flag[j]=0;           //没找到左线
                              }
                              break;
                  }
                         else
                         {
                              Left_Line[j]=0;                                  //如果没有找到边界,人为确定
                              Left_Line_flag[j]=0;           //没找到左线
                         }
         }
        for(i=mid_begin;i<91;i++)                                   //从中间向右找上升沿
         {
                 if(image[j][i+3] == IMG_BLACK
                         &&image[j][i+2] == IMG_BLACK
                          && image[j][i+1] == IMG_WHITE
                          && image[j][i] == IMG_WHITE)
                  {
                                Right_Line[j] = i+2;
                                Right_Line_flag[j]=1;                //找到右线
                                if(i+2>=91)
                                {
                                    Right_Line[j] = 93;                               //如果没有找到边界,人为确定
                                    Right_Line_flag[j]=0;            //没找到右线
                                }
                                break;
                  }
                            else
                            {
                                 Right_Line[j] = 93;                               //如果没有找到边界,人为确定
                                 Right_Line_flag[j]=0;            //没找到右线
                            }
         }
   }

    for (i = strat_row_mid; i > border_min; i--)
    {
        start_point_l[0] = i-1;
        start_point_l[1] = start_row;
        labyrinth_start_point_l[0]=i;
        labyrinth_start_point_l[1]=start_row;
        Left_Line[start_row]=i-1;
        if(Left_Line[start_row]<=2)
        {
                Left_Line_flag[start_row]=0;           //没找到左线
        }else
        {
                Left_Line_flag[start_row]=1;
        }
        if (image[start_row][i+1] == 255 &&image[start_row][i] == 255 && image[start_row][i - 1] == 0&& image[start_row][i - 2] == 0)
        {
            l_found = 1;
            break;
        }
    }

    for (i =strat_row_mid; i < border_max; i++)
    {
        start_point_r[0] = i+1;
        start_point_r[1] = start_row;
        labyrinth_start_point_r[0]=i;
        labyrinth_start_point_r[1]=start_row;
        Right_Line[start_row]=i+1;
        if(Right_Line[start_row]>=91)
        {
                Right_Line_flag[start_row]=0;           //没找到右线
        }else
        {
                Right_Line_flag[start_row]=1;
        }
        if (image[start_row][i-1] == 255 && image[start_row][i] == 255 && image[start_row][i + 1] == 0 && image[start_row][i + 2] == 0)
        {
            r_found = 1;
            break;
        }
    }

    mid_begin =(Left_Line[59]+Right_Line[59])/2;                          //更新赛道底边中点
    strat_row_mid=(Left_Line[start_row]+Right_Line[start_row])/2;                //更新赛道底边中点
    if(l_found&&r_found)return 1;                     //左右找到，可以进行八邻域
    else {
        return 0;
    }
}

//二值化迷宫巡线找边界专用
uint8 labyrinth_get_start_point(uint8 start_row,uint8(*image)[image_w])
{
    int8 i = 0,j=0,l_found = 0,r_found = 0;
    labyrinth_start_point_l[0] = 0;//x
    labyrinth_start_point_l[1] = 0;//y

    labyrinth_start_point_r[0] = 0;//x
    labyrinth_start_point_r[1] = 0;//y

    static uint8 mid_begin=47;
    static uint8 strat_row_mid=47;
    //先找最底下一行中心线
    for(j=59;j>start_row;j--)
    {
        for(i=mid_begin;i>=2;i--)                   //从中间向左找上升沿,第二帧图像起始点使用前一张图像的第二行中点开始找
         {
               if(
                    image[j][i-2] == IMG_BLACK
                          && image[j][i-1] == IMG_WHITE
                          && image[j][i] == IMG_WHITE)
                 {
                              Left_Line[j] = i-1;
                              Left_Line_flag[j]=1;           //找到左线
                              if(i-1<=2)
                              {
                                  Left_Line[j]=2;                                  //如果没有找到边界,人为确定
                                  Left_Line_flag[j]=0;           //没找到左线
                              }
                              break;
                  }
         }
        for(i=mid_begin;i<=91;i++)                                   //从中间向右找上升沿
         {
                 if(
                         image[j][i+2] == IMG_BLACK
                          && image[j][i+1] == IMG_WHITE
                          && image[j][i] == IMG_WHITE)
                  {
                                Right_Line[j] = i+1;
                                Right_Line_flag[j]=1;                //找到右线
                                if(i+1>=91)
                                {
                                    Right_Line[j] = 91;         //如果没有找到边界,人为确定
                                    Right_Line_flag[j]=0;       //没找到右线
                                }
                                break;
                  }
         }
   }

    Left_Line_flag[start_row]=1;
    Right_Line_flag[start_row]=1;

    for (i = strat_row_mid; i >= border_min; i--)
    {
        labyrinth_start_point_l[0]=i;
        labyrinth_start_point_l[1]=start_row;
        Left_Line[start_row]=i;
        if(i<=2)
        {
           Left_Line_flag[start_row]=0;           //没找到左线
        }
        if (image[start_row][i+1] == 255 &&image[start_row][i] == 255 && image[start_row][i - 1] == 0)
        {
            l_found = 1;
            break;
        }
    }

    for (i =strat_row_mid; i <= border_max; i++)
    {
        labyrinth_start_point_r[0]=i;
        labyrinth_start_point_r[1]=start_row;
        Right_Line[start_row]=i;
        if(i>=91)
        {
           Right_Line_flag[start_row]=0;           //没找到右线
        }
        if (image[start_row][i-1] == 255 && image[start_row][i] == 255 && image[start_row][i + 1] == 0)
        {
            r_found = 1;
            break;
        }
    }

    mid_begin =(Left_Line[59]+Right_Line[59])/2;                          //更新赛道底边中点
    strat_row_mid=(Left_Line[start_row]+Right_Line[start_row])/2;  //更新赛道底边中点
    if(l_found&&r_found)return 1;
    else {
        return 0;
    }
}
/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，这个是左右线一次性找完。
参数说明：
break_flag_r                 ：最多需要循环的次数
(*image)[image_w]          ：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                                   特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic                   ：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic                 ：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x                      ：左边起点横坐标
l_start_y                      ：左边起点纵坐标
r_start_x                    ：右边起点横坐标
r_start_y                      ：右边起点纵坐标
hightest                       ：循环结束所得到的最高高度
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num 110    //定义找点的数组成员个数，我使用的图像为60行，理想状态60足够，但特殊元素中边界是突变的，需要多点次数
#define White_mean 140  //灰度巡线全白阈值上限
//存放点的x，y坐标
uint8 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint8 points_r[(uint16)USE_num][2] = { {  0 } };//右线
float  rpts_l_dist[110][2]={{0}};      //等距采样透视变换左线
float  rpts_r_dist[110][2]={{0}};      //等距采样透视变换右线
float  rpts_l[(uint16)USE_num][2]={{0}};  //透视变换左线
float  rpts_r[(uint16)USE_num][2]={{0}};  //透视变换右线
float  rpts_l_fitter[(uint16)USE_num][2]={{0}};  //滤波后透视变换左线
float  rpts_r_fitter[(uint16)USE_num][2]={{0}};  //滤波后透视变换右线
float  rpts_langle[(uint16)USE_num]={0};      //左边界角度
float  rpts_rangle[(uint16)USE_num]={0};      //右边界角度
float  rpts_langle_nms[(uint16)USE_num]={0};  //进行极大抑制之后的左边界角度
float  rpts_rangle_nms[(uint16)USE_num]={0};  //进行极大抑制之后的右边界角度

uint16 out_points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 out_points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint8 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint8 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint8 gray_dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint8 gray_dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
uint8 l_hightest = 0;//左最高点
uint8 r_hightest = 0;//右最高点
uint8 hightest_x=0; //最高点的x坐标
int16 block_size = 5;                        //局部阈值计数核大小
int16 clip_value = 2;                         //局部阈值削弱
float line_blur_kernel = 3;                    //边线滤波核大小
float angle_dist =5;                           //计数角度变化率间距

void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{
    uint8 i = 0, j = 0;
    static uint8 last_x_l=0;    //上一次边界坐标
    static uint8 last_y_l=0;
    static uint8 last_x_r=0;
    static uint8 last_y_r=0;
    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //生长方向
    //顺时针(左)                 逆时针(右)
    //{4},{5},{6},         {6},{5},{4},
    //{3},    {7},         {7},    {3},
    //{2},{1},{8(0)},   {8(0)},{1},{2},
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //二值化为逆时针
    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //二值化为顺时针
    l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来
    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y
    //记录初始边界点
    last_x_l=l_start_x;
    last_y_l=l_start_y;
    last_x_r=r_start_x;
    last_y_r=r_start_y;
    //开启邻域循环
    while (break_flag--)
    {
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            //左边
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
            //右边
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y

            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用

            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }
        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//索引加一
        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y
        index_l = 0;//先清零，后使用
        //左边判断
        for (i = 0; i < 8; i++)
        {
             if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
               && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)                       //顺时针方向有0-255跳变
              {
                  temp_l[index_l][0] = search_filds_l[(i)][0];
                  temp_l[index_l][1] = search_filds_l[(i)][1];
                  index_l++;
                  dir_l[l_data_statics - 1] = (i);//记录生长方向
                  if(last_y_l>search_filds_l[i][1])                   //如果找到的是下一行
                  {
                     //直接记录
                      last_y_l=limit_a_b(last_y_l,1,59);
                      Left_Line[last_y_l-1]=search_filds_l[i][0];            //获取边界
                      last_y_l=search_filds_l[i][1];                        //更新y坐标
                      last_x_l=search_filds_l[i][0];                        //更新x坐标
                      if(last_x_l<=2)                                       //判断是否丢线
                      {
                          Left_Line_flag[last_y_l]=0;
                      }else
                      {
                          Left_Line_flag[last_y_l]=1;
                      }
                  }else if(last_y_l==search_filds_l[i][1])               //如果当前和上一个点是同一行
                  {
                      if(last_x_l!=search_filds_l[i][0])                 //在同一行但是x坐标不一样
                      {
                           Left_Line[last_y_l]=search_filds_l[i][0];          //获取边界
                           last_x_l=search_filds_l[i][0];                     //更新x坐标
                              if(last_x_l<=2)                                   //判断是否丢线
                              {
                                  Left_Line_flag[last_y_l]=0;
                              }else
                              {
                                  Left_Line_flag[last_y_l]=1;
                              }
                      }
                  }
             if (index_l)   //这个代码本来是放if(黑跳白)外边的，index_l初始8领域循环的值为0，如果多次找到，index_l
                 {          //会++,最大8，这个是在index_l次找到的边界点内取最高点那个，这是对每次八邻域循环而言，这里这样做是为了防止噪点影响
                            //不过正常图像来说，这个没啥必要
                     //更新坐标点
                     center_point_l[0] = temp_l[0][0];//x
                     center_point_l[1] = temp_l[0][1];//y
                     for (j = 0; j < index_l; j++)
                     {
                         if (center_point_l[1] > temp_l[j][1])
                         {
                             center_point_l[0] = temp_l[j][0];//x
                             center_point_l[1] = temp_l[j][1];//y
                         }
                     }

                     break;
                 }
              }
           }
        //退出while判断，三次进入同一个点，退出
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            break;
        }
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一

        index_r = 0;//先清零，后使用
        //右边判断
        for (i = 0; i < 8; i++)
        {
             if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                 && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
                  {
                      temp_r[index_r][0] = search_filds_r[(i)][0];
                      temp_r[index_r][1] = search_filds_r[(i)][1];
                      index_r++;//索引加一
                      dir_r[r_data_statics - 1] = (i);//记录生长方向
                      if(last_y_r>search_filds_r[i][1])
                      {
                          //直接记录
                          last_y_r=limit_a_b(last_y_r,1,59);
                          Right_Line[last_y_r-1]=   search_filds_r[i][0];
                          last_y_r=search_filds_r[i][1];
                          last_x_r=search_filds_r[i][0];
                          if(last_x_r>=91)
                          {
                              Right_Line_flag[last_y_r]=0;
                          }else
                          {
                              Right_Line_flag[last_y_r]=1;
                          }

                      }else if(last_y_r==search_filds_r[i][1])
                      {
                          if(last_x_r!=search_filds_r[i][0])
                          {
                                  Right_Line[last_y_r]=  search_filds_r[i][0];
                                  last_x_l=  search_filds_r[i][0];
                                  if(last_x_r>=91)
                                  {
                                      Right_Line_flag[last_y_r]=0;
                                  }else
                                  {
                                      Right_Line_flag[last_y_r]=1;
                                  }
                          }
                      }
                      if (index_r)
                       {
                           //更新坐标点
                           center_point_r[0] = temp_r[0][0];//x
                           center_point_r[1] = temp_r[0][1];//y
                           for (j = 0; j < index_r; j++)
                           {
                               if (center_point_r[1] > temp_r[j][1])
                               {
                                   center_point_r[0] = temp_r[j][0];//x
                                   center_point_r[1] = temp_r[j][1];//y
                               }
                           }
                           break;
                       }
              }
        }
    }
    //取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}
/*
函数名称：void perspective_conver()
功能说明：透视坐标变换
 */
void perspective_conver()
{
    uint8 i;
    uint8 x,y;
    //透视变换
    for (i = 0; i < data_stastics_l; i++) {
        x=points_l[i][0]*2;
        y=points_l[i][1]*2;
        rpts_l[i][0] = mapx[y][x];
        rpts_l[i][1] = mapy[y][x];
    }
    for (i = 0; i < data_stastics_r; i++) {
        x=points_l[i][0]*2;
        y=points_l[i][1]*2;
        rpts_r[i][0] = mapx[y][x];
        rpts_r[i][1] = mapy[y][x];
    }
}
/*
函数名称：angular_point_image()
功能说明：计算角点
 */
void angular_point_image()
{
    uint8 ipts0_num=data_stastics_l;
    uint8 ipts1_num=data_stastics_r;
    //透视变换
    perspective_conver();
    // 边线滤波
    blur_points(rpts_l, ipts0_num, rpts_l_fitter,line_blur_kernel);
    blur_points(rpts_r, ipts1_num, rpts_r_fitter,line_blur_kernel);
    // 边线局部角度变化率
    local_angle_points(rpts_l_fitter, ipts0_num, rpts_langle, (int)angle_dist);
    local_angle_points(rpts_r_fitter, ipts0_num, rpts_rangle, (int)angle_dist);
    // 角度变化率非极大抑制
    nms_angle(rpts_langle, ipts0_num, rpts_langle_nms, (int)(angle_dist));
    nms_angle(rpts_rangle, ipts1_num, rpts_rangle_nms, (int)(angle_dist));
}
// Y角点
int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
bool Ypt0_found, Ypt1_found;
// L角点
int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
bool Lpt0_found, Lpt1_found;
// 长直道
bool is_straight0, is_straight1;

void find_corners() {
    // 识别Y,L拐点
    Ypt0_found = Ypt1_found = Lpt0_found = Lpt1_found = false;
    is_straight0 = data_stastics_l > 20;
    is_straight1 = data_stastics_r > 20;
    for (int i = 0; i < data_stastics_l; i++) {
        if (rpts_langle_nms[i] == 0) continue;
        int im1 = clip(i -(int)angle_dist, 0, data_stastics_l - 1);
        int ip1 = clip(i + (int)angle_dist, 0, data_stastics_l - 1);
        float conf = fabs(rpts_langle[i]) - (fabs(rpts_langle[im1]) + fabs(rpts_langle[ip1])) / 2;

        //Y角点阈值
        if (Ypt0_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 40) {
            Ypt0_rpts0s_id = i;
            Ypt0_found = true;
        }
        //L角点阈值
        if (Lpt0_found == false && 70. / 180. * PI < conf && conf < 140. / 180. * PI && i < 40) {
            Lpt0_rpts0s_id = i;
            Lpt0_found = true;
        }
        //长直道阈值
        if (conf > 5. / 180. * PI && i < 20) is_straight0 = false;
        if (Ypt0_found == true && Lpt0_found == true && is_straight0 == false) break;
    }
    for (int i = 0; i < data_stastics_r; i++) {
        if (rpts_rangle_nms[i] == 0) continue;
        int im1 = clip(i - (int)angle_dist, 0, data_stastics_r - 1);
        int ip1 = clip(i + (int)angle_dist, 0, data_stastics_r - 1);
        float conf = fabs(rpts_rangle[i]) - (fabs(rpts_rangle[im1]) + fabs(rpts_rangle[ip1])) / 2;
        if (Ypt1_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 40) {
            Ypt1_rpts1s_id = i;
            Ypt1_found = true;
        }
        if (Lpt1_found == false && 70. / 180. * PI < conf && conf < 140. / 180. * PI && i < 40) {
            Lpt1_rpts1s_id = i;
            Lpt1_found = true;
        }

        if (conf > 5. / 180. * PI && i < 50) is_straight1 = false;

        if (Ypt1_found == true && Lpt1_found == true && is_straight1 == false) break;
    }
}
/*
函数名称：void image_draw_rectan(uint8(*image)[image_w])
功能说明：给图像画一个黑框，防止八领域遍历图像数组越界
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8(*image)[image_w])
{
    uint8 i = 0;
    for (i = 0; i < image_h; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][image_w - 1] = 0;
        image[i][image_w - 2] = 0;

    }
    for (i = 0; i < image_w; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
    }
}
/*
函数名称：void image_draw_rectan_labyrinth(uint8(*image)[image_w])
功能说明：给图像画一个黑框，防止迷宫遍历图像数组越界
example： image_draw_rectan_labyrinth(bin_image);
 */
void image_draw_rectan_labyrinth(uint8(*image)[image_w])
{
    uint8 i = 0;
    for (i = 0; i < image_h; i++)
    {
        image[i][0] = 0;
        image[i][image_w - 1] = 0;

    }
    for (i = 0; i < image_w; i++)
    {
        image[0][i] = 0;
    }
}
/*
功能说明：八领域最终处理函数
example： image_process(image);
 */
void image_process(uint8(*image)[image_w])
{
    uint16 i=0;
    hightest = 0;     //定义一个最高行，tip：这里的最高指的是y值的最小
    image_draw_rectan(image);                                //预处理
    data_stastics_l = 0;
    data_stastics_r = 0;

    char temp=0;                                    //临时坐标点
    float Add_Slope=0;                             //补线斜率
    Left_Add_Start=0;                           //清空每一幅图片补线起始点
    Right_Add_Start=0;                         //清空每一幅图片补线起始点
    Mid_Add_Start=0;
    Mid_Count = 0;
    Left_Last_Slope = 0;
    Right_Last_Slope = 0;
    Mid_Last_Slope=0;
    L_repire=0;
    R_repire=0;
    M_repire=0;
    //边界数据初始化
    for(int8 j=59;j>=0;j--)
    {
        Left_Line[j]=0;
        Right_Line[j]=93;
        Mid_Line[i]=46;
        Left_Line_New[i]=0;
        Right_Line_New[i]=93;
        Left_Line_flag[i]=0;
        Right_Line_flag[i]=0;
    }
    if (get_start_point(image_h - 2,image))   //找到起点了，再执行八领域，没找到就一直找
    {
     search_l_r((uint16)USE_num, image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
     //从爬取的边界线内获取中线数组
     for(i=59;i>=10;i--)
        {
             Left_Add[i] = 0;   //扫描之前都认为左中右边界都需要补线
             Right_Add[i] = 0;
             Mid_Add[i]=0;
             Mid_Line[i] = (Right_Line[i] + Left_Line[i]) /2; //计算当前行中心点
            if(Right_Line[i]>Left_Line[i])
              {
                 Width_track[i]=Right_Line[i]-Left_Line[i];             //实时赛道宽度
              }
            else
              {
                 Width_track[i]=0;
              }
/***********************边线修补**************************/
            if(i<=50 && i>=hightest)
            {
            if (Width_track[i] >Width_track[i+1] )  //本行赛道宽度大于上一行赛道宽度认为遇到突变
                        {
                    Repair_Flag = 1;    //需要补线
                            if(Left_Add[i+1])   //前一行补线了
                                {
                                        if (Left_Line[i] < Left_Line_New[i+1])  //与前二行的左边界实线比较
                                            {
                                                    Left_Add[i] = 0;                     //需要补线
                                            }else
                                            {
                                                    Left_Add[i] = 1;
                                            }
                                }
                            else    //前一行没有补线
                                {
                                        if (Left_Line[i] < Left_Line[i+1])  //与前二行的左边界实线比较
                                            {
                                              Left_Add[i] = 0;                    //需要补线
                                            }else
                                            {
                                                    Left_Add[i] = 1;
                                            }
                                }

                            if (Right_Add[i+1]) //前一行右边界补线了
                                {
                                        if (Right_Line[i] > Right_Line_New[i+1])    //与前一行的右边界实线进行比较
                                           {
                                                Right_Add[i] = 0;                             //需要补线
                                                }else{
                                                    Right_Add[i] = 1;
                                                }
                                }
                            else    //前一行右边界没有补线
                                    {
                                            if (Right_Line[i] > Right_Line[i+1])        //与前一行的右边界实线进行比较
                                            {
                                                    Right_Add[i] = 0;                          //需要补线
                                            }else{
                                                Right_Add[i] = 1;
                                        }
                                    }
                            }else{
                                Repair_Flag=0;                         //不需要补线
                            }
                        }

                        if( Repair_Flag ==1){
                        if (Left_Add[i]==0) //左边需要补线
                                {
                                    Left_Add_Start = i; //记录左边界补线开始位置

                                    if (i >= 51)        //能够识别的行数非常少，不具备参考价值
                                            {
                                                   Add_Slope = Left_Last_Slope;       //使用上一帧图像的左边界斜率
                                                   if(Add_Slope!=0)
                                                    temp =(char)((i - 59) * Add_Slope + Left_Line[59]);     //通过斜率推算补线的位置
                                                    else
                                                    temp =Left_Line[i+1];
                                            }
                                            else
                                            {
                                                  if(abs(Left_Add_Start-L_repire)<3){         //连续的补线点使用同一斜率
                                                   Add_Slope=Left_Last_Slope;
                                             }else{
                                                         Add_Slope =Slope_calculate(Left_Add_Start+1, Left_Add_Start+7,Left_Line);      //计算能识别的前几行图像斜率
                                                     }
                                                           temp =(char)((i - (Left_Add_Start+1)) * Add_Slope + Left_Line[Left_Add_Start+1]);//通过斜率推算补线的位置
                                                            Left_Last_Slope = Add_Slope;    //更新上次左边界斜率
                                                }

                        Left_Line_New[i] = LIMIT(temp, 93,0);   //不直接修改边界，只保存在补线数组里
                        L_repire=Left_Add_Start;                  //更新上一次补线的点
                        Left_Add[i]=1;                       //补点完成
                  }

                   if (Right_Add[i]==0) //右边需要补线
                                    {
                                        Right_Add_Start = i;    //记录左边界补线开始位置

                                            if (i >= 51)        //能够识别的行数非常少，不具备参考价值
                                                {
                                                        Add_Slope = Right_Last_Slope;   //使用上一帧图像的右边界斜率
                                                       if(Add_Slope!=0)
                                                        temp = (char)((i - 59) * Add_Slope + Right_Line[59]);//通过斜率推算补线的位置
                                                        else
                                                      temp = Right_Line[i+1];
                                                }
                                                else
                                                    {
                                                            if(abs(Right_Add_Start-R_repire)<3)  //连续的补线点使用同一斜率
                                                        {
                                                          Add_Slope=Right_Last_Slope;
                                                    }
                                                                    else{
                                                                      Add_Slope =Slope_calculate(Right_Add_Start+1, Right_Add_Start+7,Right_Line);      //计算能识别的前几行图像斜率
                                                                    }
                                                                temp = (char)((i - (Right_Add_Start+1)) * Add_Slope + Right_Line[Right_Add_Start+1]);//通过斜率推算补线的位置
                                                                Right_Last_Slope = Add_Slope;   //更新上次右边界斜率
                                                    }

                        Right_Line_New[i] = LIMIT(temp, 93, 0);        //不直接修改边界，只保存在补线数组里
                        R_repire=   Right_Add_Start;                  //更新上一次补线的点
                        Right_Add[i]=1;                              //补点完成
                                }

                if (Left_Add[i]==1 && Right_Add[i]==1)  //两边都需要补线
                    {
                          Width_track[i] = Right_Line_New[i] - Left_Line_New[i];  //重新计算本行赛道宽度
                          Mid_Line[i] = (Right_Line_New[i] +  Left_Line_New[i]) / 2;    //求出本行赛道中点
                          Mid_Add[i]=1;                                              //中线初步修复完成
                    }
                    else    //不需要补线或只有一边需要补线
                        {
                            if (Left_Add[i]==1) //此处最多只有一边会需要补线
                                {
                                        Width_track[i] = Right_Line[i] - Left_Line_New[i];  //重新计算本行赛道宽度
                                      Mid_Line[i]=(Right_Line[i] + Left_Line_New[i]) / 2;
                                      Mid_Add[i]=1;
                                }
                          if(Right_Add[i]==1 )
                                {
                                        Width_track[i] = Right_Line_New[i] - Left_Line[i];  //重新计算本行赛道宽度
                                      Mid_Line[i]=(Right_Line_New[i] + Left_Line[i]) / 2;
                                      Mid_Add[i]=1;
                                }

                }
                            Repair_Flag=0;
            }
/***********************对已经修补的中线进行矫正*****************************/
                if( Mid_Add[i]==1)
                {
                    Mid_Add_Start=i;
                    if (i >= 53)        //能够识别的行数非常少，不具备参考价值
                        {
                                Add_Slope = Mid_Last_Slope; //使用上一帧图像的右边界斜率
                               if(Mid_Last_Slope!=0)
                                temp = (char)((i - 59) * Add_Slope + Mid_Line[59]);//通过斜率推算补线的位置
                                else
                                temp =  Mid_Line[i+1];
                        }
                        else
                        {
                            if(abs(Mid_Add_Start-M_repire)<3)  //连续的补线点使用同一斜率
                                    {
                                          Add_Slope=Mid_Last_Slope;
                                    }
                                    else{
                                                    Add_Slope =Slope_calculate(Mid_Add_Start+1, Mid_Add_Start+6,Mid_Line);      //计算能识别的前几行图像斜率
                                            }
                                                    temp = (char)((i - (Mid_Add_Start+6)) * Add_Slope + Mid_Line[Mid_Add_Start+6]);//通过斜率推算补线的位置
                                                    Mid_Last_Slope = Add_Slope; //更新上次右边界斜率
                                            }

                  Mid_Line[i] = LIMIT(temp, 93, 0);          //保存在中线数组里
                  M_repire= Mid_Add_Start;                  //更新上一次补线的点
                  Mid_Add[i]=0;                              //矫正完成
          }
      }
   }
}
//灰度找边界
void gray_image_process(uint8(*image)[image_w],uint8 w ,uint8 h,uint8 block_size,uint8 clip_value)
{
    int num_L,num_R;
    uint16 num_point=(uint16)USE_num;
    static uint8 last_x_l=0;
    static uint8 last_y_l=0;
    static uint8 last_x_r=0;
    static uint8 last_y_r=0;
    uint16 r_data_statics=0,l_data_statics=0;
    bool hightest_flag=false;
    int8  half=block_size/2;
    int8  High=h - half - 2;
    int8  Wid=w - half - 2;
    hightest=0;
    //记录初始边界点
    last_x_l=start_point_l[0];
    last_y_l=start_point_l[1];
    last_x_r=start_point_r[0];
    last_y_r=start_point_r[1];
    num_L=sizeof(points_l) / sizeof(points_l[0]);
    num_R=sizeof(points_r) / sizeof(points_r[0]);
    gray_level_dot_find(image,half,Wid,High);  //寻找起始点
    draw_point_red(start_point_l[0],start_point_l[1]);
    draw_point_blue(start_point_r[0], start_point_r[1]);
    findline_lefthand_adaptive(image, w, h, block_size, clip_value, start_point_l[0], start_point_l[1], points_l, &num_L);//左手迷宫
    findline_righthand_adaptive(image, w, h, block_size, clip_value,start_point_r[0],start_point_r[1],  points_r, &num_R);//右手迷宫
    data_stastics_l=num_L;           //记录左边界找到的点数
    data_stastics_l=num_R;          //记录右边界找到的点数
    for(int8 i=High-1;i>10;i--)    //左右边界数据清空
    {
        Left_Line[i]= half;
        Right_Line[i]= Wid;
        Left_Line_flag[i]=0;
        Right_Line_flag[i]=0;
    }
    for(uint8 i=0;i<num_point;i++)  //提取左右边界
    {
        if(last_y_l>points_l[i][1])                   //如果找到的是下一行
        {
           //直接记录
            last_y_l=limit_a_b(last_y_l,1,59);
            Left_Line[last_y_l-1]=points_l[i][0];            //获取边界
            last_y_l=points_l[i][1];                        //更新y坐标
            last_x_l=points_l[i][0];                        //更新x坐标
            if(last_x_l<=half)                                       //判断是否丢线
            {
                Left_Line_flag[last_y_l]=0;
            }else
            {
                Left_Line_flag[last_y_l]=1;
            }
        }else if(last_y_l==points_l[i][1])               //如果当前和上一个点是同一行
        {
            if(last_x_l!=points_l[i][0])                 //在同一行但是x坐标不一样
            {
                 Left_Line[last_y_l]=points_l[i][0];          //获取边界
                 last_x_l=points_l[i][0];                     //更新x坐标
                    if(last_x_l<=half)                                   //判断是否丢线
                    {
                        Left_Line_flag[last_y_l]=0;
                    }else
                    {
                        Left_Line_flag[last_y_l]=1;
                    }
            }
        }

        if(last_y_r>points_r[i][1])
        {
            //直接记录
            last_y_r=limit_a_b(last_y_r,1,59);
            Right_Line[last_y_r-1]=  points_r[i][0];
            last_y_r=points_r[i][1];
            last_x_r=points_r[i][0];
            if(last_x_r>=w - half - 1)
            {
                Right_Line_flag[last_y_r]=0;
            }else
            {
                Right_Line_flag[last_y_r]=1;
            }

        }else if(last_y_r==points_r[i][1])
        {
            if(last_x_r!=points_r[i][0])
            {
                    Right_Line[last_y_r]=  points_r[i][0];
                    last_x_l=  points_r[i][0];
                    if(last_x_r>=w - half - 1)
                    {
                        Right_Line_flag[last_y_r]=0;
                    }else
                    {
                        Right_Line_flag[last_y_r]=1;
                    }
            }
        }
        //取出边界最高点
        l_data_statics++;
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 3
            && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 3 && hightest_flag==false)
            )
        {
            l_hightest =  points_l[l_data_statics - 1][1];//左最高点
            r_hightest = points_r[r_data_statics][1];//右最高点
            hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            hightest_flag=true;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1])&& hightest_flag==false) //如果左边比右边高了，左边等待右边
        {
            continue;
        }
        r_data_statics++;//索引加一
    }
    for(uint8 i=0;i<h;i++)  //计算中线
    {
        Mid_Line[i]=(Right_Line[i]+Left_Line[i])/2;
    }
}
//二值化迷宫
void labyrinth_image_process(uint8(*image)[image_w],uint8 w ,uint8 h)
{
    uint8 i=0;
    int num_L,num_R;
    uint16 num_point=(uint16)USE_num;
    static uint8 last_x_l=0;
    static uint8 last_y_l=0;
    static uint8 last_x_r=0;
    static uint8 last_y_r=0;
    uint16 labyrinth_r_data_statics=0,labyrinth_l_data_statics=0;
    bool hightest_flag=false;
    bool L_hightest_flag=false;
    bool R_hightest_flag=false;
    hightest=0;
    hightest_x=0;
    l_hightest=0;
    r_hightest=0;
    //边界修补
    char temp=0;                                    //临时坐标点
    float Add_Slope=0;                             //补线斜率
    Left_Add_Start=0;                           //清空每一幅图片补线起始点
    Right_Add_Start=0;                         //清空每一幅图片补线起始点
    Mid_Add_Start=0;
    Mid_Count = 0;
    Left_Last_Slope = 0;
    Right_Last_Slope = 0;
    Mid_Last_Slope=0;
    L_repire=0;
    R_repire=0;
    M_repire=0;
    //记录初始边界点
    last_x_l=labyrinth_start_point_l[0];
    last_y_l=labyrinth_start_point_l[1];
    last_x_r=labyrinth_start_point_r[0];
    last_y_r=labyrinth_start_point_r[1];
    num_L=sizeof(points_l) / sizeof(points_l[0]);
    num_R=sizeof(points_r) / sizeof(points_r[0]);
    image_draw_rectan_labyrinth(image);//画个黑框
    if (labyrinth_get_start_point(start_line_point_row,image))   //找到起点了，再执行迷宫，没找到就一直找
    {
    findline_lefthand_binaryzation(image, labyrinth_start_point_l[0], labyrinth_start_point_l[1], points_l,&num_L);
    findline_righthand_binaryzation(image, labyrinth_start_point_r[0], labyrinth_start_point_r[1], points_r,&num_R);
    data_stastics_l=num_L;           //记录左边界找到的点数
    data_stastics_l=num_R;          //记录右边界找到的点数
    }
    for(uint8 i=start_line_point_row-1;i>10;i--)    //左右边界数据清空
    {
        Left_Line[i]= 0;
        Right_Line[i]= 93;
        Left_Line_flag[i]=0;
        Right_Line_flag[i]=0;
    }
    for(uint8 i=0;i<num_point;i++)  //提取左右边界
      {
          if(last_y_l>points_l[i][1])                   //如果找到的是下一行
          {
             //直接记录
              last_y_l=limit_a_b(last_y_l,1,59);
              Left_Line[last_y_l-1]=points_l[i][0];            //获取边界
              last_y_l=points_l[i][1];                        //更新y坐标
              last_x_l=points_l[i][0];                        //更新x坐标
              if(last_x_l<=2)                                       //判断是否丢线
              {
                  Left_Line_flag[last_y_l]=0;
              }else
              {
                  Left_Line_flag[last_y_l]=1;
              }
          }else if(last_y_l==points_l[i][1])               //如果当前和上一个点是同一行
          {
              if(last_x_l!=points_l[i][0])                 //在同一行但是x坐标不一样
              {
                   Left_Line[last_y_l]=points_l[i][0];          //获取边界
                   last_x_l=points_l[i][0];                     //更新x坐标
                      if(last_x_l<=2)                                   //判断是否丢线
                      {
                          Left_Line_flag[last_y_l]=0;
                      }else
                      {
                          Left_Line_flag[last_y_l]=1;
                      }
              }
          }

          if(last_y_r>points_r[i][1])
          {
              //直接记录
              last_y_r=limit_a_b(last_y_r,1,59);
              Right_Line[last_y_r-1]=  points_r[i][0];
              last_y_r=points_r[i][1];
              last_x_r=points_r[i][0];
              if(last_x_r>=91)
              {
                  Right_Line_flag[last_y_r]=0;
              }else
              {
                  Right_Line_flag[last_y_r]=1;
              }

          }else if(last_y_r==points_r[i][1])
          {
              if(last_x_r!=points_r[i][0])
              {
                      Right_Line[last_y_r]=  points_r[i][0];
                      last_x_l=  points_r[i][0];
                      if(last_x_r>=91)
                      {
                          Right_Line_flag[last_y_r]=0;
                      }else
                      {
                          Right_Line_flag[last_y_r]=1;
                      }
              }
          }
          //取出边界最高点
          if(points_l[labyrinth_l_data_statics][1]<=10 && hightest_flag==false)
          {
              L_hightest_flag=true;
          }
          if(points_r[labyrinth_r_data_statics][1]<=10 && hightest_flag==false)
          {
              R_hightest_flag=true;
          }
          if(L_hightest_flag==true &&  R_hightest_flag==true)
          {
              hightest_flag=true;
              hightest=10;
          }

          if(L_hightest_flag==false)
          labyrinth_l_data_statics++;

          if (my_abs(points_r[labyrinth_r_data_statics][0] - points_l[labyrinth_l_data_statics - 1][0]) < 3
              && my_abs(points_r[labyrinth_r_data_statics][1] - points_l[labyrinth_l_data_statics - 1][1] < 3 && hightest_flag==false)
              )
          {
              l_hightest = points_l[labyrinth_l_data_statics - 1][1];//左最高点
              r_hightest = points_r[labyrinth_r_data_statics][1];//右最高点
              hightest = (points_r[labyrinth_r_data_statics][1] + points_l[labyrinth_l_data_statics - 1][1]) >> 1;//取出最高点
              hightest_x = (points_r[labyrinth_r_data_statics][0] + points_l[labyrinth_l_data_statics - 1][0]) >> 1;//取出最高点
              hightest_flag=true;
          }
          if ((points_r[labyrinth_r_data_statics][1] < points_l[labyrinth_l_data_statics - 1][1])&& hightest_flag==false) //如果左边比右边高了，左边等待右边
          {
              continue;
          }
          if(R_hightest_flag==false)
          labyrinth_r_data_statics++;//索引加一
    }
    //从爬取的边界线内获取中线数组
    for(i=59;i>=15;i--)
       {
            Left_Add[i] = 0;   //扫描之前都认为左中右边界都需要补线
            Right_Add[i] = 0;
            Mid_Add[i]=0;
            Mid_Line[i] = (Right_Line[i] + Left_Line[i]) /2; //计算当前行中心点
            if(Right_Line[i]>Left_Line[i])
             {
                Width_track[i]=Right_Line[i]-Left_Line[i];             //实时赛道宽度
             }
           else
             {
                Width_track[i]=0;
             }
/***********************边线修补**************************/
           if(i<=50 && i>=20)
           {
           if (Width_track[i] >Width_track[i+1] )  //本行赛道宽度大于上一行赛道宽度认为遇到突变
                       {
                   Repair_Flag = 1;    //需要补线
                           if(Left_Add[i+1])   //前一行补线了
                               {
                                       if (Left_Line[i] < Left_Line_New[i+1])  //与前二行的左边界实线比较
                                           {
                                                   Left_Add[i] = 0;                     //需要补线
                                           }else
                                           {
                                                   Left_Add[i] = 1;
                                           }
                               }
                           else    //前一行没有补线
                               {
                                       if (Left_Line[i] < Left_Line[i+1])  //与前二行的左边界实线比较
                                           {
                                             Left_Add[i] = 0;                    //需要补线
                                           }else
                                           {
                                                   Left_Add[i] = 1;
                                           }
                               }

                           if (Right_Add[i+1]) //前一行右边界补线了
                               {
                                       if (Right_Line[i] > Right_Line_New[i+1])    //与前一行的右边界实线进行比较
                                          {
                                               Right_Add[i] = 0;                             //需要补线
                                               }else{
                                                   Right_Add[i] = 1;
                                               }
                               }
                           else    //前一行右边界没有补线
                                   {
                                           if (Right_Line[i] > Right_Line[i+1])        //与前一行的右边界实线进行比较
                                           {
                                                   Right_Add[i] = 0;                          //需要补线
                                           }else{
                                               Right_Add[i] = 1;
                                       }
                                   }
                           }else{
                               Repair_Flag=0;                         //不需要补线
                           }
                       }

                       if( Repair_Flag ==1){
                       if (Left_Add[i]==0) //左边需要补线
                               {
                                   Left_Add_Start = i; //记录左边界补线开始位置

                                   if (i >= 51)        //能够识别的行数非常少，不具备参考价值
                                           {
                                                  Add_Slope = Left_Last_Slope;       //使用上一帧图像的左边界斜率
                                                  if(Add_Slope!=0)
                                                   temp =(char)((i - 59) * Add_Slope + Left_Line[59]);     //通过斜率推算补线的位置
                                                   else
                                                   temp =Left_Line[i+1];
                                           }
                                           else
                                           {
                                                 if(abs(Left_Add_Start-L_repire)<3){         //连续的补线点使用同一斜率
                                                  Add_Slope=Left_Last_Slope;
                                            }else{
                                                        Add_Slope =Slope_calculate(Left_Add_Start+1, Left_Add_Start+7,Left_Line);      //计算能识别的前几行图像斜率
                                                    }
                                                          temp =(char)((i - (Left_Add_Start+1)) * Add_Slope + Left_Line[Left_Add_Start+1]);//通过斜率推算补线的位置
                                                           Left_Last_Slope = Add_Slope;    //更新上次左边界斜率
                                               }

                       Left_Line_New[i] = LIMIT(temp, 93,0);   //不直接修改边界，只保存在补线数组里
                       L_repire=Left_Add_Start;                  //更新上一次补线的点
                       Left_Add[i]=1;                       //补点完成
                 }

                  if (Right_Add[i]==0) //右边需要补线
                                   {
                                       Right_Add_Start = i;    //记录左边界补线开始位置

                                           if (i >= 51)        //能够识别的行数非常少，不具备参考价值
                                               {
                                                       Add_Slope = Right_Last_Slope;   //使用上一帧图像的右边界斜率
                                                      if(Add_Slope!=0)
                                                       temp = (char)((i - 59) * Add_Slope + Right_Line[59]);//通过斜率推算补线的位置
                                                       else
                                                     temp = Right_Line[i+1];
                                               }
                                               else
                                                   {
                                                           if(abs(Right_Add_Start-R_repire)<3)  //连续的补线点使用同一斜率
                                                       {
                                                         Add_Slope=Right_Last_Slope;
                                                   }
                                                                   else{
                                                                     Add_Slope =Slope_calculate(Right_Add_Start+1, Right_Add_Start+7,Right_Line);      //计算能识别的前几行图像斜率
                                                                   }
                                                               temp = (char)((i - (Right_Add_Start+1)) * Add_Slope + Right_Line[Right_Add_Start+1]);//通过斜率推算补线的位置
                                                               Right_Last_Slope = Add_Slope;   //更新上次右边界斜率
                                                   }

                       Right_Line_New[i] = LIMIT(temp, 93, 0);        //不直接修改边界，只保存在补线数组里
                       R_repire=   Right_Add_Start;                  //更新上一次补线的点
                       Right_Add[i]=1;                              //补点完成
                               }

               if (Left_Add[i]==1 && Right_Add[i]==1)  //两边都需要补线
                   {
                         Width_track[i] = Right_Line_New[i] - Left_Line_New[i];  //重新计算本行赛道宽度
                         Mid_Line[i] = (Right_Line_New[i] +  Left_Line_New[i]) / 2;    //求出本行赛道中点
                         Mid_Add[i]=1;                                              //中线初步修复完成
                   }
                   else    //不需要补线或只有一边需要补线
                       {
                           if (Left_Add[i]==1) //此处最多只有一边会需要补线
                               {
                                       Width_track[i] = Right_Line[i] - Left_Line_New[i];  //重新计算本行赛道宽度
                                     Mid_Line[i]=(Right_Line[i] + Left_Line_New[i]) / 2;
                                     Mid_Add[i]=1;
                               }
                         if(Right_Add[i]==1 )
                               {
                                       Width_track[i] = Right_Line_New[i] - Left_Line[i];  //重新计算本行赛道宽度
                                     Mid_Line[i]=(Right_Line_New[i] + Left_Line[i]) / 2;
                                     Mid_Add[i]=1;
                               }

               }
                           Repair_Flag=0;
           }
/***********************对已经修补的中线进行矫正*****************************/
               if( Mid_Add[i]==1)
               {
                   Mid_Add_Start=i;
                   if (i >= 53)        //能够识别的行数非常少，不具备参考价值
                       {
                               Add_Slope = Mid_Last_Slope; //使用上一帧图像的右边界斜率
                              if(Mid_Last_Slope!=0)
                               temp = (char)((i - 59) * Add_Slope + Mid_Line[59]);//通过斜率推算补线的位置
                               else
                               temp =  Mid_Line[i+1];
                       }
                       else
                       {
                           if(abs(Mid_Add_Start-M_repire)<3)  //连续的补线点使用同一斜率
                                   {
                                         Add_Slope=Mid_Last_Slope;
                                   }
                                   else{
                                                   Add_Slope =Slope_calculate(Mid_Add_Start+1, Mid_Add_Start+6,Mid_Line);      //计算能识别的前几行图像斜率
                                           }
                                                   temp = (char)((i - (Mid_Add_Start+6)) * Add_Slope + Mid_Line[Mid_Add_Start+6]);//通过斜率推算补线的位置
                                                   Mid_Last_Slope = Add_Slope; //更新上次右边界斜率
                                           }

                 Mid_Line[i] = LIMIT(temp, 93, 0);          //保存在中线数组里
                 M_repire= Mid_Add_Start;                  //更新上一次补线的点
                 Mid_Add[i]=0;                              //矫正完成
         }
     }
}
/*
功能说明：重新提取边界数组
 */
void Get_new_Line(uint8 Line[],uint8 start_x,uint8 start_y,uint8 (*pts)[2],uint8 num)
{
    static uint8 last_x=0,last_y=0;
    last_x=start_x;
    last_y=start_y;
    for(uint8 i=0;i<num;i++)
    {
        if(last_y>pts[i][1])                   //如果找到的是下一行
        {
           //直接记录
            last_y=limit_a_b(last_y,1,59);
            Line[last_y-1]=pts[i][0];            //获取边界
            last_y=pts[i][1];                        //更新y坐标
            last_x=pts[i][0];                        //更新x坐标
        }else if(last_y==pts[i][1])                  //如果当前和上一个点是同一行
        {
            if(last_x!=pts[i][0])                     //在同一行但是x坐标不一样
            {
                 Line[last_y]=pts[i][0];              //获取边界
                 last_x=pts[i][0];                     //更新x坐标
            }
        }
     if(last_y>=58 || last_y<=20)  //到达边界停止
     {
       break;
     }
    }
}
/*
功能说明：以下为迷宫巡线算法
 */
/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */

 int8 dir_front[4][2]= {{0,  -1},
                                {1,  0},
                                {0,  1},
                                {-1, 0}};
 int8 dir_frontleft[4][2] = {{-1, -1},
                                    {1,  -1},
                                    {1,  1},
                                    {-1, 1}};
 int8 dir_frontright[4][2] = {{1,  -1},
                                     {1,  1},
                                     {-1, 1},
                                     {-1, -1}};
// 左手迷宫巡线
void findline_lefthand_adaptive(uint8(*img)[94],uint8 width,uint8 height, uint8 block_size, uint8 clip_value, uint8 x, uint8 y, uint8 (*pts)[2], int *num)
{

    int half = block_size / 2;
    int16 step = 0, dir = 0, turn = 0;
    while (step <*num  && half < y && y < height - half - 1 && turn < 4) {

        if(half < x && x < width - half - 1)
        {
            int local_thres = 0;
            for (int8 dy = -half; dy <= half; dy++) {
                for (int8 dx = -half; dx <= half; dx++) {
                    local_thres +=img[y + dy][x + dx];
                }
            }
            local_thres /= block_size * block_size;
            local_thres -= clip_value;
            uint8 front_value =img[y + dir_front[dir][1]][x + dir_front[dir][0]]; //四个点顺时针转
            uint8 frontleft_value =img[y + dir_frontleft[dir][1]][x + dir_frontleft[dir][0]];//菱形顺时针转
            if (front_value < local_thres) {
                dir = (dir + 1) % 4;
                turn++;
            } else if (frontleft_value < local_thres) {
                x += dir_front[dir][0];
                y += dir_front[dir][1];
                pts[step][0] = x;
                pts[step][1] = y;
                step++;
                turn = 0;
            } else {
                x += dir_frontleft[dir][0];
                y += dir_frontleft[dir][1];
                dir = (dir + 3) % 4;
                pts[step][0] = x;
                pts[step][1] = y;
                step++;
                turn = 0;
            }
        }
        else {
            int8 temp_x=x;
            while(!Gray_Search_Line(img,y,temp_x,y-2,temp_x,gray_thers)
                   && !(img[y][temp_x]>img[y-2][temp_x] && img[y+1][temp_x]>img[y-2][temp_x])) //符合白白黑特性
            {
                step++;
                y--;
                pts[step][0] = x;
                pts[step][1] = y;

            }
            if(x==half)
             x++;
            else if(x==width - half - 1)
             x--;
         }
       }
    *num = step;
}


// 右手迷宫巡线
void findline_righthand_adaptive(uint8(*img)[94],uint8 width,uint8 height, uint8 block_size, uint8 clip_value, uint8 x, uint8 y, uint8 (*pts)[2], int *num)
{
    int half = block_size / 2;
    int16 step = 0, dir = 0, turn = 0;
    while (step <*num && half < y && y < height - half - 1 && turn < 4) {
    if(half < x && x < width - half - 1)
    {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += img[y + dy][x + dx];
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;
        uint8 front_value = img[y + dir_front[dir][1]][x + dir_front[dir][0]];
        uint8 frontright_value =img[y + dir_frontright[dir][1]][x + dir_frontright[dir][0]];
        if (front_value < local_thres) {
            dir = (dir + 3) % 4;
            turn++;
        } else if (frontright_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }else
    {
//        if(x==half)
//           x++;
//        else if(x==width - half - 1)
//           x--;
        int8 temp_x=x;
        while(!Gray_Search_Line(img,y,temp_x,y-2,temp_x,gray_thers)
         && (!(img[y][temp_x]>img[y-2][temp_x] && img[y+1][temp_x]>img[y-2][temp_x])))//符合白白黑特性
        {
            step++;
            y--;
            pts[step][0] = x;
            pts[step][1] = y;
        }
        if(x==half)
           x++;
        else if(x==width - half - 1)
           x--;

    }
   }
    *num = step;
}

// 左手迷宫巡线,基于二值化
/*img:二值化图像    x:起点列坐标   y:起点行坐标    pts：左边线数组    num：走的步数 ,一般取边线数组大小*/
void findline_lefthand_binaryzation(uint8(*img)[94], uint8 x, uint8 y, uint8 (*pts)[2], int *num)
{
    int16 step = 0, dir = 0, turn = 0;
    while (step <*num && 10<y && y<59 && turn < 4) {
            uint8 front_value =img[y + dir_front[dir][1]][x + dir_front[dir][0]]; //四个点顺时针转
            uint8 frontleft_value =img[y + dir_frontleft[dir][1]][x + dir_frontleft[dir][0]];//菱形顺时针转
            if (front_value == 0) {
                dir = (dir + 1) % 4;
                turn++;
            } else if (frontleft_value == 0) {
                x += dir_front[dir][0];
                y += dir_front[dir][1];
                pts[step][0] = x;
                pts[step][1] = y;
                step++;
                turn = 0;
            } else {
                x += dir_frontleft[dir][0];
                y += dir_frontleft[dir][1];
                dir = (dir + 3) % 4;
                pts[step][0] = x;
                pts[step][1] = y;
                step++;
                turn = 0;
            }
     }
    *num = step;
}
// 右手迷宫巡线,基于二值化
/*img:二值化图像    x:起点列坐标   y:起点行坐标    pts：右边线数组    num：走的步数 ,一般取边线数组大小*/
void findline_righthand_binaryzation(uint8(*img)[94], uint8 x, uint8 y, uint8 (*pts)[2], int *num)
{
    int16 step = 0, dir = 0, turn = 0;
    while (step <*num && 10<y && y<59 &&  turn < 4) {   //右手，所以在我当前方向的基础上，我只需要判断前方和右上角
        uint8 front_value = img[y + dir_front[dir][1]][x + dir_front[dir][0]];
        uint8 frontright_value =img[y + dir_frontright[dir][1]][x + dir_frontright[dir][0]];
        if (front_value == 0) {    //全黑退出
            dir = (dir + 3) % 4;
            turn++;
        } else if (frontright_value == 0) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
   }
    *num = step;
}

// 左手迷宫巡线,基于二值化,适应于找角点
/*img:二值化图像    x:起点列坐标   y:起点行坐标    pts：左边线数组    num：走的步数 ,一般取边线数组大小*/
void findline_lefthand_binaryzation_angle_user(uint8(*img)[94], uint8 x, uint8 y, uint8 (*pts)[2], int *num)
{
    int16 step = 0, dir = 0, turn = 0;
    while (step <*num && 1 < x && x < 92 && 15 < y && y < 59 && turn < 4) {
            uint8 front_value =img[y + dir_front[dir][1]][x + dir_front[dir][0]]; //四个点顺时针转
            uint8 frontleft_value =img[y + dir_frontleft[dir][1]][x + dir_frontleft[dir][0]];//菱形顺时针转
            if (front_value == 0) {
                dir = (dir + 1) % 4;
                turn++;
            } else if (frontleft_value == 0) {
                x += dir_front[dir][0];
                y += dir_front[dir][1];
                pts[step][0] = x;
                pts[step][1] = y;
                step++;
                turn = 0;
            } else {
                x += dir_frontleft[dir][0];
                y += dir_frontleft[dir][1];
                dir = (dir + 3) % 4;
                pts[step][0] = x;
                pts[step][1] = y;
                step++;
                turn = 0;
            }
     }
    *num = step;
}
// 右手迷宫巡线,基于二值化,适应于找角点
/*img:二值化图像    x:起点列坐标   y:起点行坐标    pts：右边线数组    num：走的步数 ,一般取边线数组大小*/
void findline_righthand_binaryzation_angle_user(uint8(*img)[94], uint8 x, uint8 y, uint8 (*pts)[2], int *num)
{
    int16 step = 0, dir = 0, turn = 0;
    while (step <*num && 1 < x && x < 92 && 15 < y && y < 59 && turn < 4) {   //右手，所以在我当前方向的基础上，我只需要判断前方和右上角
        uint8 front_value = img[y + dir_front[dir][1]][x + dir_front[dir][0]];
        uint8 frontright_value =img[y + dir_frontright[dir][1]][x + dir_frontright[dir][0]];
        if (front_value == 0) {    //全黑退出
            dir = (dir + 3) % 4;
            turn++;
        } else if (frontright_value == 0) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
   }
    *num = step;
}
/*---------------------------------------------------------------
 【功    能】绘制中心和边界线
 【参    数】图像数组
 【返 回 值】无
 ----------------------------------------------------------------*/
void draw_line(uint8(*image)[image_w]){
    uint8 r,x;
    for(r=59;r>0;r--){
        x=LIMIT(Mid_Line[r],93,0);
        image[r][x]=0;
        x=LIMIT(Right_Line[r]-2,93,0);
        image[r][x]=0;
        x=LIMIT(Left_Line[r]+2,93,0);
        image[r][x]=0;
    }
    tft180_draw_line (0,30, 93, 30,RGB565_RED);          //在图像中画出分割线
    tft180_draw_line (0,20, 93, 20,RGB565_RED);          //在图像中画出分割线
    tft180_draw_line (0,10, 93, 10,RGB565_RED);          //在图像中画出分割线
}
/*---------------------------------------------------------------
 【功    能】使用不同颜色绘制中心和边界线
 【参    数】图像数组
 【返 回 值】无
 ----------------------------------------------------------------*/
void draw_line_differ_colour(uint8(*image)[image_w]){
    uint8 r;
    for(r=59;r>0;r--){
         if(Left_Line_flag[r]==1)
         {
             tft180_draw_point (Left_Line[r],r, RGB565_RED);
             tft180_draw_point (Left_Line[r]+1,r, RGB565_RED);
         }else {
             tft180_draw_point (Left_Line[r],r, RGB565_BLUE);
             tft180_draw_point (Left_Line[r]+1,r, RGB565_BLUE);
        }
         if(Right_Line_flag[r]==1)
         {
             tft180_draw_point (Right_Line[r],r, RGB565_RED);
             tft180_draw_point (Right_Line[r]-1,r, RGB565_RED);
         }else {
             tft180_draw_point (Right_Line[r],r, RGB565_BLUE);
             tft180_draw_point (Right_Line[r]-1,r, RGB565_BLUE);
        }
    }
    tft180_draw_line (30,50, 70, 50,RGB565_GREEN);          //在图像中画出分割线
    tft180_draw_line (30,40, 70, 40,RGB565_GREEN);          //在图像中画出分割线
    tft180_draw_line (30,30, 70, 30,RGB565_GREEN);          //在图像中画出分割线
    tft180_draw_line (30,20, 70, 20,RGB565_GREEN);          //在图像中画出分割线
    tft180_draw_line (30,10, 70, 10,RGB565_GREEN);          //在图像中画出分割线
}
/*---------------------------------------------------------------
 【功    能】将把领域边界绘制
 【参    数】图像
 【返 回 值】无
 ----------------------------------------------------------------*/
void draw_seed_line(uint8(*image)[image_w],uint8 begin)
{

    uint8 i,j;
    for(i=0;i<60;i++)
    {
        for(j=0;j<94;j++)
        {
            image[i][j]=0;
        }
    }
    for(i=0;i<begin;i++)
    {
        image[points_l[i][1]][points_l[i][0]]=255;
        image[points_r[i][1]][points_r[i][0]]=255;
    }
}
/*---------------------------------------------------------------
 【功    能】将某个点绘制出来(红色)
 【参    数】坐标
 【返 回 值】无
 ----------------------------------------------------------------*/
void draw_point_red(uint8 x,uint8 y){
   x=LIMIT(x,92,1);
   y=LIMIT(y,58,1);
   tft180_draw_point (x,y, RGB565_RED);
   tft180_draw_point (x-1,y, RGB565_RED);
   tft180_draw_point (x+1,y, RGB565_RED);
   tft180_draw_point (x,y-1, RGB565_RED);
   tft180_draw_point (x,y+1, RGB565_RED);
   tft180_draw_point (x-1,y-1, RGB565_RED);
   tft180_draw_point (x+1,y+1, RGB565_RED);
   tft180_draw_point (x-1,y+1, RGB565_RED);
   tft180_draw_point (x+1,y-1, RGB565_RED);
}
/*---------------------------------------------------------------
 【功    能】将某个点绘制出来(蓝色)
 【参    数】坐标
 【返 回 值】无
 ----------------------------------------------------------------*/
void draw_point_blue(uint8 x,uint8 y){
   x=LIMIT(x,92,1);
   y=LIMIT(y,58,1);
   tft180_draw_point (x,y, RGB565_BLUE);
   tft180_draw_point (x-1,y, RGB565_BLUE);
   tft180_draw_point (x+1,y, RGB565_BLUE);
   tft180_draw_point (x,y-1, RGB565_BLUE);
   tft180_draw_point (x,y+1, RGB565_BLUE);
   tft180_draw_point (x-1,y-1, RGB565_BLUE);
   tft180_draw_point (x+1,y+1, RGB565_BLUE);
   tft180_draw_point (x-1,y+1, RGB565_BLUE);
   tft180_draw_point (x+1,y-1, RGB565_BLUE);
}
