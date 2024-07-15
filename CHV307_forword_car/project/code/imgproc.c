/*
 * imgproc.c
 *
 *  Created on: 2023��4��26��
 *      Author: linjias
 */
#include "zf_common_headfile.h"
#define ONE_PI  (3.14159265f)
//ţ�ٵ��������ټ��� 1.0/sqrt(x)��ƽ����
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
    //���泤����
    long i;
    //��Ϊfloat
    float number = (float)num;
    float x, y;
    const float f = 1.5F;
    x = number * 0.5F;
    y = number;
    i = *(long*)&y;//����������ʾ���ڴ�ת��Ϊ����
    i = 0x5f3759df - (i >> 1); //ע����һ��
    y = *(float*)&i;
    //ţ�ٵ���
    y = y * (f - (x * y * y));
    y = y * (f - (x * y * y));
    //����ƽ����
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
// ��ͨ
float low_pass_filter1(float newdata, float fc, float Ts)
{
    static float output;
    float alpha;

    alpha = (2 * MY_PI * fc * Ts) / ((2 * MY_PI * fc * Ts) + 1);
    output = alpha * newdata + (1.f - alpha) * output;

    return output;
}

//���任��͸������ת����ԭͼ����
static float INV_M[][3] = TRANSFORMATION_MATRIX;
void map_xy(float X, float Y, int* x, int* y)
{
    float z = INV_M[2][0] * X + INV_M[2][1] * Y + INV_M[2][2];
    *x = (int)((INV_M[0][0] * X + INV_M[0][1] * Y + INV_M[0][2]) / z);
    *y = (int)((INV_M[1][0] * X + INV_M[1][1] * Y + INV_M[1][2]) / z);
}
//�����޷�
int clip(int x, int low, int up) {
    return x > up ? up : x < low ? low : x;
}
//�����޷�
float fclip(float x, float low, float up) {
    return x > up ? up : x < low ? low : x;
}
// �㼯�Ⱦ����,ʹ����������ľ���Ϊ`dist`
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
/*******��������͸����֮��Ա߽�Ⱦ�������ʹ��**********/
//�㼯�����˲�
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
/*******��������͸����֮��Ա߽�Ⱦ�������ʹ��**********/
//3�㼯�Ƕ�
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
        angle = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);    //ת��Ϊ����
        return angle;
}
// �㼯�ֲ��Ƕȱ仯��
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

        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);    //ת��Ϊ����
    }
}
// �Ǽ�������,��ȡ�ֲ����ֵ
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
/*************************���±���ֻ���޲��߽���ʹ��******************************/
uint8 Repair_Flag=0;                                                              //��Ҫ�޲��߽��־λ
uint8 Left_Line_New[63]={0}, Right_Line_New[63]={0};                             //���ұ߽��޲��������
uint8 Left_Add[60]={0}, Right_Add[60]={0},Mid_Add[60]={0};                      //���ұ߽��޲���־λ
uint8 Left_Add_Start=0, Right_Add_Start=0, Mid_Add_Start=0;                    //��¼�޲��Ŀ�ʼλ��
uint8 Mid_Count =0;                                                          //�ܹ�ʶ�����Զ���е�����
float Left_Last_Slope = 0;                                                   //�����ұ߽�б��
float Right_Last_Slope = 0;
float Mid_Last_Slope=0;
uint8 L_repire=0;                                                          //�޲���������
uint8 R_repire=0;
uint8 M_repire=0;
/*************************���ϱ���ֻ���޲��߽���ʹ��******************************/
/*---------------------------------------------------------------
 ����    �ܡ���ֵ��ͼ��������ұ߽��ұ߽��Լ�����
 ����    ������ֵ��ͼ��
 ���� �� ֵ����
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
����˵����Ѱ�������߽�ı߽����Ϊ������ѭ������ʼ��
����˵����������������,ͼ������,�Ƿ�ֱ�ӻҶ��ұ߽磬ע�⣬
 */
#define image_w  94
#define IMG_BLACK 0
#define IMG_WHITE 255
#define border_min 1
#define border_max 92
#define image_h 60
#define PI 3.14159265358979f
uint8 gray_thers=10;            //��Ⱥ���ֵ
uint8 start_point_l[2] = { 0 };//�������x��yֵ
uint8 start_point_r[2] = { 0 };//�ұ�����x��yֵ
uint8 labyrinth_start_point_l[2] = { 0 };//�Թ��������x��yֵ
uint8 labyrinth_start_point_r[2] = { 0 };//�Թ��ұ�����x��yֵ
//�Ҷ�Ѳ���ұ߽�ר��
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
//��ֵ��Ѳ���ұ߽�ר��
uint8 get_start_point(uint8 start_row,uint8(*image)[image_w])
{
    int8 i = 0,j=0,l_found = 0,r_found = 0;
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

    static uint8 mid_begin=47;                //��ʼɨ����Ļ�е�
    static uint8 strat_row_mid=47;           //��������ʼ�е�
    //���������һ��������
    for(j=59;j>start_row;j--)
    {
        for(i=mid_begin;i>2;i--)                   //���м�������������,�ڶ�֡ͼ����ʼ��ʹ��ǰһ��ͼ��ĵڶ����е㿪ʼ��
         {
               if(image[j][i-3] == IMG_BLACK
                     &&  image[j][i-2] == IMG_BLACK
                          && image[j][i-1] == IMG_WHITE
                          && image[j][i] == IMG_WHITE)
                 {
                              Left_Line[j] = i-2;
                              Left_Line_flag[j]=1;           //�ҵ�����
                              if(i-2<=2)
                              {
                                  Left_Line[j]=0;                                  //���û���ҵ��߽�,��Ϊȷ��
                                  Left_Line_flag[j]=0;           //û�ҵ�����
                              }
                              break;
                  }
                         else
                         {
                              Left_Line[j]=0;                                  //���û���ҵ��߽�,��Ϊȷ��
                              Left_Line_flag[j]=0;           //û�ҵ�����
                         }
         }
        for(i=mid_begin;i<91;i++)                                   //���м�������������
         {
                 if(image[j][i+3] == IMG_BLACK
                         &&image[j][i+2] == IMG_BLACK
                          && image[j][i+1] == IMG_WHITE
                          && image[j][i] == IMG_WHITE)
                  {
                                Right_Line[j] = i+2;
                                Right_Line_flag[j]=1;                //�ҵ�����
                                if(i+2>=91)
                                {
                                    Right_Line[j] = 93;                               //���û���ҵ��߽�,��Ϊȷ��
                                    Right_Line_flag[j]=0;            //û�ҵ�����
                                }
                                break;
                  }
                            else
                            {
                                 Right_Line[j] = 93;                               //���û���ҵ��߽�,��Ϊȷ��
                                 Right_Line_flag[j]=0;            //û�ҵ�����
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
                Left_Line_flag[start_row]=0;           //û�ҵ�����
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
                Right_Line_flag[start_row]=0;           //û�ҵ�����
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

    mid_begin =(Left_Line[59]+Right_Line[59])/2;                          //���������ױ��е�
    strat_row_mid=(Left_Line[start_row]+Right_Line[start_row])/2;                //���������ױ��е�
    if(l_found&&r_found)return 1;                     //�����ҵ������Խ��а�����
    else {
        return 0;
    }
}

//��ֵ���Թ�Ѳ���ұ߽�ר��
uint8 labyrinth_get_start_point(uint8 start_row,uint8(*image)[image_w])
{
    int8 i = 0,j=0,l_found = 0,r_found = 0;
    labyrinth_start_point_l[0] = 0;//x
    labyrinth_start_point_l[1] = 0;//y

    labyrinth_start_point_r[0] = 0;//x
    labyrinth_start_point_r[1] = 0;//y

    static uint8 mid_begin=47;
    static uint8 strat_row_mid=47;
    //���������һ��������
    for(j=59;j>start_row;j--)
    {
        for(i=mid_begin;i>=2;i--)                   //���м�������������,�ڶ�֡ͼ����ʼ��ʹ��ǰһ��ͼ��ĵڶ����е㿪ʼ��
         {
               if(
                    image[j][i-2] == IMG_BLACK
                          && image[j][i-1] == IMG_WHITE
                          && image[j][i] == IMG_WHITE)
                 {
                              Left_Line[j] = i-1;
                              Left_Line_flag[j]=1;           //�ҵ�����
                              if(i-1<=2)
                              {
                                  Left_Line[j]=2;                                  //���û���ҵ��߽�,��Ϊȷ��
                                  Left_Line_flag[j]=0;           //û�ҵ�����
                              }
                              break;
                  }
         }
        for(i=mid_begin;i<=91;i++)                                   //���м�������������
         {
                 if(
                         image[j][i+2] == IMG_BLACK
                          && image[j][i+1] == IMG_WHITE
                          && image[j][i] == IMG_WHITE)
                  {
                                Right_Line[j] = i+1;
                                Right_Line_flag[j]=1;                //�ҵ�����
                                if(i+1>=91)
                                {
                                    Right_Line[j] = 91;         //���û���ҵ��߽�,��Ϊȷ��
                                    Right_Line_flag[j]=0;       //û�ҵ�����
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
           Left_Line_flag[start_row]=0;           //û�ҵ�����
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
           Right_Line_flag[start_row]=0;           //û�ҵ�����
        }
        if (image[start_row][i-1] == 255 && image[start_row][i] == 255 && image[start_row][i + 1] == 0)
        {
            r_found = 1;
            break;
        }
    }

    mid_begin =(Left_Line[59]+Right_Line[59])/2;                          //���������ױ��е�
    strat_row_mid=(Left_Line[start_row]+Right_Line[start_row])/2;  //���������ױ��е�
    if(l_found&&r_found)return 1;
    else {
        return 0;
    }
}
/*
�������ƣ�void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

����˵������������ʽ��ʼ���ұߵ�ĺ����������������һ�������ꡣ
����˵����
break_flag_r                 �������Ҫѭ���Ĵ���
(*image)[image_w]          ����Ҫ�����ҵ��ͼ�����飬�����Ƕ�ֵͼ,�����������Ƽ���
                                   �ر�ע�⣬��Ҫ�ú궨��������Ϊ����������������ݿ����޷����ݹ���
*l_stastic                   ��ͳ��������ݣ����������ʼ�����Ա����ź�ȡ��ѭ������
*r_stastic                 ��ͳ���ұ����ݣ����������ʼ�����Ա����ź�ȡ��ѭ������
l_start_x                      �������������
l_start_y                      ��������������
r_start_x                    ���ұ���������
r_start_y                      ���ұ����������
hightest                       ��ѭ���������õ�����߸߶�
example��
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num 110    //�����ҵ�������Ա��������ʹ�õ�ͼ��Ϊ60�У�����״̬60�㹻��������Ԫ���б߽���ͻ��ģ���Ҫ������
#define White_mean 140  //�Ҷ�Ѳ��ȫ����ֵ����
//��ŵ��x��y����
uint8 points_l[(uint16)USE_num][2] = { {  0 } };//����
uint8 points_r[(uint16)USE_num][2] = { {  0 } };//����
float  rpts_l_dist[110][2]={{0}};      //�Ⱦ����͸�ӱ任����
float  rpts_r_dist[110][2]={{0}};      //�Ⱦ����͸�ӱ任����
float  rpts_l[(uint16)USE_num][2]={{0}};  //͸�ӱ任����
float  rpts_r[(uint16)USE_num][2]={{0}};  //͸�ӱ任����
float  rpts_l_fitter[(uint16)USE_num][2]={{0}};  //�˲���͸�ӱ任����
float  rpts_r_fitter[(uint16)USE_num][2]={{0}};  //�˲���͸�ӱ任����
float  rpts_langle[(uint16)USE_num]={0};      //��߽�Ƕ�
float  rpts_rangle[(uint16)USE_num]={0};      //�ұ߽�Ƕ�
float  rpts_langle_nms[(uint16)USE_num]={0};  //���м�������֮�����߽�Ƕ�
float  rpts_rangle_nms[(uint16)USE_num]={0};  //���м�������֮����ұ߽�Ƕ�

uint16 out_points_l[(uint16)USE_num][2] = { {  0 } };//����
uint16 out_points_r[(uint16)USE_num][2] = { {  0 } };//����
uint8 dir_r[(uint16)USE_num] = { 0 };//�����洢�ұ���������
uint8 dir_l[(uint16)USE_num] = { 0 };//�����洢�����������
uint8 gray_dir_r[(uint16)USE_num] = { 0 };//�����洢�ұ���������
uint8 gray_dir_l[(uint16)USE_num] = { 0 };//�����洢�����������
uint16 data_stastics_l = 0;//ͳ������ҵ���ĸ���
uint16 data_stastics_r = 0;//ͳ���ұ��ҵ���ĸ���
uint8 hightest = 0;//��ߵ�
uint8 l_hightest = 0;//����ߵ�
uint8 r_hightest = 0;//����ߵ�
uint8 hightest_x=0; //��ߵ��x����
int16 block_size = 5;                        //�ֲ���ֵ�����˴�С
int16 clip_value = 2;                         //�ֲ���ֵ����
float line_blur_kernel = 3;                    //�����˲��˴�С
float angle_dist =5;                           //�����Ƕȱ仯�ʼ��

void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{
    uint8 i = 0, j = 0;
    static uint8 last_x_l=0;    //��һ�α߽�����
    static uint8 last_y_l=0;
    static uint8 last_x_r=0;
    static uint8 last_y_r=0;
    //��߱���
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//ͳ�����
    //��������
    //˳ʱ��(��)                 ��ʱ��(��)
    //{4},{5},{6},         {6},{5},{4},
    //{3},    {7},         {7},    {3},
    //{2},{1},{8(0)},   {8(0)},{1},{2},
    //����˸�����
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //��ֵ��Ϊ��ʱ��
    //�ұ߱���
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//���������
    uint8 index_r = 0;//�����±�
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//ͳ���ұ�
    //����˸�����
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //��ֵ��Ϊ˳ʱ��
    l_data_statics = *l_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������
    r_data_statics = *r_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������
    //��һ�θ��������  ���ҵ������ֵ������
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y
    //��¼��ʼ�߽��
    last_x_l=l_start_x;
    last_y_l=l_start_y;
    last_x_r=r_start_x;
    last_y_r=r_start_y;
    //��������ѭ��
    while (break_flag--)
    {
        for (i = 0; i < 8; i++)//����8F����
        {
            //���
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
            //�ұ�
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y

            temp_l[i][0] = 0;//�����㣬��ʹ��
            temp_l[i][1] = 0;//�����㣬��ʹ��

            temp_r[i][0] = 0;//�����㣬��ʹ��
            temp_r[i][1] = 0;//�����㣬��ʹ��
        }
        //�����������䵽�Ѿ��ҵ��ĵ���
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//������һ
        //�����������䵽�Ѿ��ҵ��ĵ���
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y
        index_l = 0;//�����㣬��ʹ��
        //����ж�
        for (i = 0; i < 8; i++)
        {
             if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
               && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)                       //˳ʱ�뷽����0-255����
              {
                  temp_l[index_l][0] = search_filds_l[(i)][0];
                  temp_l[index_l][1] = search_filds_l[(i)][1];
                  index_l++;
                  dir_l[l_data_statics - 1] = (i);//��¼��������
                  if(last_y_l>search_filds_l[i][1])                   //����ҵ�������һ��
                  {
                     //ֱ�Ӽ�¼
                      last_y_l=limit_a_b(last_y_l,1,59);
                      Left_Line[last_y_l-1]=search_filds_l[i][0];            //��ȡ�߽�
                      last_y_l=search_filds_l[i][1];                        //����y����
                      last_x_l=search_filds_l[i][0];                        //����x����
                      if(last_x_l<=2)                                       //�ж��Ƿ���
                      {
                          Left_Line_flag[last_y_l]=0;
                      }else
                      {
                          Left_Line_flag[last_y_l]=1;
                      }
                  }else if(last_y_l==search_filds_l[i][1])               //�����ǰ����һ������ͬһ��
                  {
                      if(last_x_l!=search_filds_l[i][0])                 //��ͬһ�е���x���겻һ��
                      {
                           Left_Line[last_y_l]=search_filds_l[i][0];          //��ȡ�߽�
                           last_x_l=search_filds_l[i][0];                     //����x����
                              if(last_x_l<=2)                                   //�ж��Ƿ���
                              {
                                  Left_Line_flag[last_y_l]=0;
                              }else
                              {
                                  Left_Line_flag[last_y_l]=1;
                              }
                      }
                  }
             if (index_l)   //������뱾���Ƿ�if(������)��ߵģ�index_l��ʼ8����ѭ����ֵΪ0���������ҵ���index_l
                 {          //��++,���8���������index_l���ҵ��ı߽����ȡ��ߵ��Ǹ������Ƕ�ÿ�ΰ�����ѭ�����ԣ�������������Ϊ�˷�ֹ���Ӱ��
                            //��������ͼ����˵�����ûɶ��Ҫ
                     //���������
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
        //�˳�while�жϣ����ν���ͬһ���㣬�˳�
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
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//ȡ����ߵ�
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            continue;//�����߱��ұ߸��ˣ���ߵȴ��ұ�
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//��߱��ұ߸����Ѿ�����������
        {
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//������һ

        index_r = 0;//�����㣬��ʹ��
        //�ұ��ж�
        for (i = 0; i < 8; i++)
        {
             if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                 && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
                  {
                      temp_r[index_r][0] = search_filds_r[(i)][0];
                      temp_r[index_r][1] = search_filds_r[(i)][1];
                      index_r++;//������һ
                      dir_r[r_data_statics - 1] = (i);//��¼��������
                      if(last_y_r>search_filds_r[i][1])
                      {
                          //ֱ�Ӽ�¼
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
                           //���������
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
    //ȡ��ѭ������
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}
/*
�������ƣ�void perspective_conver()
����˵����͸������任
 */
void perspective_conver()
{
    uint8 i;
    uint8 x,y;
    //͸�ӱ任
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
�������ƣ�angular_point_image()
����˵��������ǵ�
 */
void angular_point_image()
{
    uint8 ipts0_num=data_stastics_l;
    uint8 ipts1_num=data_stastics_r;
    //͸�ӱ任
    perspective_conver();
    // �����˲�
    blur_points(rpts_l, ipts0_num, rpts_l_fitter,line_blur_kernel);
    blur_points(rpts_r, ipts1_num, rpts_r_fitter,line_blur_kernel);
    // ���߾ֲ��Ƕȱ仯��
    local_angle_points(rpts_l_fitter, ipts0_num, rpts_langle, (int)angle_dist);
    local_angle_points(rpts_r_fitter, ipts0_num, rpts_rangle, (int)angle_dist);
    // �Ƕȱ仯�ʷǼ�������
    nms_angle(rpts_langle, ipts0_num, rpts_langle_nms, (int)(angle_dist));
    nms_angle(rpts_rangle, ipts1_num, rpts_rangle_nms, (int)(angle_dist));
}
// Y�ǵ�
int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
bool Ypt0_found, Ypt1_found;
// L�ǵ�
int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
bool Lpt0_found, Lpt1_found;
// ��ֱ��
bool is_straight0, is_straight1;

void find_corners() {
    // ʶ��Y,L�յ�
    Ypt0_found = Ypt1_found = Lpt0_found = Lpt1_found = false;
    is_straight0 = data_stastics_l > 20;
    is_straight1 = data_stastics_r > 20;
    for (int i = 0; i < data_stastics_l; i++) {
        if (rpts_langle_nms[i] == 0) continue;
        int im1 = clip(i -(int)angle_dist, 0, data_stastics_l - 1);
        int ip1 = clip(i + (int)angle_dist, 0, data_stastics_l - 1);
        float conf = fabs(rpts_langle[i]) - (fabs(rpts_langle[im1]) + fabs(rpts_langle[ip1])) / 2;

        //Y�ǵ���ֵ
        if (Ypt0_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 40) {
            Ypt0_rpts0s_id = i;
            Ypt0_found = true;
        }
        //L�ǵ���ֵ
        if (Lpt0_found == false && 70. / 180. * PI < conf && conf < 140. / 180. * PI && i < 40) {
            Lpt0_rpts0s_id = i;
            Lpt0_found = true;
        }
        //��ֱ����ֵ
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
�������ƣ�void image_draw_rectan(uint8(*image)[image_w])
����˵������ͼ��һ���ڿ򣬷�ֹ���������ͼ������Խ��
example�� image_draw_rectan(bin_image);
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
�������ƣ�void image_draw_rectan_labyrinth(uint8(*image)[image_w])
����˵������ͼ��һ���ڿ򣬷�ֹ�Թ�����ͼ������Խ��
example�� image_draw_rectan_labyrinth(bin_image);
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
����˵�������������մ�����
example�� image_process(image);
 */
void image_process(uint8(*image)[image_w])
{
    uint16 i=0;
    hightest = 0;     //����һ������У�tip����������ָ����yֵ����С
    image_draw_rectan(image);                                //Ԥ����
    data_stastics_l = 0;
    data_stastics_r = 0;

    char temp=0;                                    //��ʱ�����
    float Add_Slope=0;                             //����б��
    Left_Add_Start=0;                           //���ÿһ��ͼƬ������ʼ��
    Right_Add_Start=0;                         //���ÿһ��ͼƬ������ʼ��
    Mid_Add_Start=0;
    Mid_Count = 0;
    Left_Last_Slope = 0;
    Right_Last_Slope = 0;
    Mid_Last_Slope=0;
    L_repire=0;
    R_repire=0;
    M_repire=0;
    //�߽����ݳ�ʼ��
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
    if (get_start_point(image_h - 2,image))   //�ҵ�����ˣ���ִ�а�����û�ҵ���һֱ��
    {
     search_l_r((uint16)USE_num, image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
     //����ȡ�ı߽����ڻ�ȡ��������
     for(i=59;i>=10;i--)
        {
             Left_Add[i] = 0;   //ɨ��֮ǰ����Ϊ�����ұ߽綼��Ҫ����
             Right_Add[i] = 0;
             Mid_Add[i]=0;
             Mid_Line[i] = (Right_Line[i] + Left_Line[i]) /2; //���㵱ǰ�����ĵ�
            if(Right_Line[i]>Left_Line[i])
              {
                 Width_track[i]=Right_Line[i]-Left_Line[i];             //ʵʱ�������
              }
            else
              {
                 Width_track[i]=0;
              }
/***********************�����޲�**************************/
            if(i<=50 && i>=hightest)
            {
            if (Width_track[i] >Width_track[i+1] )  //����������ȴ�����һ�����������Ϊ����ͻ��
                        {
                    Repair_Flag = 1;    //��Ҫ����
                            if(Left_Add[i+1])   //ǰһ�в�����
                                {
                                        if (Left_Line[i] < Left_Line_New[i+1])  //��ǰ���е���߽�ʵ�߱Ƚ�
                                            {
                                                    Left_Add[i] = 0;                     //��Ҫ����
                                            }else
                                            {
                                                    Left_Add[i] = 1;
                                            }
                                }
                            else    //ǰһ��û�в���
                                {
                                        if (Left_Line[i] < Left_Line[i+1])  //��ǰ���е���߽�ʵ�߱Ƚ�
                                            {
                                              Left_Add[i] = 0;                    //��Ҫ����
                                            }else
                                            {
                                                    Left_Add[i] = 1;
                                            }
                                }

                            if (Right_Add[i+1]) //ǰһ���ұ߽粹����
                                {
                                        if (Right_Line[i] > Right_Line_New[i+1])    //��ǰһ�е��ұ߽�ʵ�߽��бȽ�
                                           {
                                                Right_Add[i] = 0;                             //��Ҫ����
                                                }else{
                                                    Right_Add[i] = 1;
                                                }
                                }
                            else    //ǰһ���ұ߽�û�в���
                                    {
                                            if (Right_Line[i] > Right_Line[i+1])        //��ǰһ�е��ұ߽�ʵ�߽��бȽ�
                                            {
                                                    Right_Add[i] = 0;                          //��Ҫ����
                                            }else{
                                                Right_Add[i] = 1;
                                        }
                                    }
                            }else{
                                Repair_Flag=0;                         //����Ҫ����
                            }
                        }

                        if( Repair_Flag ==1){
                        if (Left_Add[i]==0) //�����Ҫ����
                                {
                                    Left_Add_Start = i; //��¼��߽粹�߿�ʼλ��

                                    if (i >= 51)        //�ܹ�ʶ��������ǳ��٣����߱��ο���ֵ
                                            {
                                                   Add_Slope = Left_Last_Slope;       //ʹ����һ֡ͼ�����߽�б��
                                                   if(Add_Slope!=0)
                                                    temp =(char)((i - 59) * Add_Slope + Left_Line[59]);     //ͨ��б�����㲹�ߵ�λ��
                                                    else
                                                    temp =Left_Line[i+1];
                                            }
                                            else
                                            {
                                                  if(abs(Left_Add_Start-L_repire)<3){         //�����Ĳ��ߵ�ʹ��ͬһб��
                                                   Add_Slope=Left_Last_Slope;
                                             }else{
                                                         Add_Slope =Slope_calculate(Left_Add_Start+1, Left_Add_Start+7,Left_Line);      //������ʶ���ǰ����ͼ��б��
                                                     }
                                                           temp =(char)((i - (Left_Add_Start+1)) * Add_Slope + Left_Line[Left_Add_Start+1]);//ͨ��б�����㲹�ߵ�λ��
                                                            Left_Last_Slope = Add_Slope;    //�����ϴ���߽�б��
                                                }

                        Left_Line_New[i] = LIMIT(temp, 93,0);   //��ֱ���޸ı߽磬ֻ�����ڲ���������
                        L_repire=Left_Add_Start;                  //������һ�β��ߵĵ�
                        Left_Add[i]=1;                       //�������
                  }

                   if (Right_Add[i]==0) //�ұ���Ҫ����
                                    {
                                        Right_Add_Start = i;    //��¼��߽粹�߿�ʼλ��

                                            if (i >= 51)        //�ܹ�ʶ��������ǳ��٣����߱��ο���ֵ
                                                {
                                                        Add_Slope = Right_Last_Slope;   //ʹ����һ֡ͼ����ұ߽�б��
                                                       if(Add_Slope!=0)
                                                        temp = (char)((i - 59) * Add_Slope + Right_Line[59]);//ͨ��б�����㲹�ߵ�λ��
                                                        else
                                                      temp = Right_Line[i+1];
                                                }
                                                else
                                                    {
                                                            if(abs(Right_Add_Start-R_repire)<3)  //�����Ĳ��ߵ�ʹ��ͬһб��
                                                        {
                                                          Add_Slope=Right_Last_Slope;
                                                    }
                                                                    else{
                                                                      Add_Slope =Slope_calculate(Right_Add_Start+1, Right_Add_Start+7,Right_Line);      //������ʶ���ǰ����ͼ��б��
                                                                    }
                                                                temp = (char)((i - (Right_Add_Start+1)) * Add_Slope + Right_Line[Right_Add_Start+1]);//ͨ��б�����㲹�ߵ�λ��
                                                                Right_Last_Slope = Add_Slope;   //�����ϴ��ұ߽�б��
                                                    }

                        Right_Line_New[i] = LIMIT(temp, 93, 0);        //��ֱ���޸ı߽磬ֻ�����ڲ���������
                        R_repire=   Right_Add_Start;                  //������һ�β��ߵĵ�
                        Right_Add[i]=1;                              //�������
                                }

                if (Left_Add[i]==1 && Right_Add[i]==1)  //���߶���Ҫ����
                    {
                          Width_track[i] = Right_Line_New[i] - Left_Line_New[i];  //���¼��㱾���������
                          Mid_Line[i] = (Right_Line_New[i] +  Left_Line_New[i]) / 2;    //������������е�
                          Mid_Add[i]=1;                                              //���߳����޸����
                    }
                    else    //����Ҫ���߻�ֻ��һ����Ҫ����
                        {
                            if (Left_Add[i]==1) //�˴����ֻ��һ�߻���Ҫ����
                                {
                                        Width_track[i] = Right_Line[i] - Left_Line_New[i];  //���¼��㱾���������
                                      Mid_Line[i]=(Right_Line[i] + Left_Line_New[i]) / 2;
                                      Mid_Add[i]=1;
                                }
                          if(Right_Add[i]==1 )
                                {
                                        Width_track[i] = Right_Line_New[i] - Left_Line[i];  //���¼��㱾���������
                                      Mid_Line[i]=(Right_Line_New[i] + Left_Line[i]) / 2;
                                      Mid_Add[i]=1;
                                }

                }
                            Repair_Flag=0;
            }
/***********************���Ѿ��޲������߽��н���*****************************/
                if( Mid_Add[i]==1)
                {
                    Mid_Add_Start=i;
                    if (i >= 53)        //�ܹ�ʶ��������ǳ��٣����߱��ο���ֵ
                        {
                                Add_Slope = Mid_Last_Slope; //ʹ����һ֡ͼ����ұ߽�б��
                               if(Mid_Last_Slope!=0)
                                temp = (char)((i - 59) * Add_Slope + Mid_Line[59]);//ͨ��б�����㲹�ߵ�λ��
                                else
                                temp =  Mid_Line[i+1];
                        }
                        else
                        {
                            if(abs(Mid_Add_Start-M_repire)<3)  //�����Ĳ��ߵ�ʹ��ͬһб��
                                    {
                                          Add_Slope=Mid_Last_Slope;
                                    }
                                    else{
                                                    Add_Slope =Slope_calculate(Mid_Add_Start+1, Mid_Add_Start+6,Mid_Line);      //������ʶ���ǰ����ͼ��б��
                                            }
                                                    temp = (char)((i - (Mid_Add_Start+6)) * Add_Slope + Mid_Line[Mid_Add_Start+6]);//ͨ��б�����㲹�ߵ�λ��
                                                    Mid_Last_Slope = Add_Slope; //�����ϴ��ұ߽�б��
                                            }

                  Mid_Line[i] = LIMIT(temp, 93, 0);          //����������������
                  M_repire= Mid_Add_Start;                  //������һ�β��ߵĵ�
                  Mid_Add[i]=0;                              //�������
          }
      }
   }
}
//�Ҷ��ұ߽�
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
    //��¼��ʼ�߽��
    last_x_l=start_point_l[0];
    last_y_l=start_point_l[1];
    last_x_r=start_point_r[0];
    last_y_r=start_point_r[1];
    num_L=sizeof(points_l) / sizeof(points_l[0]);
    num_R=sizeof(points_r) / sizeof(points_r[0]);
    gray_level_dot_find(image,half,Wid,High);  //Ѱ����ʼ��
    draw_point_red(start_point_l[0],start_point_l[1]);
    draw_point_blue(start_point_r[0], start_point_r[1]);
    findline_lefthand_adaptive(image, w, h, block_size, clip_value, start_point_l[0], start_point_l[1], points_l, &num_L);//�����Թ�
    findline_righthand_adaptive(image, w, h, block_size, clip_value,start_point_r[0],start_point_r[1],  points_r, &num_R);//�����Թ�
    data_stastics_l=num_L;           //��¼��߽��ҵ��ĵ���
    data_stastics_l=num_R;          //��¼�ұ߽��ҵ��ĵ���
    for(int8 i=High-1;i>10;i--)    //���ұ߽��������
    {
        Left_Line[i]= half;
        Right_Line[i]= Wid;
        Left_Line_flag[i]=0;
        Right_Line_flag[i]=0;
    }
    for(uint8 i=0;i<num_point;i++)  //��ȡ���ұ߽�
    {
        if(last_y_l>points_l[i][1])                   //����ҵ�������һ��
        {
           //ֱ�Ӽ�¼
            last_y_l=limit_a_b(last_y_l,1,59);
            Left_Line[last_y_l-1]=points_l[i][0];            //��ȡ�߽�
            last_y_l=points_l[i][1];                        //����y����
            last_x_l=points_l[i][0];                        //����x����
            if(last_x_l<=half)                                       //�ж��Ƿ���
            {
                Left_Line_flag[last_y_l]=0;
            }else
            {
                Left_Line_flag[last_y_l]=1;
            }
        }else if(last_y_l==points_l[i][1])               //�����ǰ����һ������ͬһ��
        {
            if(last_x_l!=points_l[i][0])                 //��ͬһ�е���x���겻һ��
            {
                 Left_Line[last_y_l]=points_l[i][0];          //��ȡ�߽�
                 last_x_l=points_l[i][0];                     //����x����
                    if(last_x_l<=half)                                   //�ж��Ƿ���
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
            //ֱ�Ӽ�¼
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
        //ȡ���߽���ߵ�
        l_data_statics++;
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 3
            && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 3 && hightest_flag==false)
            )
        {
            l_hightest =  points_l[l_data_statics - 1][1];//����ߵ�
            r_hightest = points_r[r_data_statics][1];//����ߵ�
            hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//ȡ����ߵ�
            hightest_flag=true;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1])&& hightest_flag==false) //�����߱��ұ߸��ˣ���ߵȴ��ұ�
        {
            continue;
        }
        r_data_statics++;//������һ
    }
    for(uint8 i=0;i<h;i++)  //��������
    {
        Mid_Line[i]=(Right_Line[i]+Left_Line[i])/2;
    }
}
//��ֵ���Թ�
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
    //�߽��޲�
    char temp=0;                                    //��ʱ�����
    float Add_Slope=0;                             //����б��
    Left_Add_Start=0;                           //���ÿһ��ͼƬ������ʼ��
    Right_Add_Start=0;                         //���ÿһ��ͼƬ������ʼ��
    Mid_Add_Start=0;
    Mid_Count = 0;
    Left_Last_Slope = 0;
    Right_Last_Slope = 0;
    Mid_Last_Slope=0;
    L_repire=0;
    R_repire=0;
    M_repire=0;
    //��¼��ʼ�߽��
    last_x_l=labyrinth_start_point_l[0];
    last_y_l=labyrinth_start_point_l[1];
    last_x_r=labyrinth_start_point_r[0];
    last_y_r=labyrinth_start_point_r[1];
    num_L=sizeof(points_l) / sizeof(points_l[0]);
    num_R=sizeof(points_r) / sizeof(points_r[0]);
    image_draw_rectan_labyrinth(image);//�����ڿ�
    if (labyrinth_get_start_point(start_line_point_row,image))   //�ҵ�����ˣ���ִ���Թ���û�ҵ���һֱ��
    {
    findline_lefthand_binaryzation(image, labyrinth_start_point_l[0], labyrinth_start_point_l[1], points_l,&num_L);
    findline_righthand_binaryzation(image, labyrinth_start_point_r[0], labyrinth_start_point_r[1], points_r,&num_R);
    data_stastics_l=num_L;           //��¼��߽��ҵ��ĵ���
    data_stastics_l=num_R;          //��¼�ұ߽��ҵ��ĵ���
    }
    for(uint8 i=start_line_point_row-1;i>10;i--)    //���ұ߽��������
    {
        Left_Line[i]= 0;
        Right_Line[i]= 93;
        Left_Line_flag[i]=0;
        Right_Line_flag[i]=0;
    }
    for(uint8 i=0;i<num_point;i++)  //��ȡ���ұ߽�
      {
          if(last_y_l>points_l[i][1])                   //����ҵ�������һ��
          {
             //ֱ�Ӽ�¼
              last_y_l=limit_a_b(last_y_l,1,59);
              Left_Line[last_y_l-1]=points_l[i][0];            //��ȡ�߽�
              last_y_l=points_l[i][1];                        //����y����
              last_x_l=points_l[i][0];                        //����x����
              if(last_x_l<=2)                                       //�ж��Ƿ���
              {
                  Left_Line_flag[last_y_l]=0;
              }else
              {
                  Left_Line_flag[last_y_l]=1;
              }
          }else if(last_y_l==points_l[i][1])               //�����ǰ����һ������ͬһ��
          {
              if(last_x_l!=points_l[i][0])                 //��ͬһ�е���x���겻һ��
              {
                   Left_Line[last_y_l]=points_l[i][0];          //��ȡ�߽�
                   last_x_l=points_l[i][0];                     //����x����
                      if(last_x_l<=2)                                   //�ж��Ƿ���
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
              //ֱ�Ӽ�¼
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
          //ȡ���߽���ߵ�
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
              l_hightest = points_l[labyrinth_l_data_statics - 1][1];//����ߵ�
              r_hightest = points_r[labyrinth_r_data_statics][1];//����ߵ�
              hightest = (points_r[labyrinth_r_data_statics][1] + points_l[labyrinth_l_data_statics - 1][1]) >> 1;//ȡ����ߵ�
              hightest_x = (points_r[labyrinth_r_data_statics][0] + points_l[labyrinth_l_data_statics - 1][0]) >> 1;//ȡ����ߵ�
              hightest_flag=true;
          }
          if ((points_r[labyrinth_r_data_statics][1] < points_l[labyrinth_l_data_statics - 1][1])&& hightest_flag==false) //�����߱��ұ߸��ˣ���ߵȴ��ұ�
          {
              continue;
          }
          if(R_hightest_flag==false)
          labyrinth_r_data_statics++;//������һ
    }
    //����ȡ�ı߽����ڻ�ȡ��������
    for(i=59;i>=15;i--)
       {
            Left_Add[i] = 0;   //ɨ��֮ǰ����Ϊ�����ұ߽綼��Ҫ����
            Right_Add[i] = 0;
            Mid_Add[i]=0;
            Mid_Line[i] = (Right_Line[i] + Left_Line[i]) /2; //���㵱ǰ�����ĵ�
            if(Right_Line[i]>Left_Line[i])
             {
                Width_track[i]=Right_Line[i]-Left_Line[i];             //ʵʱ�������
             }
           else
             {
                Width_track[i]=0;
             }
/***********************�����޲�**************************/
           if(i<=50 && i>=20)
           {
           if (Width_track[i] >Width_track[i+1] )  //����������ȴ�����һ�����������Ϊ����ͻ��
                       {
                   Repair_Flag = 1;    //��Ҫ����
                           if(Left_Add[i+1])   //ǰһ�в�����
                               {
                                       if (Left_Line[i] < Left_Line_New[i+1])  //��ǰ���е���߽�ʵ�߱Ƚ�
                                           {
                                                   Left_Add[i] = 0;                     //��Ҫ����
                                           }else
                                           {
                                                   Left_Add[i] = 1;
                                           }
                               }
                           else    //ǰһ��û�в���
                               {
                                       if (Left_Line[i] < Left_Line[i+1])  //��ǰ���е���߽�ʵ�߱Ƚ�
                                           {
                                             Left_Add[i] = 0;                    //��Ҫ����
                                           }else
                                           {
                                                   Left_Add[i] = 1;
                                           }
                               }

                           if (Right_Add[i+1]) //ǰһ���ұ߽粹����
                               {
                                       if (Right_Line[i] > Right_Line_New[i+1])    //��ǰһ�е��ұ߽�ʵ�߽��бȽ�
                                          {
                                               Right_Add[i] = 0;                             //��Ҫ����
                                               }else{
                                                   Right_Add[i] = 1;
                                               }
                               }
                           else    //ǰһ���ұ߽�û�в���
                                   {
                                           if (Right_Line[i] > Right_Line[i+1])        //��ǰһ�е��ұ߽�ʵ�߽��бȽ�
                                           {
                                                   Right_Add[i] = 0;                          //��Ҫ����
                                           }else{
                                               Right_Add[i] = 1;
                                       }
                                   }
                           }else{
                               Repair_Flag=0;                         //����Ҫ����
                           }
                       }

                       if( Repair_Flag ==1){
                       if (Left_Add[i]==0) //�����Ҫ����
                               {
                                   Left_Add_Start = i; //��¼��߽粹�߿�ʼλ��

                                   if (i >= 51)        //�ܹ�ʶ��������ǳ��٣����߱��ο���ֵ
                                           {
                                                  Add_Slope = Left_Last_Slope;       //ʹ����һ֡ͼ�����߽�б��
                                                  if(Add_Slope!=0)
                                                   temp =(char)((i - 59) * Add_Slope + Left_Line[59]);     //ͨ��б�����㲹�ߵ�λ��
                                                   else
                                                   temp =Left_Line[i+1];
                                           }
                                           else
                                           {
                                                 if(abs(Left_Add_Start-L_repire)<3){         //�����Ĳ��ߵ�ʹ��ͬһб��
                                                  Add_Slope=Left_Last_Slope;
                                            }else{
                                                        Add_Slope =Slope_calculate(Left_Add_Start+1, Left_Add_Start+7,Left_Line);      //������ʶ���ǰ����ͼ��б��
                                                    }
                                                          temp =(char)((i - (Left_Add_Start+1)) * Add_Slope + Left_Line[Left_Add_Start+1]);//ͨ��б�����㲹�ߵ�λ��
                                                           Left_Last_Slope = Add_Slope;    //�����ϴ���߽�б��
                                               }

                       Left_Line_New[i] = LIMIT(temp, 93,0);   //��ֱ���޸ı߽磬ֻ�����ڲ���������
                       L_repire=Left_Add_Start;                  //������һ�β��ߵĵ�
                       Left_Add[i]=1;                       //�������
                 }

                  if (Right_Add[i]==0) //�ұ���Ҫ����
                                   {
                                       Right_Add_Start = i;    //��¼��߽粹�߿�ʼλ��

                                           if (i >= 51)        //�ܹ�ʶ��������ǳ��٣����߱��ο���ֵ
                                               {
                                                       Add_Slope = Right_Last_Slope;   //ʹ����һ֡ͼ����ұ߽�б��
                                                      if(Add_Slope!=0)
                                                       temp = (char)((i - 59) * Add_Slope + Right_Line[59]);//ͨ��б�����㲹�ߵ�λ��
                                                       else
                                                     temp = Right_Line[i+1];
                                               }
                                               else
                                                   {
                                                           if(abs(Right_Add_Start-R_repire)<3)  //�����Ĳ��ߵ�ʹ��ͬһб��
                                                       {
                                                         Add_Slope=Right_Last_Slope;
                                                   }
                                                                   else{
                                                                     Add_Slope =Slope_calculate(Right_Add_Start+1, Right_Add_Start+7,Right_Line);      //������ʶ���ǰ����ͼ��б��
                                                                   }
                                                               temp = (char)((i - (Right_Add_Start+1)) * Add_Slope + Right_Line[Right_Add_Start+1]);//ͨ��б�����㲹�ߵ�λ��
                                                               Right_Last_Slope = Add_Slope;   //�����ϴ��ұ߽�б��
                                                   }

                       Right_Line_New[i] = LIMIT(temp, 93, 0);        //��ֱ���޸ı߽磬ֻ�����ڲ���������
                       R_repire=   Right_Add_Start;                  //������һ�β��ߵĵ�
                       Right_Add[i]=1;                              //�������
                               }

               if (Left_Add[i]==1 && Right_Add[i]==1)  //���߶���Ҫ����
                   {
                         Width_track[i] = Right_Line_New[i] - Left_Line_New[i];  //���¼��㱾���������
                         Mid_Line[i] = (Right_Line_New[i] +  Left_Line_New[i]) / 2;    //������������е�
                         Mid_Add[i]=1;                                              //���߳����޸����
                   }
                   else    //����Ҫ���߻�ֻ��һ����Ҫ����
                       {
                           if (Left_Add[i]==1) //�˴����ֻ��һ�߻���Ҫ����
                               {
                                       Width_track[i] = Right_Line[i] - Left_Line_New[i];  //���¼��㱾���������
                                     Mid_Line[i]=(Right_Line[i] + Left_Line_New[i]) / 2;
                                     Mid_Add[i]=1;
                               }
                         if(Right_Add[i]==1 )
                               {
                                       Width_track[i] = Right_Line_New[i] - Left_Line[i];  //���¼��㱾���������
                                     Mid_Line[i]=(Right_Line_New[i] + Left_Line[i]) / 2;
                                     Mid_Add[i]=1;
                               }

               }
                           Repair_Flag=0;
           }
/***********************���Ѿ��޲������߽��н���*****************************/
               if( Mid_Add[i]==1)
               {
                   Mid_Add_Start=i;
                   if (i >= 53)        //�ܹ�ʶ��������ǳ��٣����߱��ο���ֵ
                       {
                               Add_Slope = Mid_Last_Slope; //ʹ����һ֡ͼ����ұ߽�б��
                              if(Mid_Last_Slope!=0)
                               temp = (char)((i - 59) * Add_Slope + Mid_Line[59]);//ͨ��б�����㲹�ߵ�λ��
                               else
                               temp =  Mid_Line[i+1];
                       }
                       else
                       {
                           if(abs(Mid_Add_Start-M_repire)<3)  //�����Ĳ��ߵ�ʹ��ͬһб��
                                   {
                                         Add_Slope=Mid_Last_Slope;
                                   }
                                   else{
                                                   Add_Slope =Slope_calculate(Mid_Add_Start+1, Mid_Add_Start+6,Mid_Line);      //������ʶ���ǰ����ͼ��б��
                                           }
                                                   temp = (char)((i - (Mid_Add_Start+6)) * Add_Slope + Mid_Line[Mid_Add_Start+6]);//ͨ��б�����㲹�ߵ�λ��
                                                   Mid_Last_Slope = Add_Slope; //�����ϴ��ұ߽�б��
                                           }

                 Mid_Line[i] = LIMIT(temp, 93, 0);          //����������������
                 M_repire= Mid_Add_Start;                  //������һ�β��ߵĵ�
                 Mid_Add[i]=0;                              //�������
         }
     }
}
/*
����˵����������ȡ�߽�����
 */
void Get_new_Line(uint8 Line[],uint8 start_x,uint8 start_y,uint8 (*pts)[2],uint8 num)
{
    static uint8 last_x=0,last_y=0;
    last_x=start_x;
    last_y=start_y;
    for(uint8 i=0;i<num;i++)
    {
        if(last_y>pts[i][1])                   //����ҵ�������һ��
        {
           //ֱ�Ӽ�¼
            last_y=limit_a_b(last_y,1,59);
            Line[last_y-1]=pts[i][0];            //��ȡ�߽�
            last_y=pts[i][1];                        //����y����
            last_x=pts[i][0];                        //����x����
        }else if(last_y==pts[i][1])                  //�����ǰ����һ������ͬһ��
        {
            if(last_x!=pts[i][0])                     //��ͬһ�е���x���겻һ��
            {
                 Line[last_y]=pts[i][0];              //��ȡ�߽�
                 last_x=pts[i][0];                     //����x����
            }
        }
     if(last_y>=58 || last_y<=20)  //����߽�ֹͣ
     {
       break;
     }
    }
}
/*
����˵��������Ϊ�Թ�Ѳ���㷨
 */
/* ǰ�������壺
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
// �����Թ�Ѳ��
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
            uint8 front_value =img[y + dir_front[dir][1]][x + dir_front[dir][0]]; //�ĸ���˳ʱ��ת
            uint8 frontleft_value =img[y + dir_frontleft[dir][1]][x + dir_frontleft[dir][0]];//����˳ʱ��ת
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
                   && !(img[y][temp_x]>img[y-2][temp_x] && img[y+1][temp_x]>img[y-2][temp_x])) //���ϰװ׺�����
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


// �����Թ�Ѳ��
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
         && (!(img[y][temp_x]>img[y-2][temp_x] && img[y+1][temp_x]>img[y-2][temp_x])))//���ϰװ׺�����
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

// �����Թ�Ѳ��,���ڶ�ֵ��
/*img:��ֵ��ͼ��    x:���������   y:���������    pts�����������    num���ߵĲ��� ,һ��ȡ���������С*/
void findline_lefthand_binaryzation(uint8(*img)[94], uint8 x, uint8 y, uint8 (*pts)[2], int *num)
{
    int16 step = 0, dir = 0, turn = 0;
    while (step <*num && 10<y && y<59 && turn < 4) {
            uint8 front_value =img[y + dir_front[dir][1]][x + dir_front[dir][0]]; //�ĸ���˳ʱ��ת
            uint8 frontleft_value =img[y + dir_frontleft[dir][1]][x + dir_frontleft[dir][0]];//����˳ʱ��ת
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
// �����Թ�Ѳ��,���ڶ�ֵ��
/*img:��ֵ��ͼ��    x:���������   y:���������    pts���ұ�������    num���ߵĲ��� ,һ��ȡ���������С*/
void findline_righthand_binaryzation(uint8(*img)[94], uint8 x, uint8 y, uint8 (*pts)[2], int *num)
{
    int16 step = 0, dir = 0, turn = 0;
    while (step <*num && 10<y && y<59 &&  turn < 4) {   //���֣��������ҵ�ǰ����Ļ����ϣ���ֻ��Ҫ�ж�ǰ�������Ͻ�
        uint8 front_value = img[y + dir_front[dir][1]][x + dir_front[dir][0]];
        uint8 frontright_value =img[y + dir_frontright[dir][1]][x + dir_frontright[dir][0]];
        if (front_value == 0) {    //ȫ���˳�
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

// �����Թ�Ѳ��,���ڶ�ֵ��,��Ӧ���ҽǵ�
/*img:��ֵ��ͼ��    x:���������   y:���������    pts�����������    num���ߵĲ��� ,һ��ȡ���������С*/
void findline_lefthand_binaryzation_angle_user(uint8(*img)[94], uint8 x, uint8 y, uint8 (*pts)[2], int *num)
{
    int16 step = 0, dir = 0, turn = 0;
    while (step <*num && 1 < x && x < 92 && 15 < y && y < 59 && turn < 4) {
            uint8 front_value =img[y + dir_front[dir][1]][x + dir_front[dir][0]]; //�ĸ���˳ʱ��ת
            uint8 frontleft_value =img[y + dir_frontleft[dir][1]][x + dir_frontleft[dir][0]];//����˳ʱ��ת
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
// �����Թ�Ѳ��,���ڶ�ֵ��,��Ӧ���ҽǵ�
/*img:��ֵ��ͼ��    x:���������   y:���������    pts���ұ�������    num���ߵĲ��� ,һ��ȡ���������С*/
void findline_righthand_binaryzation_angle_user(uint8(*img)[94], uint8 x, uint8 y, uint8 (*pts)[2], int *num)
{
    int16 step = 0, dir = 0, turn = 0;
    while (step <*num && 1 < x && x < 92 && 15 < y && y < 59 && turn < 4) {   //���֣��������ҵ�ǰ����Ļ����ϣ���ֻ��Ҫ�ж�ǰ�������Ͻ�
        uint8 front_value = img[y + dir_front[dir][1]][x + dir_front[dir][0]];
        uint8 frontright_value =img[y + dir_frontright[dir][1]][x + dir_frontright[dir][0]];
        if (front_value == 0) {    //ȫ���˳�
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
 ����    �ܡ��������ĺͱ߽���
 ����    ����ͼ������
 ���� �� ֵ����
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
    tft180_draw_line (0,30, 93, 30,RGB565_RED);          //��ͼ���л����ָ���
    tft180_draw_line (0,20, 93, 20,RGB565_RED);          //��ͼ���л����ָ���
    tft180_draw_line (0,10, 93, 10,RGB565_RED);          //��ͼ���л����ָ���
}
/*---------------------------------------------------------------
 ����    �ܡ�ʹ�ò�ͬ��ɫ�������ĺͱ߽���
 ����    ����ͼ������
 ���� �� ֵ����
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
    tft180_draw_line (30,50, 70, 50,RGB565_GREEN);          //��ͼ���л����ָ���
    tft180_draw_line (30,40, 70, 40,RGB565_GREEN);          //��ͼ���л����ָ���
    tft180_draw_line (30,30, 70, 30,RGB565_GREEN);          //��ͼ���л����ָ���
    tft180_draw_line (30,20, 70, 20,RGB565_GREEN);          //��ͼ���л����ָ���
    tft180_draw_line (30,10, 70, 10,RGB565_GREEN);          //��ͼ���л����ָ���
}
/*---------------------------------------------------------------
 ����    �ܡ���������߽����
 ����    ����ͼ��
 ���� �� ֵ����
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
 ����    �ܡ���ĳ������Ƴ���(��ɫ)
 ����    ��������
 ���� �� ֵ����
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
 ����    �ܡ���ĳ������Ƴ���(��ɫ)
 ����    ��������
 ���� �� ֵ����
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
