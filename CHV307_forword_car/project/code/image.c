/*
 * image.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "zf_common_headfile.h"
#include "math.h"
//不需要修改变量
uint8 Left_Line[60]={0}, Mid_Line[60]={0}, Right_Line[60]={0}; //左中右边界线,默认找压缩后的图像
uint8 Left_Line_flag[60]={0}, Right_Line_flag[60]={0};       //找到边界线标志位，1为找到，0为没有找到
uint8 Width_track[60]={0};                                   //每行赛道的宽度
uint8 Loss_L_line=0,Loss_R_line=0;                  //左右丢线行数
uint8 No_Loss_L_line=0,No_Loss_R_line=0;             //左右不丢线数
uint8 REPIRE_LINE=0;                   //补线标志位
uint8 corss_deal_mid=0;                //十字中线特殊处理
bool average_steer_duty=0;              //可以进行平均打角标志位
uint8 Cross_Among=0;            //在十字之间标志位
bool l_gray_point=0,r_gray_point=0;   //左右灰度判断是否为丢边界,已经在二值化处理时进行了判断
bool vert_gray_point=0;  //竖直差比和标志位，专用于判断断路
uint8 Width_straight[60]={2,4,5,6,7,8,8,10,10,11,      //直道宽度,dubug查看，或者上位机(32cm高度)
                       11,12,12,12,12,13,13,15,15,18,
                       20,20,21,23,24,24,25,26,27,29,
                       30,31,32,33,35,35,37,38,39,41,
                       41,43,43,45,46,47,49,49,51,51,
                       53,53,55,57,57,59,59,61,62,65};
uint8 Out_door_type=1;        //2为出右库,1为出左库
float Turn_dt=0.02;                                                        //角速度累加时间
float Car_speed_dt=0.005;                                                     //距离累加时间
//59到30为45cm,到20为80cm,到10为165cm
float L_straight_slope=-0.56;         //左直道斜率，车越偏左这个值越小
float R_straight_slope=0.56;         //右直道斜率，车越偏右这个值越大
bool L_straightaway=0;                //左直道标志位
bool R_straightaway=0;                //右直道标志位
bool Barrier_brake=0;                  //障碍物刹车标志位
//以下为元素找左右边界角点变量
uint8 angle_thres=45;        //角点阈值
uint8 bottom_gray_threa=30;     //底部左右上下边界判断差比和阈值
uint8 Break_road_add_thres=15;   //断路增强阈值
#define element_line_num  35                                     //边线数组大小
uint8 element_rpts_l[(uint16)element_line_num][2];              //左边界数组x,y
uint8 element_rpts_r[(uint16)element_line_num][2];             //右边界数组x,y
float element_a_l[(uint16)element_line_num];                  //左边界角度变化率
float element_a_r[(uint16)element_line_num];                 //右边界角度变化率
float element_outangle_l[(uint16)element_line_num];         //抑制完成角度边界
float element_outangle_r[(uint16)element_line_num];        //抑制完成角度边界
int   element_rpts_num_l=(uint16)element_line_num;        //数组长度
int   element_rpts_num_r=(uint16)element_line_num;       //数组长度
uint8 element_l_angle_point=0,element_r_angle_point=0;  //左上角点，右上角点数组下标
bool left_angle_point=0;                             //左角点找到标志
bool right_angle_point=0;                            //右角点找到标志
uint8 l_angle_point_x,l_angle_point_y,r_angle_point_x,r_angle_point_y;    //记录角点原始坐标

uint8 l_upangle_point_x,l_upangle_point_y,l_downangle_point_x,l_downangle_point_y;   //记录角点上原始坐标
uint8 r_upangle_point_x,r_upangle_point_y,r_downangle_point_x,r_downangle_point_y;    //记录角点下原始坐标

float practical_l_angle_point_x,practical_l_angle_point_y,practical_r_angle_point_x,practical_r_angle_point_y;    //记录透视角点坐标

float practical_l_upangle_point_x,practical_l_upangle_point_y,practical_l_downangle_point_x,practical_l_downangle_point_y;   //记录透视上角点坐标
float practical_r_upangle_point_x,practical_r_upangle_point_y,practical_r_downangle_point_x,practical_r_downangle_point_y; //记录透视下角点坐标

float l_original_angle,r_original_angle;       //角点原始角度
float l_practical_angle,r_practical_angle;   //角点透视变换角度

bool l_ring_check_point=0;              //圆环左角点存在
bool r_ring_check_point=0;              //圆环右角点存在
uint8 l_ring_check_point_x;
uint8 l_ring_check_point_y;
uint8 r_ring_check_point_x;
uint8 r_ring_check_point_y;

bool l_cross_check_point=0;              //十字左角点存在
bool r_cross_check_point=0;              //十字右角点存在
uint8 l_cross_check_point_x=0;
uint8 l_cross_check_point_y=0;
uint8 r_cross_check_point_x=0;
uint8 r_cross_check_point_y=0;

bool l_barrier_check_point=0;              //障碍物左角点存在
bool r_barrier_check_point=0;              //障碍物右角点存在
uint8 l_barrier_check_point_x=0;
uint8 l_barrier_check_point_y=0;
uint8 r_barrier_check_point_x=0;
uint8 r_barrier_check_point_y=0;

bool l_break_road_check_point=0;              //断路左角点存在
bool r_break_road_check_point=0;              //断路右角点存在
uint8 l_break_road_check_point_x=0;
uint8 l_break_road_check_point_y=0;
uint8 r_break_road_check_point_x=0;
uint8 r_break_road_check_point_y=0;

bool l_garage_check_point=0;              //车库左角点存在
bool r_garage_check_point=0;              //车库右角点存在
uint8 l_garage_check_point_x=0;
uint8 l_garage_check_point_y=0;
uint8 r_garage_check_point_x=0;
uint8 r_garage_check_point_y=0;
//角点方向判断
bool l_grow_inside=false,r_grow_inside=false,l_grow_outside=false,r_grow_outside=false;
//透视角点标志位
bool l_small_angle_point=0;  //左边小角点,40度到80度
bool l_big_angle_point=0;  //左边大角点，80度到140度

bool r_small_angle_point=0;  //右边小角点,40度到80度
bool r_big_angle_point=0;  //右边大角点，80度到140度

uint8 l_small_angle_point_id=0; //下标
uint8 l_big_angle_point_id=0;
uint8 r_small_angle_point_id=0;
uint8 r_big_angle_point_id=0;
/*---------------------------------------------------------------
 【功    能】透视角点计算
 【参    数】无
 【返 回 值】无
 【特 别 说 明】本程序参考了上交方案，他们的采样距离是2cm,我们在缩小一半图像之后
 【特 别 说 明】本程序使用的是一半图像来算，没问题，但是角点位置不对，建议用原图像来找
 ----------------------------------------------------------------*/
void Fine_perspective_angle_point(uint8(*image)[image_w])
{
    l_practical_angle=0;
    r_practical_angle=0;
    uint8 x,y;
    element_rpts_num_l=(uint16)element_line_num;
    element_rpts_num_r=(uint16)element_line_num;
    int rpts_num_l=(uint16)element_line_num;
    int rpts_num_r=(uint16)element_line_num;
    int angle_dist=7;    //计算角度的间距
    int nms_angle_dist=7*2+1;    //角度非极大值抑制距离
    if(l_gray_point==1)
    {
      findline_lefthand_binaryzation_angle_user(image, labyrinth_start_point_l[0], labyrinth_start_point_l[1], element_rpts_l,&element_rpts_num_l);  //重新扫描边界
    }
    else
    {
      element_rpts_num_l=0;
    }

    if(r_gray_point==1)
    {
      findline_righthand_binaryzation_angle_user(image, labyrinth_start_point_r[0], labyrinth_start_point_r[1], element_rpts_r,&element_rpts_num_r);   //重新扫描
    }
    else
    {
      element_rpts_num_r=0;
    }

    //透视坐标转化
    for (uint8 i = 0; i < element_rpts_num_l; i++) {
        x=element_rpts_l[i][0]*2;
        y=element_rpts_l[i][1]*2;
        rpts_l[i][0] = mapx[y][x];
        rpts_l[i][1] = mapy[y][x];
    }
    for (uint8 i = 0; i < element_rpts_num_r; i++) {
        x=element_rpts_r[i][0]*2;
        y=element_rpts_r[i][1]*2;
        rpts_r[i][0] = mapx[y][x];
        rpts_r[i][1] = mapy[y][x];
    }
    // 边线等距采样
    resample_points(rpts_l, element_rpts_num_l, rpts_l_dist, &rpts_num_l, 0.02 * pixel_per_meter);
    resample_points(rpts_r, element_rpts_num_r, rpts_r_dist, &rpts_num_r, 0.02 * pixel_per_meter);
    // 边线局部角度变化率
    local_angle_points(rpts_l_dist, element_rpts_num_l, element_a_l, angle_dist);
    local_angle_points(rpts_r_dist, element_rpts_num_r, element_a_r, angle_dist);
    //角度变化率非极大抑制
    nms_angle(element_a_l, element_rpts_num_l, element_outangle_l, nms_angle_dist);
    nms_angle(element_a_r, element_rpts_num_r, element_outangle_r, nms_angle_dist);
    //识别大，小拐点
    l_small_angle_point=0;
    l_big_angle_point=0;
    r_small_angle_point=0;
    r_big_angle_point=0;
    for (int i = 0; i < element_rpts_num_l; i++) {            //左角点寻找
        if (element_outangle_l[i] == 0) continue;
        int im1 = clip(i -angle_dist, 0, element_rpts_num_l - 1);
        int ip1 = clip(i + angle_dist, 0, element_rpts_num_l - 1);
        float conf = fabs(element_a_l[i]) - (fabs(element_a_l[im1]) + fabs(element_a_l[ip1])) / 2;
        //左小角点阈值
        if (l_small_angle_point == false && 30.0*0.0174 < conf && conf < 60.0*0.0174) {
            l_small_angle_point_id = i;
            l_small_angle_point = true;

        }
        //左大角点阈值
        if (l_big_angle_point == false && 60.0 *0.0174 < conf && conf < 140.0 *0.0174) {
            l_big_angle_point_id = i;
            l_big_angle_point = true;
            l_angle_point_x=element_rpts_l[i][0]; //x
            l_angle_point_y=element_rpts_l[i][1];  //y
            l_practical_angle=conf/0.0174;
        }
    }

    for (int i = 0; i < element_rpts_num_r; i++) {
        if (element_outangle_r[i] == 0) continue;
        int im1 = clip(i - angle_dist, 0, element_rpts_num_r - 1);
        int ip1 = clip(i + angle_dist, 0, element_rpts_num_r - 1);
        float conf = fabs(element_a_r[i]) - (fabs(element_a_r[im1]) + fabs(element_a_r[ip1])) / 2;
        //右小角点
        if (r_small_angle_point == false && 40.0 *0.0174 < conf && conf < 80.0 *0.0174) {

            r_small_angle_point_id = i;
            r_small_angle_point = true;
        }
        //右大角点
        if (r_big_angle_point == false && 80.0 *0.0174 < conf && conf < 140.0 *0.0174) {
            r_big_angle_point_id = i;
            r_big_angle_point = true;
            r_angle_point_x=element_rpts_r[i][0]; //x
            r_angle_point_y=element_rpts_r[i][1];  //y
            r_practical_angle=conf/0.0174;
        }
    }
//显示角点
//    if(l_big_angle_point==true)
//     draw_point_red(l_angle_point_x, l_angle_point_y);
//    if(r_big_angle_point==true)
//     draw_point_blue(r_angle_point_x, r_angle_point_y);
}
/*---------------------------------------------------------------
 【功    能】寻找角点(不进行逆透视计算)
 【参    数】图像
 【返 回 值】无
 【详细说明】利用修改后的巡线固定步数重新爬取边界,寻找角点，计算逆透视角点，区分元素
 ----------------------------------------------------------------*/
void Fine_element_angle_point(uint8(*image)[image_w])
{
      uint8 numcut1=0;
      uint8 i=0;
      uint8 limit_up=0,limit_down=0;  //上下限幅
      uint8 angle_dist=7;             //采样间距
      uint8 nms_angle_dist=7*2+1;       //非极大值抑制距离
      uint8 start_l_x=Left_Line[55];
      uint8 start_l_y=55;
      uint8 start_r_x=Right_Line[55];
      uint8 start_r_y=55;

      left_angle_point=0;           //角度清空
      right_angle_point=0;          //角度清空
      l_ring_check_point=0;         //圆环左角点
      r_ring_check_point=0;        //圆环右角点
      l_cross_check_point=0;      //十字左角点
      r_cross_check_point=0;     //十字右角点
      l_garage_check_point=0;   //车库左角点
      r_garage_check_point=0;  //车库右角点
      l_break_road_check_point=0;  //断路左角点
      r_break_road_check_point=0; //断路右角点
      l_barrier_check_point=0;   //障碍物左角点
      r_barrier_check_point=0;  //障碍物右角点
      element_rpts_num_l=(uint16)element_line_num;
      element_rpts_num_r=(uint16)element_line_num;
      for(i=0;i<(uint16)element_line_num;i++)  //角度清空
      {
          element_a_l[i] = 0;
          element_a_r[i] = 0;
          element_outangle_l[i]=0;
          element_outangle_r[i]=0;
      }
      if(l_gray_point) //起点存在
      {
         findline_lefthand_binaryzation_angle_user(image, start_l_x,start_l_y, element_rpts_l,&element_rpts_num_l);  //重新扫描边界
      }
      else                        //起点不存在
      {
          element_rpts_num_l=0;
      }
      if(r_gray_point)
      {
          findline_righthand_binaryzation_angle_user(image,start_r_x, start_r_y, element_rpts_r,&element_rpts_num_r);   //重新扫描
      }
      else
      {
          element_rpts_num_r=0;
      }
      numcut1=element_rpts_num_l-1;
      for(i=0;i<element_rpts_num_l;i++)
      {
          if (i <= 0 || i >= numcut1)   //边界点角度默认0
          {
              element_a_l[i] = 0;
              continue;
          }

          limit_up=clip(i+angle_dist,0,numcut1);
          limit_down=clip(i-angle_dist,0,numcut1);

          element_a_l[i]=180-fabs(getAngle(element_rpts_l[limit_up][0],
                  element_rpts_l[limit_up][1],element_rpts_l[i][0],
                  element_rpts_l[i][1],element_rpts_l[limit_down][0],
                  element_rpts_l[limit_down][1]));
      }

      numcut1=element_rpts_num_r-1;
      for(i=0;i<element_rpts_num_r;i++)
      {
          if (i <= 0 || i >= numcut1)   //边界点角度默认0
          {
              element_a_r[i] = 0;
              continue;
          }

          limit_up=clip(i+angle_dist,0,numcut1);
          limit_down=clip(i-angle_dist,0,numcut1);

          element_a_r[i]=180-fabs(getAngle(element_rpts_r[limit_up][0],
                  element_rpts_r[limit_up][1],element_rpts_r[i][0],
                  element_rpts_r[i][1],element_rpts_r[limit_down][0],
                  element_rpts_r[limit_down][1]));
      }

          nms_angle(element_a_l, element_rpts_num_l, element_outangle_l, nms_angle_dist);   //角度极大值抑制
          nms_angle(element_a_r, element_rpts_num_r, element_outangle_r,nms_angle_dist);

          for(i=0;i<element_rpts_num_l;i++)         //提取左边界角点
          {
              if(element_outangle_l[i]==0)
                  continue;
              //断路
             if(element_outangle_l[i]>30  && l_break_road_check_point==0)
             {
                 l_break_road_check_point=1;
                 l_break_road_check_point_x=element_rpts_l[i][0];
                 l_break_road_check_point_y=i;
             }
              if(element_outangle_l[i]>angle_thres)
              {
                  //圆环角点
                  if(element_outangle_l[i]<140 && element_outangle_l[i]>90&& l_ring_check_point==0)   //圆环左角点
                  {
                      l_ring_check_point=1;
                      l_ring_check_point_x=element_rpts_l[i][0];
                      l_ring_check_point_y=element_rpts_l[i][1];
                  }
                  //十字或者车库角点
                  if(element_outangle_l[i]<130 && element_outangle_l[i]>70 && (l_cross_check_point==0 || l_garage_check_point==0))
                  {
                      if(l_cross_check_point==0)
                      {
                          l_cross_check_point=1;
                          l_cross_check_point_x=element_rpts_l[i][0];
                          l_cross_check_point_y=element_rpts_l[i][1];
                      }
                      if(l_garage_check_point==0)
                      {
                          l_garage_check_point=1;
                          l_garage_check_point_x=element_rpts_l[i][0];
                          l_garage_check_point_y=element_rpts_l[i][1];
                      }
                  }
                 //障碍物
                 if(element_outangle_l[i]>angle_thres && l_barrier_check_point==0)
                 {
                     l_barrier_check_point=1;
                     l_barrier_check_point_x=element_rpts_l[i][0];
                     l_barrier_check_point_y=element_rpts_l[i][1];
                 }

                 //正常判断
                 if(left_angle_point==0)
                 {
                 l_angle_point_x= element_rpts_l[i][0];
                 l_angle_point_y= element_rpts_l[i][1];
                 l_original_angle=element_outangle_l[i];    //角点角度
                 left_angle_point=true;
                 element_l_angle_point=i;
                 }
              }
          }

          for(i=0;i<element_rpts_num_r;i++)          //提取角点
          {
              if(element_outangle_r[i]==0)
                  continue;
              //断路
             if(element_outangle_r[i]>30  && r_break_road_check_point==0)
             {
                 r_break_road_check_point=1;
                 r_break_road_check_point_x=element_rpts_r[i][0];
                 r_break_road_check_point_y=i;
             }
              if(element_outangle_r[i]>angle_thres)
              {
                  //圆环角点
                  if(element_outangle_r[i]<140 && element_outangle_r[i]>90 && r_ring_check_point==0)   //圆环右角点
                  {
                      r_ring_check_point=1;
                      r_ring_check_point_x=element_rpts_r[i][0];
                      r_ring_check_point_y=element_rpts_r[i][1];
                  }
                  //十字或者车库角点
                  if(element_outangle_r[i]<130 && element_outangle_r[i]>70 && (r_cross_check_point==0 || r_garage_check_point==0))
                  {
                      if(r_cross_check_point==0)
                      {
                          r_cross_check_point=1;
                          r_cross_check_point_x=element_rpts_r[i][0];
                          r_cross_check_point_y=element_rpts_r[i][1];
                      }
                      if(r_garage_check_point==0)
                      {
                          r_garage_check_point=1;
                          r_garage_check_point_x=element_rpts_r[i][0];
                          r_garage_check_point_y=element_rpts_r[i][1];
                      }
                  }
                 //障碍物
                 if(element_outangle_r[i]>angle_thres && r_barrier_check_point==0)
                 {
                     r_barrier_check_point=1;
                     r_barrier_check_point_x=element_rpts_r[i][0];
                     r_barrier_check_point_y=element_rpts_r[i][1];
                 }
                 //正常判断
                 if(right_angle_point==0)
                 {
                 r_angle_point_x= element_rpts_r[i][0];
                 r_angle_point_y= element_rpts_r[i][1];
                 r_original_angle=element_outangle_r[i];
                 right_angle_point=true;
                 element_r_angle_point=i;
            }
        }
    }
  // 显示角点
//          if(l_break_road_check_point==true)
//          {
//              draw_point_red(l_break_road_check_point_x, l_break_road_check_point_y);
//          }
//          if(r_break_road_check_point==true)
//          {
//             draw_point_blue(r_break_road_check_point_x, r_break_road_check_point_y);
//          }
}
/*---------------------------------------------------------------
 【功    能】计算二值化角点透视角度
 【参    数】无
 【返 回 值】无
 ----------------------------------------------------------------*/
void Calculate_perspective_angle()
{
    uint8 angle_dist=5;             //采样距离
    uint8 limit_up,limit_down;     //限幅
    l_practical_angle=0;
    r_practical_angle=0;
    if(left_angle_point==true)    //左角点存在
    {
       //对原图角点坐标进行透视转化
        practical_l_angle_point_x = mapx[element_rpts_l[element_l_angle_point][1]*2][element_rpts_l[element_l_angle_point][0]*2];
        practical_l_angle_point_y = mapy[element_rpts_l[element_l_angle_point][1]*2][element_rpts_l[element_l_angle_point][0]*2];
        //远处采样点
        limit_up=clip(element_l_angle_point+angle_dist,0,element_rpts_num_l-1);
        practical_l_upangle_point_x=mapx[element_rpts_l[limit_up][1]*2][element_rpts_l[limit_up][0]*2];
        practical_l_upangle_point_y=mapy[element_rpts_l[limit_up][1]*2][element_rpts_l[limit_up][0]*2];
        //近处采样点
        limit_down=clip(element_l_angle_point-angle_dist,0,element_rpts_num_l-1);
        practical_l_downangle_point_x=mapx[element_rpts_l[limit_down][1]*2][element_rpts_l[limit_down][0]*2];
        practical_l_downangle_point_y=mapy[element_rpts_l[limit_down][1]*2][element_rpts_l[limit_down][0]*2];
        //计算透视角度
        l_practical_angle=fabs(Three_angle_points(practical_l_downangle_point_x,practical_l_downangle_point_y,practical_l_angle_point_x,practical_l_angle_point_y,practical_l_upangle_point_x,practical_l_upangle_point_y));
        l_practical_angle/=0.0174;
    }
    if(right_angle_point==true)    //右角点存在
    {
       //对原图角点坐标进行透视转化
        practical_r_angle_point_x = mapx[element_rpts_r[element_r_angle_point][1]*2][element_rpts_r[element_r_angle_point][0]*2];
        practical_r_angle_point_y = mapy[element_rpts_r[element_r_angle_point][1]*2][element_rpts_r[element_r_angle_point][0]*2];
        //远处采样点
        limit_up=clip(element_r_angle_point+5,0,element_rpts_num_r-1);
        practical_r_upangle_point_x=mapx[element_rpts_r[limit_up][1]*2][element_rpts_r[limit_up][0]*2];
        practical_r_upangle_point_y=mapy[element_rpts_r[limit_up][1]*2][element_rpts_r[limit_up][0]*2];
        //近处采样点
        limit_down=clip(element_r_angle_point-5,0,element_rpts_num_r-1);
        practical_r_downangle_point_x=mapx[element_rpts_r[limit_down][1]*2][element_rpts_r[limit_down][0]*2];
        practical_r_downangle_point_y=mapy[element_rpts_r[limit_down][1]*2][element_rpts_r[limit_down][0]*2];
        //计算透视角度
        r_practical_angle=fabs(Three_angle_points(practical_r_downangle_point_x,practical_r_downangle_point_y,practical_r_angle_point_x,practical_r_angle_point_y,practical_r_upangle_point_x,practical_r_upangle_point_y));
        r_practical_angle/=0.0174;
    }
}
/*---------------------------------------------------------------
 【功    能】判断角点是向内还是向外
 【参    数】无
 【返 回 值】无
 【详细说明】向内和向外可以区分障碍物和断路，经过非极大值抑制，留下的是局部最大值
 【详细说明】既然大于45°的存在，那就必存在角点，剩下的就是靠角度大小区分元素
 ----------------------------------------------------------------*/
void Find_angle_point_grow_dir()
{
     int8 l_count=0,r_count=0;
     uint8 upper_l,upper_r;
     l_grow_inside=false,r_grow_inside=false,l_grow_outside=false,r_grow_outside=false;
     if(left_angle_point==true)                                           //存在左角点
     {
         upper_l=clip(element_l_angle_point+15,0,element_rpts_num_l-1);    //向上判断15个点
         for(uint8 i=element_l_angle_point+1;i<upper_l;i++)
         {
            if (element_rpts_l[i][0]<element_rpts_l[element_l_angle_point][0])    //左角点向外生长
            {
                l_count++;
            }
            else if(element_rpts_l[i][0]>element_rpts_l[element_l_angle_point][0])  //左角点向内生长
            {
                l_count--;
            }
            if(l_count>7)
            {
                l_grow_outside=true;
                break;
            }
            if(l_count<-7)
            {
                l_grow_inside=true;
                break;
            }

         }
     }

     if(right_angle_point==true)
     {
         upper_r=clip(element_r_angle_point+15,0,element_rpts_num_r-1);
         for(uint8 i=element_r_angle_point+1;i<upper_r;i++)
         {
            if(element_rpts_r[i][0]>element_rpts_r[element_r_angle_point][0])//右角点向外生长
            {
                r_count++;
            }
            else if(element_rpts_r[i][0]<element_rpts_r[element_r_angle_point][0])//右角点向内生长
            {
                r_count--;
            }

            if(r_count>7)
            {
                r_grow_outside=true;
                break;
            }
            if(r_count<-7)
            {
                r_grow_inside=true;
                break;
            }

         }

     }
}
/*---------------------------------------------------------------
 【功    能】角点测试函数
 【参    数】图像数组
 ----------------------------------------------------------------*/
void Test_angle_point(uint8(*image)[image_w])
{
        Fine_element_angle_point(image);       //判断角点
        Find_angle_point_grow_dir();          //判断角点生长方向
        Calculate_perspective_angle();            //计算透视角点角度
        if(left_angle_point==true)       //二值化左角点
        {
            draw_point_red(l_angle_point_x, l_angle_point_y);
        }
        if(right_angle_point==true)      //二值化右角点
        {
           draw_point_blue(r_angle_point_x, r_angle_point_y);
        }
       draw_point_red(labyrinth_start_point_l[0], labyrinth_start_point_l[1]);  //左边界起点
       draw_point_blue(labyrinth_start_point_r[0], labyrinth_start_point_r[1]);//右边界起点
       tft180_show_uint(95,0,left_angle_point,1);
       tft180_show_uint(108,0,right_angle_point,1);

       tft180_show_float(95,16,l_original_angle,3,2);
       tft180_show_float(95,32,r_original_angle,3,2);
       tft180_show_float(95,48,l_practical_angle,3,2);
       tft180_show_float(95,64,r_practical_angle,3,2);

       tft180_show_uint(0,80,l_grow_inside,1);
       tft180_show_uint(20,80,r_grow_inside,1);
       tft180_show_uint(40,80,l_grow_outside,1);
       tft180_show_uint(60,80,r_grow_outside,1);
}
/*---------------------------------------------------------------
 【功    能】严格直线判断函数
 【参    数】传入数组,判断边界类型
 【返 回 值】1为直线 0为非直线
 ----------------------------------------------------------------*/
uint8 Straight_line_judgment(uint8 arr[],uint8 type,int8 st,int8 en)
{
    short i,sum=0;
    short start=st,end=en;
    float kk;
    kk=((float)arr[start]-(float)arr[end])/(start-end);
    sum=0;
    short l_num=0,r_num=0;
    for(i=end;i>=start;i--)
    {
        if(fabs((arr[end]+(float)(i-end)*kk)-arr[i])<3 ) sum++;
        else break;

        if(type==1)
        {
          if(Left_Line_flag[i]==0)         //6个点丢线，直接判断为非直道
          {
              l_num++;
          }
          if(l_num>5)
          {
              sum=0;
              break;
          }

        }
        else if(type==2)
        {
            if(Right_Line_flag[i]==0)
            {
                r_num++;
            }
            if(r_num>5)
            {
                sum=0;
                break;
            }
        }

    }
    if(sum>=((end-start)-4) && kk>-1.2 && kk<1.2)  return 1;
    else return 0;
}
/*---------------------------------------------------------------
 【功    能】平移边界线
 【参    数】开始行  结束行  type:1 平移左边界  2  平移右边界
 【返 回 值】无
 ----------------------------------------------------------------*/
void Reair_Mid_Line(uint8 begin,uint8 end,uint8 type)
{
    uint8 i=0;
    for(i=begin;i<=end;i++)
    {
        if(type==1)
        {
            Mid_Line[i]=clip(Left_Line[i]+Width_straight[i]/2,0,93);
        }else if(type==2)
        {
            Mid_Line[i]=clip(Right_Line[i]-Width_straight[i]/2,0,93);
        }
    }
}
/*---------------------------------------------------------------
 【功    能】S弯判断
 【参    数】无
 【返 回 值】无
 【修改日期】2023/4/25
 ----------------------------------------------------------------*/
bool s_state[2]={0,0};              //小S,中S
void Check_Small_S()
{
    uint8 i;
    bool  point1_flag=0;           //中线>拐点标志位
    bool  point2_flag=0;          //中线<拐点标志位
    uint8 mid_line_num=0;      //中线连续数
    s_state[0]=0;
    if(Flag_Cross==0 && L_CirCle_State==0 && R_CirCle_State==0 &&
        L_slope_cross_state==0 && R_slope_cross_state==0
       && hightest<15)            //十字与圆环不进行S弯处理
    {
    for(i=50;i>=30;i--)
    {
        if(i-8<hightest)   break;
        //>弯判断,寻找转折点
        if(Mid_Line[i]-Mid_Line[i+6]>=3 && Mid_Line[i]-Mid_Line[i+7]>=3 &&  Mid_Line[i]-Mid_Line[i+8]>=3 &&
           Mid_Line[i]-Mid_Line[i-6]>=3 && Mid_Line[i]-Mid_Line[i-7]>=3 &&  Mid_Line[i]-Mid_Line[i-8]>=3 && point1_flag==0)
        {
            point1_flag=1;
        }
        //<弯判断,寻找转折点
        if(Mid_Line[i+6]-Mid_Line[i]>=3 && Mid_Line[i+7]-Mid_Line[i]>=3 &&  Mid_Line[i+8]-Mid_Line[i]>=3 &&
            Mid_Line[i-6]-Mid_Line[i]>=3 && Mid_Line[i-7]-Mid_Line[i]>=3 &&  Mid_Line[i-8]-Mid_Line[i]>=3 && point2_flag==0)
        {
             point2_flag=1;
        }
        if(Left_Line_flag[i] && Right_Line_flag[i])
        {
            mid_line_num++;
        }
    }
    if(point1_flag && point2_flag && mid_line_num>25)   //双拐点存在,中线几乎不丢
    {
            s_state[0]=1;        //确认是小S
    }
  }
}
/*---------------------------------------------------------------
 【功    能】适当修补边界，通过平移边界获取中线
 【参    数】无
 【返 回 值】无
 【修改日期】2023/4/24
 ----------------------------------------------------------------*/
void Repire()
{
    int8  i=0;
    uint8 l_i=20,r_i=20;
    if(L_CirCle_State==0 || R_CirCle_State==0)  //圆环不进行处理，留给圆环处理函数进行操作
    {

    for(i=55;i>=20;i--)
    {
        if(Left_Line_flag[i]==0)
        {
            l_i=i+1;              //记录丢线开始点
            break;
        }
    }
    for(i=55;i>=20;i--)
    {
        if(Right_Line_flag[i]==0)
        {
            r_i=i+1;
            break;
        }
    }

    if(l_i<r_i && L_straightaway==1 && l_gray_point==1 && r_gray_point==1)  //只对两边底部起点存在，且单边是直道进行平移
    {
        Reair_Mid_Line(l_i,r_i,1);
    }
    else if(l_i>r_i && R_straightaway==1 && l_gray_point==1 && r_gray_point==1) //只对两边底部起点存在，且单边是直道进行平移
    {
        Reair_Mid_Line(r_i,l_i,2);
    }

    }
    //判断是否在十字里面
    if(Flag_Cross==1 && Cross_Among==0)  //经过第一次十字
    {
        Cross_Among=1;
    }
    if( Flag_Cross==0 && Cross_Among==1) //在十字里面
    {
        Cross_Among=2;
    }
    if( Flag_Cross==1 && Cross_Among==2)                          //出十字
    {
        Cross_Among=0;
    }
}
/***********************C     D*******************
                       *       *
                    *             */
/******************A               B***************/
#define cross_line_num   40                                      //边线数组大小
uint8 AX=0,AY=0;                                                //十字左下拐点
uint8 BX=0,BY=0;                                               //十字右下拐点
uint8 CX=0,CY=0;                                              //十字左上拐点
uint8 DX=0,DY=0;                                             //十字右上拐点
uint8 up_l_angle_point=0,up_r_angle_point=0;        //左上角点，右上角点数组下标
uint8 Flag_Cross=0;                                                        //十字路口状态位
/*---------------------------------------------------------------
 【功    能】正入十字(车在十字入口时白色部分占35行)
 【参    数】二值化图像
 【返 回 值】无
 【修改日期】2023/4/24
 ----------------------------------------------------------------*/
void cross_deal(uint8(*image)[image_w])
{
     uint8 temp_x=0;
     uint8 numcut1 = 0;
     float L_slope=0,R_slope=0;                //左右边界斜率
     uint8 i=0,White_row=0,repire=0;
     uint8 l_lock=0,r_lock=0;                //十字锁
     corss_deal_mid=0;
     element_rpts_num_l=(uint16)element_line_num;        //数组长度
     element_rpts_num_r=(uint16)element_line_num;       //数组长度
     uint8 limit_up=0,limit_down=0;  //上下限幅
     uint8 angle_dist=7;                //采样间距
     uint8 nums_angle_dist=7*2+1;           //角度非极大抑制距离
     if(Flag_Cross==0 && L_CirCle_State==0 && R_CirCle_State==0)                                                          //清0
     {
         White_row=Fine_width_baihang(2,25,35,image,4); //判断是否出现连续白行(函数4行白行符合)
         if(White_row)
         {
             if(l_cross_check_point && r_cross_check_point
               && l_grow_outside && r_grow_outside)   //十字角点判断
             {
                 regression(2,25, 55);                 //右线拟合
                 R_slope=parameterB;
                 regression(1, 25, 55);                 //左线拟合
                 L_slope=parameterB;
                 if(L_slope>0.1 && R_slope<(-0.1))       //十字的左右斜率正负会对调
                 Flag_Cross=1;
                 Beep_set(1);
             }
         }
     }

  if(Flag_Cross!=0)
  {
     switch(Flag_Cross)
     {
     case 1:
     AY=Find_guaidian(30 ,58,Left_Line,1,1);
     BY=Find_guaidian(30 ,58,Right_Line,2,1);
     if(AY==0)
     {
         AY=59;
     }
     if(BY==0)
     {
         BY=59;
     }
     if(AY<=48 && BY<=48)     //如果拐点较远，直接斜率补线
     {
      AX=Left_Line[AY];
      BX=Right_Line[BY];
      Add_line_user_twopoint_slope(1,Left_Line, AX,AY,Left_Line[59],59,30);
      Add_line_user_twopoint_slope(2,Right_Line, BX,BY,Right_Line[59],59,30);
      for(i=48;i>=20;i--)
      {
        Mid_Line[i]=clip((Left_Line[i]+Right_Line[i])/2,0,93);
      }
     }
     else if(AY<=48 && BY>48)  //哪边远补哪边
     {
         AX=Left_Line[AY];
         Add_line_user_twopoint_slope(1,Left_Line, AX,AY,Left_Line[59],59,30);
         for(i=48;i>=20;i--)
         {
           Mid_Line[i]=clip(Left_Line[i]+Width_straight[i]/2,0,93);
         }
     }
     else if(AY>48 && BY<=48)//哪边远补哪边
     {
         BX=Right_Line[BY];
         Add_line_user_twopoint_slope(2,Right_Line, BX,BY,Right_Line[59],59,30);
         for(i=48;i>=20;i--)
         {
           Mid_Line[i]=clip(Right_Line[i]-Width_straight[i]/2,0,93);
         }
     }
     else //都不远，开始两点补线
     {
         Flag_Cross=2;                  //即将进入十字
     }
   break;
   case 2:
   if(Left_Line_flag[start_line_point_row]==0 && Right_Line_flag[start_line_point_row]==0
      &&Left_Line_flag[start_line_point_row-1]==0 && Right_Line_flag[start_line_point_row-1]==0
      &&Left_Line_flag[start_line_point_row-2]==0 && Right_Line_flag[start_line_point_row-2]==0)
      {
          Flag_Cross=3;                  //进入十字
      }
   for(i=50;i>20;i--)        //判断丢线恢复高度
   {
       if(l_lock==0 && Left_Line_flag[i]==0 && Left_Line_flag[i-1]==0 && Left_Line_flag[i-2]==1
          && Left_Line_flag[i-3]==1)
       {
           l_lock=i-2;
       }
       if(r_lock==0 && Right_Line_flag[i]==0 && Right_Line_flag[i-1]==0 && Right_Line_flag[i-2]==1
          && Right_Line_flag[i-3]==1)
       {
           r_lock=i-2;
       }
   }

   if(l_lock==0)   l_lock=10;
   if(r_lock==0)   r_lock=10;

   if(l_lock<r_lock)    //右边矮
   {
       BX=0;BY=0;                                        //清空坐标
       DX=0;DY=0;
       BY=Find_guaidian(45 ,start_line_point_row,Right_Line,2,1);  //寻找右下拐点B,从下往上找
       if(BY!=0)
        {
           BX=Right_Line[BY];
        }
       else
        {
           BY=59;
           BX=47+Width_straight[BY]/2;
        }
       for(i=BY-4;i>20;i-=3)                     //从右下拐点向上扫描
              {
                 if(image[i][BX]==0 && image[i-1][BX]==0)
                 {
                         DY=i;                              //记录遇到的第一个右黑点位置
                         break;
                 }
             }
       if(DY!=0)                      //如果有十字特征
          {
               DX=BX;
          }
       else                                   //如果没找到D的Y坐标,给定值
          {
               DY=BY-35;
               DX=BX;
          }
       for(i=DX;i>26;i-=2)                     //从第一个右黑点位置  向左扫描
           {
                  if(image[DY][i]==255 && image[DY][i-1]==255)
                  {
                          DX=i;
                          break;
                  }
           }
       if(DX==BX)                          //如果没找到，给定值
       {
           DX=BX-15;
       }

       Add_line(2,Right_Line,DX,DY,BX,BY);            //右补线
       for(i=59;i>=20;i--)
          Mid_Line[i]=Right_Line[i]-Width_straight[i]/2;
   }

       if(l_lock>r_lock)          //左边矮
       {
           AX=0;AY=0;              //清空坐标
           CX=0;CY=0;
           AY=Find_guaidian(45 ,start_line_point_row,Left_Line,1,1);   //寻找左下拐点A,从下往上找
           if(AY!=0)                                                //如果找到
           {
              AX=Left_Line[AY];                                      //记录拐点列坐标
           }else
           {
             AY=59;
             AX=47-Width_straight[AY]/2;                                                 //没找到给定值
           }
           for(i=AY-4;i>20;i-=3)                       //从左下拐点向上扫描
              {
                 if(image[i][AX]==0 && image[i-1][AX]==0)
                    {
                         CY=i;                   //记录遇到的第一个左黑点位置
                         break;
                     }
                }
           if(CY!=0)                      //如果有十字特征
              {
                   CX=AX;
              }
           else                                   //如果没找到C的Y坐标,给定值
              {
                   CY=AY-35;
                   CX=AX;
              }
           for(i=CX;i<66;i+=2)                     //从第一个左黑点位置  向右扫描
               {
                if(image[CY][i]==255 && image[CY][i+1]==255)
                  {
                              CX=i;
                              break;
                  }
               }
           if(CX==AX)                          //如果没找到，给定值
           {
               CX=AX+15;
           }
               Add_line(1,Left_Line,CX,CY,AX,AY);             //左补线
               for(i=59;i>=20;i--)
                  Mid_Line[i]=Left_Line[i]+Width_straight[i]/2;
       }

       if(l_lock==r_lock)   //车不偏
       {
       AX=0;AY=0;BX=0;BY=0;                                        //清空坐标
       CX=0;CY=0;DX=0;DY=0;
       AY=Find_guaidian(45 ,start_line_point_row,Left_Line,1,1);   //寻找左下拐点A,从下往上找
       BY=Find_guaidian(45 ,start_line_point_row,Right_Line,2,1);  //寻找右下拐点B,从下往上找
       if(AY!=0)                                                //如果找到
       {
          AX=Left_Line[AY];                                      //记录拐点列坐标
       }else
       {
         AY=59;
         AX=47-Width_straight[AY]/2;                                                 //没找到给定值
       }
      if(BY!=0)
       {
          BX=Right_Line[BY];
       }
      else
       {
          BY=59;
          BX=47+Width_straight[BY]/2;
       }
     for(i=AY-4;i>20;i-=3)                       //从左下拐点向上扫描
        {
           if(image[i][AX]==0 && image[i-1][AX]==0)
              {
                   CY=i;                   //记录遇到的第一个左黑点位置
                   break;
               }
          }
     for(i=BY-4;i>20;i-=3)                     //从右下拐点向上扫描
            {
               if(image[i][BX]==0 && image[i-1][BX]==0)
               {
                       DY=i;                              //记录遇到的第一个右黑点位置
                       break;
               }
           }
    if(CY!=0 && DY!=0)                      //如果有十字特征
       {
            CX=AX;
            DX=BX;
       }
    else                                   //如果没找到C,D的Y坐标,给定值
       {
            CY=AY-35;
            DY=BY-35;
            CX=AX;
            DX=BX;
       }
    for(i=CX;i<66;i+=2)                     //从第一个左黑点位置  向右扫描
        {
         if(image[CY][i]==255 && image[CY][i+1]==255)
           {
                       CX=i;
                       repire+=1;
                       break;
           }
        }
    for(i=DX;i>26;i-=2)                     //从第一个右黑点位置  向左扫描
        {
               if(image[DY][i]==255 && image[DY][i-1]==255)
               {
                       DX=i;
                       repire+=1;
                       break;
               }
        }

    if(CX==AX && DX==BX)                          //如果没找到，给定值
    {
        CX=AX+15;
        DX=BX-15;
    }
    if(repire==2)                                   //允许补线标志位
     {
        Add_line(1,Left_Line,CX,CY,AX,AY);             //左补线
        Add_line(2,Right_Line,DX,DY,BX,BY);            //右补线
        corss_deal_mid=(CY+DY)/2;
        for(i=59;i>=corss_deal_mid;i--)
           Mid_Line[i]=(Right_Line[i]+Left_Line[i])/2;
     }
   }
    break;
    case 3:
        //退出十字状态判断
       if(Left_Line_flag[start_line_point_row-1]==1 && Right_Line_flag[start_line_point_row-1]==1 &&
          Left_Line_flag[start_line_point_row-2]==1 && Right_Line_flag[start_line_point_row-2]==1 &&
          Left_Line_flag[start_line_point_row-3]==1 && Right_Line_flag[start_line_point_row-3]==1&&
          Left_Line_flag[start_line_point_row-5]==1 && Right_Line_flag[start_line_point_row-5]==1 )
          {
            Flag_Cross=0;
            Beep_set(0);
          }

       for(i=50;i>15;i--)        //判断丢线恢复高度
       {
           if(l_lock==0 && Left_Line_flag[i]==0 && Left_Line_flag[i-1]==0 && Left_Line_flag[i-2]==1
              && Left_Line_flag[i-3]==1)
           {
               l_lock=i-2;
           }
           if(r_lock==0 && Right_Line_flag[i]==0 && Right_Line_flag[i-1]==0 && Right_Line_flag[i-2]==1
              && Right_Line_flag[i-3]==1)
           {
               r_lock=i-2;
           }
       }

       if(l_lock==0)   l_lock=10;
       if(r_lock==0)   r_lock=10;
       element_rpts_num_l=(uint16)element_line_num;        //数组长度
       element_rpts_num_r=(uint16)element_line_num;       //数组长度

       if(l_lock<r_lock)    //右边矮
       {
           for(i=50;i>20;i--)
           {
              if(image[i][80]==255 && image[i-1][80]==255 && image[i-2][80]==0 && image[i-3][80]==0 )
              {
                 DX=80;
                 DY=i-1;
                 break;
              }
           }
           findline_righthand_binaryzation_angle_user(image, DX, DY, element_rpts_r,&element_rpts_num_r);   //重新扫描
           for(i=0;i<(uint16)element_line_num;i++)  //角度清空
           {
               element_a_r[i] = 0;
               element_outangle_r[i]=0;
           }
           numcut1=element_rpts_num_r-1;
           for(i=0;i<element_rpts_num_r;i++)
           {
               if (i <= 0 || i >= numcut1)
               {
                   element_a_r[i] = 0;
                   continue;
               }

               limit_up=clip(i+angle_dist,0,numcut1);
               limit_down=clip(i-angle_dist,0,numcut1);

               element_a_r[i]=180-fabs(getAngle(element_rpts_r[limit_up][0],
                       element_rpts_r[limit_up][1],element_rpts_r[i][0],
                       element_rpts_r[i][1],element_rpts_r[limit_down][0],
                       element_rpts_r[limit_down][1]));
           }
           nms_angle(element_a_r, element_rpts_num_r, element_outangle_r,nums_angle_dist);
           for(i=0;i<element_rpts_num_r;i++)          //提取角点
           {
               if(element_outangle_r[i]==0)
                   continue;
               if(element_outangle_r[i]>40)
               {
                  BX= element_rpts_r[i][0];
                  BY= element_rpts_r[i][1];
                  up_r_angle_point=i;
                  break;
               }
           }

           for(i=BY;i>15;i--)       //寻找恢复点进行补线
           {
               if(Right_Line_flag[i]==1 && Right_Line_flag[i-1]==1)
               {
                   BX= Right_Line[i];
                   BY=i;
                   break;
               }
           }

           if(r_lock>35)
           {
               Add_line_other(2, Right_Line, BY-15, BY, BY, 59);
           }else{
               temp_x=78 ;
               Add_line(2,Right_Line,BX,BY,temp_x,59);            //右补线
           }

              for(i=59;i>20;i--)
              {
                  Mid_Line[i]=(Right_Line[i]-Width_straight[i]/2);
              }
       }

       if(l_lock>r_lock)          //左边矮
       {
           for(i=50;i>20;i--)
           {
              if(image[i][13]==255 && image[i-1][13]==255 && image[i-2][13]==0 && image[i-3][13]==0 )
              {
                 CX=13;
                 CY=i-1;
                 break;
              }
           }
           findline_lefthand_binaryzation_angle_user(image, CX, CY, element_rpts_l,&element_rpts_num_l);  //重新扫描边界
           for(i=0;i<(uint16)element_line_num;i++)  //角度清空
           {
               element_a_l[i] = 0;
               element_outangle_l[i]=0;
           }
           numcut1=element_rpts_num_l-1;
           for(i=0;i<element_rpts_num_l;i++)
           {
               if (i <= 0 || i >= numcut1)
               {
                   element_a_l[i] = 0;
                   continue;
               }

               limit_up=clip(i+angle_dist,0,numcut1);
               limit_down=clip(i-angle_dist,0,numcut1);

               element_a_l[i]=180-fabs(getAngle(element_rpts_l[limit_up][0],
                       element_rpts_l[limit_up][1],element_rpts_l[i][0],
                       element_rpts_l[i][1],element_rpts_l[limit_down][0],
                       element_rpts_l[limit_down][1]));
           }
           nms_angle(element_a_l, element_rpts_num_l, element_outangle_l, nums_angle_dist);   //角度极大值抑制
           for(i=0;i<element_rpts_num_l;i++)         //提取角点
           {
               if(element_outangle_l[i]==0)
                   continue;
               if(element_outangle_l[i]>40)
               {
                  AX=element_rpts_l[i][0];
                  AY=element_rpts_l[i][1];
                  up_l_angle_point=i;
                  break;
               }
           }

           for(i=AY;i>15;i--)       //寻找恢复点进行补线
           {
               if(Left_Line_flag[i]==1 && Left_Line_flag[i-1]==1)
               {
                   AX= Left_Line[i];
                   AY=i;
                   break;
               }
           }

           if(l_lock>35)
           {
               Add_line_other(1, Left_Line, AY-15, AY, AY, 59);
           }else{
               temp_x=15;
               Add_line(1,Left_Line,AX,AY,temp_x,59);            //左补线
           }

           for(i=59;i>=20;i--)
           {
              Mid_Line[i]=(Left_Line[i]+Width_straight[i]/2);
           }
       }

    if(l_lock==r_lock)      //要么相等要么没找到
     {
       for(i=50;i>20;i--)
       {
          if(image[i][78]==255 && image[i-1][78]==255 && image[i-2][78]==0 && image[i-3][78]==0 )
          {
             DX=78;
             DY=i-1;
             break;
          }
       }
       for(i=50;i>20;i--)
       {
          if(image[i][15]==255 && image[i-1][15]==255 && image[i-2][15]==0 && image[i-3][15]==0 )
          {
             CX=15;
             CY=i-1;
             break;
          }
       }
       //以下为角点寻找
       findline_lefthand_binaryzation_angle_user(image, CX, CY, element_rpts_l,&element_rpts_num_l);  //重新扫描边界
       findline_righthand_binaryzation_angle_user(image, DX, DY, element_rpts_r,&element_rpts_num_r);   //重新扫描
       for(i=0;i<(uint16)element_line_num;i++)  //角度清空
       {
           element_a_l[i] = 0;
           element_a_r[i] = 0;
           element_outangle_l[i]=0;
           element_outangle_r[i]=0;
       }

       numcut1=element_rpts_num_l-1;
       for(i=0;i<element_rpts_num_l;i++)
       {
           if (i <= 0 || i >= numcut1)
           {
               element_a_l[i] = 0;
               continue;
           }
           limit_up=clip(i+angle_dist,0,numcut1);
           limit_down=clip(i-angle_dist,0,numcut1);

           element_a_l[i]=180-fabs(getAngle(element_rpts_l[limit_up][0],
                   element_rpts_l[limit_up][1],element_rpts_l[i][0],
                   element_rpts_l[i][1],element_rpts_l[limit_down][0],
                   element_rpts_l[limit_down][1]));
       }

       numcut1=element_rpts_num_r-1;
       for(i=0;i<element_rpts_num_r;i++)
       {
           if (i <= 0 || i >= numcut1)
           {
               element_a_r[i] = 0;
               continue;
           }
           limit_up=clip(i+angle_dist,0,numcut1);
           limit_down=clip(i-angle_dist,0,numcut1);

           element_a_r[i]=180-fabs(getAngle(element_rpts_r[limit_up][0],
                   element_rpts_r[limit_up][1],element_rpts_r[i][0],
                   element_rpts_r[i][1],element_rpts_r[limit_down][0],
                   element_rpts_r[limit_down][1]));
       }
       nms_angle(element_a_l, element_rpts_num_l, element_outangle_l,nums_angle_dist);   //角度极大值抑制
       nms_angle(element_a_r, element_rpts_num_r, element_outangle_r,nums_angle_dist);
       for(i=0;i<element_rpts_num_l;i++)         //提取角点
       {
           if(element_outangle_l[i]==0)
               continue;
           if(element_outangle_l[i]>40)
           {
              AX=element_rpts_l[i][0];
              AY=element_rpts_l[i][1];
              up_l_angle_point=i;
              break;
           }
       }

       for(i=0;i<element_rpts_num_r;i++)          //提取角点
       {
           if(element_outangle_r[i]==0)
               continue;
           if(element_outangle_r[i]>40)
           {
              BX= element_rpts_r[i][0];
              BY= element_rpts_r[i][1];
              up_r_angle_point=i;
              break;
           }
       }
           Add_line(1,Left_Line,AX,AY,13,59);             //左补线
           Add_line(2,Right_Line,BX,BY,80,59);            //右补线
           corss_deal_mid=(AY+BY)/2;
           for(i=59;i>corss_deal_mid;i--)
           {
              Mid_Line[i]=(Left_Line[i]+Right_Line[i])/2;
           }
      }
     break;
    }
  }
}
/*---------------------------------------------------------------
 【功    能】斜入十字处理
 【参    数】二值化图像
 【返 回 值】无
 ----------------------------------------------------------------*/
uint8 slope_cross=0;                   //斜入标志位
uint8 L_slope_cross_state=0;          //左斜入标志位
uint8 R_slope_cross_state=0;         //右斜入标志位
void  slope_cross_deal(uint8(*image)[image_w])
{
    uint8 L_down_y=0,L_down_x=0,R_down_y=0,R_down_x=0; //单边补线坐标
    uint8 L_down_y0=0,L_down_x0=0,R_down_y0=0,R_down_x0=0; //单边补线坐标
    uint8 L_white_row=0,R_white_row=0;
    uint8 L_loss_num=0,R_loss_num=0;
    uint8 l_lock=0,r_lock=0;         //判断断线恢复高度,可以进行单边补线
    static bool L_slope_cross_lock=0,R_slope_cross_lock=0;      //左右斜入十字锁
    bool advance_L=0,advance_R=0;                    //左右斜入预先处理
    float slope=0;                                     //补线斜率
    uint8 temp_x=0;                                   //补线临时坐标
    uint8 temp_y=0;                                  //补线临时坐标
    uint8 limit_down,limit_up;                      //上下限幅
    uint8 angle_dist=7;                            //采样距离
    uint8 nums_angle_dist=7*2+1;                  //角度非极大抑制

    if(slope_cross==0  && L_CirCle_State==0 && R_CirCle_State==0)
    {
        for(uint8 i=start_line_point_row;i>=30;i--)//58-30=28//进行左右丢线数量判断，斜入有一边是很多丢线的
        {
            if(Left_Line_flag[i]==0)
            {
                L_loss_num++;
            }
            if(Right_Line_flag[i]==0)
            {
                R_loss_num++;
            }
        }

        //R_loss_num<5适合小十字，如果是大十字，删掉这个条件,角点符合<型
        if(L_loss_num>=24 && r_cross_check_point && r_grow_outside && R_loss_num<=5)//左边大量丢线
        {
             if(advance_R==0)
             advance_R=1;                     //对右边界进行处理
             if(Left_Line_flag[r_cross_check_point_y]==0 &&
                 Left_Line_flag[r_cross_check_point_y+1]==0 &&
                 Left_Line_flag[r_cross_check_point_y+2]==0 &&
                 Left_Line_flag[r_cross_check_point_y+3]==0)   //右角点对应左边线大量丢失
             {
                  R_slope_cross_lock=1;
             }

             if(R_slope_cross_lock==1)
             {
                 for(uint8 n=40;n>=15;n--) //判断左边界是否断开
                 {
                     if(Left_Line_flag[n]==0 && Left_Line_flag[n+1]==0 &&
                        Left_Line_flag[n-1]==1 && Left_Line_flag[n-2]==1)
                     {
                         slope_cross=1;
                         R_slope_cross_state=1;           //右斜入
                         Beep_set(1);
                         break;
                     }
                 }
             }
         }
        //L_loss_num<9适合小十字，如果是大十字，删掉这个条件，角点符合>型
        else if(L_loss_num<=5 && l_cross_check_point && l_grow_outside && R_loss_num>=24)
        {
            if(advance_L==0)
            advance_L=1;                     //对左边界进行处理
            if(Right_Line_flag[l_cross_check_point_y]==0 &&
               Right_Line_flag[l_cross_check_point_y+1]==0 &&
               Right_Line_flag[l_cross_check_point_y+2]==0 &&
               Right_Line_flag[l_cross_check_point_y+3]==0 )   //左角点对应左边线大量丢失
            {
                L_slope_cross_lock=1;
            }
            if(L_slope_cross_lock==1)
            {
                for(uint8 n=40;n>=15;n--)    //判断丢线断点
                {
                    if(Right_Line_flag[n]==0 && Right_Line_flag[n+1]==0 &&
                            Right_Line_flag[n-1]==1 &&  Right_Line_flag[n-2]==1)
                    {
                        slope_cross=1;
                        L_slope_cross_state=1;
                        Beep_set(1);
                        break;
                    }
                }
            }
        }
        else//进行常规判断,处理轻微斜入
        {
            L_white_row=Fine_baihang(1, 30, 37, image, 4);  //判断左边白行数
            R_white_row=Fine_baihang(2, 30, 37, image, 4);  //判断左边白行数
            //左角点，角点向外，右边大量白行，左边没有大量白行，确认是左斜入十字
            if(l_cross_check_point && l_grow_outside && R_white_row==1 && L_white_row==0)
            {
                if(Right_Line_flag[l_cross_check_point_y]==0 &&
                   Right_Line_flag[l_cross_check_point_y+1]==0 &&
                   Right_Line_flag[l_cross_check_point_y+2]==0 &&   //斜入的话，一边角点肯定是高的
                   Right_Line_flag[l_cross_check_point_y+3]==0 )   //左角点对应右边线大量丢失
                {
                    L_slope_cross_lock=1;
                }
                if(L_slope_cross_lock==1)
                {
                    for(uint8 n=40;n>=15;n--)          //寻找右边界断点
                    {
                        if(Right_Line_flag[n]==0 && Right_Line_flag[n+1]==0 &&
                                Right_Line_flag[n-1]==1 && Right_Line_flag[n-2]==1)
                        {
                            slope_cross=1;
                            L_slope_cross_state=1;
                            Beep_set(1);
                            break;
                        }
                    }
                }
            }
            //右角点，角点向外，左边大量白行，右边没有大量白行，确认是右斜入十字
            else if(r_cross_check_point && r_grow_outside && R_white_row==0 && L_white_row)
            {
                if(Left_Line_flag[r_angle_point_y]==0 &&
                    Left_Line_flag[r_angle_point_y+1]==0 &&
                    Left_Line_flag[r_angle_point_y+2]==0 &&  //斜入的话，一边角点肯定是高的
                    Left_Line_flag[r_angle_point_y+3]==0)   //右角点对应左边线大量丢失
                {
                    R_slope_cross_lock=1;
                }
                if(R_slope_cross_lock==1)
                {
                    for(uint8 n=40;n>=15;n--) //判断是否边界断开
                    {
                        if(Left_Line_flag[n]==0 && Left_Line_flag[n+1]==0 &&
                                Left_Line_flag[n-1]==1 && Left_Line_flag[n-2]==1)
                        {
                            slope_cross=1;
                            R_slope_cross_state=1;
                            Beep_set(1);
                            break;
                        }
                    }
                }
            }
        }
    }


    if(advance_R)    //提前补线
    {
    //直接斜率补线
    AY=40;
    AX=Right_Line[AY];
    temp_y=clip(AY+15,0,59);
    temp_x=Right_Line[temp_y];
    slope=AX-temp_x;
    slope=slope/(AY-temp_y);
    slope=LIMIT(slope,2,R_straight_slope);//斜率限幅，求得的斜率不能突变,对于斜入十字来说是需要这样处理的
    REPIRE_LINE=1;          //进行斜率补偿
    for(uint8 i=AY;i>=20;i--)
    {
      temp_x=slope*(i-AY)+AX;
      Right_Line[i]=clip(temp_x,0,93);
    }
    for(uint8 i=20;i<=AY;i++)
    {
      Mid_Line[i]=clip(Right_Line[i]-Width_straight[i]/2,0,93);
    }
 }
    if(advance_L)//提前补线
    {
    //直接斜率补线
    AY=40;
    AX=Left_Line[AY];
    temp_y=clip(AY+15,0,59);
    temp_x=Left_Line[temp_y];
    slope=AX-temp_x;
    slope=slope/(AY-temp_y);
    slope=LIMIT(slope,L_straight_slope,-2);//斜率限幅，求得的斜率不能突变,对于斜入十字来说是需要这样处理的
    REPIRE_LINE=1;          //进行斜率补偿
    for(uint8 i=AY;i>=20;i--)
    {
      temp_x=slope*(i-AY)+AX;
      Left_Line[i]=clip(temp_x,0,93);
    }
    for(uint8 i=20;i<=AY;i++)
    {
      Mid_Line[i]=clip(Left_Line[i]+Width_straight[i]/2,0,93);
    }
 }

    if(L_slope_cross_state!=0)    //开始处理左斜入
    {
    float temp_slope=0;
    switch(L_slope_cross_state)
    {
    case 1 :        //入十字之前拐点较远，直接利用近处斜率向上补线，48为近处限幅
                   //直接斜率补线
         temp_slope=Slope_calculate(49, start_line_point_row, Left_Line);
         AY=Find_guaidian(30, 53, Left_Line, 1, 1);
         AX=Left_Line[AY];
         temp_y=clip(AY+12,0,59);
         slope=AX-Left_Line[temp_y];
         slope=slope/(AY-temp_y);
         if(fabs(slope-temp_slope)>0.2)   slope=temp_slope; //斜率限幅
         slope=LIMIT(slope,L_straight_slope,-2);//斜率限幅，求得的斜率不能突变,对于斜入十字来说是需要这样处理的
         REPIRE_LINE=1;          //进行斜率补偿
         for(uint8 i=AY;i>=20;i--)
         {
             temp_x=slope*(i-AY)+AX;
             Left_Line[i]=clip(temp_x,0,93);
         }
         for(uint8 i=20;i<=AY;i++)
         {
             Mid_Line[i]=clip(Left_Line[i]+Width_straight[i]/2,0,93);
         }
         if((l_angle_point_y>=48 && l_grow_outside) || AY>=48)//左角点在近处，角点向外生长
         {                                                    //拐点角点同时判断
               L_slope_cross_state=2;
         }
    break;
    case 2 :         //近处拐点，此时利用上下两点进行补线
        //进行找左边界两拐点补线
        L_down_y=Find_guaidian(47 ,58,Left_Line,1,1);
        if(L_down_y)
        {
            L_down_x=Left_Line[L_down_y];
        }
        else
        {
            L_down_y=59;
            L_down_x=47-Width_straight[59]/2;
        }
        for(uint8 i=L_down_y-4;i>15;i-=3)
        {
            if(image[i][L_down_x]==0 && image[i-1][L_down_x]==0)
            {
                L_down_y0=i;
                break;
            }
        }
        for(uint8 i=L_down_x;i<73;i++)
        {
            if(image[L_down_y0][i]==255 && image[L_down_y0][i+1]==255)
            {
                L_down_x0=i;
                break;
            }
        }
        Add_line(1,Left_Line, L_down_x0, L_down_y0, L_down_x, L_down_y);
        for(uint8 i=20;i<=59;i++)
        {
            if(i<=L_down_y0)
            Mid_Line[i]=46;
            else
            Mid_Line[i]=clip(Left_Line[i]+Width_straight[i]/2,0,93);

        }
        REPIRE_LINE=1;          //进行斜率补偿
        //十字里面判断
        if(Left_Line_flag[start_line_point_row-1]==0 && Right_Line_flag[start_line_point_row-1]==0 &&
        Left_Line_flag[start_line_point_row-2]==0 && Right_Line_flag[start_line_point_row-2]==0 &&
        Left_Line_flag[start_line_point_row-3]==0 && Right_Line_flag[start_line_point_row-3]==0)
        {
            L_slope_cross_state=3;
        }

    break;
    case 3 :     //当前已经进入十字，只能利用上面两个角点补线
        if(Left_Line_flag[start_line_point_row-1]==1 && Right_Line_flag[start_line_point_row-1]==1 &&
        Left_Line_flag[start_line_point_row-2]==1 && Right_Line_flag[start_line_point_row-2]==1 &&
        Left_Line_flag[start_line_point_row-3]==1 && Right_Line_flag[start_line_point_row-3]==1 )
        {
          if(OFF_Element_test==1)
          {
             Door_count=track_num;
          }
          L_slope_cross_state=0;
          slope_cross=0;
          Beep_set(0);
        }

        uint8 numcut1 = 0;

        for(uint8 i=start_line_point_row;i>15;i--)        //判断丢线恢复高度
          {
              if(l_lock==0 && Left_Line_flag[i]==0 && Left_Line_flag[i-1]==0 && Left_Line_flag[i-2]==1
                 && Left_Line_flag[i-3]==1)
              {
                  l_lock=i-2;
              }
              if(r_lock==0 && Right_Line_flag[i]==0 && Right_Line_flag[i-1]==0 && Right_Line_flag[i-2]==1
                 && Right_Line_flag[i-3]==1)
              {
                  r_lock=i-2;
              }

              if(l_lock && r_lock)
              {
                  break;
              }
          }

          if(l_lock==0)   l_lock=10;
          if(r_lock==0)   r_lock=10;

          element_rpts_num_l=(uint16)element_line_num;        //数组长度
           element_rpts_num_r=(uint16)element_line_num;       //数组长度

           if(l_lock>r_lock)    //右边远
           {
               for(uint8 i=50;i>20;i--)
               {
                  if(image[i][80]==255 && image[i-1][80]==255 && image[i-2][80]==0 && image[i-3][80]==0 )
                  {
                     DX=80;
                     DY=i-1;
                     break;
                  }
               }
               findline_righthand_binaryzation_angle_user(image, DX, DY, element_rpts_r,&element_rpts_num_r);   //重新扫描
               for(uint8 i=0;i<(uint16)element_line_num;i++)  //角度清空
               {
                   element_a_r[i] = 0;
                   element_outangle_r[i]=0;
               }
               numcut1=element_rpts_num_r-1;
               for(uint8 i=0;i<element_rpts_num_r;i++)
               {
                   if (i <= 0 || i >= numcut1)
                   {
                       element_a_r[i] = 0;
                       continue;
                   }

                   limit_up=clip(i+angle_dist,0,numcut1);
                   limit_down=clip(i-angle_dist,0,numcut1);

                   element_a_r[i]=180-fabs(getAngle(element_rpts_r[limit_up][0],
                           element_rpts_r[limit_up][1],element_rpts_r[i][0],
                           element_rpts_r[i][1],element_rpts_r[limit_down][0],
                           element_rpts_r[limit_down][1]));
               }
               nms_angle(element_a_r, element_rpts_num_r, element_outangle_r,nums_angle_dist);
               for(uint8 i=0;i<element_rpts_num_r;i++)          //提取角点
               {
                   if(element_outangle_r[i]==0)
                       continue;
                   if(element_outangle_r[i]>40)
                   {
                      BX= element_rpts_r[i][0];
                      BY= element_rpts_r[i][1];
                      up_r_angle_point=i;
                      break;
                   }
               }
               for(uint8 i=BY;i>15;i--)       //寻找恢复点进行补线
               {
                   if(Right_Line_flag[i]==1 && Right_Line_flag[i-1]==1)
                   {
                       BX= Right_Line[i];
                       BY=i;
                       break;
                   }
               }

               if(r_lock>35)
               {
                   Add_line_other(2, Right_Line, BY-15, BY, BY, 59);
               }else{
                   temp_x=78 ;
                   Add_line(2,Right_Line,BX,BY,temp_x,59);            //右补线
               }

               for(uint8 i=59;i>BY;i--)
               {
                  Mid_Line[i]=(Right_Line[i]-Width_straight[i]/2);
               }
                REPIRE_LINE=1;          //进行斜率补偿
           }

           if(l_lock<r_lock)          //左边远
           {
               for(uint8 i=50;i>20;i--)
               {
                  if(image[i][13]==255 && image[i-1][13]==255 && image[i-2][13]==0 && image[i-3][13]==0 )
                  {
                     CX=13;
                     CY=i-1;
                     break;
                  }
               }
               findline_lefthand_binaryzation_angle_user(image, CX, CY, element_rpts_l,&element_rpts_num_l);  //重新扫描边界
               for(uint8 i=0;i<(uint16)element_line_num;i++)  //角度清空
               {
                   element_a_l[i] = 0;
                   element_outangle_l[i]=0;
               }
               numcut1=element_rpts_num_l-1;
               for(uint8 i=0;i<element_rpts_num_l;i++)
               {
                   if (i <= 0 || i >= numcut1)
                   {
                       element_a_l[i] = 0;
                       continue;
                   }

                   limit_up=clip(i+angle_dist,0,numcut1);
                   limit_down=clip(i-angle_dist,0,numcut1);

                   element_a_l[i]=180-fabs(getAngle(element_rpts_l[limit_up][0],
                           element_rpts_l[limit_up][1],element_rpts_l[i][0],
                           element_rpts_l[i][1],element_rpts_l[limit_down][0],
                           element_rpts_l[limit_down][1]));
               }
               nms_angle(element_a_l, element_rpts_num_l, element_outangle_l, nums_angle_dist);   //角度极大值抑制
               for(uint8 i=0;i<element_rpts_num_l;i++)         //提取角点
               {
                   if(element_outangle_l[i]==0)
                       continue;
                   if(element_outangle_l[i]>40)
                   {
                      AX=element_rpts_l[i][0];
                      AY=element_rpts_l[i][1];
                      up_l_angle_point=i;
                      break;
                   }
               }
               for(uint8 i=AY;i>15;i--)
               {
                   if(Left_Line_flag[i]==1 && Left_Line_flag[i-1]==1)
                   {
                       AX= Left_Line[i];
                       AY=i;
                       break;
                   }
               }

               if(l_lock>35)
               {
                   Add_line_other(1, Left_Line, AY-15, AY, AY, 59);
               }else{
                   temp_x=15;
                   Add_line(1,Left_Line,AX,AY,temp_x,59);             //左补线
               }

               for(uint8 i=59;i>=AY;i--)
               {
                  Mid_Line[i]=(Left_Line[i]+Width_straight[i]/2);
               }
               REPIRE_LINE=1;          //进行斜率补偿
           }

       if(l_lock==r_lock)      //要么相等要么没找到
        {
          for(uint8 i=50;i>20;i--)
          {
             if(image[i][78]==255 && image[i-1][78]==255 && image[i-2][78]==0 && image[i-3][78]==0 )
             {
                DX=78;
                DY=i-1;
                break;
             }
          }
          for(uint8 i=50;i>20;i--)
          {
             if(image[i][15]==255 && image[i-1][15]==255 && image[i-2][15]==0 && image[i-3][15]==0 )
             {
                CX=15;
                CY=i-1;
                break;
             }
          }
          //以下为角点寻找
          findline_lefthand_binaryzation_angle_user(image, CX, CY, element_rpts_l,&element_rpts_num_l);  //重新扫描边界
          findline_righthand_binaryzation_angle_user(image, DX, DY, element_rpts_r,&element_rpts_num_r);   //重新扫描
          for(uint8 i=0;i<(uint16)element_line_num;i++)  //角度清空
          {
              element_a_l[i] = 0;
              element_a_r[i] = 0;
              element_outangle_l[i]=0;
              element_outangle_r[i]=0;
          }

          numcut1=element_rpts_num_l-1;
          for(uint8 i=0;i<element_rpts_num_l;i++)
          {
              if (i <= 0 || i >= numcut1)
              {
                  element_a_l[i] = 0;
                  continue;
              }
              limit_up=clip(i+angle_dist,0,numcut1);
              limit_down=clip(i-angle_dist,0,numcut1);

              element_a_l[i]=180-fabs(getAngle(element_rpts_l[limit_up][0],
                      element_rpts_l[limit_up][1],element_rpts_l[i][0],
                      element_rpts_l[i][1],element_rpts_l[limit_down][0],
                      element_rpts_l[limit_down][1]));
          }

          numcut1=element_rpts_num_r-1;
          for(uint8 i=0;i<element_rpts_num_r;i++)
          {
              if (i <= 0 || i >= numcut1)
              {
                  element_a_r[i] = 0;
                  continue;
              }
              limit_up=clip(i+angle_dist,0,numcut1);
              limit_down=clip(i-angle_dist,0,numcut1);

              element_a_r[i]=180-fabs(getAngle(element_rpts_r[limit_up][0],
                      element_rpts_r[limit_up][1],element_rpts_r[i][0],
                      element_rpts_r[i][1],element_rpts_r[limit_down][0],
                      element_rpts_r[limit_down][1]));
          }
          nms_angle(element_a_l, element_rpts_num_l, element_outangle_l,nums_angle_dist);   //角度极大值抑制
          nms_angle(element_a_r, element_rpts_num_r, element_outangle_r,nums_angle_dist);
          for(uint8 i=0;i<element_rpts_num_l;i++)         //提取角点
          {
              if(element_outangle_l[i]==0)
                  continue;
              if(element_outangle_l[i]>40)
              {
                 AX=element_rpts_l[i][0];
                 AY=element_rpts_l[i][1];
                 up_l_angle_point=i;
                 break;
              }
          }

          for(uint8 i=0;i<element_rpts_num_r;i++)          //提取角点
          {
              if(element_outangle_r[i]==0)
                  continue;
              if(element_outangle_r[i]>40)
              {
                 BX= element_rpts_r[i][0];
                 BY= element_rpts_r[i][1];
                 up_r_angle_point=i;
                 break;
              }
          }
              Add_line(1,Left_Line,AX,AY,47-Width_straight[59]/2,59);             //左补线
              Add_line(2,Right_Line,BX,BY,47+Width_straight[59]/2,59);            //右补线
              corss_deal_mid=(AY+BY)/2;
              for(uint8 i=59;i>corss_deal_mid;i--)
              {
                 Mid_Line[i]=(Left_Line[i]+Right_Line[i])/2;
              }
              REPIRE_LINE=1;          //进行斜率补偿
         }
    break;
    }
  }

    if(R_slope_cross_state!=0)    //开始处理右斜入
    {
       float temp_slope=0;
       switch(R_slope_cross_state)
       {
       case 1:
          //直接斜率补线
          temp_slope=Slope_calculate(49, start_line_point_row, Right_Line); //拟合一下底部斜率
          AY=Find_guaidian(30, 53, Right_Line, 2, 1);
          AX=Right_Line[AY];
          temp_y=clip(AY+12,0,59);
          slope=AX-Right_Line[temp_y];
          slope=slope/(AY-temp_y);
          if(fabs(slope-temp_slope)>0.2)  slope=temp_slope;
          slope=LIMIT(slope,2,R_straight_slope);//斜率限幅，求得的斜率不能突变,对于斜入十字来说是需要这样处理的
          REPIRE_LINE=1;          //进行斜率补偿
          for(uint8 i=AY;i>=20;i--)
          {
              temp_x=slope*(i-AY)+AX;
              Right_Line[i]=clip(temp_x,0,93);
          }
          for(uint8 i=20;i<=AY;i++)
          {
              Mid_Line[i]=Right_Line[i]-Width_straight[i]/2;
          }
          if((r_angle_point_y>=48 && r_grow_outside) || AY>=48)//右角点座标很高，角点向外生长
          {                                                    //拐点角点同时判断
                R_slope_cross_state=2;
          }
       break;
       case 2: //这个状态不适合斜率补线，点数过少
               //进行找右边界两拐点补线
       R_down_y=Find_guaidian(47,58,Right_Line,2,1);
       if(R_down_y)
       {
           R_down_x=Right_Line[R_down_y];
       }else {
           R_down_y=59;
           R_down_x=47+Width_straight[59]/2;
       }
       //向上找黑点
       for(uint8 i=R_down_y-4;i>15;i-=3)
       {
           if(image[i][R_down_x]==0 && image[i-1][R_down_x]==0)
           {
               R_down_y0=i;
               break;
           }
       }
       //向左找白点
       for(uint8 i=R_down_x;i>20;i--)
       {
           if(image[R_down_y0][i]==255 && image[R_down_y0][i-1]==255)
           {
               R_down_x0=i;
               break;
           }
       }
       //两点补线
       Add_line(2,Right_Line, R_down_x0, R_down_y0, R_down_x, R_down_y);
       REPIRE_LINE=1;          //进行斜率补偿
       for(uint8 i=20;i<=59;i++)
       {
           if(i<=R_down_y0)
           Mid_Line[i]=46;
           else
           Mid_Line[i]=clip(Right_Line[i]-Width_straight[i]/2,0,93);
       }

       if(Left_Line_flag[start_line_point_row-1]==0 && Right_Line_flag[start_line_point_row-1]==0 &&
       Left_Line_flag[start_line_point_row-2]==0 && Right_Line_flag[start_line_point_row-2]==0 &&
       Left_Line_flag[start_line_point_row-3]==0 && Right_Line_flag[start_line_point_row-3]==0 )
       {
           R_slope_cross_state=3;
       }

       break;
       case 3:

       if(Left_Line_flag[start_line_point_row-1]==1 && Right_Line_flag[start_line_point_row-1]==1 &&
       Left_Line_flag[start_line_point_row-2]==1 && Right_Line_flag[start_line_point_row-2]==1 &&
       Left_Line_flag[start_line_point_row-3]==1 && Right_Line_flag[start_line_point_row-3]==1 )
       {
         if(OFF_Element_test==1)
         {
          Door_count=track_num;
         }
         R_slope_cross_state=0;
         slope_cross=0;
         Beep_set(0);
       }

       uint8 numcut1 = 0;

       for(uint8 i=start_line_point_row;i>15;i--)        //判断丢线恢复高度
         {
             if(l_lock==0 && Left_Line_flag[i]==0 && Left_Line_flag[i-1]==0 && Left_Line_flag[i-2]==1
                && Left_Line_flag[i-3]==1)
             {
                 l_lock=i-2;
             }
             if(r_lock==0 && Right_Line_flag[i]==0 && Right_Line_flag[i-1]==0 && Right_Line_flag[i-2]==1
                && Right_Line_flag[i-3]==1)
             {
                 r_lock=i-2;
             }
             if(l_lock && r_lock)
             {
                 break;
             }

         }

         if(l_lock==0)   l_lock=10;
         if(r_lock==0)   r_lock=10;

         element_rpts_num_l=(uint16)element_line_num;        //数组长度
         element_rpts_num_r=(uint16)element_line_num;       //数组长度

         if(l_lock>r_lock)    //右边远
         {
             for(uint8 i=50;i>20;i--)
             {
                if(image[i][80]==255 && image[i-1][80]==255 && image[i-2][80]==0 && image[i-3][80]==0 )
                {
                   DX=80;
                   DY=i-1;
                   break;
                }
             }
             findline_righthand_binaryzation_angle_user(image, DX, DY, element_rpts_r,&element_rpts_num_r);   //重新扫描
             for(uint8 i=0;i<(uint16)element_line_num;i++)  //角度清空
             {
                 element_a_r[i] = 0;
                 element_outangle_r[i]=0;
             }
             numcut1=element_rpts_num_r-1;
             for(uint8 i=0;i<element_rpts_num_r;i++)
             {
                 if (i <= 0 || i >= numcut1)
                 {
                     element_a_r[i] = 0;
                     continue;
                 }

                 limit_up=clip(i+angle_dist,0,numcut1);
                 limit_down=clip(i-angle_dist,0,numcut1);

                 element_a_r[i]=180-fabs(getAngle(element_rpts_r[limit_up][0],
                         element_rpts_r[limit_up][1],element_rpts_r[i][0],
                         element_rpts_r[i][1],element_rpts_r[limit_down][0],
                         element_rpts_r[limit_down][1]));
             }
             nms_angle(element_a_r, element_rpts_num_r, element_outangle_r,nums_angle_dist);
             for(uint8 i=0;i<element_rpts_num_r;i++)          //提取角点
             {
                 if(element_outangle_r[i]==0)
                     continue;
                 if(element_outangle_r[i]>40)
                 {
                    BX= element_rpts_r[i][0];
                    BY= element_rpts_r[i][1];
                    up_r_angle_point=i;
                    break;
                 }
             }
             for(uint8 i=BY;i>15;i--)       //寻找恢复点进行补线
             {
                 if(Right_Line_flag[i]==1 && Right_Line_flag[i-1]==1)
                 {
                     BX= Right_Line[i];
                     BY=i;
                     break;
                 }
             }

             if(r_lock>35)
             {
                 Add_line_other(2, Right_Line, BY-15, BY, BY, 59);
             }else{
                 temp_x=78 ;
                 Add_line(2,Right_Line,BX,BY,temp_x,59);            //右补线
             }

             for(uint8 i=59;i>BY;i--)
             {
                Mid_Line[i]=(Right_Line[i]-Width_straight[i]/2);
             }
              REPIRE_LINE=1;          //进行斜率补偿
         }

         if(l_lock<r_lock)          //左边远
         {
             for(uint8 i=50;i>20;i--)
             {
                if(image[i][13]==255 && image[i-1][13]==255 && image[i-2][13]==0 && image[i-3][13]==0 )
                {
                   CX=13;
                   CY=i-1;
                   break;
                }
             }
             findline_lefthand_binaryzation_angle_user(image, CX, CY, element_rpts_l,&element_rpts_num_l);  //重新扫描边界
             for(uint8 i=0;i<(uint16)element_line_num;i++)  //角度清空
             {
                 element_a_l[i] = 0;
                 element_outangle_l[i]=0;
             }
             numcut1=element_rpts_num_l-1;
             for(uint8 i=0;i<element_rpts_num_l;i++)
             {
                 if (i <= 0 || i >= numcut1)
                 {
                     element_a_l[i] = 0;
                     continue;
                 }

                 limit_up=clip(i+angle_dist,0,numcut1);
                 limit_down=clip(i-angle_dist,0,numcut1);

                 element_a_l[i]=180-fabs(getAngle(element_rpts_l[limit_up][0],
                         element_rpts_l[limit_up][1],element_rpts_l[i][0],
                         element_rpts_l[i][1],element_rpts_l[limit_down][0],
                         element_rpts_l[limit_down][1]));
             }
             nms_angle(element_a_l, element_rpts_num_l, element_outangle_l, nums_angle_dist);   //角度极大值抑制
             for(uint8 i=0;i<element_rpts_num_l;i++)         //提取角点
             {
                 if(element_outangle_l[i]==0)
                     continue;
                 if(element_outangle_l[i]>40)
                 {
                    AX=element_rpts_l[i][0];
                    AY=element_rpts_l[i][1];
                    up_l_angle_point=i;
                    break;
                 }
             }
             for(uint8 i=AY;i>15;i--)
             {
                 if(Left_Line_flag[i]==1 && Left_Line_flag[i-1]==1)
                 {
                     AX= Left_Line[i];
                     AY=i;
                     break;
                 }
             }

             if(l_lock>35)
             {
                 Add_line_other(1, Left_Line, AY-15, AY, AY, 59);
             }else{
                 temp_x=15;
                 Add_line(1,Left_Line,AX,AY,temp_x,59);             //左补线
             }

             for(uint8 i=59;i>=AY;i--)
             {
                Mid_Line[i]=(Left_Line[i]+Width_straight[i]/2);
             }
             REPIRE_LINE=1;          //进行斜率补偿
         }

      if(l_lock==r_lock)      //要么相等要么没找到
       {
         for(uint8 i=50;i>20;i--)
         {
            if(image[i][78]==255 && image[i-1][78]==255 && image[i-2][78]==0 && image[i-3][78]==0 )
            {
               DX=78;
               DY=i-1;
               break;
            }
         }
         for(uint8 i=50;i>20;i--)
         {
            if(image[i][15]==255 && image[i-1][15]==255 && image[i-2][15]==0 && image[i-3][15]==0 )
            {
               CX=15;
               CY=i-1;
               break;
            }
         }
         //以下为角点寻找
         findline_lefthand_binaryzation_angle_user(image, CX, CY, element_rpts_l,&element_rpts_num_l);  //重新扫描边界
         findline_righthand_binaryzation_angle_user(image, DX, DY, element_rpts_r,&element_rpts_num_r);   //重新扫描
         for(uint8 i=0;i<(uint16)element_line_num;i++)  //角度清空
         {
             element_a_l[i] = 0;
             element_a_r[i] = 0;
             element_outangle_l[i]=0;
             element_outangle_r[i]=0;
         }

         numcut1=element_rpts_num_l-1;
         for(uint8 i=0;i<element_rpts_num_l;i++)
         {
             if (i <= 0 || i >= numcut1)
             {
                 element_a_l[i] = 0;
                 continue;
             }
             limit_up=clip(i+angle_dist,0,numcut1);
             limit_down=clip(i-angle_dist,0,numcut1);

             element_a_l[i]=180-fabs(getAngle(element_rpts_l[limit_up][0],
                     element_rpts_l[limit_up][1],element_rpts_l[i][0],
                     element_rpts_l[i][1],element_rpts_l[limit_down][0],
                     element_rpts_l[limit_down][1]));
         }

         numcut1=element_rpts_num_r-1;
         for(uint8 i=0;i<element_rpts_num_r;i++)
         {
             if (i <= 0 || i >= numcut1)
             {
                 element_a_r[i] = 0;
                 continue;
             }
             limit_up=clip(i+angle_dist,0,numcut1);
             limit_down=clip(i-angle_dist,0,numcut1);

             element_a_r[i]=180-fabs(getAngle(element_rpts_r[limit_up][0],
                     element_rpts_r[limit_up][1],element_rpts_r[i][0],
                     element_rpts_r[i][1],element_rpts_r[limit_down][0],
                     element_rpts_r[limit_down][1]));
         }
         nms_angle(element_a_l, element_rpts_num_l, element_outangle_l,nums_angle_dist);   //角度极大值抑制
         nms_angle(element_a_r, element_rpts_num_r, element_outangle_r,nums_angle_dist);
         for(uint8 i=0;i<element_rpts_num_l;i++)         //提取角点
         {
             if(element_outangle_l[i]==0)
                 continue;
             if(element_outangle_l[i]>40)
             {
                AX=element_rpts_l[i][0];
                AY=element_rpts_l[i][1];
                up_l_angle_point=i;
                break;
             }
         }

         for(uint8 i=0;i<element_rpts_num_r;i++)          //提取角点
         {
             if(element_outangle_r[i]==0)
                 continue;
             if(element_outangle_r[i]>40)
             {
                BX= element_rpts_r[i][0];
                BY= element_rpts_r[i][1];
                up_r_angle_point=i;
                break;
             }
         }

         for(uint8 i=BY;i>15;i--)       //寻找恢复点进行补线
         {
             if(Right_Line_flag[i]==1 && Right_Line_flag[i-1]==1)
             {
                 BX= Right_Line[i];
                 BY=i;
                 break;
             }
         }

         for(uint8 i=AY;i>15;i--)       //寻找恢复点进行补线
         {
             if(Left_Line_flag[i]==1 && Left_Line_flag[i-1]==1)
             {
                 AX= Left_Line[i];
                 AY=i;
                 break;
             }
         }

             Add_line(1,Left_Line,AX,AY,47-Width_straight[59]/2,59);             //左补线
             Add_line(2,Right_Line,BX,BY,47+Width_straight[59]/2,59);            //右补线
             corss_deal_mid=(AY+BY)/2;
             for(uint8 i=59;i>corss_deal_mid;i--)
             {
                Mid_Line[i]=(Left_Line[i]+Right_Line[i])/2;
             }
             REPIRE_LINE=1;          //进行斜率补偿
        }
        break;
       }
   }
}
/*---------------------------------------------------------------
 【功    能】正入圆环处理
 【参    数】二值化图像
 【返 回 值】无
 ----------------------------------------------------------------*/
uint8 Flag_L_CirCle=0;                               //左圆环标志位
uint8 Flag_R_CirCle=0;                              //右圆环标志位
uint8 L_CirCle_State=0;                            //左圆环状态标志位
uint8 R_CirCle_State=0;                           //右圆环状态标志位
uint8 In_Poit_x=0,In_Poit_y=0;                   //进圆环拐点
uint8 Out_Poit_x=0,Out_Poit_y=0;                //出环拐点坐标
float R_slope=0,L_slope=0;                     //左右边界斜率
float Ring_Left_angle=0,Ring_Right_angle=0;   //左右边界角度
float Ring_add_angle=0;                      //累加的角度
float slope_to_angle=0;                     //斜率转化为角度
bool can_slopetoangle=0;                    //可以进行斜率打角标志位
int Add_steer_duty=0;                      //累加占空比
int16 Average_duty=0;                      //平均占空比
int16 Add_steer_duty_count=0;             //累加占空比计数
//右圆环辅助判断函数
void assist_Right_ring_deal()
{
    static bool can_check=false;
    uint8 i=0;
    uint8 acr_flag=0;
    uint8 dz=0,dj=0;               //单增和单减标志位
    if((Inductor_Value[0]+Inductor_Value[3])>= 85 && Inductor_Value[2]>=25 && can_check==false && R_CirCle_State<3) //水平电感值很大,靠近圆环
    {
        can_check=true;
    }

    if(L_straightaway && R_straightaway)
    {
        can_check=false;
    }

    if(can_check==true)
    {
    for(i=5;i<=50;i++) //寻找弧度
    {
          if(points_r[i][0]>points_r[i+1][0] && dj<=9)   //先判断单减
          {
              dj++;
          }
          if(points_r[i][0]<points_r[i+1][0] && dz<=9 && dj>=9)//当单减符合一定数量才可以判断单增
          {
              dz++;
          }
          if(dz>=9 && dj>=9)//符合圆弧特征
          {
              acr_flag=1;
              break;
          }
    }
     if(acr_flag==1 && left_angle_point==0)      //存在圆弧，左边无角点，确定是圆环                                                               //存在弧线
      {
         if(R_CirCle_State!=3)          //强制进入左圆环状态3
         R_CirCle_State=3;
         can_check=false;
         acr_flag=0;
     }
  }
}
//左圆环辅助判断函数
void assist_Left_ring_deal()
{
    static bool can_check=false;
    uint8 i=0;
    uint8 acr_flag=0;
    uint8 dz=0,dj=0;               //单增和单减标志位
    if((Inductor_Value[0]+Inductor_Value[3])>=85 && Inductor_Value[1]>=25 && can_check==false && L_CirCle_State<3) //水平电感值很大,靠近圆环
    {
        can_check=true;
    }

    if(L_straightaway && R_straightaway)
    {
        can_check=false;
    }

    if(can_check==true)
    {
    for(i=5;i<=45;i++) //寻找弧度
    {
          if(points_l[i][0]<points_l[i+1][0] && dz<=9)   //先判断单增
          {
              dz++;
          }
          if(points_l[i][0]>points_l[i+1][0] && dj<=9 && dz>=9)//当单增符合一定数量才可以判断单减
          {
              dj++;
          }

          if(dz>=9 && dj>=9)//符合圆弧特征
          {
              acr_flag=1;
              break;
          }
    }
     if(acr_flag==1 && right_angle_point==0)      //存在圆弧，右边无角点，确定是圆环                                                               //存在弧线
      {
         if(L_CirCle_State!=3)          //强制进入左圆环状态3
         L_CirCle_State=3;
         can_check=false;
         acr_flag=0;
     }
  }
}
void Ring_Deal(uint8(*image)[image_w]){
  uint8 i=0;
  uint8 white_row=0;               //连续白行计数
  static uint8 Angle_flag=0;   //可以进行角度累加标志位
  average_steer_duty=0;       //可以进行平均打角标志位
  can_slopetoangle=0;        //斜率打角标志位清0
  element_rpts_num_l=(uint16)element_line_num;        //数组长度
  element_rpts_num_r=(uint16)element_line_num;       //数组长度
  static bool L_CirCle_State_lock=0;                //左圆环状态锁
  static bool R_CirCle_State_lock=0;               //右圆环状态锁
  uint8 angle_dist=7;                             //采样距离
  uint8 nums_angle_dist=7*2+1;                   //角度非极大值抑制
  uint8 limit_up=0,limit_down=0;                //角点变量限幅
  uint8 dz=0,dj=0;                             //单增单减标志位
  //下面两个变量是针对小中圆环的状态二进行处理，如果有大圆环，删掉即可
  int8 loss_to_acc_count_row=0;

  if(Angle_flag)           //角度允许累加
  {
  Ring_add_angle+=Turn_speed *Turn_dt ;           //角速度累加
  if(fabs(Ring_add_angle)>20) Ring_add_angle=0;  //过大清空
  }

  assist_Left_ring_deal();               //电感辅助判断左圆环
  assist_Right_ring_deal();               //电感辅助判断右圆环

  //左圆环预判断
  if(L_CirCle_State==0)
  {
      if(l_ring_check_point==1 && l_grow_outside==1 && R_straightaway)   //存在圆环左角点且向外，右边严格直道
      {
          regression(2, 25, 50);                    //拟合右边界
          R_slope=parameterB;
          regression(1, 25, 55);                    //拟合左边界
          if(R_slope>0.35 && R_slope<0.85 && parameterB>0.17)   //右直道斜率范围，左边界斜率极性相反
          {
           //左角点位置对应右边界不丢线
           if(Right_Line_flag[l_ring_check_point_y] && Right_Line_flag[l_ring_check_point_y-1] && Right_Line_flag[l_ring_check_point_y-2])
           {
           Flag_L_CirCle=1;       //左圆环标志位
           L_CirCle_State=1;
           L_CirCle_State_lock=0;         //开锁
           Flag_Cross=0;               //清空
           }
          }
      }
  }
  //右圆环预判断
  if(R_CirCle_State==0 && L_CirCle_State==0)      //左右圆环不能同时判断到
  {
      if(r_ring_check_point==1 && r_grow_outside==1 && L_straightaway)   //存在圆环右角点且向外，左边严格直道
      {
          regression(1, 25, 50);                    //拟合左边界
          L_slope=parameterB;
          regression(2, 25, 55);                    //拟合右边界
          if(L_slope>-0.85 && L_slope<-0.35 && parameterB<-0.17)   //左直道斜率范围，右边界斜率极性相反
          {
          if(Left_Line_flag[r_ring_check_point_y] && Left_Line_flag[r_ring_check_point_y-1] && Left_Line_flag[r_ring_check_point_y-2])
          {
           Flag_R_CirCle=1;        //右圆环标志位
           R_CirCle_State=1;
           R_CirCle_State_lock=0;         //开锁
           Flag_Cross=0;         //清空
          }
          }
      }
  }
/****************************开始处理左环状态******************************/
 if(Flag_L_CirCle)   //左环处理
 {
 switch(L_CirCle_State)
 {
    case 1:                  //当前状态寻右边线
    if(( l_gray_point==0 && !Left_Line_flag[50] && !Left_Line_flag[49] && !Left_Line_flag[48]
       &&!Left_Line_flag[47] && !Left_Line_flag[46]&&
       !Left_Line_flag[45]) && l_ring_check_point==0) //如果左边界底下全丢和圆环角点消失,说明靠近圆环圆弧
      {                                                                                                                                                                 //拐点消失之后的弧度很明显
             L_CirCle_State=2;  //进入下一个状态
             L_CirCle_State_lock=1;   //上锁
      }

    if(R_straightaway==0)                 //如果右边界不是直道
       {
         L_CirCle_State=0;
       }
     break;
     case 2:    //当前状态寻右边线
     if(L_CirCle_State_lock==1)       //上锁
      {
          for(i=55;i>=40;i--)        //判断丢丢不丢不丢
          {
              if(Left_Line_flag[i]==0 && Left_Line_flag[i-1]==0 &&
                 Left_Line_flag[i-2]==1 &&  Left_Line_flag[i-3]==1)
              {
                  L_CirCle_State_lock=0;       //解锁
                  break;
              }
          }
      }

     if(L_CirCle_State_lock==0) //判断弧度
     {
       for(i=0;i<50;i++)
       {
           if(points_l[i][0] < points_l[i+1][0] && dz<12)//先判断单调增加
           {
               dz++;
           }
           if(points_l[i][0] > points_l[i+1][0] && dj<12 && dz>=12) //单调增加之后才可以判断单调递减
           {
               dj++;
           }
           if(dz>=12 && dj>=12)
           {
               loss_to_acc_count_row=1;   //确认有圆弧
               break;
           }
       }
     }

     if(Left_Line_flag[50] && Left_Line_flag[49] && Left_Line_flag[48]
       && Left_Line_flag[47] && Left_Line_flag[46]&&
       Left_Line_flag[45] && left_angle_point==1
     && L_CirCle_State_lock==0 && l_gray_point==1 && loss_to_acc_count_row)      //左角点出现 ，没有上锁，确认是圆弧
       {
            L_CirCle_State=3;
            L_CirCle_State_lock=1;            //上锁
       }

     if(R_straightaway==0)  //如果右边界不是直道
       {
           L_CirCle_State=0;
       }
     break;
     case 3:                          //当前状态寻右边线
         if(L_CirCle_State_lock==1)
         {
             for(i=40;i>15;i--)
             {
                 if(Left_Line_flag[i]==0 && Left_Line_flag[i-1]==0 &&
                    Left_Line_flag[i-2]==1 &&  Left_Line_flag[i-3]==1)
                 {
                     L_CirCle_State_lock=0; //解锁
                     break;
                 }
             }
         }

     if(                         //圆弧消失判断,如果越高，则越提前进入状态4
        ( !Left_Line_flag[40] && !Left_Line_flag[41]
          &&!Left_Line_flag[42] && !Left_Line_flag[43]
          &&!Left_Line_flag[44] && !Left_Line_flag[45]
          &&left_angle_point==0 && L_CirCle_State_lock==0))  //如果左边界靠近底下丢线 ，角点消失
             {
                           Beep_set(1);
                           L_CirCle_State=4;                //进入下一个状态
                           In_Poit_y=0;                    //清空坐标等待下次补线
                           In_Poit_x=0;
                           Ring_add_angle=0;
                           Angle_flag=1;       //可以进行角度累加
             }
         if(R_straightaway==0)  //如果右边界不是直道
            {
             L_CirCle_State=0;
            }
     break;
     case 4:
         for(i=45;i>=20;i--)     //寻找迷宫起点
         {
             if(image[i][25]==255 && image[i-1][25]==255 && image[i-2][25]==0 && image[i-3][25]==0)
             {
                 In_Poit_y=i-1;
                 In_Poit_x=25;
                 break;
             }
         }
         if(In_Poit_y!=0)
         {
                  findline_lefthand_binaryzation_angle_user(image, In_Poit_x, In_Poit_y, element_rpts_l,&element_rpts_num_l);  //重新扫描边界
                  for(i=0;i<element_rpts_num_l;i++)  //角度清空
                  {
                      element_a_l[i] = 0;
                      element_outangle_l[i]=0;
                  }
                  for(i=0;i<element_rpts_num_l;i++)  //计算角度变化率
                  {
                      if (i <= 0 || i >= element_rpts_num_l-1)
                      {
                          element_a_l[i] = 0;
                          continue;
                      }
                      limit_up=clip(i+angle_dist,0,element_rpts_num_l-1);
                      limit_down=clip(i-angle_dist,0,element_rpts_num_l-1);

                      element_a_l[i]=180-getAngle(element_rpts_l[limit_up][0],
                              element_rpts_l[limit_up][1],element_rpts_l[i][0],
                              element_rpts_l[i][1],element_rpts_l[limit_down][0],
                              element_rpts_l[limit_down][1]);
                  }

                  nms_angle(element_a_l, element_rpts_num_l, element_outangle_l,nums_angle_dist);   //角度极大值抑制

                  for(i=0;i<element_rpts_num_l;i++)         //提取角点
                  {
                      if(element_outangle_l[i]==0)
                          continue;
                      if(element_outangle_l[i]>50)
                      {

                         In_Poit_x= element_rpts_l[i][0];
                         In_Poit_y= element_rpts_l[i][1];
                         break;
                      }
                  }
         }
         slope_to_angle=In_Poit_x-Right_Line[59];            //计算补线斜率
         slope_to_angle=slope_to_angle/(In_Poit_y-59);       //计算补线斜率
         slope_to_angle=atanf(slope_to_angle)*57.32;          //直接转化为角度
         can_slopetoangle=1;                                //进行斜率打角
         //下面判断是否快进入圆环
         if((!Right_Line_flag[50] && !Right_Line_flag[49] &&
            !Right_Line_flag[48] && !Right_Line_flag[47] &&
            !Right_Line_flag[46] && !Right_Line_flag[45]) && r_gray_point==0 )      //如果右边界靠近底下丢线
           {
              L_CirCle_State=5;                                //进入下一个状态
           }
     break;

     case 5:
         for(i=45;i>=20;i--)     //寻找迷宫起点
          {
              if(image[i][35]==255 && image[i-1][35]==255 && image[i-2][35]==0 && image[i-3][35]==0)
              {
                  In_Poit_y=i-1;
                  In_Poit_x=35;
                  break;
              }
          }
          if(In_Poit_y!=0)
          {
                   findline_lefthand_binaryzation_angle_user(image, In_Poit_x, In_Poit_y, element_rpts_l,&element_rpts_num_l);  //重新扫描边界
                   for(i=0;i<element_rpts_num_l;i++)  //角度清空
                   {
                       element_a_l[i] = 0;
                       element_outangle_l[i]=0;
                   }
                   for(i=0;i<element_rpts_num_l;i++)  //计算角度变化率
                   {
                       if (i <= 0 || i >= element_rpts_num_l-1)
                       {
                           element_a_l[i] = 0;
                           continue;
                       }

                       limit_up=clip(i+angle_dist,0,element_rpts_num_l-1);
                       limit_down=clip(i-angle_dist,0,element_rpts_num_l-1);

                       element_a_l[i]=180-getAngle(element_rpts_l[limit_up][0],
                               element_rpts_l[limit_up][1],element_rpts_l[i][0],
                               element_rpts_l[i][1],element_rpts_l[limit_down][0],
                               element_rpts_l[limit_down][1]);
                   }

                   nms_angle(element_a_l, element_rpts_num_l, element_outangle_l, nums_angle_dist);   //角度极大值抑制

                   for(i=0;i<element_rpts_num_l;i++)         //提取角点
                   {
                       if(element_outangle_l[i]==0)
                           continue;
                       if(element_outangle_l[i]>50)
                       {
                          In_Poit_x= element_rpts_l[i][0];
                          In_Poit_y= element_rpts_l[i][1];
                          break;
                       }
                   }
          }

         slope_to_angle=In_Poit_x-Right_Line[59];            //计算补线斜率
         slope_to_angle=slope_to_angle/(In_Poit_y-59);       //计算补线斜率
         slope_to_angle=atanf(slope_to_angle)*57.32;
         can_slopetoangle=1;                                 //进行斜率打角
        if(Ring_add_angle>3.1 ||( Right_Line_flag[50] &&
           Right_Line_flag[45] &&  Right_Line_flag[40] && r_gray_point==1 ) )  //判断是否已经入环
           {
              L_CirCle_State=6;     //进入下一个状态
              Out_Poit_x=0;         //清空坐标
              Out_Poit_y=0;
              slope_to_angle=0;
              Add_steer_duty=0;               //清空占空比
              Add_steer_duty_count=0;           //清空计数
           }
     break;
     case 6 :      //当前状态已经进入圆环，正常循迹（如果出现快到出环拐点位置直接直行，我感觉是没有判断到状态7）
         Add_steer_duty+=streeing_duty;         //占空比累加
         Add_steer_duty_count++;               //累加次数

         if(Ring_add_angle>6.4 && (right_angle_point && r_angle_point_y>=20 && r_angle_point_y<=45))//如果已经接近出环(环内误差取底下30行，所以这个角度刚好是角点没有低于30行的最大值)
         {
         Out_Poit_y=Find_guaidian(20, 45, Right_Line, 2, 1);   //寻找右拐点
         if((r_original_angle >55 && r_grow_outside) || (Out_Poit_y>=20&& Out_Poit_y<=45)) //存在角点,向外生长,或者有拐点
         {
             if(Out_Poit_y>=20&& Out_Poit_y<=45)  //拐点条件
             {
             Out_Poit_x=r_angle_point_x;  //记录右角点x坐标
             Out_Poit_y=r_angle_point_y; //记录右角点y坐标
             }
             else {
             Out_Poit_x=Right_Line[Out_Poit_y];  //记录右角点x坐标
            }

             for(i=Out_Poit_y-1;i>=Out_Poit_y-15;i--)
              {
                 if(image[i][Out_Poit_x]==255) //从下往上扫描,是否有连续白点(防误判)
                 {
                     white_row++;
                     if(white_row>=10)        //往上有连续5行白点
                     {
                         L_CirCle_State=7;  //说明准备出环，进入下一个状态
                         L_slope_cross_state=0;    //防十字
                         R_slope_cross_state=0;     //防十字
                         Flag_Cross=0;              //防十字
                         Average_duty= Add_steer_duty/Add_steer_duty_count;//计算平均占空比
                         Add_steer_duty=0;
                         Add_steer_duty_count=0;
                         Out_Poit_y=0;    //清空坐标
                         Out_Poit_x=0;   //清空坐标
                         break;
                     }
                 }else{
                     white_row=0;
                 }
             }
         }
       }

     break;
     case 7:                  //进行平均打角
         average_steer_duty=1;             //进行平均打角
         if(r_gray_point==0 && right_angle_point==0 )
         {
            L_CirCle_State=8;                                                       //进入出环状态
         }
     break;
     case 8:                                                                         //补线继续出环
         average_steer_duty=1;             //进行平均打角
         if(r_gray_point==1 && Ring_add_angle>8 && R_straightaway==1)  //右边赛道恢复
         {
            Beep_set(0);
            L_CirCle_State=9;                                                       //进入出环状态
            Ring_add_angle=0;
         }

     break;
     case 9 :                                        //这个状态寻右边界
         if((Right_Line_flag[50] && Right_Line_flag[49] &&
            Right_Line_flag[48] && Right_Line_flag[47] &&
            Right_Line_flag[46] && Right_Line_flag[45] &&
            Left_Line_flag[50]&& Left_Line_flag[49] &&
            Left_Line_flag[48]&& Left_Line_flag[47] &&
            Left_Line_flag[46]&& Left_Line_flag[45]) || (L_straightaway && R_straightaway))  //如果左右边界都不丢线
         {
            if(OFF_Element_test==1)
            {
              Door_count=track_num;
            }
            L_CirCle_State=0;  //已经完全出环
            Flag_L_CirCle=0;     //清空
            Average_duty=0;   //平均占 空比清0
            slope_to_angle=0;    //斜率转角度清0
            Add_steer_duty_count=0;//清空计数
            Add_steer_duty=0;  //累计的占空比清0
            Angle_flag=0;
            Ring_add_angle=0;
         }
     break;
 }
/*******************圆环状态处理*******************************/
         if(L_CirCle_State==1 || L_CirCle_State==2 || L_CirCle_State==3 ||  L_CirCle_State==9 )           //寻右边线
           {
              for(i=59;i>20;i--)
                  Mid_Line[i]=clip((Right_Line[i]-Width_straight[i]/2),0,93);
           }
  }
 /****************************开始处理右环状态******************************/
 if(Flag_R_CirCle)   //右环处理,右环参数和左环不一样，一样参数下，右环进不去
  {
  switch(R_CirCle_State)
  {
     case 1:                  //当前状态寻左边线
     if((r_gray_point==0 && !Right_Line_flag[50] && !Right_Line_flag[49] && !Right_Line_flag[48]
        && !Right_Line_flag[47] && !Right_Line_flag[46]&&
        !Right_Line_flag[45]) &&  r_ring_check_point==0 ) //如果右边界底下全丢和圆环角点消失,说明靠近圆环圆弧
         {                                                                                                                                                                 //拐点消失之后的弧度很明显
              R_CirCle_State=2;  //进入下一个状态
              R_CirCle_State_lock=1;
         }
     if(L_straightaway==0)  //如果左边界不是直道
        {
            R_CirCle_State=0;
        }
      break;
      case 2:                            //当前状态寻左边线
          if(R_CirCle_State_lock==1)
          {
              for(i=57;i>=40;i--)        //判断丢丢不丢不丢
              {
                  if(Right_Line_flag[i]==0 && Right_Line_flag[i-1]==0 &&
                     Right_Line_flag[i-2]==1 &&  Right_Line_flag[i-3]==1)
                  {
                      R_CirCle_State_lock=0;
                      break;
                  }
              }
          }

          if(R_CirCle_State_lock==0) //判断弧度
          {
            for(i=0;i<50;i++)
            {
                if(points_r[i][0] > points_r[i+1][0] && dj<12)  //先判断单减
                {
                    dj++;
                }

                if(points_r[i][0] < points_r[i+1][0] && dz<12 && dj>=12) //单减符合之后才可以判断单增
                {
                    dz++;
                }

                if(dz>=12 && dj>=12)
                {
                    loss_to_acc_count_row=1;   //确认有圆弧
                    break;
                }
            }
          }

           if(Right_Line_flag[50] && Right_Line_flag[49] && Right_Line_flag[48]
              && Right_Line_flag[47] && Right_Line_flag[46]&&
              Right_Line_flag[45] && right_angle_point==1 && R_CirCle_State_lock==0
              && r_gray_point==1 && loss_to_acc_count_row==1)      //右角点出现 ，确认是圆弧
            {
               R_CirCle_State=3;
               R_CirCle_State_lock=1;  //上锁
            }
           if(L_straightaway==0)  //如果左边界不是直道
              {
                  R_CirCle_State=0;
              }

      break;
      case 3:                          //当前状态寻左边线
          if(R_CirCle_State_lock==1)
          {
              for(i=40;i>15;i--)
              {
                  if(Right_Line_flag[i]==0 && Right_Line_flag[i-1]==0 &&
                     Right_Line_flag[i-2]==1 &&  Right_Line_flag[i-3]==1)
                  {
                      R_CirCle_State_lock=0; //解锁
                      break;
                  }
              }
          }


          if(                         //圆弧消失判断,如果越高，则越提前进入状态4
             !Right_Line_flag[40] && !Right_Line_flag[41]
             &&!Right_Line_flag[42] && !Right_Line_flag[43]
             &&!Right_Line_flag[44] && !Right_Line_flag[45]
             && right_angle_point==0 && R_CirCle_State_lock==0)  //如果右边界靠近底下丢线 ，角点消失
              {
                            Beep_set(1);
                            R_CirCle_State=4;                //进入下一个状态
                            In_Poit_y=0;                    //清空坐标等待下次补线
                            In_Poit_x=0;
                            Ring_add_angle=0;
                            Angle_flag=1;                   //可以进行角度累加
              }
          if(L_straightaway==0)  //如果左边界不是直道
             {
                 R_CirCle_State=0;
             }
      break;
      case 4:
          for(i=45;i>=20;i--)     //寻找迷宫起点
          {
              if(image[i][68]==255 && image[i-1][68]==255 && image[i-2][68]==0 && image[i-3][68]==0)
              {
                  In_Poit_y=i-1;
                  In_Poit_x=68;
                  break;
              }
          }
          if(In_Poit_y!=0)
          {
                   findline_righthand_binaryzation_angle_user(image, In_Poit_x, In_Poit_y, element_rpts_r,&element_rpts_num_r);  //重新扫描边界
                   for(i=0;i<element_rpts_num_r;i++)  //角度清空
                   {
                       element_a_r[i] = 0;
                       element_outangle_r[i]=0;
                   }
                   for(i=0;i<element_rpts_num_r;i++)  //计算角度变化率
                   {
                       if (i <= 0 || i >= element_rpts_num_r-1)
                       {
                           element_a_r[i] = 0;
                           continue;
                       }

                       limit_up=clip(i+angle_dist,0,element_rpts_num_r-1);
                       limit_down=clip(i-angle_dist,0,element_rpts_num_r-1);

                       element_a_r[i]=180-getAngle(element_rpts_r[limit_up][0],
                               element_rpts_r[limit_up][1],element_rpts_r[i][0],
                               element_rpts_r[i][1],element_rpts_r[limit_down][0],
                               element_rpts_r[limit_down][1]);
                   }

                   nms_angle(element_a_r, element_rpts_num_r, element_outangle_r, angle_dist);   //角度极大值抑制

                   for(i=0;i<element_rpts_num_r;i++)         //提取角点
                   {
                       if(element_outangle_r[i]==0)
                           continue;
                       if(element_outangle_r[i]>70)
                       {
                          In_Poit_x= element_rpts_r[i][0];
                          In_Poit_y= element_rpts_r[i][1];
                          break;
                       }
                   }
                //draw_point_blue(In_Poit_x, In_Poit_y);
          }
          slope_to_angle=In_Poit_x-Left_Line[59];            //计算补线斜率
          slope_to_angle=slope_to_angle/(In_Poit_y-59);       //计算补线斜率
          slope_to_angle=atanf(slope_to_angle)*57.32;          //直接转化为角度
          can_slopetoangle=1;
          //下面判断是否快进入圆环
          if( (!Left_Line_flag[50] && !Left_Line_flag[49] &&
             !Left_Line_flag[48] && !Left_Line_flag[47] &&
             !Left_Line_flag[46] && !Left_Line_flag[45]) && l_gray_point==0)      //如果左边界靠近底下丢线
            {
               R_CirCle_State=5;                                //进入下一个状态
            }
      break;

      case 5:
          for(i=45;i>=20;i--)     //寻找迷宫起点
           {
               if(image[i][58]==255 && image[i-1][58]==255 && image[i-2][58]==0 && image[i-3][58]==0)
               {
                   In_Poit_y=i-1;
                   In_Poit_x=58;
                   break;
               }
           }
           if(In_Poit_y!=0)
           {
                    findline_righthand_binaryzation_angle_user(image, In_Poit_x, In_Poit_y, element_rpts_r,&element_rpts_num_r);  //重新扫描边界
                    for(i=0;i<element_rpts_num_r;i++)  //角度清空
                    {
                        element_a_r[i] = 0;
                        element_outangle_r[i]=0;
                    }
                    for(i=0;i<element_rpts_num_r;i++)  //计算角度变化率
                    {
                        if (i <= 0 || i >= element_rpts_num_r-1)
                        {
                            element_a_r[i] = 0;
                            continue;
                        }

                        limit_up=clip(i+angle_dist,0,element_rpts_num_r-1);
                        limit_down=clip(i-angle_dist,0,element_rpts_num_r-1);

                        element_a_r[i]=180-getAngle(element_rpts_r[limit_up][0],
                                element_rpts_r[limit_up][1],element_rpts_r[i][0],
                                element_rpts_r[i][1],element_rpts_r[limit_down][0],
                                element_rpts_r[limit_down][1]);
                    }

                    nms_angle(element_a_r, element_rpts_num_r, element_outangle_r, nums_angle_dist);   //角度极大值抑制

                    for(i=0;i<element_rpts_num_r;i++)         //提取角点
                    {
                        if(element_outangle_r[i]==0)
                            continue;
                        if(element_outangle_r[i]>50)
                        {
                           In_Poit_x= element_rpts_r[i][0];
                           In_Poit_y= element_rpts_r[i][1];
                           break;
                        }
                    }
                    //draw_point_blue(In_Poit_x, In_Poit_y);
           }
          slope_to_angle=In_Poit_x-Left_Line[59];            //计算补线斜率
          slope_to_angle=slope_to_angle/(In_Poit_y-59);       //计算补线斜率
          slope_to_angle=atanf(slope_to_angle)*57.32;
          can_slopetoangle=1;
          if(Ring_add_angle<-3.2 || ( Left_Line_flag[50] &&
              Left_Line_flag[45] &&  Left_Line_flag[40] && l_gray_point==1))  //判断是否已经入环
            {
               R_CirCle_State=6;     //进入下一个状态
               Out_Poit_x=0;         //清空坐标
               Out_Poit_y=0;
               slope_to_angle=0;
               Add_steer_duty=0;               //清空占空比
               Add_steer_duty_count=0;           //清空计数
            }
      break;
      case 6 :                                     //当前状态已经进入圆环，正常循迹
          Add_steer_duty+=streeing_duty;         //占空比累加
          Add_steer_duty_count++;               //累加次数

          if(Ring_add_angle<-6.4 && (left_angle_point && l_angle_point_y>=20 && l_angle_point_y<=45 )) //如果已经接近出环
          {
          Out_Poit_y=Find_guaidian(20, 45, Left_Line, 1, 1);   //寻找左拐点
          if(( l_original_angle >55 && l_grow_outside ) || (Out_Poit_y>=20 && Out_Poit_y<=45) )  //存在角点,向外生长
          {
             if(Out_Poit_y>=20 && Out_Poit_y<=45)  //<30说明是角点条件成立，否则是拐点条件
              {
              Out_Poit_x=l_angle_point_x;  //记录右角点x坐标
              Out_Poit_y=l_angle_point_y; //记录右角点y坐标
              }
              else {
              Out_Poit_x=Left_Line[Out_Poit_y];  //记录右角点x坐标
              }

              for(i=Out_Poit_y-1;i>=Out_Poit_y-15;i--)
               {
                  if(image[i][Out_Poit_x]==255) //从下往上扫描,是否有连续白点
                  {
                      white_row++;
                      if(white_row>=10)        //往上有连续10行白点
                      {
                          R_CirCle_State=7;  //说明准备出环，进入下一个状态
                          L_slope_cross_state=0;     //防十字
                          R_slope_cross_state=0;     //防十字
                          Flag_Cross=0;          //防十字
                          Average_duty= Add_steer_duty/Add_steer_duty_count;//计算平均占空比
                          Add_steer_duty=0;
                          Add_steer_duty_count=0;
                          Out_Poit_y=0;    //清空坐标
                          Out_Poit_x=0;   //清空坐标
                          break;
                      }
                  }else{
                      white_row=0;
                  }
              }
          }
        }

      break;
      case 7:                                                                         //补线出环
          average_steer_duty=1;             //进行平均打角
          if(l_gray_point==0  &&  left_angle_point==0 )           //如果右边界都丢线
          {
             R_CirCle_State=8;                                                       //进入出环状态
          }

      break;
      case 8:                                                                         //补线继续出环
          average_steer_duty=1;             //进行平均打角
          if( l_gray_point==1 && Ring_add_angle<-8 && L_straightaway==1 )  //左边赛道恢复
          {
             Beep_set(0);
             R_CirCle_State=9;                                                       //进入出环状态
             Ring_add_angle=0;
          }

      break;
      case 9 :                                        //这个状态寻左边界
          if((Right_Line_flag[50] && Right_Line_flag[49] &&
             Right_Line_flag[48] && Right_Line_flag[47] &&
             Right_Line_flag[46] && Right_Line_flag[45] &&
             Left_Line_flag[50]&& Left_Line_flag[49] &&
             Left_Line_flag[48]&& Left_Line_flag[47] &&
             Left_Line_flag[46]&& Left_Line_flag[45]) || (L_straightaway && R_straightaway))  //如果左右边界都不丢线
          {
             if(OFF_Element_test==1)
             {
                Door_count=track_num;
             }
             R_CirCle_State=0;  //已经完全出环
             Flag_R_CirCle=0;
             Average_duty=0;   //平均占空比清0
             slope_to_angle=0;    //斜率转角度清0
             Add_steer_duty_count=0;//清空计数
             Add_steer_duty=0;  //累计的占空比清0
             Angle_flag=0;
             Ring_add_angle=0;
          }
      break;
  }
 /*******************圆环状态处理*******************************/
     if(R_CirCle_State==1 || R_CirCle_State==2 || R_CirCle_State==3 ||  R_CirCle_State==9 )           //寻右边线
       {
          for(i=59;i>=20;i--)
             Mid_Line[i]=clip((Left_Line[i]+Width_straight[i]/2),0,93);
       }
   }
}
/*---------------------------------------------------------------
 【功    能】障碍物处理
 【参    数】图像数组，tof测得的距离
 【返 回 值】无
 ----------------------------------------------------------------*/
uint8 Barrier_state=0;       //障碍物状态
float Barrier_Add_angle=0;      //累加的角度
float  out_angle=1.4;          //出去角度
float  r_out_angle=-1.4;     //右绕行出去
int16  out_bypass_duty=-110;  //绕出去舵机打角
int16  in_bypass_duty=110;   //绕回来舵机打角
uint16 check_distance=900;  //检测距离mm
bool Barrier_out_count=0;   //过障碍物进行减速
void  Barrier_Deal(uint8(*image)[image_w],float tof_distance )
{
    /***********************预判断*****************************/
    bool l_straight=0,r_straight=0;
    if(tof_distance< check_distance && hightest>=14 && Barrier_state==0)
    {
        l_straight=Straight_line_judgment(Left_Line,1,35,59);
        r_straight=Straight_line_judgment(Right_Line,2,35,59);
        if(l_straight && r_straight)
        Barrier_state=1;                                //进入障碍物标志位1
    }
    if(left_angle_point==true && right_angle_point==true
      && l_grow_inside==1 && r_grow_inside==1 && Barrier_state==1) //存在双角点且向内
    {
               Barrier_state=2;                                    //进入障碍物标志位1
               Barrier_Add_angle=0;
               streeing_duty=out_bypass_duty;                   //在没有完全行驶出赛道之前舵机打死
               Beep_set(1);                                     //蜂鸣器响
    }

    switch(Barrier_state)
    {
    case 2:                                 //绕行出去
        if(Barrier_dir==1)  //左绕
        {
            streeing_duty=out_bypass_duty;
            Barrier_Add_angle+=Turn_speed*Turn_dt;   //记录角度
            if(Barrier_Add_angle>out_angle && ((Inductor_Value[0]+Inductor_Value[1]+Inductor_Value[2]+Inductor_Value[3])<7))
            {
                Turn_speed=0;
                Barrier_state=3;
                Barrier_Add_angle=0;
            }
        }
        else {
            streeing_duty=-out_bypass_duty;
            Barrier_Add_angle+=Turn_speed*Turn_dt;   //记录角度
            if(Barrier_Add_angle< r_out_angle && (Inductor_Value[0]+Inductor_Value[1]+Inductor_Value[2]+Inductor_Value[3])<7)      //右绕行出去
            {
                Turn_speed=0;
                Barrier_state=3;
                Barrier_Add_angle=0;
            }
        }
    break;
    case 3:                                //绕行回来
      if(Barrier_dir==1)
      {
          streeing_duty=in_bypass_duty;
      }
      else {
          streeing_duty=-in_bypass_duty;
      }
      if((Inductor_Value[0]+Inductor_Value[1]+Inductor_Value[2]+Inductor_Value[3])>13)
       {
            if(OFF_Element_test==1)
            {
              Door_count=track_num;
            }
            Barrier_out_count=1;
            Barrier_state=0;
            Barrier_Add_angle=0;
       }
    break;
   }
}
/*---------------------------------------------------------------
 【功    能】最终处理函数
 【参    数】二值化图像
 【返 回 值】无
 ----------------------------------------------------------------*/
void Final_deal(uint8(*image)[image_w]){
    if(OFF_Element_test==0)        //非元素测试阶段
    {
    Out_door_deal(Out_door_type);
    if(OFF_Charge_Resue)          //1为开启充电救援
    Charge_Resue(Out_door_type);
    }
    else {
    Out_door_state=1;            //元素测试阶段,不执行出库
    }

    if(Out_door_state==1 && Door_count!=track_num) //出库成功或者入库完成
    {
    REPIRE_LINE=0;            //补线标志位清空
    if(Distance>0.15)        //出库一小段距离进行元素识别
    {
    L_straightaway=0;                //左直道标志位
    R_straightaway=0;                //右直道标志位
    L_straightaway=Straight_line_judgment(Left_Line,1,20,55);    //OK，使用没啥问题
    R_straightaway=Straight_line_judgment(Right_Line,2,20,55);   //OK，使用没啥问题
    Repire();                                                  //只对直道进行修补,OK，使用没啥问题
    In_door_deal(Out_door_type,image);                       //入右库,OK，使用没啥问题
    Ring_Deal(image);                                      //正入圆环
    if(Flag_Cross==0)
    {
     slope_cross_deal(User_image);                     //斜入十字
    }
    if(slope_cross==0)
    {
     cross_deal(User_image);                        //正入十字
    }
    Check_Small_S();                              //s弯判断
    Ramp_Deal(gyrox);                            //坡道判断
    Break_Road_Deal(image,dl1a_distance_mm);    //断路处理
    Barrier_Deal(image,dl1a_distance_mm);      //障碍物处理
    if(L_CirCle_State>=3 && R_CirCle_State>=3)
    {
      L_slope_cross_state=0;
      R_slope_cross_state=0;
      Flag_Cross=0;
    }
  }
 }
    steer_deal();
}

/*---------------------------------------------------------------
 【功    能】舵机处理函数
 【参    数】无
 【返 回 值】无
 ----------------------------------------------------------------*/
void steer_deal()
{
    if(Barrier_state==2 ||  Barrier_state==3  || Out_door_state==0)
    {                                        //如果在障碍物状态或者出车库状态
    if(Out_door_type==1 && Out_door_state==0)  //出左库
    {
        streeing_duty=-120;
    }else if(Out_door_type==2 && Out_door_state==0)//出右库
    {
        streeing_duty= 120;
    }
    final_err=0;
    }

    else
    {
    if(can_slopetoangle==1)//进行斜率打角
    {
        streeing_duty=slopetoangle_user_PID(slope_to_angle);
    }
    else if(average_steer_duty) //进行平均打角
    {
        streeing_duty=Average_duty;
    }
    else if(Charge_Resue_state==2) //救援状态,倒退舵机取反
    {
        final_err=turn_error(Level_Deviation);
        streeing_duty=-Steering_gear_PID(final_err)*1.6;
    }
    else  //正常状态
    {
        final_err=turn_error(Level_Deviation);
        if(Break_road_state==0)
        streeing_duty=Steering_gear_PID(final_err);
        else {                                                 //断路
        streeing_duty =Inductor_user_PID(final_err);
      }
    }
  }
}
/*---------------------------------------------------------------
 【功    能】出车库（执行一次)(可以检测并出库,这个函数暂时不用改了)
 【参    数】type ：1 出左库  2 出右库
 【返 回 值】无
 【使    用】Out_door_deal(1)(舵机负为左)
 ----------------------------------------------------------------*/
float Out_L_door_angle=1.5;                            //出左车库需要累加的角度
float Out_R_door_angle=-1.5;                          //出右车库需要累加的角度
float ADD_angle=0;                                   //累加的角度
uint8 Out_door_state=0;                             //出车库状态
void Out_door_deal(uint8 type)
{
     if(Out_door_state==0 && type==2)                                                //执行右出库
     {
      ADD_angle+=Turn_speed*Turn_dt;                                         //角度累加
      if( ADD_angle<Out_R_door_angle)
      {
          Out_door_state=1;                                                //出库成功
          ADD_angle=0;
      }
     }

     if(Out_door_state==0 && type==1)                                                //执行左出库
     {
      ADD_angle+=Turn_speed*Turn_dt;                                         //角度累加
      if(ADD_angle>Out_L_door_angle)
      {
          Out_door_state=1;                                                //出库成功
          ADD_angle=0;
      }
     }
}
/*---------------------------------------------------------------
 【功    能】入车库(可以检测并入库,这个函数暂时不用改了)
 【参    数】入库类型type 1 入左库  2 入右库  二值化图像
 【返 回 值】无
 ----------------------------------------------------------------*/
uint8 Indooor_state=0;                            //斑马线识别标志位
uint8 Door_count=0;                              //遇到斑马线次数,入库成功后值置成3，然后停车
float L_Angle_door=1.0;                           //左入库累计角度
float R_Angle_door=-1.0;                          //右入库累计角度
float Add_angle_door=0;                           //入库累计角度
float Add_distance_no_door=0;                      //不入库停车距离
float Add_distance_door=0;                       //识别到斑马线之后行驶一段距离才能再次识别
void In_door_deal(uint8 type,uint8(*image)[image_w])
{
    uint8 i=0,j=0;
    uint8 garage_count=0,region=0;                        //斑马线跳变点计数
    uint8 white_black=1,black_white=1;                   //白黑，黑白跳变点
    if((Indooor_state>=2 && type==2) || Door_count==2)                   //右库重新处理边界，防止抖动
    {
        uint8 pts_l[2]={1,59};
        bool fine_start_point=0;
        int num_L=50;
        for(i=3;i<=47;i++)
        {
            if(image[57][i]==0&& image[57][i+1]==0&&image[57][i+2]==255&&image[57][i+3]==255)
            {
                pts_l[0]=i+2; //x
                pts_l[1]=57;//y
                fine_start_point=1;
                break;
            }
        }
        if(fine_start_point)
        {
        findline_lefthand_binaryzation(image, pts_l[0],pts_l[1], points_l,&num_L);//重新扫描边界
        Get_new_Line(Left_Line,pts_l[0],pts_l[1],points_l,num_L);                //重新获取边界
        }
    }
    if((Indooor_state>=2 && type==1) || Door_count==2)                   //左库重新处理边界，防止抖动
    {
        uint8 pts_r[2]={1,59};
        bool fine_start_point=0;
        int num_R=50;
        for(i=90;i>=47;i--)
        {
            if(image[57][i]==0&& image[57][i-1]==0&&image[57][i-2]==255&&image[57][i-3]==255)
            {
                pts_r[0]=i-2; //x
                pts_r[1]=57;//y
                fine_start_point=1;
                break;
            }
        }
        if(fine_start_point)
        {
        findline_righthand_binaryzation(image, pts_r[0],pts_r[1], points_r,&num_R);//重新扫描边界
        Get_new_Line(Right_Line,pts_r[0],pts_r[1],points_r,num_R);                //重新获取边界
        }
    }

    if(Indooor_state && type==2)                   //入右库
    {
        for(i=59;i>=20;i--)
        {
            Mid_Line[i]=clip(Left_Line[i]+Width_straight[i]/2,0,93);
        }
    }else if(Indooor_state && type==1)             //入左库
    {
        for(i=59;i>=20;i--)
        {
            Mid_Line[i]=clip(Right_Line[i]-Width_straight[i]/2,0,93);
        }
    }

    switch(type)                           //入库类别
    {
    case 2:                                //右库处理
        switch(Indooor_state)
        {
        case 0:
            if(left_angle_point==0 && r_garage_check_point==1
               && r_grow_outside==1 && L_straightaway)       //左边无角点，左边直道，右边车库角点且向外生长
            {
                if(Left_Line_flag[r_garage_check_point_y]&&
                   Left_Line_flag[r_garage_check_point_y-1]&&
                   Left_Line_flag[r_garage_check_point_y-2])  //左边界的右角点坐标不丢线
                {
                    Indooor_state=1;          //进入下一个状态
                }
            }
        break;
        case 1:
            if(r_gray_point==0 && l_gray_point==1) //然后进入右边丢线，左边不丢
            {
                Indooor_state=2;          //进入下一个状态
                 Beep_set(1);
            }
        break;
        case 2:
            for(i =45;i>=40;i--)    //开始横向扫描8行判断是否存在黑白跳变点
            {
                garage_count= 0;
                for(j =80;j>47-Width_straight[i]/2;j--)      //右往左扫,47-宽度一半这个情况，要保证姿态正
                  {
                    if(image[i][j]==255)           //如果当前点为白
                    {
                     white_black=1;
                     }
                    else
                    {
                     white_black=0;
                    }
                   if(white_black!=black_white)        //black_white原始值为1,代表白点
                    {
                    black_white = white_black;         //存在跳变
                    garage_count++;                    //跳变数+1
                    }
                   if(garage_count>9)                  //一行存在10个跳变点
                    {
                       region++;                      //行数+1
                       break;
                    }
                 }
                if(region>=3)       //3行以上,确定是斑马线
                  {
                   L_CirCle_State=0;                  //车库防圆环误判
                   R_CirCle_State=0;                  //车库防圆环误判
                   Indooor_state=3;            //进入下一个状态
                   Beep_set(0);
                   Door_count+=1;               //路过次数+1
                   break;
                  }
             }

       break;
       case 3:
           Add_distance_door+=Car_Speed*Car_speed_dt;
           if(Add_distance_door>0.1)           //走了0.4m
           {
               Indooor_state=0;            //已经路过一次斑马线
               Door_count+=1;               //路过次数+1
               Add_distance_door=0;
           }
      break;
    }
    break;

    case 1:                           //左库处理
      switch(Indooor_state)
        {
        case 0:
        if(right_angle_point==0 && l_garage_check_point==1
           && l_grow_outside==1 && R_straightaway)       //右边无角点，右边直道，左边车库角点且向外生长
        {
            if(Right_Line_flag[l_garage_check_point_y]&&
               Right_Line_flag[l_garage_check_point_y-1]&&
               Right_Line_flag[l_garage_check_point_y-2])  //右边界的左角点坐标不丢线
            {
                Indooor_state=1;          //进入下一个状态
            }
        }
        break;
        case 1:
            if(l_gray_point==0 && r_gray_point==1)
            {
              Indooor_state=2;          //进入下一个状态
              Beep_set(1);
            }
        break;
        case 2:
            for(i=45;i>=40;i--)                                    //开始横向扫描判断是否存在黑白跳变点
            {
               garage_count = 0;
               for(j =13;j<47+Width_straight[i]/2;j++)      //左往右扫
                  {
                    if(image[i][j]==255)           //如果当前点为白
                    {
                     white_black=1;
                     }
                    else
                    {
                     white_black=0;
                     }
                   if(white_black!=black_white)        //black_white原始值为1,代表白点
                    {
                    black_white = white_black;         //存在跳变
                    garage_count++;                    //跳变数+1
                    }
                   if(garage_count>9)                  //一行存在10个跳变点
                    {
                       region++;                      //行数+1
                       break;
                    }
                 }
             if(region>=3)                                //3行以上,确定是斑马线
               {
                L_CirCle_State=0;                      //防圆环
                R_CirCle_State=0;                     //防圆环
                Indooor_state=3;                     //进入下一个状态
                Beep_set(0);
                Door_count+=1;               //路过次数+1
                break;
               }
             }
       break;
       case 3:
            Add_distance_door+=Car_Speed*Car_speed_dt;
            if(Add_distance_door>0.1)           //走了0.05m
            {
                Indooor_state=0;            //已经路过一次斑马线
                Door_count+=1;               //路过次数+1
                Add_distance_door=0;
            }
       break;
    }
     break;
  }
}
/*---------------------------------------------------------------
 【功    能】出车库之后进行充电救援
 【参    数】dir_type 1  左       2    右
 ----------------------------------------------------------------*/
uint8 Charge_Resue_state=0;   //救援状态
uint8 Charge_Resue_count=0;    //充电使能发射计数
int16 L_charge_big_thes=45,L_charge_small_thes=10,R_charge_big_thes=45,R_charge_small_thes=13;//可以停止开始倒车的电感阈值
void Charge_Resue(uint8 dir_type)
{
     static bool successful_recure=0; //成功救援标志位
     if(Out_door_state==1 && Charge_Resue_state==0 && successful_recure==0) //出库成功之后立刻进入救援状态
     {
         Charge_Resue_state=1;
     }

     if(Charge_Resue_count==30 && car_state==16)//延时1.5s,停稳再发射
     {
        gpio_set_level(B14,1); //使能开启充电发射
     }

    switch(Charge_Resue_state)
      {
      case 1:
      if( ((Inductor_Value[0]>L_charge_big_thes && Inductor_Value[1]<L_charge_small_thes)
         || (Inductor_Value[2]<R_charge_small_thes && Inductor_Value[3]>R_charge_big_thes) ) && successful_recure==0)  //电感值为直道状态
      {
          Charge_Resue_state=2;
      }
      break;

      case 2:            //此时为倒退,舵机取反
      if(dir_type==2)   //巡左边界 ,右救援
      {
          for(uint8 i=59;i>20;i--)
          {
              Mid_Line[i]=clip(Left_Line[i]+Width_straight[i]/2,0,93);
          }
          REPIRE_LINE=1;                      //重新确认中线起始点
      }

      if(dir_type==1)    //巡右边界 ,左救援
      {
          for(uint8 i=59;i>20;i--)
          {
              Mid_Line[i]=clip(Right_Line[i]-Width_straight[i]/2,0,93);
          }
          REPIRE_LINE=1;                      //重新确认中线起始点
      }

      if(r_gray_point==0 && l_gray_point==1 && successful_recure==0 && dir_type==2)   //右边边界丢线，可以停车，等待1s进行充电
      {
         successful_recure=1;
         Charge_Resue_state=3;
         Distance=0;   //清0,等待后车自检进行前进
      }

      if(r_gray_point==1 && l_gray_point==0 && successful_recure==0 && dir_type==1)   //左边边界丢线，可以停车，等待1s进行充电
      {
         successful_recure=1;
         Charge_Resue_state=3;
         Distance=0;   //清0,等待后车自检进行前进
      }
      break;
   }
}
/*---------------------------------------------------------------
 【功    能】坡道处理
 【参    数】某轴角速度
 【返 回 值】无
 ----------------------------------------------------------------*/
uint8 Ramp_state=0;      //坡道标志位
float add_gyro_x=0;      //x轴角速度积分
void Ramp_Deal(float gyro)
{
    add_gyro_x+=gyro*Turn_dt;
    static bool clear_flag=0;
    if(add_gyro_x < (-15) && Ramp_state==0)       //角速度很大（负）正在上坡
    {
       Ramp_state=1;
       clear_flag=0;
    }
    if(add_gyro_x > (10) && Ramp_state==1)         //角速度很大（正）正在下坡
    {
       Ramp_state=2;
    }
    if(fabs(gyro)<10 && Ramp_state==0) add_gyro_x=0;          //准备上坡，提前清空
    if(clear_flag==0 && (gyro)>6 && Ramp_state==1)              //准备下坡，提前清空
      {
        add_gyro_x=0;       //准备下坡，提前清空
        clear_flag=1;
      }
    if(Ramp_state==1)
    {
      Barrier_state=0;               //障碍物
    }
    if(Ramp_state==2)                //坡道清空所有标志位
    {
        L_CirCle_State=0;         //左圆环
        R_CirCle_State=0; //右圆环
        Flag_Cross=0;     //十字
        L_slope_cross_state=0;   //左写入十字
        R_slope_cross_state=0;   //右斜入十字
        Barrier_state=0;          //障碍物
        Beep_set(0);
    }
}
/*---------------------------------------------------------------
 【功    能】断路处理
 【参    数】图像数组，tof测得的距离
 【返 回 值】无
 ----------------------------------------------------------------*/
uint8 Break_road_state=0;                               //断路状态标志位
void Break_Road_Deal(uint8(*image)[image_w],float tof_distance)
{
    uint8 i=0,count=0;
    uint8 black_row=0;
    float l_slope=0,r_slope=0;  //左右斜率
    uint8 l_up=0,l_down=0;
    uint8 r_up=0,r_down=0;
    uint8 line_count=0;
    uint8 l_in_count=0,r_in_count=0,upper_l,upper_r;
    uint8 l_dz_count=0,r_dj_count=0;    //左右边界单调^计数
    bool assist_black_row=0;
   /***********************预判断*****************************/
  /***********************弯道断路判断*****************************/
    if(Break_road_state==0 && hightest>=25 && hightest<=50 && hightest_x>23
     && hightest_x<70 && L_CirCle_State==0 && R_CirCle_State==0
     && L_slope_cross_state==0 && R_slope_cross_state==0
     && Flag_Cross==0 && Indooor_state==0)                                //图像近处截短,非重要元素阶段
    {
        assist_black_row=Fine_heihang(2, hightest-5, hightest, image, 2);//寻找黑行
        regression(1, hightest+2, start_line_point_row);     //拟合左边界
        l_slope=parameterB;                                   //记录左边界斜率
        regression(2, hightest+2, start_line_point_row);   //拟合右边界
        r_slope=parameterB;                                 //记录右边界斜率
        /***********************疑是断路判断*****************************/
        if((l_gray_point || r_gray_point))  //进行弯道断路特征判断
        {
            for(i=59;i>=hightest;i--)
            {
               if(Left_Line_flag[i]==1 && Right_Line_flag[i]==1)
               {
                   line_count++;
               }
               if(Left_Line[i-1]>Left_Line[i])
               {
                   l_dz_count++;
               }
               if(Right_Line[i-1]<Right_Line[i])
               {
                   r_dj_count++;
               }
            }
             //如果断开处一边丢线比较多，把abs(59-hightest-line_count)<=10去掉
            if(abs(59-hightest-line_count)<=10 && ((r_slope>=1.05 && l_slope<= -0.8) || (r_slope>=0.8 && l_slope<= -1.05)))//符合弯道特征
            {
                if( (abs(59-hightest-l_dz_count)<8 || abs(59-hightest-r_dj_count)<8)
                  &&tof_distance>5500  && assist_black_row)
                    Break_road_state=1;
            }
        }
        //拐点判断
        //右边界
        for(i=10;i<=50;i++)
        {
          if( (points_r[i+1][0] < points_r[i][0]) && (points_r[i+1][1] < points_r[i][1]) && r_up<10)
          {
              r_up++;
          }
          if(r_down<10 && r_up>=10 && (points_r[i+1][0] < points_r[i][0]) && (points_r[i+1][1] > points_r[i][1]))
          {
              r_down++;
          }
            if(r_up>=10 && r_down>=10)
             {
                 break;
             }
        }
        //左边界判断
        for(i=10;i<=50;i++)
        {
          if((points_l[i+1][0] > points_l[i][0]) && (points_l[i+1][1] < points_l[i][1]) && l_up<10)
          {
              l_up++;
          }
          if(l_down<10 && l_up>=10 && (points_l[i+1][0] > points_l[i][0]) && (points_l[i+1][1] > points_l[i][1]))
          {
              l_down++;
          }
          if(l_up>=10 &&  l_down>=10)
          {
              break;
          }
        }

        if((l_up>=10 && l_down>=10) && (r_up>=10 && r_down>=10)    //双^点判断断路
          && l_slope<-0.85 && r_slope>0.85 && tof_distance>2500 && assist_black_row)
        {
            Break_road_state=1;
        }

        if( ((r_slope>=1.05 && l_slope<= -0.85) || (r_slope>=0.85 && l_slope<= -1.05) )//单独判断断路
           && image[hightest-2][hightest_x]==0 && image[hightest-3][hightest_x]==0
           && tof_distance>2500 && assist_black_row)
        {
           Break_road_state=1;
        }
       //弯道角点判断
       if(l_break_road_check_point && r_break_road_check_point)
        {
             upper_l=clip(l_break_road_check_point_y+13,0,element_rpts_num_l-1);    //向上判断13个点
             for(i=l_break_road_check_point_y;i<upper_l;i++)
             {
                 if(element_rpts_l[i][0]>element_rpts_l[l_break_road_check_point_y][0])  //左角点向内生长
                 {
                     l_in_count++;
                 }
                 if(l_in_count>7)
                 {
                     break;
                 }
             }
             upper_r=clip(r_break_road_check_point_y+13,0,element_rpts_num_r-1);    //向上判断13个点
             for(i=r_break_road_check_point_y;i<upper_r;i++)
             {
                 if(element_rpts_r[i][0]<element_rpts_r[r_break_road_check_point_y][0])  //左角点向内生长
                 {
                     r_in_count++;
                 }
                 if(r_in_count>7)
                 {
                     break;
                 }
             }
             if(l_in_count>7 && r_in_count>7 && tof_distance>2500 && l_slope<-0.85 && r_slope>0.85 && assist_black_row)
             {
                 Break_road_state=1;
             }
       }
   }
  //直道处截断判断
  if(left_angle_point==true && right_angle_point==true && Break_road_state==0) //存在双角点
   {
     if(l_grow_inside==1 && r_grow_inside==1)           //角点向内
       {
           black_row=Fine_heihang(2, 25, 30, image, 3);
           if(tof_distance>2500 && hightest>=30 && black_row)          //前方无障碍物
           {
               if(L_CirCle_State==0 || Flag_Cross==0 || Barrier_state==0 || L_slope_cross_state==0 || R_slope_cross_state==0) //在其他元素不进行判断
               {
                  Break_road_state=1;
                  Beep_set(1);
               }
           }
       }
    }

   if( Break_road_state == 1 && vert_gray_point == 1 )    //出现底部差比和点
   {
       Break_road_state = 2;
   }

   if(Break_road_state==2 && vert_gray_point==0)    //底部差比和点消失
   {
       Break_road_state=3;
   }

   if(Break_road_state==3)
    {
        for(uint8 j=start_line_point_row;j>=56;j--)    //底部正方形白色检测
        {
            for(i=40;i<=54;i++)
            {
                if(image[j][i]==255)
                {
                    count++;
                }
            }
        }
        if(count>38 && ( l_gray_point==1 || r_gray_point==1 ) )  //正方形检测正常，底部边界点恢复
        {
         if(OFF_Element_test==1)
         {
          Door_count=track_num;
         }
         Beep_set(0);
         Break_road_state=0;                                   //退出断路判断
        }
    }
 }
/*---------------------------------------------------------------
 【功    能】利用两点斜率补线函数
 【参    数】begin为上面的点,end为下面的点，line为边线或中线数组,num为补线长度
 【返 回 值】无
 ----------------------------------------------------------------*/
void Add_line_user_twopoint_slope(uint8 type,uint8 line[], uint8 begin_x, uint8 begin_y,uint8 end_x, uint8 end_y,uint8 num)
{
  uint8 i=0,x=0;
  float slope=0;
  int8 N=begin_y-num;
  N=clip(N,20,59);
  slope = begin_x - end_x;
  slope = slope/(begin_y-end_y);
  if(type==1) //左线
  {
   slope =LIMIT(slope,L_straight_slope/2.0,-2);
  }
  else if(type==2)  //右线
  {
   slope =LIMIT(slope,2,R_straight_slope/2.0);
  }
  for(i=begin_y;i>=N;i--){
        x=slope*(i-end_y)+end_x;
        line[i]=LIMIT(x,93,0);
    }
}
/*---------------------------------------------------------------
 【功    能】两点斜率补线函数
 【参    数】x1,y1为上面的点,x2,y2为下面的点，line为边线或中线数组
 【返 回 值】无
 ----------------------------------------------------------------*/
void Add_line(uint8 type,uint8 line[], uint8 x1, uint8 y1,uint8 x2, uint8 y2)
{
  uint8 i=0,x=0;
  float slope=0;
  slope = x1 - x2;
  slope = slope/(y1-y2);
  if(type==1) //左线
  {
   slope =LIMIT(slope,L_straight_slope/2,-1.4);
  }
  else if(type==2)  //右线
  {
   slope =LIMIT(slope,1.4,R_straight_slope/2);
  }
  for(i=y2;i>y1;i--){
        x=slope*(i-y2)+x2;
        line[i]=LIMIT(x,93,0);
    }
}
/*---------------------------------------------------------------
 【功    能】最小二乘法补线函数
 【参    数】begin和end为最小二乘法计算斜率的长度，begin_y和end_y为需要补线的长度，line为边线或中线数组
 【返 回 值】无
 ----------------------------------------------------------------*/
void Add_line_other(uint8 type,uint8 line[], uint8 begin, uint8 end,uint8 begin_y,uint8 end_y)
{
  uint8 i=0,temp=0;
  float slope=0;
  slope =Slope_calculate(begin,end,line);
  if(type==1) //左线
  {
   slope =LIMIT(slope,L_straight_slope,-1.2);
  }
  else if(type==2)  //右线
  {
   slope =LIMIT(slope,1.2,R_straight_slope);
  }

    for(i=begin_y;i<end_y;i++){
        temp = (char)((i - begin) * slope + line[begin]);//通过斜率推算补线的位置
        line[i]=LIMIT(temp,93,0);
    }
}
/*---------------------------------------------------------------
 【功    能】最小二乘法补线函数
 【参    数】begin和end为最小二乘法计算斜率的长度，begin_y和end_y为需要补线的长度，line为边线或中线数组
 【返 回 值】无
 ----------------------------------------------------------------*/
void Add_line_other_plan(uint8 type,uint8 line[],uint8 out_line[], uint8 begin, uint8 end,uint8 begin_y,uint8 end_y)
{
  uint8 i=0,temp=0;
  float slope=0;
  slope =Slope_calculate(begin,end,line);
  if(type==1) //左线
  {
   slope =LIMIT(slope,L_straight_slope/2.0,-2);
  }
  else if(type==2)  //右线
  {
   slope =LIMIT(slope,2,R_straight_slope/2.0);
  }
  for(i=begin_y;i<end_y;i++){
  temp = (char)((i - begin) * slope + line[begin]);//通过斜率推算补线的位置
  out_line[i]=LIMIT(temp,93,0);
  }
}
/*---------------------------------------------------------------
 【功    能】最小二乘法补线函数
 【参    数】end为补线最后点，begin_y和end_y为需要补线的长度，line为边线或中线数组，slope为补线斜率
 【返 回 值】无
 ----------------------------------------------------------------*/
void Add_line_constant(uint8 type,uint8 line[],uint8 begin_y,uint8 end_y,uint8 end_x,uint8 end_Y,float slope)
{
  uint8 i=0,temp=0;
  float temp_slope=slope;
  if(type==1) //左线
  {
      temp_slope =LIMIT(temp_slope,L_straight_slope/2.0,-2);
  }
  else if(type==2)  //右线
  {
      temp_slope =LIMIT(temp_slope,2,R_straight_slope/2.0);
  }
  for(i=begin_y;i<=end_y;i++){
       temp = (char)((i - end_Y) * temp_slope + end_x);//通过斜率推算补线的位置
        line[i]=LIMIT(temp,93,0);
  }
}
/*---------------------------------------------------------------
 【功    能】用中线数组结合电感误差求车的转向偏差(取图形底下50行)
 【参    数】电感水平误差,误差权值
 【返 回 值】转向偏差
 ----------------------------------------------------------------*/
float turn_error(float inductor_error)
{
  int8 i = 0;
  float error = 0;
  float Line_Ratio_sum=0;      //比例系数和
  uint8 Hightest=20;
  uint8 end=start_line_point_row;
  uint8 mid_line=46;          //默认中线起点
  float Mid_slope=0;
  static int16 Line_Ratio[60]={
                              1,1,1,1,1,1,1,1,1,1,
                              1,1,1,1,1,1,1,1,1,1,
                              4,4,4,4,4,5,5,5,5,5,
                              13,13,13,13,13,15,17,17,17,17,
                              17,17,17,19,19,16,16,16,16,10,
                              10,10,10,10,10,10,10,10,10,10
                            };
  static int16 Ring_Ratio[60]={                         //圆环权值，靠近赛道底边
                              1,1,1,1,1,1,1,1,1,1,
                              1,1,1,1,1,1,1,1,1,1,
                              1,1,1,1,1,1,1,1,1,1,
                              12,12,12,12,12,10,10,10,10,10,
                              13,13,13,13,13,14,14,14,14,14,
                              10,10,10,10,10,10,10,10,10,10
                            };

  if(REPIRE_LINE)     //如果补线了，或者平移边界了，重新确认中线误差计算起点
  {
      Mid_slope=Slope_calculate(45, start_line_point_row, Mid_Line);
      if(Mid_slope>0.15)   //中线左偏
      {
          if(Mid_Line[start_line_point_row]>47)
          {
              mid_line= Mid_Line[start_line_point_row];
          }
      }
      if(Mid_slope<-0.15)//中线右偏
      {
          if(Mid_Line[start_line_point_row]<47)
          {
              mid_line= Mid_Line[start_line_point_row];
          }
      }
  }

  if(L_CirCle_State==6)   //现在是对左圆环特殊处理
  {
      for(i = end; i>=Hightest; i--)
      {
          Line_Ratio_sum+=Ring_Ratio[i];
          error += (Mid_Line[i]  - mid_line)*Ring_Ratio[i];
      }
      Mid_slope=Slope_calculate(35, start_line_point_row, Right_Line);       //圆环内进行误差补偿
  }

  else if(R_CirCle_State==6)    //现在是对右圆环特殊处理
  {
      for(i = end; i>=Hightest; i--)
        {
          Line_Ratio_sum+=Ring_Ratio[i];
          error += (Mid_Line[i]  - mid_line)*Ring_Ratio[i];
        }
      Mid_slope=Slope_calculate(35, start_line_point_row, Left_Line);       //圆环内进行误差补偿
  }
  else                                                    //正常赛道
  {
      for(i = end; i>=Hightest; i--)
        {
          Line_Ratio_sum+=Line_Ratio[i];
          error += (Mid_Line[i] - mid_line)*Line_Ratio[i];
        }
   }

  if(Break_road_state>=1) //断路切换电感循迹
  {
   error=(-inductor_error);
  }
  else
  {
       if(L_CirCle_State==6 || R_CirCle_State==6)
       error =error/Line_Ratio_sum+Mid_slope*(-4.5);        //圆环斜率补偿
       else
       error =error/Line_Ratio_sum+Mid_slope*(-5);        //斜率补偿(补线斜率补偿)
  }
  return error;
}
/*---------------------------------------------------------------
 【功    能】寻找拐点
 【参    数】uint8 Line[]边线数组    flag :  1 左线   2  右线    UD:  1： 下往上扫   0 ：上往下扫
 【返 回 值】 i行坐标 0无
 ----------------------------------------------------------------*/
uint8 Find_guaidian(uint8 begin ,uint8 end,uint8 Line[],uint8 flag,uint8 UD)
{
  if(begin<4) begin=4;
  if(end>57 ) end=57;
  uint8 hang=0;
  if(UD)
  {
    switch(flag)
    {
        case 1:
            for(uint8 i = end;i>=begin;i--)
            {
                       if(Line[i]>=Line[i+2] && Line[i]>=Line[i+1] && Line[i]-Line[i-1]>=1 && Line[i]-Line[i-2]>=1)//找到拐点
                           {
                                                                 hang= i;     //返回该拐点的行
                                                                 break;
                           }
                  }
            break;
        case 2:
            for(uint8 i = end;i>=begin;i--)
            {
                    if(Line[i-1]-Line[i]>=1 && Line[i+1]>=Line[i] && Line[i-2]-Line[i]>=1 && Line[i+2]>=Line[i])//找到右边线有拐点
                         {
                                                         hang= i;     //返回该拐点的行
                                                         break;
                         }
           }
            break;
    }
    }
  else
  {
      switch(flag)
          {
              case 1:
                  for(uint8 i = begin;i<=end;i++)
                  {
                           if(abs(Line[i]-Line[i+1])>6 && abs(Line[i-1]-Line[i])<=2 &&  abs(Line[i-2]-Line[i])<=2 && abs(Line[i]-Line[i+2])>6)//找到拐点
                           {
                                                               hang= i;     //返回该拐点的行
                                                                 break;
                           }
                  }
                  break;
              case 2:
                  for(uint8 i = begin;i<=end;i++)
                  {
                           if(abs(Line[i]-Line[i+1])>6 && abs(Line[i-1]-Line[i])<=2 &&  abs(Line[i-2]-Line[i])<=2 && abs(Line[i]-Line[i+2])>6 )//找到右边线有拐点
                           {
                                                       hang= i;     //返回该拐点的行
                                                         break;
                           }
                  }
                  break;
          }

        }
       if(hang){
             return hang;
         }else{
             return 0;
         }
}
/*---------------------------------------------------------------
 【功    能】寻找多行空白,从下往上扫描,默认7行停止扫描
 【参    数】type寻找类型 1为左边，2为全行,3为右边
 【返 回 值】 1有空白 0无
 ----------------------------------------------------------------*/
uint8 Fine_baihang(uint8 type,uint8 begin ,uint8 end,uint8(*image)[image_w],uint8 Hang)
{
    uint8 bai=0;
    uint8 hang=0;
    uint8 check=0;
    uint8 i,j;
    if(type==2){
    for(i=end;i>=begin;i--)
    {
        for(j=1;j<=91;j+=2)
        {
            if(image[i][j]==255)
            {
                bai+=1;
            }
        }
        if(bai>=42)
        {
            hang+=1;
        }
        bai=0;
        if(hang>=Hang)
        {
            check=1;
            break;
        }
    }
}
    if(type==1){         //左
    for( i=end;i>=begin;i--)
    {
        for( j=0;j<=47;j+=2)
        {
            if(image[i][j]==255)
            {
                bai++;
            }
        }
        if(bai>=19)
        {
            hang++;
        }
        bai=0;
        if(hang>=Hang)
        {
            check= 1;
            break;
        }
    }
}
    if(type==3){         //右
    for( i=end;i>=begin;i--)
    {
        for( j=47;j<=91;j+=2)
        {
            if(image[i][j]==255)
            {
                bai++;
            }
        }
        if(bai>=19)
        {
            hang++;
        }
        bai=0;
        if(hang>=Hang)
        {
            check= 1;
            break;
        }
    }
}
    return check;
}
/*---------------------------------------------------------------
 【功    能】寻找多行突变白行,大于赛道宽度计数，从下往上扫描
 【参    数】type寻找类型 1为左边，2为全行,3为右边
 【返 回 值】 1有白行 0无
 ----------------------------------------------------------------*/
uint8 Fine_width_baihang(uint8 type,uint8 begin ,uint8 end,uint8(*image)[image_w],uint8 Hang)
{
    uint8 bai=0;
    uint8 hang=0;
    uint8 check=0;
    uint8 i,j;
    if(type==2){
    for(i=end;i>=begin;i--)
    {
        for(j=1;j<=91;j+=2)
        {
            if(image[i][j]==255)
            {
                bai+=1;
            }
        }
        if((bai-Width_straight[i]/2)>10)
        {
            hang+=1;
        }
        bai=0;
        if(hang>=Hang)
        {
            check=1;
            break;
        }
    }
}
    if(type==1){         //左
    for( i=end;i>=begin;i--)
    {
        for( j=0;j<=47;j+=2)
        {
            if(image[i][j]==255)
            {
                bai++;
            }
        }
        if((bai-Width_straight[i]/4)>5)
        {
            hang++;
        }
        bai=0;
        if(hang>=Hang)
        {
            check= 1;
            break;
        }
    }
}
    if(type==3){         //右
    for( i=end;i>=begin;i--)
    {
        for( j=47;j<=91;j+=2)
        {
            if(image[i][j]==255)
            {
                bai++;
            }
        }
        if((bai-Width_straight[i]/4)>5)
        {
            hang++;
        }
        bai=0;
        if(hang>=Hang)
        {
            check= 1;
            break;
        }
    }
}
    return check;
}
/*---------------------------------------------------------------
 【功    能】寻找多行黑色,从下往上扫描,默认4行停止扫描
 【参    数】type寻找类型 1为左边，2为全行,3为右边
 【返 回 值】 1有黑行 0无
 ----------------------------------------------------------------*/
uint8 Fine_heihang(uint8 type,uint8 begin ,uint8 end,uint8(*image)[image_w],uint8 hang)
{
    uint8 hei=0;
    uint8 Hang=0;
    uint8 check=0;
    if(type==2){                      //全行
    for(uint8 i=end;i>=begin;i--)
    {
        for(uint8 j=1;j<=91;j+=2)
        {
            if(image[i][j]==0)
            {
                hei++;
            }
        }
        if(hei>=43)
        {
            Hang++;
        }
        hei=0;
        if(Hang>hang)
        {
            check= 1;
            break;
        }
    }
}
    if(type==1){         //左
    for(uint8 i=end;i>=begin;i--)
    {
        for(uint8 j=1;j<47;j+=2)
        {
            if(image[i][j]==0)
            {
                hei++;
            }
        }
        if(hei>=19)
        {
            Hang++;
        }
        hei=0;
        if(Hang>hang)
        {
            check= 1;
            break;
        }
    }
}
    if(type==3){         //右
    for(uint8 i=end;i>=begin;i--)
    {
        for(uint8 j=47;j<=91;j+=2)
        {
            if(image[i][j]==0)
            {
                hei++;
            }
        }
        if(hei>=19)
        {
            Hang++;
        }
        hei=0;
        if(Hang>hang)
        {
            check= 1;
            break;
        }
       }
     }
    return check;
}


