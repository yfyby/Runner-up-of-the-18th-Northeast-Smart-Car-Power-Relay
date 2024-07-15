/*
 * image.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "zf_common_headfile.h"
#include "math.h"
//����Ҫ�޸ı���
uint8 Left_Line[60]={0}, Mid_Line[60]={0}, Right_Line[60]={0}; //�����ұ߽���,Ĭ����ѹ�����ͼ��
uint8 Left_Line_flag[60]={0}, Right_Line_flag[60]={0};       //�ҵ��߽��߱�־λ��1Ϊ�ҵ���0Ϊû���ҵ�
uint8 Width_track[60]={0};                                   //ÿ�������Ŀ��
uint8 Loss_L_line=0,Loss_R_line=0;                  //���Ҷ�������
uint8 No_Loss_L_line=0,No_Loss_R_line=0;             //���Ҳ�������
uint8 REPIRE_LINE=0;                   //���߱�־λ
uint8 corss_deal_mid=0;                //ʮ���������⴦��
bool average_steer_duty=0;              //���Խ���ƽ����Ǳ�־λ
uint8 Cross_Among=0;            //��ʮ��֮���־λ
bool l_gray_point=0,r_gray_point=0;   //���һҶ��ж��Ƿ�Ϊ���߽�,�Ѿ��ڶ�ֵ������ʱ�������ж�
bool vert_gray_point=0;  //��ֱ��Ⱥͱ�־λ��ר�����ж϶�·
uint8 Width_straight[60]={2,4,5,6,7,8,8,10,10,11,      //ֱ�����,dubug�鿴��������λ��(32cm�߶�)
                       11,12,12,12,12,13,13,15,15,18,
                       20,20,21,23,24,24,25,26,27,29,
                       30,31,32,33,35,35,37,38,39,41,
                       41,43,43,45,46,47,49,49,51,51,
                       53,53,55,57,57,59,59,61,62,65};
uint8 Out_door_type=1;        //2Ϊ���ҿ�,1Ϊ�����
float Turn_dt=0.02;                                                        //���ٶ��ۼ�ʱ��
float Car_speed_dt=0.005;                                                     //�����ۼ�ʱ��
//59��30Ϊ45cm,��20Ϊ80cm,��10Ϊ165cm
float L_straight_slope=-0.56;         //��ֱ��б�ʣ���Խƫ�����ֵԽС
float R_straight_slope=0.56;         //��ֱ��б�ʣ���Խƫ�����ֵԽ��
bool L_straightaway=0;                //��ֱ����־λ
bool R_straightaway=0;                //��ֱ����־λ
bool Barrier_brake=0;                  //�ϰ���ɲ����־λ
//����ΪԪ�������ұ߽�ǵ����
uint8 angle_thres=45;        //�ǵ���ֵ
uint8 bottom_gray_threa=30;     //�ײ��������±߽��жϲ�Ⱥ���ֵ
uint8 Break_road_add_thres=15;   //��·��ǿ��ֵ
#define element_line_num  35                                     //���������С
uint8 element_rpts_l[(uint16)element_line_num][2];              //��߽�����x,y
uint8 element_rpts_r[(uint16)element_line_num][2];             //�ұ߽�����x,y
float element_a_l[(uint16)element_line_num];                  //��߽�Ƕȱ仯��
float element_a_r[(uint16)element_line_num];                 //�ұ߽�Ƕȱ仯��
float element_outangle_l[(uint16)element_line_num];         //������ɽǶȱ߽�
float element_outangle_r[(uint16)element_line_num];        //������ɽǶȱ߽�
int   element_rpts_num_l=(uint16)element_line_num;        //���鳤��
int   element_rpts_num_r=(uint16)element_line_num;       //���鳤��
uint8 element_l_angle_point=0,element_r_angle_point=0;  //���Ͻǵ㣬���Ͻǵ������±�
bool left_angle_point=0;                             //��ǵ��ҵ���־
bool right_angle_point=0;                            //�ҽǵ��ҵ���־
uint8 l_angle_point_x,l_angle_point_y,r_angle_point_x,r_angle_point_y;    //��¼�ǵ�ԭʼ����

uint8 l_upangle_point_x,l_upangle_point_y,l_downangle_point_x,l_downangle_point_y;   //��¼�ǵ���ԭʼ����
uint8 r_upangle_point_x,r_upangle_point_y,r_downangle_point_x,r_downangle_point_y;    //��¼�ǵ���ԭʼ����

float practical_l_angle_point_x,practical_l_angle_point_y,practical_r_angle_point_x,practical_r_angle_point_y;    //��¼͸�ӽǵ�����

float practical_l_upangle_point_x,practical_l_upangle_point_y,practical_l_downangle_point_x,practical_l_downangle_point_y;   //��¼͸���Ͻǵ�����
float practical_r_upangle_point_x,practical_r_upangle_point_y,practical_r_downangle_point_x,practical_r_downangle_point_y; //��¼͸���½ǵ�����

float l_original_angle,r_original_angle;       //�ǵ�ԭʼ�Ƕ�
float l_practical_angle,r_practical_angle;   //�ǵ�͸�ӱ任�Ƕ�

bool l_ring_check_point=0;              //Բ����ǵ����
bool r_ring_check_point=0;              //Բ���ҽǵ����
uint8 l_ring_check_point_x;
uint8 l_ring_check_point_y;
uint8 r_ring_check_point_x;
uint8 r_ring_check_point_y;

bool l_cross_check_point=0;              //ʮ����ǵ����
bool r_cross_check_point=0;              //ʮ���ҽǵ����
uint8 l_cross_check_point_x=0;
uint8 l_cross_check_point_y=0;
uint8 r_cross_check_point_x=0;
uint8 r_cross_check_point_y=0;

bool l_barrier_check_point=0;              //�ϰ�����ǵ����
bool r_barrier_check_point=0;              //�ϰ����ҽǵ����
uint8 l_barrier_check_point_x=0;
uint8 l_barrier_check_point_y=0;
uint8 r_barrier_check_point_x=0;
uint8 r_barrier_check_point_y=0;

bool l_break_road_check_point=0;              //��·��ǵ����
bool r_break_road_check_point=0;              //��·�ҽǵ����
uint8 l_break_road_check_point_x=0;
uint8 l_break_road_check_point_y=0;
uint8 r_break_road_check_point_x=0;
uint8 r_break_road_check_point_y=0;

bool l_garage_check_point=0;              //������ǵ����
bool r_garage_check_point=0;              //�����ҽǵ����
uint8 l_garage_check_point_x=0;
uint8 l_garage_check_point_y=0;
uint8 r_garage_check_point_x=0;
uint8 r_garage_check_point_y=0;
//�ǵ㷽���ж�
bool l_grow_inside=false,r_grow_inside=false,l_grow_outside=false,r_grow_outside=false;
//͸�ӽǵ��־λ
bool l_small_angle_point=0;  //���С�ǵ�,40�ȵ�80��
bool l_big_angle_point=0;  //��ߴ�ǵ㣬80�ȵ�140��

bool r_small_angle_point=0;  //�ұ�С�ǵ�,40�ȵ�80��
bool r_big_angle_point=0;  //�ұߴ�ǵ㣬80�ȵ�140��

uint8 l_small_angle_point_id=0; //�±�
uint8 l_big_angle_point_id=0;
uint8 r_small_angle_point_id=0;
uint8 r_big_angle_point_id=0;
/*---------------------------------------------------------------
 ����    �ܡ�͸�ӽǵ����
 ����    ������
 ���� �� ֵ����
 ���� �� ˵ ����������ο����Ͻ����������ǵĲ���������2cm,��������Сһ��ͼ��֮��
 ���� �� ˵ ����������ʹ�õ���һ��ͼ�����㣬û���⣬���ǽǵ�λ�ò��ԣ�������ԭͼ������
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
    int angle_dist=7;    //����Ƕȵļ��
    int nms_angle_dist=7*2+1;    //�ǶȷǼ���ֵ���ƾ���
    if(l_gray_point==1)
    {
      findline_lefthand_binaryzation_angle_user(image, labyrinth_start_point_l[0], labyrinth_start_point_l[1], element_rpts_l,&element_rpts_num_l);  //����ɨ��߽�
    }
    else
    {
      element_rpts_num_l=0;
    }

    if(r_gray_point==1)
    {
      findline_righthand_binaryzation_angle_user(image, labyrinth_start_point_r[0], labyrinth_start_point_r[1], element_rpts_r,&element_rpts_num_r);   //����ɨ��
    }
    else
    {
      element_rpts_num_r=0;
    }

    //͸������ת��
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
    // ���ߵȾ����
    resample_points(rpts_l, element_rpts_num_l, rpts_l_dist, &rpts_num_l, 0.02 * pixel_per_meter);
    resample_points(rpts_r, element_rpts_num_r, rpts_r_dist, &rpts_num_r, 0.02 * pixel_per_meter);
    // ���߾ֲ��Ƕȱ仯��
    local_angle_points(rpts_l_dist, element_rpts_num_l, element_a_l, angle_dist);
    local_angle_points(rpts_r_dist, element_rpts_num_r, element_a_r, angle_dist);
    //�Ƕȱ仯�ʷǼ�������
    nms_angle(element_a_l, element_rpts_num_l, element_outangle_l, nms_angle_dist);
    nms_angle(element_a_r, element_rpts_num_r, element_outangle_r, nms_angle_dist);
    //ʶ���С�յ�
    l_small_angle_point=0;
    l_big_angle_point=0;
    r_small_angle_point=0;
    r_big_angle_point=0;
    for (int i = 0; i < element_rpts_num_l; i++) {            //��ǵ�Ѱ��
        if (element_outangle_l[i] == 0) continue;
        int im1 = clip(i -angle_dist, 0, element_rpts_num_l - 1);
        int ip1 = clip(i + angle_dist, 0, element_rpts_num_l - 1);
        float conf = fabs(element_a_l[i]) - (fabs(element_a_l[im1]) + fabs(element_a_l[ip1])) / 2;
        //��С�ǵ���ֵ
        if (l_small_angle_point == false && 30.0*0.0174 < conf && conf < 60.0*0.0174) {
            l_small_angle_point_id = i;
            l_small_angle_point = true;

        }
        //���ǵ���ֵ
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
        //��С�ǵ�
        if (r_small_angle_point == false && 40.0 *0.0174 < conf && conf < 80.0 *0.0174) {

            r_small_angle_point_id = i;
            r_small_angle_point = true;
        }
        //�Ҵ�ǵ�
        if (r_big_angle_point == false && 80.0 *0.0174 < conf && conf < 140.0 *0.0174) {
            r_big_angle_point_id = i;
            r_big_angle_point = true;
            r_angle_point_x=element_rpts_r[i][0]; //x
            r_angle_point_y=element_rpts_r[i][1];  //y
            r_practical_angle=conf/0.0174;
        }
    }
//��ʾ�ǵ�
//    if(l_big_angle_point==true)
//     draw_point_red(l_angle_point_x, l_angle_point_y);
//    if(r_big_angle_point==true)
//     draw_point_blue(r_angle_point_x, r_angle_point_y);
}
/*---------------------------------------------------------------
 ����    �ܡ�Ѱ�ҽǵ�(��������͸�Ӽ���)
 ����    ����ͼ��
 ���� �� ֵ����
 ����ϸ˵���������޸ĺ��Ѳ�߹̶�����������ȡ�߽�,Ѱ�ҽǵ㣬������͸�ӽǵ㣬����Ԫ��
 ----------------------------------------------------------------*/
void Fine_element_angle_point(uint8(*image)[image_w])
{
      uint8 numcut1=0;
      uint8 i=0;
      uint8 limit_up=0,limit_down=0;  //�����޷�
      uint8 angle_dist=7;             //�������
      uint8 nms_angle_dist=7*2+1;       //�Ǽ���ֵ���ƾ���
      uint8 start_l_x=Left_Line[55];
      uint8 start_l_y=55;
      uint8 start_r_x=Right_Line[55];
      uint8 start_r_y=55;

      left_angle_point=0;           //�Ƕ����
      right_angle_point=0;          //�Ƕ����
      l_ring_check_point=0;         //Բ����ǵ�
      r_ring_check_point=0;        //Բ���ҽǵ�
      l_cross_check_point=0;      //ʮ����ǵ�
      r_cross_check_point=0;     //ʮ���ҽǵ�
      l_garage_check_point=0;   //������ǵ�
      r_garage_check_point=0;  //�����ҽǵ�
      l_break_road_check_point=0;  //��·��ǵ�
      r_break_road_check_point=0; //��·�ҽǵ�
      l_barrier_check_point=0;   //�ϰ�����ǵ�
      r_barrier_check_point=0;  //�ϰ����ҽǵ�
      element_rpts_num_l=(uint16)element_line_num;
      element_rpts_num_r=(uint16)element_line_num;
      for(i=0;i<(uint16)element_line_num;i++)  //�Ƕ����
      {
          element_a_l[i] = 0;
          element_a_r[i] = 0;
          element_outangle_l[i]=0;
          element_outangle_r[i]=0;
      }
      if(l_gray_point) //������
      {
         findline_lefthand_binaryzation_angle_user(image, start_l_x,start_l_y, element_rpts_l,&element_rpts_num_l);  //����ɨ��߽�
      }
      else                        //��㲻����
      {
          element_rpts_num_l=0;
      }
      if(r_gray_point)
      {
          findline_righthand_binaryzation_angle_user(image,start_r_x, start_r_y, element_rpts_r,&element_rpts_num_r);   //����ɨ��
      }
      else
      {
          element_rpts_num_r=0;
      }
      numcut1=element_rpts_num_l-1;
      for(i=0;i<element_rpts_num_l;i++)
      {
          if (i <= 0 || i >= numcut1)   //�߽��Ƕ�Ĭ��0
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
          if (i <= 0 || i >= numcut1)   //�߽��Ƕ�Ĭ��0
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

          nms_angle(element_a_l, element_rpts_num_l, element_outangle_l, nms_angle_dist);   //�Ƕȼ���ֵ����
          nms_angle(element_a_r, element_rpts_num_r, element_outangle_r,nms_angle_dist);

          for(i=0;i<element_rpts_num_l;i++)         //��ȡ��߽�ǵ�
          {
              if(element_outangle_l[i]==0)
                  continue;
              //��·
             if(element_outangle_l[i]>30  && l_break_road_check_point==0)
             {
                 l_break_road_check_point=1;
                 l_break_road_check_point_x=element_rpts_l[i][0];
                 l_break_road_check_point_y=i;
             }
              if(element_outangle_l[i]>angle_thres)
              {
                  //Բ���ǵ�
                  if(element_outangle_l[i]<140 && element_outangle_l[i]>90&& l_ring_check_point==0)   //Բ����ǵ�
                  {
                      l_ring_check_point=1;
                      l_ring_check_point_x=element_rpts_l[i][0];
                      l_ring_check_point_y=element_rpts_l[i][1];
                  }
                  //ʮ�ֻ��߳���ǵ�
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
                 //�ϰ���
                 if(element_outangle_l[i]>angle_thres && l_barrier_check_point==0)
                 {
                     l_barrier_check_point=1;
                     l_barrier_check_point_x=element_rpts_l[i][0];
                     l_barrier_check_point_y=element_rpts_l[i][1];
                 }

                 //�����ж�
                 if(left_angle_point==0)
                 {
                 l_angle_point_x= element_rpts_l[i][0];
                 l_angle_point_y= element_rpts_l[i][1];
                 l_original_angle=element_outangle_l[i];    //�ǵ�Ƕ�
                 left_angle_point=true;
                 element_l_angle_point=i;
                 }
              }
          }

          for(i=0;i<element_rpts_num_r;i++)          //��ȡ�ǵ�
          {
              if(element_outangle_r[i]==0)
                  continue;
              //��·
             if(element_outangle_r[i]>30  && r_break_road_check_point==0)
             {
                 r_break_road_check_point=1;
                 r_break_road_check_point_x=element_rpts_r[i][0];
                 r_break_road_check_point_y=i;
             }
              if(element_outangle_r[i]>angle_thres)
              {
                  //Բ���ǵ�
                  if(element_outangle_r[i]<140 && element_outangle_r[i]>90 && r_ring_check_point==0)   //Բ���ҽǵ�
                  {
                      r_ring_check_point=1;
                      r_ring_check_point_x=element_rpts_r[i][0];
                      r_ring_check_point_y=element_rpts_r[i][1];
                  }
                  //ʮ�ֻ��߳���ǵ�
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
                 //�ϰ���
                 if(element_outangle_r[i]>angle_thres && r_barrier_check_point==0)
                 {
                     r_barrier_check_point=1;
                     r_barrier_check_point_x=element_rpts_r[i][0];
                     r_barrier_check_point_y=element_rpts_r[i][1];
                 }
                 //�����ж�
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
  // ��ʾ�ǵ�
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
 ����    �ܡ������ֵ���ǵ�͸�ӽǶ�
 ����    ������
 ���� �� ֵ����
 ----------------------------------------------------------------*/
void Calculate_perspective_angle()
{
    uint8 angle_dist=5;             //��������
    uint8 limit_up,limit_down;     //�޷�
    l_practical_angle=0;
    r_practical_angle=0;
    if(left_angle_point==true)    //��ǵ����
    {
       //��ԭͼ�ǵ��������͸��ת��
        practical_l_angle_point_x = mapx[element_rpts_l[element_l_angle_point][1]*2][element_rpts_l[element_l_angle_point][0]*2];
        practical_l_angle_point_y = mapy[element_rpts_l[element_l_angle_point][1]*2][element_rpts_l[element_l_angle_point][0]*2];
        //Զ��������
        limit_up=clip(element_l_angle_point+angle_dist,0,element_rpts_num_l-1);
        practical_l_upangle_point_x=mapx[element_rpts_l[limit_up][1]*2][element_rpts_l[limit_up][0]*2];
        practical_l_upangle_point_y=mapy[element_rpts_l[limit_up][1]*2][element_rpts_l[limit_up][0]*2];
        //����������
        limit_down=clip(element_l_angle_point-angle_dist,0,element_rpts_num_l-1);
        practical_l_downangle_point_x=mapx[element_rpts_l[limit_down][1]*2][element_rpts_l[limit_down][0]*2];
        practical_l_downangle_point_y=mapy[element_rpts_l[limit_down][1]*2][element_rpts_l[limit_down][0]*2];
        //����͸�ӽǶ�
        l_practical_angle=fabs(Three_angle_points(practical_l_downangle_point_x,practical_l_downangle_point_y,practical_l_angle_point_x,practical_l_angle_point_y,practical_l_upangle_point_x,practical_l_upangle_point_y));
        l_practical_angle/=0.0174;
    }
    if(right_angle_point==true)    //�ҽǵ����
    {
       //��ԭͼ�ǵ��������͸��ת��
        practical_r_angle_point_x = mapx[element_rpts_r[element_r_angle_point][1]*2][element_rpts_r[element_r_angle_point][0]*2];
        practical_r_angle_point_y = mapy[element_rpts_r[element_r_angle_point][1]*2][element_rpts_r[element_r_angle_point][0]*2];
        //Զ��������
        limit_up=clip(element_r_angle_point+5,0,element_rpts_num_r-1);
        practical_r_upangle_point_x=mapx[element_rpts_r[limit_up][1]*2][element_rpts_r[limit_up][0]*2];
        practical_r_upangle_point_y=mapy[element_rpts_r[limit_up][1]*2][element_rpts_r[limit_up][0]*2];
        //����������
        limit_down=clip(element_r_angle_point-5,0,element_rpts_num_r-1);
        practical_r_downangle_point_x=mapx[element_rpts_r[limit_down][1]*2][element_rpts_r[limit_down][0]*2];
        practical_r_downangle_point_y=mapy[element_rpts_r[limit_down][1]*2][element_rpts_r[limit_down][0]*2];
        //����͸�ӽǶ�
        r_practical_angle=fabs(Three_angle_points(practical_r_downangle_point_x,practical_r_downangle_point_y,practical_r_angle_point_x,practical_r_angle_point_y,practical_r_upangle_point_x,practical_r_upangle_point_y));
        r_practical_angle/=0.0174;
    }
}
/*---------------------------------------------------------------
 ����    �ܡ��жϽǵ������ڻ�������
 ����    ������
 ���� �� ֵ����
 ����ϸ˵�������ں�������������ϰ���Ͷ�·�������Ǽ���ֵ���ƣ����µ��Ǿֲ����ֵ
 ����ϸ˵������Ȼ����45��Ĵ��ڣ��Ǿͱش��ڽǵ㣬ʣ�µľ��ǿ��Ƕȴ�С����Ԫ��
 ----------------------------------------------------------------*/
void Find_angle_point_grow_dir()
{
     int8 l_count=0,r_count=0;
     uint8 upper_l,upper_r;
     l_grow_inside=false,r_grow_inside=false,l_grow_outside=false,r_grow_outside=false;
     if(left_angle_point==true)                                           //������ǵ�
     {
         upper_l=clip(element_l_angle_point+15,0,element_rpts_num_l-1);    //�����ж�15����
         for(uint8 i=element_l_angle_point+1;i<upper_l;i++)
         {
            if (element_rpts_l[i][0]<element_rpts_l[element_l_angle_point][0])    //��ǵ���������
            {
                l_count++;
            }
            else if(element_rpts_l[i][0]>element_rpts_l[element_l_angle_point][0])  //��ǵ���������
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
            if(element_rpts_r[i][0]>element_rpts_r[element_r_angle_point][0])//�ҽǵ���������
            {
                r_count++;
            }
            else if(element_rpts_r[i][0]<element_rpts_r[element_r_angle_point][0])//�ҽǵ���������
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
 ����    �ܡ��ǵ���Ժ���
 ����    ����ͼ������
 ----------------------------------------------------------------*/
void Test_angle_point(uint8(*image)[image_w])
{
        Fine_element_angle_point(image);       //�жϽǵ�
        Find_angle_point_grow_dir();          //�жϽǵ���������
        Calculate_perspective_angle();            //����͸�ӽǵ�Ƕ�
        if(left_angle_point==true)       //��ֵ����ǵ�
        {
            draw_point_red(l_angle_point_x, l_angle_point_y);
        }
        if(right_angle_point==true)      //��ֵ���ҽǵ�
        {
           draw_point_blue(r_angle_point_x, r_angle_point_y);
        }
       draw_point_red(labyrinth_start_point_l[0], labyrinth_start_point_l[1]);  //��߽����
       draw_point_blue(labyrinth_start_point_r[0], labyrinth_start_point_r[1]);//�ұ߽����
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
 ����    �ܡ��ϸ�ֱ���жϺ���
 ����    ������������,�жϱ߽�����
 ���� �� ֵ��1Ϊֱ�� 0Ϊ��ֱ��
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
          if(Left_Line_flag[i]==0)         //6���㶪�ߣ�ֱ���ж�Ϊ��ֱ��
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
 ����    �ܡ�ƽ�Ʊ߽���
 ����    ������ʼ��  ������  type:1 ƽ����߽�  2  ƽ���ұ߽�
 ���� �� ֵ����
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
 ����    �ܡ�S���ж�
 ����    ������
 ���� �� ֵ����
 ���޸����ڡ�2023/4/25
 ----------------------------------------------------------------*/
bool s_state[2]={0,0};              //СS,��S
void Check_Small_S()
{
    uint8 i;
    bool  point1_flag=0;           //����>�յ��־λ
    bool  point2_flag=0;          //����<�յ��־λ
    uint8 mid_line_num=0;      //����������
    s_state[0]=0;
    if(Flag_Cross==0 && L_CirCle_State==0 && R_CirCle_State==0 &&
        L_slope_cross_state==0 && R_slope_cross_state==0
       && hightest<15)            //ʮ����Բ��������S�䴦��
    {
    for(i=50;i>=30;i--)
    {
        if(i-8<hightest)   break;
        //>���ж�,Ѱ��ת�۵�
        if(Mid_Line[i]-Mid_Line[i+6]>=3 && Mid_Line[i]-Mid_Line[i+7]>=3 &&  Mid_Line[i]-Mid_Line[i+8]>=3 &&
           Mid_Line[i]-Mid_Line[i-6]>=3 && Mid_Line[i]-Mid_Line[i-7]>=3 &&  Mid_Line[i]-Mid_Line[i-8]>=3 && point1_flag==0)
        {
            point1_flag=1;
        }
        //<���ж�,Ѱ��ת�۵�
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
    if(point1_flag && point2_flag && mid_line_num>25)   //˫�յ����,���߼�������
    {
            s_state[0]=1;        //ȷ����СS
    }
  }
}
/*---------------------------------------------------------------
 ����    �ܡ��ʵ��޲��߽磬ͨ��ƽ�Ʊ߽��ȡ����
 ����    ������
 ���� �� ֵ����
 ���޸����ڡ�2023/4/24
 ----------------------------------------------------------------*/
void Repire()
{
    int8  i=0;
    uint8 l_i=20,r_i=20;
    if(L_CirCle_State==0 || R_CirCle_State==0)  //Բ�������д�������Բ�����������в���
    {

    for(i=55;i>=20;i--)
    {
        if(Left_Line_flag[i]==0)
        {
            l_i=i+1;              //��¼���߿�ʼ��
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

    if(l_i<r_i && L_straightaway==1 && l_gray_point==1 && r_gray_point==1)  //ֻ�����ߵײ������ڣ��ҵ�����ֱ������ƽ��
    {
        Reair_Mid_Line(l_i,r_i,1);
    }
    else if(l_i>r_i && R_straightaway==1 && l_gray_point==1 && r_gray_point==1) //ֻ�����ߵײ������ڣ��ҵ�����ֱ������ƽ��
    {
        Reair_Mid_Line(r_i,l_i,2);
    }

    }
    //�ж��Ƿ���ʮ������
    if(Flag_Cross==1 && Cross_Among==0)  //������һ��ʮ��
    {
        Cross_Among=1;
    }
    if( Flag_Cross==0 && Cross_Among==1) //��ʮ������
    {
        Cross_Among=2;
    }
    if( Flag_Cross==1 && Cross_Among==2)                          //��ʮ��
    {
        Cross_Among=0;
    }
}
/***********************C     D*******************
                       *       *
                    *             */
/******************A               B***************/
#define cross_line_num   40                                      //���������С
uint8 AX=0,AY=0;                                                //ʮ�����¹յ�
uint8 BX=0,BY=0;                                               //ʮ�����¹յ�
uint8 CX=0,CY=0;                                              //ʮ�����Ϲյ�
uint8 DX=0,DY=0;                                             //ʮ�����Ϲյ�
uint8 up_l_angle_point=0,up_r_angle_point=0;        //���Ͻǵ㣬���Ͻǵ������±�
uint8 Flag_Cross=0;                                                        //ʮ��·��״̬λ
/*---------------------------------------------------------------
 ����    �ܡ�����ʮ��(����ʮ�����ʱ��ɫ����ռ35��)
 ����    ������ֵ��ͼ��
 ���� �� ֵ����
 ���޸����ڡ�2023/4/24
 ----------------------------------------------------------------*/
void cross_deal(uint8(*image)[image_w])
{
     uint8 temp_x=0;
     uint8 numcut1 = 0;
     float L_slope=0,R_slope=0;                //���ұ߽�б��
     uint8 i=0,White_row=0,repire=0;
     uint8 l_lock=0,r_lock=0;                //ʮ����
     corss_deal_mid=0;
     element_rpts_num_l=(uint16)element_line_num;        //���鳤��
     element_rpts_num_r=(uint16)element_line_num;       //���鳤��
     uint8 limit_up=0,limit_down=0;  //�����޷�
     uint8 angle_dist=7;                //�������
     uint8 nums_angle_dist=7*2+1;           //�ǶȷǼ������ƾ���
     if(Flag_Cross==0 && L_CirCle_State==0 && R_CirCle_State==0)                                                          //��0
     {
         White_row=Fine_width_baihang(2,25,35,image,4); //�ж��Ƿ������������(����4�а��з���)
         if(White_row)
         {
             if(l_cross_check_point && r_cross_check_point
               && l_grow_outside && r_grow_outside)   //ʮ�ֽǵ��ж�
             {
                 regression(2,25, 55);                 //�������
                 R_slope=parameterB;
                 regression(1, 25, 55);                 //�������
                 L_slope=parameterB;
                 if(L_slope>0.1 && R_slope<(-0.1))       //ʮ�ֵ�����б��������Ե�
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
     if(AY<=48 && BY<=48)     //����յ��Զ��ֱ��б�ʲ���
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
     else if(AY<=48 && BY>48)  //�ı�Զ���ı�
     {
         AX=Left_Line[AY];
         Add_line_user_twopoint_slope(1,Left_Line, AX,AY,Left_Line[59],59,30);
         for(i=48;i>=20;i--)
         {
           Mid_Line[i]=clip(Left_Line[i]+Width_straight[i]/2,0,93);
         }
     }
     else if(AY>48 && BY<=48)//�ı�Զ���ı�
     {
         BX=Right_Line[BY];
         Add_line_user_twopoint_slope(2,Right_Line, BX,BY,Right_Line[59],59,30);
         for(i=48;i>=20;i--)
         {
           Mid_Line[i]=clip(Right_Line[i]-Width_straight[i]/2,0,93);
         }
     }
     else //����Զ����ʼ���㲹��
     {
         Flag_Cross=2;                  //��������ʮ��
     }
   break;
   case 2:
   if(Left_Line_flag[start_line_point_row]==0 && Right_Line_flag[start_line_point_row]==0
      &&Left_Line_flag[start_line_point_row-1]==0 && Right_Line_flag[start_line_point_row-1]==0
      &&Left_Line_flag[start_line_point_row-2]==0 && Right_Line_flag[start_line_point_row-2]==0)
      {
          Flag_Cross=3;                  //����ʮ��
      }
   for(i=50;i>20;i--)        //�ж϶��߻ָ��߶�
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

   if(l_lock<r_lock)    //�ұ߰�
   {
       BX=0;BY=0;                                        //�������
       DX=0;DY=0;
       BY=Find_guaidian(45 ,start_line_point_row,Right_Line,2,1);  //Ѱ�����¹յ�B,����������
       if(BY!=0)
        {
           BX=Right_Line[BY];
        }
       else
        {
           BY=59;
           BX=47+Width_straight[BY]/2;
        }
       for(i=BY-4;i>20;i-=3)                     //�����¹յ�����ɨ��
              {
                 if(image[i][BX]==0 && image[i-1][BX]==0)
                 {
                         DY=i;                              //��¼�����ĵ�һ���Һڵ�λ��
                         break;
                 }
             }
       if(DY!=0)                      //�����ʮ������
          {
               DX=BX;
          }
       else                                   //���û�ҵ�D��Y����,����ֵ
          {
               DY=BY-35;
               DX=BX;
          }
       for(i=DX;i>26;i-=2)                     //�ӵ�һ���Һڵ�λ��  ����ɨ��
           {
                  if(image[DY][i]==255 && image[DY][i-1]==255)
                  {
                          DX=i;
                          break;
                  }
           }
       if(DX==BX)                          //���û�ҵ�������ֵ
       {
           DX=BX-15;
       }

       Add_line(2,Right_Line,DX,DY,BX,BY);            //�Ҳ���
       for(i=59;i>=20;i--)
          Mid_Line[i]=Right_Line[i]-Width_straight[i]/2;
   }

       if(l_lock>r_lock)          //��߰�
       {
           AX=0;AY=0;              //�������
           CX=0;CY=0;
           AY=Find_guaidian(45 ,start_line_point_row,Left_Line,1,1);   //Ѱ�����¹յ�A,����������
           if(AY!=0)                                                //����ҵ�
           {
              AX=Left_Line[AY];                                      //��¼�յ�������
           }else
           {
             AY=59;
             AX=47-Width_straight[AY]/2;                                                 //û�ҵ�����ֵ
           }
           for(i=AY-4;i>20;i-=3)                       //�����¹յ�����ɨ��
              {
                 if(image[i][AX]==0 && image[i-1][AX]==0)
                    {
                         CY=i;                   //��¼�����ĵ�һ����ڵ�λ��
                         break;
                     }
                }
           if(CY!=0)                      //�����ʮ������
              {
                   CX=AX;
              }
           else                                   //���û�ҵ�C��Y����,����ֵ
              {
                   CY=AY-35;
                   CX=AX;
              }
           for(i=CX;i<66;i+=2)                     //�ӵ�һ����ڵ�λ��  ����ɨ��
               {
                if(image[CY][i]==255 && image[CY][i+1]==255)
                  {
                              CX=i;
                              break;
                  }
               }
           if(CX==AX)                          //���û�ҵ�������ֵ
           {
               CX=AX+15;
           }
               Add_line(1,Left_Line,CX,CY,AX,AY);             //����
               for(i=59;i>=20;i--)
                  Mid_Line[i]=Left_Line[i]+Width_straight[i]/2;
       }

       if(l_lock==r_lock)   //����ƫ
       {
       AX=0;AY=0;BX=0;BY=0;                                        //�������
       CX=0;CY=0;DX=0;DY=0;
       AY=Find_guaidian(45 ,start_line_point_row,Left_Line,1,1);   //Ѱ�����¹յ�A,����������
       BY=Find_guaidian(45 ,start_line_point_row,Right_Line,2,1);  //Ѱ�����¹յ�B,����������
       if(AY!=0)                                                //����ҵ�
       {
          AX=Left_Line[AY];                                      //��¼�յ�������
       }else
       {
         AY=59;
         AX=47-Width_straight[AY]/2;                                                 //û�ҵ�����ֵ
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
     for(i=AY-4;i>20;i-=3)                       //�����¹յ�����ɨ��
        {
           if(image[i][AX]==0 && image[i-1][AX]==0)
              {
                   CY=i;                   //��¼�����ĵ�һ����ڵ�λ��
                   break;
               }
          }
     for(i=BY-4;i>20;i-=3)                     //�����¹յ�����ɨ��
            {
               if(image[i][BX]==0 && image[i-1][BX]==0)
               {
                       DY=i;                              //��¼�����ĵ�һ���Һڵ�λ��
                       break;
               }
           }
    if(CY!=0 && DY!=0)                      //�����ʮ������
       {
            CX=AX;
            DX=BX;
       }
    else                                   //���û�ҵ�C,D��Y����,����ֵ
       {
            CY=AY-35;
            DY=BY-35;
            CX=AX;
            DX=BX;
       }
    for(i=CX;i<66;i+=2)                     //�ӵ�һ����ڵ�λ��  ����ɨ��
        {
         if(image[CY][i]==255 && image[CY][i+1]==255)
           {
                       CX=i;
                       repire+=1;
                       break;
           }
        }
    for(i=DX;i>26;i-=2)                     //�ӵ�һ���Һڵ�λ��  ����ɨ��
        {
               if(image[DY][i]==255 && image[DY][i-1]==255)
               {
                       DX=i;
                       repire+=1;
                       break;
               }
        }

    if(CX==AX && DX==BX)                          //���û�ҵ�������ֵ
    {
        CX=AX+15;
        DX=BX-15;
    }
    if(repire==2)                                   //�����߱�־λ
     {
        Add_line(1,Left_Line,CX,CY,AX,AY);             //����
        Add_line(2,Right_Line,DX,DY,BX,BY);            //�Ҳ���
        corss_deal_mid=(CY+DY)/2;
        for(i=59;i>=corss_deal_mid;i--)
           Mid_Line[i]=(Right_Line[i]+Left_Line[i])/2;
     }
   }
    break;
    case 3:
        //�˳�ʮ��״̬�ж�
       if(Left_Line_flag[start_line_point_row-1]==1 && Right_Line_flag[start_line_point_row-1]==1 &&
          Left_Line_flag[start_line_point_row-2]==1 && Right_Line_flag[start_line_point_row-2]==1 &&
          Left_Line_flag[start_line_point_row-3]==1 && Right_Line_flag[start_line_point_row-3]==1&&
          Left_Line_flag[start_line_point_row-5]==1 && Right_Line_flag[start_line_point_row-5]==1 )
          {
            Flag_Cross=0;
            Beep_set(0);
          }

       for(i=50;i>15;i--)        //�ж϶��߻ָ��߶�
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
       element_rpts_num_l=(uint16)element_line_num;        //���鳤��
       element_rpts_num_r=(uint16)element_line_num;       //���鳤��

       if(l_lock<r_lock)    //�ұ߰�
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
           findline_righthand_binaryzation_angle_user(image, DX, DY, element_rpts_r,&element_rpts_num_r);   //����ɨ��
           for(i=0;i<(uint16)element_line_num;i++)  //�Ƕ����
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
           for(i=0;i<element_rpts_num_r;i++)          //��ȡ�ǵ�
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

           for(i=BY;i>15;i--)       //Ѱ�һָ�����в���
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
               Add_line(2,Right_Line,BX,BY,temp_x,59);            //�Ҳ���
           }

              for(i=59;i>20;i--)
              {
                  Mid_Line[i]=(Right_Line[i]-Width_straight[i]/2);
              }
       }

       if(l_lock>r_lock)          //��߰�
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
           findline_lefthand_binaryzation_angle_user(image, CX, CY, element_rpts_l,&element_rpts_num_l);  //����ɨ��߽�
           for(i=0;i<(uint16)element_line_num;i++)  //�Ƕ����
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
           nms_angle(element_a_l, element_rpts_num_l, element_outangle_l, nums_angle_dist);   //�Ƕȼ���ֵ����
           for(i=0;i<element_rpts_num_l;i++)         //��ȡ�ǵ�
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

           for(i=AY;i>15;i--)       //Ѱ�һָ�����в���
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
               Add_line(1,Left_Line,AX,AY,temp_x,59);            //����
           }

           for(i=59;i>=20;i--)
           {
              Mid_Line[i]=(Left_Line[i]+Width_straight[i]/2);
           }
       }

    if(l_lock==r_lock)      //Ҫô���Ҫôû�ҵ�
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
       //����Ϊ�ǵ�Ѱ��
       findline_lefthand_binaryzation_angle_user(image, CX, CY, element_rpts_l,&element_rpts_num_l);  //����ɨ��߽�
       findline_righthand_binaryzation_angle_user(image, DX, DY, element_rpts_r,&element_rpts_num_r);   //����ɨ��
       for(i=0;i<(uint16)element_line_num;i++)  //�Ƕ����
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
       nms_angle(element_a_l, element_rpts_num_l, element_outangle_l,nums_angle_dist);   //�Ƕȼ���ֵ����
       nms_angle(element_a_r, element_rpts_num_r, element_outangle_r,nums_angle_dist);
       for(i=0;i<element_rpts_num_l;i++)         //��ȡ�ǵ�
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

       for(i=0;i<element_rpts_num_r;i++)          //��ȡ�ǵ�
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
           Add_line(1,Left_Line,AX,AY,13,59);             //����
           Add_line(2,Right_Line,BX,BY,80,59);            //�Ҳ���
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
 ����    �ܡ�б��ʮ�ִ���
 ����    ������ֵ��ͼ��
 ���� �� ֵ����
 ----------------------------------------------------------------*/
uint8 slope_cross=0;                   //б���־λ
uint8 L_slope_cross_state=0;          //��б���־λ
uint8 R_slope_cross_state=0;         //��б���־λ
void  slope_cross_deal(uint8(*image)[image_w])
{
    uint8 L_down_y=0,L_down_x=0,R_down_y=0,R_down_x=0; //���߲�������
    uint8 L_down_y0=0,L_down_x0=0,R_down_y0=0,R_down_x0=0; //���߲�������
    uint8 L_white_row=0,R_white_row=0;
    uint8 L_loss_num=0,R_loss_num=0;
    uint8 l_lock=0,r_lock=0;         //�ж϶��߻ָ��߶�,���Խ��е��߲���
    static bool L_slope_cross_lock=0,R_slope_cross_lock=0;      //����б��ʮ����
    bool advance_L=0,advance_R=0;                    //����б��Ԥ�ȴ���
    float slope=0;                                     //����б��
    uint8 temp_x=0;                                   //������ʱ����
    uint8 temp_y=0;                                  //������ʱ����
    uint8 limit_down,limit_up;                      //�����޷�
    uint8 angle_dist=7;                            //��������
    uint8 nums_angle_dist=7*2+1;                  //�ǶȷǼ�������

    if(slope_cross==0  && L_CirCle_State==0 && R_CirCle_State==0)
    {
        for(uint8 i=start_line_point_row;i>=30;i--)//58-30=28//�������Ҷ��������жϣ�б����һ���Ǻܶඪ�ߵ�
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

        //R_loss_num<5�ʺ�Сʮ�֣�����Ǵ�ʮ�֣�ɾ���������,�ǵ����<��
        if(L_loss_num>=24 && r_cross_check_point && r_grow_outside && R_loss_num<=5)//��ߴ�������
        {
             if(advance_R==0)
             advance_R=1;                     //���ұ߽���д���
             if(Left_Line_flag[r_cross_check_point_y]==0 &&
                 Left_Line_flag[r_cross_check_point_y+1]==0 &&
                 Left_Line_flag[r_cross_check_point_y+2]==0 &&
                 Left_Line_flag[r_cross_check_point_y+3]==0)   //�ҽǵ��Ӧ����ߴ�����ʧ
             {
                  R_slope_cross_lock=1;
             }

             if(R_slope_cross_lock==1)
             {
                 for(uint8 n=40;n>=15;n--) //�ж���߽��Ƿ�Ͽ�
                 {
                     if(Left_Line_flag[n]==0 && Left_Line_flag[n+1]==0 &&
                        Left_Line_flag[n-1]==1 && Left_Line_flag[n-2]==1)
                     {
                         slope_cross=1;
                         R_slope_cross_state=1;           //��б��
                         Beep_set(1);
                         break;
                     }
                 }
             }
         }
        //L_loss_num<9�ʺ�Сʮ�֣�����Ǵ�ʮ�֣�ɾ������������ǵ����>��
        else if(L_loss_num<=5 && l_cross_check_point && l_grow_outside && R_loss_num>=24)
        {
            if(advance_L==0)
            advance_L=1;                     //����߽���д���
            if(Right_Line_flag[l_cross_check_point_y]==0 &&
               Right_Line_flag[l_cross_check_point_y+1]==0 &&
               Right_Line_flag[l_cross_check_point_y+2]==0 &&
               Right_Line_flag[l_cross_check_point_y+3]==0 )   //��ǵ��Ӧ����ߴ�����ʧ
            {
                L_slope_cross_lock=1;
            }
            if(L_slope_cross_lock==1)
            {
                for(uint8 n=40;n>=15;n--)    //�ж϶��߶ϵ�
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
        else//���г����ж�,������΢б��
        {
            L_white_row=Fine_baihang(1, 30, 37, image, 4);  //�ж���߰�����
            R_white_row=Fine_baihang(2, 30, 37, image, 4);  //�ж���߰�����
            //��ǵ㣬�ǵ����⣬�ұߴ������У����û�д������У�ȷ������б��ʮ��
            if(l_cross_check_point && l_grow_outside && R_white_row==1 && L_white_row==0)
            {
                if(Right_Line_flag[l_cross_check_point_y]==0 &&
                   Right_Line_flag[l_cross_check_point_y+1]==0 &&
                   Right_Line_flag[l_cross_check_point_y+2]==0 &&   //б��Ļ���һ�߽ǵ�϶��Ǹߵ�
                   Right_Line_flag[l_cross_check_point_y+3]==0 )   //��ǵ��Ӧ�ұ��ߴ�����ʧ
                {
                    L_slope_cross_lock=1;
                }
                if(L_slope_cross_lock==1)
                {
                    for(uint8 n=40;n>=15;n--)          //Ѱ���ұ߽�ϵ�
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
            //�ҽǵ㣬�ǵ����⣬��ߴ������У��ұ�û�д������У�ȷ������б��ʮ��
            else if(r_cross_check_point && r_grow_outside && R_white_row==0 && L_white_row)
            {
                if(Left_Line_flag[r_angle_point_y]==0 &&
                    Left_Line_flag[r_angle_point_y+1]==0 &&
                    Left_Line_flag[r_angle_point_y+2]==0 &&  //б��Ļ���һ�߽ǵ�϶��Ǹߵ�
                    Left_Line_flag[r_angle_point_y+3]==0)   //�ҽǵ��Ӧ����ߴ�����ʧ
                {
                    R_slope_cross_lock=1;
                }
                if(R_slope_cross_lock==1)
                {
                    for(uint8 n=40;n>=15;n--) //�ж��Ƿ�߽�Ͽ�
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


    if(advance_R)    //��ǰ����
    {
    //ֱ��б�ʲ���
    AY=40;
    AX=Right_Line[AY];
    temp_y=clip(AY+15,0,59);
    temp_x=Right_Line[temp_y];
    slope=AX-temp_x;
    slope=slope/(AY-temp_y);
    slope=LIMIT(slope,2,R_straight_slope);//б���޷�����õ�б�ʲ���ͻ��,����б��ʮ����˵����Ҫ���������
    REPIRE_LINE=1;          //����б�ʲ���
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
    if(advance_L)//��ǰ����
    {
    //ֱ��б�ʲ���
    AY=40;
    AX=Left_Line[AY];
    temp_y=clip(AY+15,0,59);
    temp_x=Left_Line[temp_y];
    slope=AX-temp_x;
    slope=slope/(AY-temp_y);
    slope=LIMIT(slope,L_straight_slope,-2);//б���޷�����õ�б�ʲ���ͻ��,����б��ʮ����˵����Ҫ���������
    REPIRE_LINE=1;          //����б�ʲ���
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

    if(L_slope_cross_state!=0)    //��ʼ������б��
    {
    float temp_slope=0;
    switch(L_slope_cross_state)
    {
    case 1 :        //��ʮ��֮ǰ�յ��Զ��ֱ�����ý���б�����ϲ��ߣ�48Ϊ�����޷�
                   //ֱ��б�ʲ���
         temp_slope=Slope_calculate(49, start_line_point_row, Left_Line);
         AY=Find_guaidian(30, 53, Left_Line, 1, 1);
         AX=Left_Line[AY];
         temp_y=clip(AY+12,0,59);
         slope=AX-Left_Line[temp_y];
         slope=slope/(AY-temp_y);
         if(fabs(slope-temp_slope)>0.2)   slope=temp_slope; //б���޷�
         slope=LIMIT(slope,L_straight_slope,-2);//б���޷�����õ�б�ʲ���ͻ��,����б��ʮ����˵����Ҫ���������
         REPIRE_LINE=1;          //����б�ʲ���
         for(uint8 i=AY;i>=20;i--)
         {
             temp_x=slope*(i-AY)+AX;
             Left_Line[i]=clip(temp_x,0,93);
         }
         for(uint8 i=20;i<=AY;i++)
         {
             Mid_Line[i]=clip(Left_Line[i]+Width_straight[i]/2,0,93);
         }
         if((l_angle_point_y>=48 && l_grow_outside) || AY>=48)//��ǵ��ڽ������ǵ���������
         {                                                    //�յ�ǵ�ͬʱ�ж�
               L_slope_cross_state=2;
         }
    break;
    case 2 :         //�����յ㣬��ʱ��������������в���
        //��������߽����յ㲹��
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
        REPIRE_LINE=1;          //����б�ʲ���
        //ʮ�������ж�
        if(Left_Line_flag[start_line_point_row-1]==0 && Right_Line_flag[start_line_point_row-1]==0 &&
        Left_Line_flag[start_line_point_row-2]==0 && Right_Line_flag[start_line_point_row-2]==0 &&
        Left_Line_flag[start_line_point_row-3]==0 && Right_Line_flag[start_line_point_row-3]==0)
        {
            L_slope_cross_state=3;
        }

    break;
    case 3 :     //��ǰ�Ѿ�����ʮ�֣�ֻ���������������ǵ㲹��
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

        for(uint8 i=start_line_point_row;i>15;i--)        //�ж϶��߻ָ��߶�
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

          element_rpts_num_l=(uint16)element_line_num;        //���鳤��
           element_rpts_num_r=(uint16)element_line_num;       //���鳤��

           if(l_lock>r_lock)    //�ұ�Զ
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
               findline_righthand_binaryzation_angle_user(image, DX, DY, element_rpts_r,&element_rpts_num_r);   //����ɨ��
               for(uint8 i=0;i<(uint16)element_line_num;i++)  //�Ƕ����
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
               for(uint8 i=0;i<element_rpts_num_r;i++)          //��ȡ�ǵ�
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
               for(uint8 i=BY;i>15;i--)       //Ѱ�һָ�����в���
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
                   Add_line(2,Right_Line,BX,BY,temp_x,59);            //�Ҳ���
               }

               for(uint8 i=59;i>BY;i--)
               {
                  Mid_Line[i]=(Right_Line[i]-Width_straight[i]/2);
               }
                REPIRE_LINE=1;          //����б�ʲ���
           }

           if(l_lock<r_lock)          //���Զ
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
               findline_lefthand_binaryzation_angle_user(image, CX, CY, element_rpts_l,&element_rpts_num_l);  //����ɨ��߽�
               for(uint8 i=0;i<(uint16)element_line_num;i++)  //�Ƕ����
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
               nms_angle(element_a_l, element_rpts_num_l, element_outangle_l, nums_angle_dist);   //�Ƕȼ���ֵ����
               for(uint8 i=0;i<element_rpts_num_l;i++)         //��ȡ�ǵ�
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
                   Add_line(1,Left_Line,AX,AY,temp_x,59);             //����
               }

               for(uint8 i=59;i>=AY;i--)
               {
                  Mid_Line[i]=(Left_Line[i]+Width_straight[i]/2);
               }
               REPIRE_LINE=1;          //����б�ʲ���
           }

       if(l_lock==r_lock)      //Ҫô���Ҫôû�ҵ�
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
          //����Ϊ�ǵ�Ѱ��
          findline_lefthand_binaryzation_angle_user(image, CX, CY, element_rpts_l,&element_rpts_num_l);  //����ɨ��߽�
          findline_righthand_binaryzation_angle_user(image, DX, DY, element_rpts_r,&element_rpts_num_r);   //����ɨ��
          for(uint8 i=0;i<(uint16)element_line_num;i++)  //�Ƕ����
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
          nms_angle(element_a_l, element_rpts_num_l, element_outangle_l,nums_angle_dist);   //�Ƕȼ���ֵ����
          nms_angle(element_a_r, element_rpts_num_r, element_outangle_r,nums_angle_dist);
          for(uint8 i=0;i<element_rpts_num_l;i++)         //��ȡ�ǵ�
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

          for(uint8 i=0;i<element_rpts_num_r;i++)          //��ȡ�ǵ�
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
              Add_line(1,Left_Line,AX,AY,47-Width_straight[59]/2,59);             //����
              Add_line(2,Right_Line,BX,BY,47+Width_straight[59]/2,59);            //�Ҳ���
              corss_deal_mid=(AY+BY)/2;
              for(uint8 i=59;i>corss_deal_mid;i--)
              {
                 Mid_Line[i]=(Left_Line[i]+Right_Line[i])/2;
              }
              REPIRE_LINE=1;          //����б�ʲ���
         }
    break;
    }
  }

    if(R_slope_cross_state!=0)    //��ʼ������б��
    {
       float temp_slope=0;
       switch(R_slope_cross_state)
       {
       case 1:
          //ֱ��б�ʲ���
          temp_slope=Slope_calculate(49, start_line_point_row, Right_Line); //���һ�µײ�б��
          AY=Find_guaidian(30, 53, Right_Line, 2, 1);
          AX=Right_Line[AY];
          temp_y=clip(AY+12,0,59);
          slope=AX-Right_Line[temp_y];
          slope=slope/(AY-temp_y);
          if(fabs(slope-temp_slope)>0.2)  slope=temp_slope;
          slope=LIMIT(slope,2,R_straight_slope);//б���޷�����õ�б�ʲ���ͻ��,����б��ʮ����˵����Ҫ���������
          REPIRE_LINE=1;          //����б�ʲ���
          for(uint8 i=AY;i>=20;i--)
          {
              temp_x=slope*(i-AY)+AX;
              Right_Line[i]=clip(temp_x,0,93);
          }
          for(uint8 i=20;i<=AY;i++)
          {
              Mid_Line[i]=Right_Line[i]-Width_straight[i]/2;
          }
          if((r_angle_point_y>=48 && r_grow_outside) || AY>=48)//�ҽǵ�����ܸߣ��ǵ���������
          {                                                    //�յ�ǵ�ͬʱ�ж�
                R_slope_cross_state=2;
          }
       break;
       case 2: //���״̬���ʺ�б�ʲ��ߣ���������
               //�������ұ߽����յ㲹��
       R_down_y=Find_guaidian(47,58,Right_Line,2,1);
       if(R_down_y)
       {
           R_down_x=Right_Line[R_down_y];
       }else {
           R_down_y=59;
           R_down_x=47+Width_straight[59]/2;
       }
       //�����Һڵ�
       for(uint8 i=R_down_y-4;i>15;i-=3)
       {
           if(image[i][R_down_x]==0 && image[i-1][R_down_x]==0)
           {
               R_down_y0=i;
               break;
           }
       }
       //�����Ұ׵�
       for(uint8 i=R_down_x;i>20;i--)
       {
           if(image[R_down_y0][i]==255 && image[R_down_y0][i-1]==255)
           {
               R_down_x0=i;
               break;
           }
       }
       //���㲹��
       Add_line(2,Right_Line, R_down_x0, R_down_y0, R_down_x, R_down_y);
       REPIRE_LINE=1;          //����б�ʲ���
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

       for(uint8 i=start_line_point_row;i>15;i--)        //�ж϶��߻ָ��߶�
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

         element_rpts_num_l=(uint16)element_line_num;        //���鳤��
         element_rpts_num_r=(uint16)element_line_num;       //���鳤��

         if(l_lock>r_lock)    //�ұ�Զ
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
             findline_righthand_binaryzation_angle_user(image, DX, DY, element_rpts_r,&element_rpts_num_r);   //����ɨ��
             for(uint8 i=0;i<(uint16)element_line_num;i++)  //�Ƕ����
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
             for(uint8 i=0;i<element_rpts_num_r;i++)          //��ȡ�ǵ�
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
             for(uint8 i=BY;i>15;i--)       //Ѱ�һָ�����в���
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
                 Add_line(2,Right_Line,BX,BY,temp_x,59);            //�Ҳ���
             }

             for(uint8 i=59;i>BY;i--)
             {
                Mid_Line[i]=(Right_Line[i]-Width_straight[i]/2);
             }
              REPIRE_LINE=1;          //����б�ʲ���
         }

         if(l_lock<r_lock)          //���Զ
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
             findline_lefthand_binaryzation_angle_user(image, CX, CY, element_rpts_l,&element_rpts_num_l);  //����ɨ��߽�
             for(uint8 i=0;i<(uint16)element_line_num;i++)  //�Ƕ����
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
             nms_angle(element_a_l, element_rpts_num_l, element_outangle_l, nums_angle_dist);   //�Ƕȼ���ֵ����
             for(uint8 i=0;i<element_rpts_num_l;i++)         //��ȡ�ǵ�
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
                 Add_line(1,Left_Line,AX,AY,temp_x,59);             //����
             }

             for(uint8 i=59;i>=AY;i--)
             {
                Mid_Line[i]=(Left_Line[i]+Width_straight[i]/2);
             }
             REPIRE_LINE=1;          //����б�ʲ���
         }

      if(l_lock==r_lock)      //Ҫô���Ҫôû�ҵ�
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
         //����Ϊ�ǵ�Ѱ��
         findline_lefthand_binaryzation_angle_user(image, CX, CY, element_rpts_l,&element_rpts_num_l);  //����ɨ��߽�
         findline_righthand_binaryzation_angle_user(image, DX, DY, element_rpts_r,&element_rpts_num_r);   //����ɨ��
         for(uint8 i=0;i<(uint16)element_line_num;i++)  //�Ƕ����
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
         nms_angle(element_a_l, element_rpts_num_l, element_outangle_l,nums_angle_dist);   //�Ƕȼ���ֵ����
         nms_angle(element_a_r, element_rpts_num_r, element_outangle_r,nums_angle_dist);
         for(uint8 i=0;i<element_rpts_num_l;i++)         //��ȡ�ǵ�
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

         for(uint8 i=0;i<element_rpts_num_r;i++)          //��ȡ�ǵ�
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

         for(uint8 i=BY;i>15;i--)       //Ѱ�һָ�����в���
         {
             if(Right_Line_flag[i]==1 && Right_Line_flag[i-1]==1)
             {
                 BX= Right_Line[i];
                 BY=i;
                 break;
             }
         }

         for(uint8 i=AY;i>15;i--)       //Ѱ�һָ�����в���
         {
             if(Left_Line_flag[i]==1 && Left_Line_flag[i-1]==1)
             {
                 AX= Left_Line[i];
                 AY=i;
                 break;
             }
         }

             Add_line(1,Left_Line,AX,AY,47-Width_straight[59]/2,59);             //����
             Add_line(2,Right_Line,BX,BY,47+Width_straight[59]/2,59);            //�Ҳ���
             corss_deal_mid=(AY+BY)/2;
             for(uint8 i=59;i>corss_deal_mid;i--)
             {
                Mid_Line[i]=(Left_Line[i]+Right_Line[i])/2;
             }
             REPIRE_LINE=1;          //����б�ʲ���
        }
        break;
       }
   }
}
/*---------------------------------------------------------------
 ����    �ܡ�����Բ������
 ����    ������ֵ��ͼ��
 ���� �� ֵ����
 ----------------------------------------------------------------*/
uint8 Flag_L_CirCle=0;                               //��Բ����־λ
uint8 Flag_R_CirCle=0;                              //��Բ����־λ
uint8 L_CirCle_State=0;                            //��Բ��״̬��־λ
uint8 R_CirCle_State=0;                           //��Բ��״̬��־λ
uint8 In_Poit_x=0,In_Poit_y=0;                   //��Բ���յ�
uint8 Out_Poit_x=0,Out_Poit_y=0;                //�����յ�����
float R_slope=0,L_slope=0;                     //���ұ߽�б��
float Ring_Left_angle=0,Ring_Right_angle=0;   //���ұ߽�Ƕ�
float Ring_add_angle=0;                      //�ۼӵĽǶ�
float slope_to_angle=0;                     //б��ת��Ϊ�Ƕ�
bool can_slopetoangle=0;                    //���Խ���б�ʴ�Ǳ�־λ
int Add_steer_duty=0;                      //�ۼ�ռ�ձ�
int16 Average_duty=0;                      //ƽ��ռ�ձ�
int16 Add_steer_duty_count=0;             //�ۼ�ռ�ձȼ���
//��Բ�������жϺ���
void assist_Right_ring_deal()
{
    static bool can_check=false;
    uint8 i=0;
    uint8 acr_flag=0;
    uint8 dz=0,dj=0;               //�����͵�����־λ
    if((Inductor_Value[0]+Inductor_Value[3])>= 85 && Inductor_Value[2]>=25 && can_check==false && R_CirCle_State<3) //ˮƽ���ֵ�ܴ�,����Բ��
    {
        can_check=true;
    }

    if(L_straightaway && R_straightaway)
    {
        can_check=false;
    }

    if(can_check==true)
    {
    for(i=5;i<=50;i++) //Ѱ�һ���
    {
          if(points_r[i][0]>points_r[i+1][0] && dj<=9)   //���жϵ���
          {
              dj++;
          }
          if(points_r[i][0]<points_r[i+1][0] && dz<=9 && dj>=9)//����������һ�������ſ����жϵ���
          {
              dz++;
          }
          if(dz>=9 && dj>=9)//����Բ������
          {
              acr_flag=1;
              break;
          }
    }
     if(acr_flag==1 && left_angle_point==0)      //����Բ��������޽ǵ㣬ȷ����Բ��                                                               //���ڻ���
      {
         if(R_CirCle_State!=3)          //ǿ�ƽ�����Բ��״̬3
         R_CirCle_State=3;
         can_check=false;
         acr_flag=0;
     }
  }
}
//��Բ�������жϺ���
void assist_Left_ring_deal()
{
    static bool can_check=false;
    uint8 i=0;
    uint8 acr_flag=0;
    uint8 dz=0,dj=0;               //�����͵�����־λ
    if((Inductor_Value[0]+Inductor_Value[3])>=85 && Inductor_Value[1]>=25 && can_check==false && L_CirCle_State<3) //ˮƽ���ֵ�ܴ�,����Բ��
    {
        can_check=true;
    }

    if(L_straightaway && R_straightaway)
    {
        can_check=false;
    }

    if(can_check==true)
    {
    for(i=5;i<=45;i++) //Ѱ�һ���
    {
          if(points_l[i][0]<points_l[i+1][0] && dz<=9)   //���жϵ���
          {
              dz++;
          }
          if(points_l[i][0]>points_l[i+1][0] && dj<=9 && dz>=9)//����������һ�������ſ����жϵ���
          {
              dj++;
          }

          if(dz>=9 && dj>=9)//����Բ������
          {
              acr_flag=1;
              break;
          }
    }
     if(acr_flag==1 && right_angle_point==0)      //����Բ�����ұ��޽ǵ㣬ȷ����Բ��                                                               //���ڻ���
      {
         if(L_CirCle_State!=3)          //ǿ�ƽ�����Բ��״̬3
         L_CirCle_State=3;
         can_check=false;
         acr_flag=0;
     }
  }
}
void Ring_Deal(uint8(*image)[image_w]){
  uint8 i=0;
  uint8 white_row=0;               //�������м���
  static uint8 Angle_flag=0;   //���Խ��нǶ��ۼӱ�־λ
  average_steer_duty=0;       //���Խ���ƽ����Ǳ�־λ
  can_slopetoangle=0;        //б�ʴ�Ǳ�־λ��0
  element_rpts_num_l=(uint16)element_line_num;        //���鳤��
  element_rpts_num_r=(uint16)element_line_num;       //���鳤��
  static bool L_CirCle_State_lock=0;                //��Բ��״̬��
  static bool R_CirCle_State_lock=0;               //��Բ��״̬��
  uint8 angle_dist=7;                             //��������
  uint8 nums_angle_dist=7*2+1;                   //�ǶȷǼ���ֵ����
  uint8 limit_up=0,limit_down=0;                //�ǵ�����޷�
  uint8 dz=0,dj=0;                             //����������־λ
  //�����������������С��Բ����״̬�����д�������д�Բ����ɾ������
  int8 loss_to_acc_count_row=0;

  if(Angle_flag)           //�Ƕ������ۼ�
  {
  Ring_add_angle+=Turn_speed *Turn_dt ;           //���ٶ��ۼ�
  if(fabs(Ring_add_angle)>20) Ring_add_angle=0;  //�������
  }

  assist_Left_ring_deal();               //��и����ж���Բ��
  assist_Right_ring_deal();               //��и����ж���Բ��

  //��Բ��Ԥ�ж�
  if(L_CirCle_State==0)
  {
      if(l_ring_check_point==1 && l_grow_outside==1 && R_straightaway)   //����Բ����ǵ������⣬�ұ��ϸ�ֱ��
      {
          regression(2, 25, 50);                    //����ұ߽�
          R_slope=parameterB;
          regression(1, 25, 55);                    //�����߽�
          if(R_slope>0.35 && R_slope<0.85 && parameterB>0.17)   //��ֱ��б�ʷ�Χ����߽�б�ʼ����෴
          {
           //��ǵ�λ�ö�Ӧ�ұ߽粻����
           if(Right_Line_flag[l_ring_check_point_y] && Right_Line_flag[l_ring_check_point_y-1] && Right_Line_flag[l_ring_check_point_y-2])
           {
           Flag_L_CirCle=1;       //��Բ����־λ
           L_CirCle_State=1;
           L_CirCle_State_lock=0;         //����
           Flag_Cross=0;               //���
           }
          }
      }
  }
  //��Բ��Ԥ�ж�
  if(R_CirCle_State==0 && L_CirCle_State==0)      //����Բ������ͬʱ�жϵ�
  {
      if(r_ring_check_point==1 && r_grow_outside==1 && L_straightaway)   //����Բ���ҽǵ������⣬����ϸ�ֱ��
      {
          regression(1, 25, 50);                    //�����߽�
          L_slope=parameterB;
          regression(2, 25, 55);                    //����ұ߽�
          if(L_slope>-0.85 && L_slope<-0.35 && parameterB<-0.17)   //��ֱ��б�ʷ�Χ���ұ߽�б�ʼ����෴
          {
          if(Left_Line_flag[r_ring_check_point_y] && Left_Line_flag[r_ring_check_point_y-1] && Left_Line_flag[r_ring_check_point_y-2])
          {
           Flag_R_CirCle=1;        //��Բ����־λ
           R_CirCle_State=1;
           R_CirCle_State_lock=0;         //����
           Flag_Cross=0;         //���
          }
          }
      }
  }
/****************************��ʼ������״̬******************************/
 if(Flag_L_CirCle)   //�󻷴���
 {
 switch(L_CirCle_State)
 {
    case 1:                  //��ǰ״̬Ѱ�ұ���
    if(( l_gray_point==0 && !Left_Line_flag[50] && !Left_Line_flag[49] && !Left_Line_flag[48]
       &&!Left_Line_flag[47] && !Left_Line_flag[46]&&
       !Left_Line_flag[45]) && l_ring_check_point==0) //�����߽����ȫ����Բ���ǵ���ʧ,˵������Բ��Բ��
      {                                                                                                                                                                 //�յ���ʧ֮��Ļ��Ⱥ�����
             L_CirCle_State=2;  //������һ��״̬
             L_CirCle_State_lock=1;   //����
      }

    if(R_straightaway==0)                 //����ұ߽粻��ֱ��
       {
         L_CirCle_State=0;
       }
     break;
     case 2:    //��ǰ״̬Ѱ�ұ���
     if(L_CirCle_State_lock==1)       //����
      {
          for(i=55;i>=40;i--)        //�ж϶�����������
          {
              if(Left_Line_flag[i]==0 && Left_Line_flag[i-1]==0 &&
                 Left_Line_flag[i-2]==1 &&  Left_Line_flag[i-3]==1)
              {
                  L_CirCle_State_lock=0;       //����
                  break;
              }
          }
      }

     if(L_CirCle_State_lock==0) //�жϻ���
     {
       for(i=0;i<50;i++)
       {
           if(points_l[i][0] < points_l[i+1][0] && dz<12)//���жϵ�������
           {
               dz++;
           }
           if(points_l[i][0] > points_l[i+1][0] && dj<12 && dz>=12) //��������֮��ſ����жϵ����ݼ�
           {
               dj++;
           }
           if(dz>=12 && dj>=12)
           {
               loss_to_acc_count_row=1;   //ȷ����Բ��
               break;
           }
       }
     }

     if(Left_Line_flag[50] && Left_Line_flag[49] && Left_Line_flag[48]
       && Left_Line_flag[47] && Left_Line_flag[46]&&
       Left_Line_flag[45] && left_angle_point==1
     && L_CirCle_State_lock==0 && l_gray_point==1 && loss_to_acc_count_row)      //��ǵ���� ��û��������ȷ����Բ��
       {
            L_CirCle_State=3;
            L_CirCle_State_lock=1;            //����
       }

     if(R_straightaway==0)  //����ұ߽粻��ֱ��
       {
           L_CirCle_State=0;
       }
     break;
     case 3:                          //��ǰ״̬Ѱ�ұ���
         if(L_CirCle_State_lock==1)
         {
             for(i=40;i>15;i--)
             {
                 if(Left_Line_flag[i]==0 && Left_Line_flag[i-1]==0 &&
                    Left_Line_flag[i-2]==1 &&  Left_Line_flag[i-3]==1)
                 {
                     L_CirCle_State_lock=0; //����
                     break;
                 }
             }
         }

     if(                         //Բ����ʧ�ж�,���Խ�ߣ���Խ��ǰ����״̬4
        ( !Left_Line_flag[40] && !Left_Line_flag[41]
          &&!Left_Line_flag[42] && !Left_Line_flag[43]
          &&!Left_Line_flag[44] && !Left_Line_flag[45]
          &&left_angle_point==0 && L_CirCle_State_lock==0))  //�����߽翿�����¶��� ���ǵ���ʧ
             {
                           Beep_set(1);
                           L_CirCle_State=4;                //������һ��״̬
                           In_Poit_y=0;                    //�������ȴ��´β���
                           In_Poit_x=0;
                           Ring_add_angle=0;
                           Angle_flag=1;       //���Խ��нǶ��ۼ�
             }
         if(R_straightaway==0)  //����ұ߽粻��ֱ��
            {
             L_CirCle_State=0;
            }
     break;
     case 4:
         for(i=45;i>=20;i--)     //Ѱ���Թ����
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
                  findline_lefthand_binaryzation_angle_user(image, In_Poit_x, In_Poit_y, element_rpts_l,&element_rpts_num_l);  //����ɨ��߽�
                  for(i=0;i<element_rpts_num_l;i++)  //�Ƕ����
                  {
                      element_a_l[i] = 0;
                      element_outangle_l[i]=0;
                  }
                  for(i=0;i<element_rpts_num_l;i++)  //����Ƕȱ仯��
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

                  nms_angle(element_a_l, element_rpts_num_l, element_outangle_l,nums_angle_dist);   //�Ƕȼ���ֵ����

                  for(i=0;i<element_rpts_num_l;i++)         //��ȡ�ǵ�
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
         slope_to_angle=In_Poit_x-Right_Line[59];            //���㲹��б��
         slope_to_angle=slope_to_angle/(In_Poit_y-59);       //���㲹��б��
         slope_to_angle=atanf(slope_to_angle)*57.32;          //ֱ��ת��Ϊ�Ƕ�
         can_slopetoangle=1;                                //����б�ʴ��
         //�����ж��Ƿ�����Բ��
         if((!Right_Line_flag[50] && !Right_Line_flag[49] &&
            !Right_Line_flag[48] && !Right_Line_flag[47] &&
            !Right_Line_flag[46] && !Right_Line_flag[45]) && r_gray_point==0 )      //����ұ߽翿�����¶���
           {
              L_CirCle_State=5;                                //������һ��״̬
           }
     break;

     case 5:
         for(i=45;i>=20;i--)     //Ѱ���Թ����
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
                   findline_lefthand_binaryzation_angle_user(image, In_Poit_x, In_Poit_y, element_rpts_l,&element_rpts_num_l);  //����ɨ��߽�
                   for(i=0;i<element_rpts_num_l;i++)  //�Ƕ����
                   {
                       element_a_l[i] = 0;
                       element_outangle_l[i]=0;
                   }
                   for(i=0;i<element_rpts_num_l;i++)  //����Ƕȱ仯��
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

                   nms_angle(element_a_l, element_rpts_num_l, element_outangle_l, nums_angle_dist);   //�Ƕȼ���ֵ����

                   for(i=0;i<element_rpts_num_l;i++)         //��ȡ�ǵ�
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

         slope_to_angle=In_Poit_x-Right_Line[59];            //���㲹��б��
         slope_to_angle=slope_to_angle/(In_Poit_y-59);       //���㲹��б��
         slope_to_angle=atanf(slope_to_angle)*57.32;
         can_slopetoangle=1;                                 //����б�ʴ��
        if(Ring_add_angle>3.1 ||( Right_Line_flag[50] &&
           Right_Line_flag[45] &&  Right_Line_flag[40] && r_gray_point==1 ) )  //�ж��Ƿ��Ѿ��뻷
           {
              L_CirCle_State=6;     //������һ��״̬
              Out_Poit_x=0;         //�������
              Out_Poit_y=0;
              slope_to_angle=0;
              Add_steer_duty=0;               //���ռ�ձ�
              Add_steer_duty_count=0;           //��ռ���
           }
     break;
     case 6 :      //��ǰ״̬�Ѿ�����Բ��������ѭ����������ֿ쵽�����յ�λ��ֱ��ֱ�У��Ҹо���û���жϵ�״̬7��
         Add_steer_duty+=streeing_duty;         //ռ�ձ��ۼ�
         Add_steer_duty_count++;               //�ۼӴ���

         if(Ring_add_angle>6.4 && (right_angle_point && r_angle_point_y>=20 && r_angle_point_y<=45))//����Ѿ��ӽ�����(�������ȡ����30�У���������Ƕȸպ��ǽǵ�û�е���30�е����ֵ)
         {
         Out_Poit_y=Find_guaidian(20, 45, Right_Line, 2, 1);   //Ѱ���ҹյ�
         if((r_original_angle >55 && r_grow_outside) || (Out_Poit_y>=20&& Out_Poit_y<=45)) //���ڽǵ�,��������,�����йյ�
         {
             if(Out_Poit_y>=20&& Out_Poit_y<=45)  //�յ�����
             {
             Out_Poit_x=r_angle_point_x;  //��¼�ҽǵ�x����
             Out_Poit_y=r_angle_point_y; //��¼�ҽǵ�y����
             }
             else {
             Out_Poit_x=Right_Line[Out_Poit_y];  //��¼�ҽǵ�x����
            }

             for(i=Out_Poit_y-1;i>=Out_Poit_y-15;i--)
              {
                 if(image[i][Out_Poit_x]==255) //��������ɨ��,�Ƿ��������׵�(������)
                 {
                     white_row++;
                     if(white_row>=10)        //����������5�а׵�
                     {
                         L_CirCle_State=7;  //˵��׼��������������һ��״̬
                         L_slope_cross_state=0;    //��ʮ��
                         R_slope_cross_state=0;     //��ʮ��
                         Flag_Cross=0;              //��ʮ��
                         Average_duty= Add_steer_duty/Add_steer_duty_count;//����ƽ��ռ�ձ�
                         Add_steer_duty=0;
                         Add_steer_duty_count=0;
                         Out_Poit_y=0;    //�������
                         Out_Poit_x=0;   //�������
                         break;
                     }
                 }else{
                     white_row=0;
                 }
             }
         }
       }

     break;
     case 7:                  //����ƽ�����
         average_steer_duty=1;             //����ƽ�����
         if(r_gray_point==0 && right_angle_point==0 )
         {
            L_CirCle_State=8;                                                       //�������״̬
         }
     break;
     case 8:                                                                         //���߼�������
         average_steer_duty=1;             //����ƽ�����
         if(r_gray_point==1 && Ring_add_angle>8 && R_straightaway==1)  //�ұ������ָ�
         {
            Beep_set(0);
            L_CirCle_State=9;                                                       //�������״̬
            Ring_add_angle=0;
         }

     break;
     case 9 :                                        //���״̬Ѱ�ұ߽�
         if((Right_Line_flag[50] && Right_Line_flag[49] &&
            Right_Line_flag[48] && Right_Line_flag[47] &&
            Right_Line_flag[46] && Right_Line_flag[45] &&
            Left_Line_flag[50]&& Left_Line_flag[49] &&
            Left_Line_flag[48]&& Left_Line_flag[47] &&
            Left_Line_flag[46]&& Left_Line_flag[45]) || (L_straightaway && R_straightaway))  //������ұ߽綼������
         {
            if(OFF_Element_test==1)
            {
              Door_count=track_num;
            }
            L_CirCle_State=0;  //�Ѿ���ȫ����
            Flag_L_CirCle=0;     //���
            Average_duty=0;   //ƽ��ռ �ձ���0
            slope_to_angle=0;    //б��ת�Ƕ���0
            Add_steer_duty_count=0;//��ռ���
            Add_steer_duty=0;  //�ۼƵ�ռ�ձ���0
            Angle_flag=0;
            Ring_add_angle=0;
         }
     break;
 }
/*******************Բ��״̬����*******************************/
         if(L_CirCle_State==1 || L_CirCle_State==2 || L_CirCle_State==3 ||  L_CirCle_State==9 )           //Ѱ�ұ���
           {
              for(i=59;i>20;i--)
                  Mid_Line[i]=clip((Right_Line[i]-Width_straight[i]/2),0,93);
           }
  }
 /****************************��ʼ�����һ�״̬******************************/
 if(Flag_R_CirCle)   //�һ�����,�һ��������󻷲�һ����һ�������£��һ�����ȥ
  {
  switch(R_CirCle_State)
  {
     case 1:                  //��ǰ״̬Ѱ�����
     if((r_gray_point==0 && !Right_Line_flag[50] && !Right_Line_flag[49] && !Right_Line_flag[48]
        && !Right_Line_flag[47] && !Right_Line_flag[46]&&
        !Right_Line_flag[45]) &&  r_ring_check_point==0 ) //����ұ߽����ȫ����Բ���ǵ���ʧ,˵������Բ��Բ��
         {                                                                                                                                                                 //�յ���ʧ֮��Ļ��Ⱥ�����
              R_CirCle_State=2;  //������һ��״̬
              R_CirCle_State_lock=1;
         }
     if(L_straightaway==0)  //�����߽粻��ֱ��
        {
            R_CirCle_State=0;
        }
      break;
      case 2:                            //��ǰ״̬Ѱ�����
          if(R_CirCle_State_lock==1)
          {
              for(i=57;i>=40;i--)        //�ж϶�����������
              {
                  if(Right_Line_flag[i]==0 && Right_Line_flag[i-1]==0 &&
                     Right_Line_flag[i-2]==1 &&  Right_Line_flag[i-3]==1)
                  {
                      R_CirCle_State_lock=0;
                      break;
                  }
              }
          }

          if(R_CirCle_State_lock==0) //�жϻ���
          {
            for(i=0;i<50;i++)
            {
                if(points_r[i][0] > points_r[i+1][0] && dj<12)  //���жϵ���
                {
                    dj++;
                }

                if(points_r[i][0] < points_r[i+1][0] && dz<12 && dj>=12) //��������֮��ſ����жϵ���
                {
                    dz++;
                }

                if(dz>=12 && dj>=12)
                {
                    loss_to_acc_count_row=1;   //ȷ����Բ��
                    break;
                }
            }
          }

           if(Right_Line_flag[50] && Right_Line_flag[49] && Right_Line_flag[48]
              && Right_Line_flag[47] && Right_Line_flag[46]&&
              Right_Line_flag[45] && right_angle_point==1 && R_CirCle_State_lock==0
              && r_gray_point==1 && loss_to_acc_count_row==1)      //�ҽǵ���� ��ȷ����Բ��
            {
               R_CirCle_State=3;
               R_CirCle_State_lock=1;  //����
            }
           if(L_straightaway==0)  //�����߽粻��ֱ��
              {
                  R_CirCle_State=0;
              }

      break;
      case 3:                          //��ǰ״̬Ѱ�����
          if(R_CirCle_State_lock==1)
          {
              for(i=40;i>15;i--)
              {
                  if(Right_Line_flag[i]==0 && Right_Line_flag[i-1]==0 &&
                     Right_Line_flag[i-2]==1 &&  Right_Line_flag[i-3]==1)
                  {
                      R_CirCle_State_lock=0; //����
                      break;
                  }
              }
          }


          if(                         //Բ����ʧ�ж�,���Խ�ߣ���Խ��ǰ����״̬4
             !Right_Line_flag[40] && !Right_Line_flag[41]
             &&!Right_Line_flag[42] && !Right_Line_flag[43]
             &&!Right_Line_flag[44] && !Right_Line_flag[45]
             && right_angle_point==0 && R_CirCle_State_lock==0)  //����ұ߽翿�����¶��� ���ǵ���ʧ
              {
                            Beep_set(1);
                            R_CirCle_State=4;                //������һ��״̬
                            In_Poit_y=0;                    //�������ȴ��´β���
                            In_Poit_x=0;
                            Ring_add_angle=0;
                            Angle_flag=1;                   //���Խ��нǶ��ۼ�
              }
          if(L_straightaway==0)  //�����߽粻��ֱ��
             {
                 R_CirCle_State=0;
             }
      break;
      case 4:
          for(i=45;i>=20;i--)     //Ѱ���Թ����
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
                   findline_righthand_binaryzation_angle_user(image, In_Poit_x, In_Poit_y, element_rpts_r,&element_rpts_num_r);  //����ɨ��߽�
                   for(i=0;i<element_rpts_num_r;i++)  //�Ƕ����
                   {
                       element_a_r[i] = 0;
                       element_outangle_r[i]=0;
                   }
                   for(i=0;i<element_rpts_num_r;i++)  //����Ƕȱ仯��
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

                   nms_angle(element_a_r, element_rpts_num_r, element_outangle_r, angle_dist);   //�Ƕȼ���ֵ����

                   for(i=0;i<element_rpts_num_r;i++)         //��ȡ�ǵ�
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
          slope_to_angle=In_Poit_x-Left_Line[59];            //���㲹��б��
          slope_to_angle=slope_to_angle/(In_Poit_y-59);       //���㲹��б��
          slope_to_angle=atanf(slope_to_angle)*57.32;          //ֱ��ת��Ϊ�Ƕ�
          can_slopetoangle=1;
          //�����ж��Ƿ�����Բ��
          if( (!Left_Line_flag[50] && !Left_Line_flag[49] &&
             !Left_Line_flag[48] && !Left_Line_flag[47] &&
             !Left_Line_flag[46] && !Left_Line_flag[45]) && l_gray_point==0)      //�����߽翿�����¶���
            {
               R_CirCle_State=5;                                //������һ��״̬
            }
      break;

      case 5:
          for(i=45;i>=20;i--)     //Ѱ���Թ����
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
                    findline_righthand_binaryzation_angle_user(image, In_Poit_x, In_Poit_y, element_rpts_r,&element_rpts_num_r);  //����ɨ��߽�
                    for(i=0;i<element_rpts_num_r;i++)  //�Ƕ����
                    {
                        element_a_r[i] = 0;
                        element_outangle_r[i]=0;
                    }
                    for(i=0;i<element_rpts_num_r;i++)  //����Ƕȱ仯��
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

                    nms_angle(element_a_r, element_rpts_num_r, element_outangle_r, nums_angle_dist);   //�Ƕȼ���ֵ����

                    for(i=0;i<element_rpts_num_r;i++)         //��ȡ�ǵ�
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
          slope_to_angle=In_Poit_x-Left_Line[59];            //���㲹��б��
          slope_to_angle=slope_to_angle/(In_Poit_y-59);       //���㲹��б��
          slope_to_angle=atanf(slope_to_angle)*57.32;
          can_slopetoangle=1;
          if(Ring_add_angle<-3.2 || ( Left_Line_flag[50] &&
              Left_Line_flag[45] &&  Left_Line_flag[40] && l_gray_point==1))  //�ж��Ƿ��Ѿ��뻷
            {
               R_CirCle_State=6;     //������һ��״̬
               Out_Poit_x=0;         //�������
               Out_Poit_y=0;
               slope_to_angle=0;
               Add_steer_duty=0;               //���ռ�ձ�
               Add_steer_duty_count=0;           //��ռ���
            }
      break;
      case 6 :                                     //��ǰ״̬�Ѿ�����Բ��������ѭ��
          Add_steer_duty+=streeing_duty;         //ռ�ձ��ۼ�
          Add_steer_duty_count++;               //�ۼӴ���

          if(Ring_add_angle<-6.4 && (left_angle_point && l_angle_point_y>=20 && l_angle_point_y<=45 )) //����Ѿ��ӽ�����
          {
          Out_Poit_y=Find_guaidian(20, 45, Left_Line, 1, 1);   //Ѱ����յ�
          if(( l_original_angle >55 && l_grow_outside ) || (Out_Poit_y>=20 && Out_Poit_y<=45) )  //���ڽǵ�,��������
          {
             if(Out_Poit_y>=20 && Out_Poit_y<=45)  //<30˵���ǽǵ����������������ǹյ�����
              {
              Out_Poit_x=l_angle_point_x;  //��¼�ҽǵ�x����
              Out_Poit_y=l_angle_point_y; //��¼�ҽǵ�y����
              }
              else {
              Out_Poit_x=Left_Line[Out_Poit_y];  //��¼�ҽǵ�x����
              }

              for(i=Out_Poit_y-1;i>=Out_Poit_y-15;i--)
               {
                  if(image[i][Out_Poit_x]==255) //��������ɨ��,�Ƿ��������׵�
                  {
                      white_row++;
                      if(white_row>=10)        //����������10�а׵�
                      {
                          R_CirCle_State=7;  //˵��׼��������������һ��״̬
                          L_slope_cross_state=0;     //��ʮ��
                          R_slope_cross_state=0;     //��ʮ��
                          Flag_Cross=0;          //��ʮ��
                          Average_duty= Add_steer_duty/Add_steer_duty_count;//����ƽ��ռ�ձ�
                          Add_steer_duty=0;
                          Add_steer_duty_count=0;
                          Out_Poit_y=0;    //�������
                          Out_Poit_x=0;   //�������
                          break;
                      }
                  }else{
                      white_row=0;
                  }
              }
          }
        }

      break;
      case 7:                                                                         //���߳���
          average_steer_duty=1;             //����ƽ�����
          if(l_gray_point==0  &&  left_angle_point==0 )           //����ұ߽綼����
          {
             R_CirCle_State=8;                                                       //�������״̬
          }

      break;
      case 8:                                                                         //���߼�������
          average_steer_duty=1;             //����ƽ�����
          if( l_gray_point==1 && Ring_add_angle<-8 && L_straightaway==1 )  //��������ָ�
          {
             Beep_set(0);
             R_CirCle_State=9;                                                       //�������״̬
             Ring_add_angle=0;
          }

      break;
      case 9 :                                        //���״̬Ѱ��߽�
          if((Right_Line_flag[50] && Right_Line_flag[49] &&
             Right_Line_flag[48] && Right_Line_flag[47] &&
             Right_Line_flag[46] && Right_Line_flag[45] &&
             Left_Line_flag[50]&& Left_Line_flag[49] &&
             Left_Line_flag[48]&& Left_Line_flag[47] &&
             Left_Line_flag[46]&& Left_Line_flag[45]) || (L_straightaway && R_straightaway))  //������ұ߽綼������
          {
             if(OFF_Element_test==1)
             {
                Door_count=track_num;
             }
             R_CirCle_State=0;  //�Ѿ���ȫ����
             Flag_R_CirCle=0;
             Average_duty=0;   //ƽ��ռ�ձ���0
             slope_to_angle=0;    //б��ת�Ƕ���0
             Add_steer_duty_count=0;//��ռ���
             Add_steer_duty=0;  //�ۼƵ�ռ�ձ���0
             Angle_flag=0;
             Ring_add_angle=0;
          }
      break;
  }
 /*******************Բ��״̬����*******************************/
     if(R_CirCle_State==1 || R_CirCle_State==2 || R_CirCle_State==3 ||  R_CirCle_State==9 )           //Ѱ�ұ���
       {
          for(i=59;i>=20;i--)
             Mid_Line[i]=clip((Left_Line[i]+Width_straight[i]/2),0,93);
       }
   }
}
/*---------------------------------------------------------------
 ����    �ܡ��ϰ��ﴦ��
 ����    ����ͼ�����飬tof��õľ���
 ���� �� ֵ����
 ----------------------------------------------------------------*/
uint8 Barrier_state=0;       //�ϰ���״̬
float Barrier_Add_angle=0;      //�ۼӵĽǶ�
float  out_angle=1.4;          //��ȥ�Ƕ�
float  r_out_angle=-1.4;     //�����г�ȥ
int16  out_bypass_duty=-110;  //�Ƴ�ȥ������
int16  in_bypass_duty=110;   //�ƻ���������
uint16 check_distance=900;  //������mm
bool Barrier_out_count=0;   //���ϰ�����м���
void  Barrier_Deal(uint8(*image)[image_w],float tof_distance )
{
    /***********************Ԥ�ж�*****************************/
    bool l_straight=0,r_straight=0;
    if(tof_distance< check_distance && hightest>=14 && Barrier_state==0)
    {
        l_straight=Straight_line_judgment(Left_Line,1,35,59);
        r_straight=Straight_line_judgment(Right_Line,2,35,59);
        if(l_straight && r_straight)
        Barrier_state=1;                                //�����ϰ����־λ1
    }
    if(left_angle_point==true && right_angle_point==true
      && l_grow_inside==1 && r_grow_inside==1 && Barrier_state==1) //����˫�ǵ�������
    {
               Barrier_state=2;                                    //�����ϰ����־λ1
               Barrier_Add_angle=0;
               streeing_duty=out_bypass_duty;                   //��û����ȫ��ʻ������֮ǰ�������
               Beep_set(1);                                     //��������
    }

    switch(Barrier_state)
    {
    case 2:                                 //���г�ȥ
        if(Barrier_dir==1)  //����
        {
            streeing_duty=out_bypass_duty;
            Barrier_Add_angle+=Turn_speed*Turn_dt;   //��¼�Ƕ�
            if(Barrier_Add_angle>out_angle && ((Inductor_Value[0]+Inductor_Value[1]+Inductor_Value[2]+Inductor_Value[3])<7))
            {
                Turn_speed=0;
                Barrier_state=3;
                Barrier_Add_angle=0;
            }
        }
        else {
            streeing_duty=-out_bypass_duty;
            Barrier_Add_angle+=Turn_speed*Turn_dt;   //��¼�Ƕ�
            if(Barrier_Add_angle< r_out_angle && (Inductor_Value[0]+Inductor_Value[1]+Inductor_Value[2]+Inductor_Value[3])<7)      //�����г�ȥ
            {
                Turn_speed=0;
                Barrier_state=3;
                Barrier_Add_angle=0;
            }
        }
    break;
    case 3:                                //���л���
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
 ����    �ܡ����մ�����
 ����    ������ֵ��ͼ��
 ���� �� ֵ����
 ----------------------------------------------------------------*/
void Final_deal(uint8(*image)[image_w]){
    if(OFF_Element_test==0)        //��Ԫ�ز��Խ׶�
    {
    Out_door_deal(Out_door_type);
    if(OFF_Charge_Resue)          //1Ϊ��������Ԯ
    Charge_Resue(Out_door_type);
    }
    else {
    Out_door_state=1;            //Ԫ�ز��Խ׶�,��ִ�г���
    }

    if(Out_door_state==1 && Door_count!=track_num) //����ɹ�����������
    {
    REPIRE_LINE=0;            //���߱�־λ���
    if(Distance>0.15)        //����һС�ξ������Ԫ��ʶ��
    {
    L_straightaway=0;                //��ֱ����־λ
    R_straightaway=0;                //��ֱ����־λ
    L_straightaway=Straight_line_judgment(Left_Line,1,20,55);    //OK��ʹ��ûɶ����
    R_straightaway=Straight_line_judgment(Right_Line,2,20,55);   //OK��ʹ��ûɶ����
    Repire();                                                  //ֻ��ֱ�������޲�,OK��ʹ��ûɶ����
    In_door_deal(Out_door_type,image);                       //���ҿ�,OK��ʹ��ûɶ����
    Ring_Deal(image);                                      //����Բ��
    if(Flag_Cross==0)
    {
     slope_cross_deal(User_image);                     //б��ʮ��
    }
    if(slope_cross==0)
    {
     cross_deal(User_image);                        //����ʮ��
    }
    Check_Small_S();                              //s���ж�
    Ramp_Deal(gyrox);                            //�µ��ж�
    Break_Road_Deal(image,dl1a_distance_mm);    //��·����
    Barrier_Deal(image,dl1a_distance_mm);      //�ϰ��ﴦ��
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
 ����    �ܡ����������
 ����    ������
 ���� �� ֵ����
 ----------------------------------------------------------------*/
void steer_deal()
{
    if(Barrier_state==2 ||  Barrier_state==3  || Out_door_state==0)
    {                                        //������ϰ���״̬���߳�����״̬
    if(Out_door_type==1 && Out_door_state==0)  //�����
    {
        streeing_duty=-120;
    }else if(Out_door_type==2 && Out_door_state==0)//���ҿ�
    {
        streeing_duty= 120;
    }
    final_err=0;
    }

    else
    {
    if(can_slopetoangle==1)//����б�ʴ��
    {
        streeing_duty=slopetoangle_user_PID(slope_to_angle);
    }
    else if(average_steer_duty) //����ƽ�����
    {
        streeing_duty=Average_duty;
    }
    else if(Charge_Resue_state==2) //��Ԯ״̬,���˶��ȡ��
    {
        final_err=turn_error(Level_Deviation);
        streeing_duty=-Steering_gear_PID(final_err)*1.6;
    }
    else  //����״̬
    {
        final_err=turn_error(Level_Deviation);
        if(Break_road_state==0)
        streeing_duty=Steering_gear_PID(final_err);
        else {                                                 //��·
        streeing_duty =Inductor_user_PID(final_err);
      }
    }
  }
}
/*---------------------------------------------------------------
 ����    �ܡ������⣨ִ��һ��)(���Լ�Ⲣ����,���������ʱ���ø���)
 ����    ����type ��1 �����  2 ���ҿ�
 ���� �� ֵ����
 ��ʹ    �á�Out_door_deal(1)(�����Ϊ��)
 ----------------------------------------------------------------*/
float Out_L_door_angle=1.5;                            //���󳵿���Ҫ�ۼӵĽǶ�
float Out_R_door_angle=-1.5;                          //���ҳ�����Ҫ�ۼӵĽǶ�
float ADD_angle=0;                                   //�ۼӵĽǶ�
uint8 Out_door_state=0;                             //������״̬
void Out_door_deal(uint8 type)
{
     if(Out_door_state==0 && type==2)                                                //ִ���ҳ���
     {
      ADD_angle+=Turn_speed*Turn_dt;                                         //�Ƕ��ۼ�
      if( ADD_angle<Out_R_door_angle)
      {
          Out_door_state=1;                                                //����ɹ�
          ADD_angle=0;
      }
     }

     if(Out_door_state==0 && type==1)                                                //ִ�������
     {
      ADD_angle+=Turn_speed*Turn_dt;                                         //�Ƕ��ۼ�
      if(ADD_angle>Out_L_door_angle)
      {
          Out_door_state=1;                                                //����ɹ�
          ADD_angle=0;
      }
     }
}
/*---------------------------------------------------------------
 ����    �ܡ��복��(���Լ�Ⲣ���,���������ʱ���ø���)
 ����    �����������type 1 �����  2 ���ҿ�  ��ֵ��ͼ��
 ���� �� ֵ����
 ----------------------------------------------------------------*/
uint8 Indooor_state=0;                            //������ʶ���־λ
uint8 Door_count=0;                              //���������ߴ���,���ɹ���ֵ�ó�3��Ȼ��ͣ��
float L_Angle_door=1.0;                           //������ۼƽǶ�
float R_Angle_door=-1.0;                          //������ۼƽǶ�
float Add_angle_door=0;                           //����ۼƽǶ�
float Add_distance_no_door=0;                      //�����ͣ������
float Add_distance_door=0;                       //ʶ�𵽰�����֮����ʻһ�ξ�������ٴ�ʶ��
void In_door_deal(uint8 type,uint8(*image)[image_w])
{
    uint8 i=0,j=0;
    uint8 garage_count=0,region=0;                        //��������������
    uint8 white_black=1,black_white=1;                   //�׺ڣ��ڰ������
    if((Indooor_state>=2 && type==2) || Door_count==2)                   //�ҿ����´���߽磬��ֹ����
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
        findline_lefthand_binaryzation(image, pts_l[0],pts_l[1], points_l,&num_L);//����ɨ��߽�
        Get_new_Line(Left_Line,pts_l[0],pts_l[1],points_l,num_L);                //���»�ȡ�߽�
        }
    }
    if((Indooor_state>=2 && type==1) || Door_count==2)                   //������´���߽磬��ֹ����
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
        findline_righthand_binaryzation(image, pts_r[0],pts_r[1], points_r,&num_R);//����ɨ��߽�
        Get_new_Line(Right_Line,pts_r[0],pts_r[1],points_r,num_R);                //���»�ȡ�߽�
        }
    }

    if(Indooor_state && type==2)                   //���ҿ�
    {
        for(i=59;i>=20;i--)
        {
            Mid_Line[i]=clip(Left_Line[i]+Width_straight[i]/2,0,93);
        }
    }else if(Indooor_state && type==1)             //�����
    {
        for(i=59;i>=20;i--)
        {
            Mid_Line[i]=clip(Right_Line[i]-Width_straight[i]/2,0,93);
        }
    }

    switch(type)                           //������
    {
    case 2:                                //�ҿ⴦��
        switch(Indooor_state)
        {
        case 0:
            if(left_angle_point==0 && r_garage_check_point==1
               && r_grow_outside==1 && L_straightaway)       //����޽ǵ㣬���ֱ�����ұ߳���ǵ�����������
            {
                if(Left_Line_flag[r_garage_check_point_y]&&
                   Left_Line_flag[r_garage_check_point_y-1]&&
                   Left_Line_flag[r_garage_check_point_y-2])  //��߽���ҽǵ����겻����
                {
                    Indooor_state=1;          //������һ��״̬
                }
            }
        break;
        case 1:
            if(r_gray_point==0 && l_gray_point==1) //Ȼ������ұ߶��ߣ���߲���
            {
                Indooor_state=2;          //������һ��״̬
                 Beep_set(1);
            }
        break;
        case 2:
            for(i =45;i>=40;i--)    //��ʼ����ɨ��8���ж��Ƿ���ںڰ������
            {
                garage_count= 0;
                for(j =80;j>47-Width_straight[i]/2;j--)      //������ɨ,47-���һ����������Ҫ��֤��̬��
                  {
                    if(image[i][j]==255)           //�����ǰ��Ϊ��
                    {
                     white_black=1;
                     }
                    else
                    {
                     white_black=0;
                    }
                   if(white_black!=black_white)        //black_whiteԭʼֵΪ1,����׵�
                    {
                    black_white = white_black;         //��������
                    garage_count++;                    //������+1
                    }
                   if(garage_count>9)                  //һ�д���10�������
                    {
                       region++;                      //����+1
                       break;
                    }
                 }
                if(region>=3)       //3������,ȷ���ǰ�����
                  {
                   L_CirCle_State=0;                  //�����Բ������
                   R_CirCle_State=0;                  //�����Բ������
                   Indooor_state=3;            //������һ��״̬
                   Beep_set(0);
                   Door_count+=1;               //·������+1
                   break;
                  }
             }

       break;
       case 3:
           Add_distance_door+=Car_Speed*Car_speed_dt;
           if(Add_distance_door>0.1)           //����0.4m
           {
               Indooor_state=0;            //�Ѿ�·��һ�ΰ�����
               Door_count+=1;               //·������+1
               Add_distance_door=0;
           }
      break;
    }
    break;

    case 1:                           //��⴦��
      switch(Indooor_state)
        {
        case 0:
        if(right_angle_point==0 && l_garage_check_point==1
           && l_grow_outside==1 && R_straightaway)       //�ұ��޽ǵ㣬�ұ�ֱ������߳���ǵ�����������
        {
            if(Right_Line_flag[l_garage_check_point_y]&&
               Right_Line_flag[l_garage_check_point_y-1]&&
               Right_Line_flag[l_garage_check_point_y-2])  //�ұ߽����ǵ����겻����
            {
                Indooor_state=1;          //������һ��״̬
            }
        }
        break;
        case 1:
            if(l_gray_point==0 && r_gray_point==1)
            {
              Indooor_state=2;          //������һ��״̬
              Beep_set(1);
            }
        break;
        case 2:
            for(i=45;i>=40;i--)                                    //��ʼ����ɨ���ж��Ƿ���ںڰ������
            {
               garage_count = 0;
               for(j =13;j<47+Width_straight[i]/2;j++)      //������ɨ
                  {
                    if(image[i][j]==255)           //�����ǰ��Ϊ��
                    {
                     white_black=1;
                     }
                    else
                    {
                     white_black=0;
                     }
                   if(white_black!=black_white)        //black_whiteԭʼֵΪ1,����׵�
                    {
                    black_white = white_black;         //��������
                    garage_count++;                    //������+1
                    }
                   if(garage_count>9)                  //һ�д���10�������
                    {
                       region++;                      //����+1
                       break;
                    }
                 }
             if(region>=3)                                //3������,ȷ���ǰ�����
               {
                L_CirCle_State=0;                      //��Բ��
                R_CirCle_State=0;                     //��Բ��
                Indooor_state=3;                     //������һ��״̬
                Beep_set(0);
                Door_count+=1;               //·������+1
                break;
               }
             }
       break;
       case 3:
            Add_distance_door+=Car_Speed*Car_speed_dt;
            if(Add_distance_door>0.1)           //����0.05m
            {
                Indooor_state=0;            //�Ѿ�·��һ�ΰ�����
                Door_count+=1;               //·������+1
                Add_distance_door=0;
            }
       break;
    }
     break;
  }
}
/*---------------------------------------------------------------
 ����    �ܡ�������֮����г���Ԯ
 ����    ����dir_type 1  ��       2    ��
 ----------------------------------------------------------------*/
uint8 Charge_Resue_state=0;   //��Ԯ״̬
uint8 Charge_Resue_count=0;    //���ʹ�ܷ������
int16 L_charge_big_thes=45,L_charge_small_thes=10,R_charge_big_thes=45,R_charge_small_thes=13;//����ֹͣ��ʼ�����ĵ����ֵ
void Charge_Resue(uint8 dir_type)
{
     static bool successful_recure=0; //�ɹ���Ԯ��־λ
     if(Out_door_state==1 && Charge_Resue_state==0 && successful_recure==0) //����ɹ�֮�����̽����Ԯ״̬
     {
         Charge_Resue_state=1;
     }

     if(Charge_Resue_count==30 && car_state==16)//��ʱ1.5s,ͣ���ٷ���
     {
        gpio_set_level(B14,1); //ʹ�ܿ�����緢��
     }

    switch(Charge_Resue_state)
      {
      case 1:
      if( ((Inductor_Value[0]>L_charge_big_thes && Inductor_Value[1]<L_charge_small_thes)
         || (Inductor_Value[2]<R_charge_small_thes && Inductor_Value[3]>R_charge_big_thes) ) && successful_recure==0)  //���ֵΪֱ��״̬
      {
          Charge_Resue_state=2;
      }
      break;

      case 2:            //��ʱΪ����,���ȡ��
      if(dir_type==2)   //Ѳ��߽� ,�Ҿ�Ԯ
      {
          for(uint8 i=59;i>20;i--)
          {
              Mid_Line[i]=clip(Left_Line[i]+Width_straight[i]/2,0,93);
          }
          REPIRE_LINE=1;                      //����ȷ��������ʼ��
      }

      if(dir_type==1)    //Ѳ�ұ߽� ,���Ԯ
      {
          for(uint8 i=59;i>20;i--)
          {
              Mid_Line[i]=clip(Right_Line[i]-Width_straight[i]/2,0,93);
          }
          REPIRE_LINE=1;                      //����ȷ��������ʼ��
      }

      if(r_gray_point==0 && l_gray_point==1 && successful_recure==0 && dir_type==2)   //�ұ߽߱綪�ߣ�����ͣ�����ȴ�1s���г��
      {
         successful_recure=1;
         Charge_Resue_state=3;
         Distance=0;   //��0,�ȴ����Լ����ǰ��
      }

      if(r_gray_point==1 && l_gray_point==0 && successful_recure==0 && dir_type==1)   //��߽߱綪�ߣ�����ͣ�����ȴ�1s���г��
      {
         successful_recure=1;
         Charge_Resue_state=3;
         Distance=0;   //��0,�ȴ����Լ����ǰ��
      }
      break;
   }
}
/*---------------------------------------------------------------
 ����    �ܡ��µ�����
 ����    ����ĳ����ٶ�
 ���� �� ֵ����
 ----------------------------------------------------------------*/
uint8 Ramp_state=0;      //�µ���־λ
float add_gyro_x=0;      //x����ٶȻ���
void Ramp_Deal(float gyro)
{
    add_gyro_x+=gyro*Turn_dt;
    static bool clear_flag=0;
    if(add_gyro_x < (-15) && Ramp_state==0)       //���ٶȺܴ󣨸�����������
    {
       Ramp_state=1;
       clear_flag=0;
    }
    if(add_gyro_x > (10) && Ramp_state==1)         //���ٶȺܴ�������������
    {
       Ramp_state=2;
    }
    if(fabs(gyro)<10 && Ramp_state==0) add_gyro_x=0;          //׼�����£���ǰ���
    if(clear_flag==0 && (gyro)>6 && Ramp_state==1)              //׼�����£���ǰ���
      {
        add_gyro_x=0;       //׼�����£���ǰ���
        clear_flag=1;
      }
    if(Ramp_state==1)
    {
      Barrier_state=0;               //�ϰ���
    }
    if(Ramp_state==2)                //�µ�������б�־λ
    {
        L_CirCle_State=0;         //��Բ��
        R_CirCle_State=0; //��Բ��
        Flag_Cross=0;     //ʮ��
        L_slope_cross_state=0;   //��д��ʮ��
        R_slope_cross_state=0;   //��б��ʮ��
        Barrier_state=0;          //�ϰ���
        Beep_set(0);
    }
}
/*---------------------------------------------------------------
 ����    �ܡ���·����
 ����    ����ͼ�����飬tof��õľ���
 ���� �� ֵ����
 ----------------------------------------------------------------*/
uint8 Break_road_state=0;                               //��·״̬��־λ
void Break_Road_Deal(uint8(*image)[image_w],float tof_distance)
{
    uint8 i=0,count=0;
    uint8 black_row=0;
    float l_slope=0,r_slope=0;  //����б��
    uint8 l_up=0,l_down=0;
    uint8 r_up=0,r_down=0;
    uint8 line_count=0;
    uint8 l_in_count=0,r_in_count=0,upper_l,upper_r;
    uint8 l_dz_count=0,r_dj_count=0;    //���ұ߽絥��^����
    bool assist_black_row=0;
   /***********************Ԥ�ж�*****************************/
  /***********************�����·�ж�*****************************/
    if(Break_road_state==0 && hightest>=25 && hightest<=50 && hightest_x>23
     && hightest_x<70 && L_CirCle_State==0 && R_CirCle_State==0
     && L_slope_cross_state==0 && R_slope_cross_state==0
     && Flag_Cross==0 && Indooor_state==0)                                //ͼ������ض�,����ҪԪ�ؽ׶�
    {
        assist_black_row=Fine_heihang(2, hightest-5, hightest, image, 2);//Ѱ�Һ���
        regression(1, hightest+2, start_line_point_row);     //�����߽�
        l_slope=parameterB;                                   //��¼��߽�б��
        regression(2, hightest+2, start_line_point_row);   //����ұ߽�
        r_slope=parameterB;                                 //��¼�ұ߽�б��
        /***********************���Ƕ�·�ж�*****************************/
        if((l_gray_point || r_gray_point))  //���������·�����ж�
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
             //����Ͽ���һ�߶��߱Ƚ϶࣬��abs(59-hightest-line_count)<=10ȥ��
            if(abs(59-hightest-line_count)<=10 && ((r_slope>=1.05 && l_slope<= -0.8) || (r_slope>=0.8 && l_slope<= -1.05)))//�����������
            {
                if( (abs(59-hightest-l_dz_count)<8 || abs(59-hightest-r_dj_count)<8)
                  &&tof_distance>5500  && assist_black_row)
                    Break_road_state=1;
            }
        }
        //�յ��ж�
        //�ұ߽�
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
        //��߽��ж�
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

        if((l_up>=10 && l_down>=10) && (r_up>=10 && r_down>=10)    //˫^���ж϶�·
          && l_slope<-0.85 && r_slope>0.85 && tof_distance>2500 && assist_black_row)
        {
            Break_road_state=1;
        }

        if( ((r_slope>=1.05 && l_slope<= -0.85) || (r_slope>=0.85 && l_slope<= -1.05) )//�����ж϶�·
           && image[hightest-2][hightest_x]==0 && image[hightest-3][hightest_x]==0
           && tof_distance>2500 && assist_black_row)
        {
           Break_road_state=1;
        }
       //����ǵ��ж�
       if(l_break_road_check_point && r_break_road_check_point)
        {
             upper_l=clip(l_break_road_check_point_y+13,0,element_rpts_num_l-1);    //�����ж�13����
             for(i=l_break_road_check_point_y;i<upper_l;i++)
             {
                 if(element_rpts_l[i][0]>element_rpts_l[l_break_road_check_point_y][0])  //��ǵ���������
                 {
                     l_in_count++;
                 }
                 if(l_in_count>7)
                 {
                     break;
                 }
             }
             upper_r=clip(r_break_road_check_point_y+13,0,element_rpts_num_r-1);    //�����ж�13����
             for(i=r_break_road_check_point_y;i<upper_r;i++)
             {
                 if(element_rpts_r[i][0]<element_rpts_r[r_break_road_check_point_y][0])  //��ǵ���������
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
  //ֱ�����ض��ж�
  if(left_angle_point==true && right_angle_point==true && Break_road_state==0) //����˫�ǵ�
   {
     if(l_grow_inside==1 && r_grow_inside==1)           //�ǵ�����
       {
           black_row=Fine_heihang(2, 25, 30, image, 3);
           if(tof_distance>2500 && hightest>=30 && black_row)          //ǰ�����ϰ���
           {
               if(L_CirCle_State==0 || Flag_Cross==0 || Barrier_state==0 || L_slope_cross_state==0 || R_slope_cross_state==0) //������Ԫ�ز������ж�
               {
                  Break_road_state=1;
                  Beep_set(1);
               }
           }
       }
    }

   if( Break_road_state == 1 && vert_gray_point == 1 )    //���ֵײ���Ⱥ͵�
   {
       Break_road_state = 2;
   }

   if(Break_road_state==2 && vert_gray_point==0)    //�ײ���Ⱥ͵���ʧ
   {
       Break_road_state=3;
   }

   if(Break_road_state==3)
    {
        for(uint8 j=start_line_point_row;j>=56;j--)    //�ײ������ΰ�ɫ���
        {
            for(i=40;i<=54;i++)
            {
                if(image[j][i]==255)
                {
                    count++;
                }
            }
        }
        if(count>38 && ( l_gray_point==1 || r_gray_point==1 ) )  //�����μ���������ײ��߽��ָ�
        {
         if(OFF_Element_test==1)
         {
          Door_count=track_num;
         }
         Beep_set(0);
         Break_road_state=0;                                   //�˳���·�ж�
        }
    }
 }
/*---------------------------------------------------------------
 ����    �ܡ���������б�ʲ��ߺ���
 ����    ����beginΪ����ĵ�,endΪ����ĵ㣬lineΪ���߻���������,numΪ���߳���
 ���� �� ֵ����
 ----------------------------------------------------------------*/
void Add_line_user_twopoint_slope(uint8 type,uint8 line[], uint8 begin_x, uint8 begin_y,uint8 end_x, uint8 end_y,uint8 num)
{
  uint8 i=0,x=0;
  float slope=0;
  int8 N=begin_y-num;
  N=clip(N,20,59);
  slope = begin_x - end_x;
  slope = slope/(begin_y-end_y);
  if(type==1) //����
  {
   slope =LIMIT(slope,L_straight_slope/2.0,-2);
  }
  else if(type==2)  //����
  {
   slope =LIMIT(slope,2,R_straight_slope/2.0);
  }
  for(i=begin_y;i>=N;i--){
        x=slope*(i-end_y)+end_x;
        line[i]=LIMIT(x,93,0);
    }
}
/*---------------------------------------------------------------
 ����    �ܡ�����б�ʲ��ߺ���
 ����    ����x1,y1Ϊ����ĵ�,x2,y2Ϊ����ĵ㣬lineΪ���߻���������
 ���� �� ֵ����
 ----------------------------------------------------------------*/
void Add_line(uint8 type,uint8 line[], uint8 x1, uint8 y1,uint8 x2, uint8 y2)
{
  uint8 i=0,x=0;
  float slope=0;
  slope = x1 - x2;
  slope = slope/(y1-y2);
  if(type==1) //����
  {
   slope =LIMIT(slope,L_straight_slope/2,-1.4);
  }
  else if(type==2)  //����
  {
   slope =LIMIT(slope,1.4,R_straight_slope/2);
  }
  for(i=y2;i>y1;i--){
        x=slope*(i-y2)+x2;
        line[i]=LIMIT(x,93,0);
    }
}
/*---------------------------------------------------------------
 ����    �ܡ���С���˷����ߺ���
 ����    ����begin��endΪ��С���˷�����б�ʵĳ��ȣ�begin_y��end_yΪ��Ҫ���ߵĳ��ȣ�lineΪ���߻���������
 ���� �� ֵ����
 ----------------------------------------------------------------*/
void Add_line_other(uint8 type,uint8 line[], uint8 begin, uint8 end,uint8 begin_y,uint8 end_y)
{
  uint8 i=0,temp=0;
  float slope=0;
  slope =Slope_calculate(begin,end,line);
  if(type==1) //����
  {
   slope =LIMIT(slope,L_straight_slope,-1.2);
  }
  else if(type==2)  //����
  {
   slope =LIMIT(slope,1.2,R_straight_slope);
  }

    for(i=begin_y;i<end_y;i++){
        temp = (char)((i - begin) * slope + line[begin]);//ͨ��б�����㲹�ߵ�λ��
        line[i]=LIMIT(temp,93,0);
    }
}
/*---------------------------------------------------------------
 ����    �ܡ���С���˷����ߺ���
 ����    ����begin��endΪ��С���˷�����б�ʵĳ��ȣ�begin_y��end_yΪ��Ҫ���ߵĳ��ȣ�lineΪ���߻���������
 ���� �� ֵ����
 ----------------------------------------------------------------*/
void Add_line_other_plan(uint8 type,uint8 line[],uint8 out_line[], uint8 begin, uint8 end,uint8 begin_y,uint8 end_y)
{
  uint8 i=0,temp=0;
  float slope=0;
  slope =Slope_calculate(begin,end,line);
  if(type==1) //����
  {
   slope =LIMIT(slope,L_straight_slope/2.0,-2);
  }
  else if(type==2)  //����
  {
   slope =LIMIT(slope,2,R_straight_slope/2.0);
  }
  for(i=begin_y;i<end_y;i++){
  temp = (char)((i - begin) * slope + line[begin]);//ͨ��б�����㲹�ߵ�λ��
  out_line[i]=LIMIT(temp,93,0);
  }
}
/*---------------------------------------------------------------
 ����    �ܡ���С���˷����ߺ���
 ����    ����endΪ�������㣬begin_y��end_yΪ��Ҫ���ߵĳ��ȣ�lineΪ���߻��������飬slopeΪ����б��
 ���� �� ֵ����
 ----------------------------------------------------------------*/
void Add_line_constant(uint8 type,uint8 line[],uint8 begin_y,uint8 end_y,uint8 end_x,uint8 end_Y,float slope)
{
  uint8 i=0,temp=0;
  float temp_slope=slope;
  if(type==1) //����
  {
      temp_slope =LIMIT(temp_slope,L_straight_slope/2.0,-2);
  }
  else if(type==2)  //����
  {
      temp_slope =LIMIT(temp_slope,2,R_straight_slope/2.0);
  }
  for(i=begin_y;i<=end_y;i++){
       temp = (char)((i - end_Y) * temp_slope + end_x);//ͨ��б�����㲹�ߵ�λ��
        line[i]=LIMIT(temp,93,0);
  }
}
/*---------------------------------------------------------------
 ����    �ܡ������������ϵ������󳵵�ת��ƫ��(ȡͼ�ε���50��)
 ����    �������ˮƽ���,���Ȩֵ
 ���� �� ֵ��ת��ƫ��
 ----------------------------------------------------------------*/
float turn_error(float inductor_error)
{
  int8 i = 0;
  float error = 0;
  float Line_Ratio_sum=0;      //����ϵ����
  uint8 Hightest=20;
  uint8 end=start_line_point_row;
  uint8 mid_line=46;          //Ĭ���������
  float Mid_slope=0;
  static int16 Line_Ratio[60]={
                              1,1,1,1,1,1,1,1,1,1,
                              1,1,1,1,1,1,1,1,1,1,
                              4,4,4,4,4,5,5,5,5,5,
                              13,13,13,13,13,15,17,17,17,17,
                              17,17,17,19,19,16,16,16,16,10,
                              10,10,10,10,10,10,10,10,10,10
                            };
  static int16 Ring_Ratio[60]={                         //Բ��Ȩֵ�����������ױ�
                              1,1,1,1,1,1,1,1,1,1,
                              1,1,1,1,1,1,1,1,1,1,
                              1,1,1,1,1,1,1,1,1,1,
                              12,12,12,12,12,10,10,10,10,10,
                              13,13,13,13,13,14,14,14,14,14,
                              10,10,10,10,10,10,10,10,10,10
                            };

  if(REPIRE_LINE)     //��������ˣ�����ƽ�Ʊ߽��ˣ�����ȷ���������������
  {
      Mid_slope=Slope_calculate(45, start_line_point_row, Mid_Line);
      if(Mid_slope>0.15)   //������ƫ
      {
          if(Mid_Line[start_line_point_row]>47)
          {
              mid_line= Mid_Line[start_line_point_row];
          }
      }
      if(Mid_slope<-0.15)//������ƫ
      {
          if(Mid_Line[start_line_point_row]<47)
          {
              mid_line= Mid_Line[start_line_point_row];
          }
      }
  }

  if(L_CirCle_State==6)   //�����Ƕ���Բ�����⴦��
  {
      for(i = end; i>=Hightest; i--)
      {
          Line_Ratio_sum+=Ring_Ratio[i];
          error += (Mid_Line[i]  - mid_line)*Ring_Ratio[i];
      }
      Mid_slope=Slope_calculate(35, start_line_point_row, Right_Line);       //Բ���ڽ�������
  }

  else if(R_CirCle_State==6)    //�����Ƕ���Բ�����⴦��
  {
      for(i = end; i>=Hightest; i--)
        {
          Line_Ratio_sum+=Ring_Ratio[i];
          error += (Mid_Line[i]  - mid_line)*Ring_Ratio[i];
        }
      Mid_slope=Slope_calculate(35, start_line_point_row, Left_Line);       //Բ���ڽ�������
  }
  else                                                    //��������
  {
      for(i = end; i>=Hightest; i--)
        {
          Line_Ratio_sum+=Line_Ratio[i];
          error += (Mid_Line[i] - mid_line)*Line_Ratio[i];
        }
   }

  if(Break_road_state>=1) //��·�л����ѭ��
  {
   error=(-inductor_error);
  }
  else
  {
       if(L_CirCle_State==6 || R_CirCle_State==6)
       error =error/Line_Ratio_sum+Mid_slope*(-4.5);        //Բ��б�ʲ���
       else
       error =error/Line_Ratio_sum+Mid_slope*(-5);        //б�ʲ���(����б�ʲ���)
  }
  return error;
}
/*---------------------------------------------------------------
 ����    �ܡ�Ѱ�ҹյ�
 ����    ����uint8 Line[]��������    flag :  1 ����   2  ����    UD:  1�� ������ɨ   0 ��������ɨ
 ���� �� ֵ�� i������ 0��
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
                       if(Line[i]>=Line[i+2] && Line[i]>=Line[i+1] && Line[i]-Line[i-1]>=1 && Line[i]-Line[i-2]>=1)//�ҵ��յ�
                           {
                                                                 hang= i;     //���ظùյ����
                                                                 break;
                           }
                  }
            break;
        case 2:
            for(uint8 i = end;i>=begin;i--)
            {
                    if(Line[i-1]-Line[i]>=1 && Line[i+1]>=Line[i] && Line[i-2]-Line[i]>=1 && Line[i+2]>=Line[i])//�ҵ��ұ����йյ�
                         {
                                                         hang= i;     //���ظùյ����
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
                           if(abs(Line[i]-Line[i+1])>6 && abs(Line[i-1]-Line[i])<=2 &&  abs(Line[i-2]-Line[i])<=2 && abs(Line[i]-Line[i+2])>6)//�ҵ��յ�
                           {
                                                               hang= i;     //���ظùյ����
                                                                 break;
                           }
                  }
                  break;
              case 2:
                  for(uint8 i = begin;i<=end;i++)
                  {
                           if(abs(Line[i]-Line[i+1])>6 && abs(Line[i-1]-Line[i])<=2 &&  abs(Line[i-2]-Line[i])<=2 && abs(Line[i]-Line[i+2])>6 )//�ҵ��ұ����йյ�
                           {
                                                       hang= i;     //���ظùյ����
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
 ����    �ܡ�Ѱ�Ҷ��пհ�,��������ɨ��,Ĭ��7��ֹͣɨ��
 ����    ����typeѰ������ 1Ϊ��ߣ�2Ϊȫ��,3Ϊ�ұ�
 ���� �� ֵ�� 1�пհ� 0��
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
    if(type==1){         //��
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
    if(type==3){         //��
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
 ����    �ܡ�Ѱ�Ҷ���ͻ�����,����������ȼ�������������ɨ��
 ����    ����typeѰ������ 1Ϊ��ߣ�2Ϊȫ��,3Ϊ�ұ�
 ���� �� ֵ�� 1�а��� 0��
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
    if(type==1){         //��
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
    if(type==3){         //��
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
 ����    �ܡ�Ѱ�Ҷ��к�ɫ,��������ɨ��,Ĭ��4��ֹͣɨ��
 ����    ����typeѰ������ 1Ϊ��ߣ�2Ϊȫ��,3Ϊ�ұ�
 ���� �� ֵ�� 1�к��� 0��
 ----------------------------------------------------------------*/
uint8 Fine_heihang(uint8 type,uint8 begin ,uint8 end,uint8(*image)[image_w],uint8 hang)
{
    uint8 hei=0;
    uint8 Hang=0;
    uint8 check=0;
    if(type==2){                      //ȫ��
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
    if(type==1){         //��
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
    if(type==3){         //��
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


