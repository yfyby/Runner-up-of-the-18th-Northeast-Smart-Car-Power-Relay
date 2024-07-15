/*
 * ui_key.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "ui_key.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "zf_device_tft180.h"
#include "image.h"
#include "inductor.h"
#include "system.h"
#include "imgproc.h"
extern uint8 User_image[60][94];
extern uint8 mt9v03x_image[120][188];
bool shield_key=0;
/********************************************************************************************************************
 * ����������ѯ�������л��˵����� 8*16�����С��160*128��
 ********************************************************************************************************************/
#define KEY1                    (A8 )       //����������ǰ��
#define KEY2                    (B10 )     //��������������
#define KEY3                    (D8 )     //�����������м�����
#define KEY4                    (B12 )   //��������������
#define KEY5                    (B11 )  //��������������

void Key_init()
 {
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);     // ��ʼ�� KEY1 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);    // ��ʼ�� KEY2 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);   // ��ʼ�� KEY3 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);  // ��ʼ�� KEY4 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY5, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� KEY5 ���� Ĭ�ϸߵ�ƽ ��������
 }

 /*********��0��***********/
void fun_0()
{
    shield_key=0;
    tft180_show_string(0,0,"USTH");
    if(flag_offset==1)  //������ȥ��Ư�ɹ��ɹ�
    tft180_show_string(0,16,"Car can Start");
    if(flag_offset==0)
    {
    tft180_show_string(0,32,"Car can't Start");
    if(flag_offset==1)
    tft180_clear();
    }
    tft180_show_string(0,48,"Enter to menu");
    tft180_show_string(0,64,"flag_offset=");
    tft180_show_uint(100,64,flag_offset,1);
}
/*********��1��***********/
void fun_a1()
{
   shield_key=0;
   tft180_show_string(0,16,">");
   tft180_show_string(8,16,"show and mode");             //��ʾ
   tft180_show_string(8,32,"para_adjust1");    //������1
   tft180_show_string(8,48,"para_adjust2");   //������2
   tft180_show_string(8,64,"return");
}

void fun_b1()
{
   shield_key=0;
   tft180_show_string(0,32,">");
   tft180_show_string(8,16,"show and mode");             //��ʾ
   tft180_show_string(8,32,"para_adjust1");    //������1
   tft180_show_string(8,48,"para_adjust2");   //������2
   tft180_show_string(8,64,"return");
}

void fun_c1()
{
   shield_key=0;
   tft180_show_string(0,48,">");
   tft180_show_string(8,16,"show and mode");             //��ʾ
   tft180_show_string(8,32,"para_adjust1");    //������1
   tft180_show_string(8,48,"para_adjust2");   //������2
   tft180_show_string(8,64,"return");
}

void fun_d1()
{
   shield_key=0;
   tft180_show_string(0,64,">");
   tft180_show_string(8,16,"show and mode");             //��ʾ
   tft180_show_string(8,32,"para_adjust1");    //������1
   tft180_show_string(8,48,"para_adjust2");   //������2
   tft180_show_string(8,64,"return");
}

/*********��2��***********/
void fun_a21()
{
   shield_key=0;
   tft180_show_string(0,16,">");
   tft180_show_string(8,16,"mode select");       //ģʽѡ��
   tft180_show_string(8,32,"user show");        //��ʾʹ�õ�����
   tft180_show_string(8,48,"blank show");     //�հ׻���
   tft180_show_string(8,64,"return");
}

void fun_a22()
{
   shield_key=0;
   tft180_show_string(0,32,">");
   tft180_show_string(8,16,"mode select");   //ģʽѡ��
   tft180_show_string(8,32,"user show");    //��ʾʹ�õ�����
   tft180_show_string(8,48,"blank show"); //�հ׻���
   tft180_show_string(8,64,"return");
}

void fun_a23()
{
   shield_key=0;
   tft180_show_string(0,48,">");
   tft180_show_string(8,16,"mode select");     //ģʽѡ��
   tft180_show_string(8,32,"user show");    //��ʾ��������ʱ��ʾ��ͼ�������1
   tft180_show_string(8,48,"blank show");     //�հ׻���
   tft180_show_string(8,64,"return");
}

void fun_a24()
{
   shield_key=0;
   tft180_show_string(0,64,">");
   tft180_show_string(8,16,"mode select");     //ģʽѡ��
   tft180_show_string(8,32,"user show");    //��ʾ��������ʱ��ʾ��ͼ�������1
   tft180_show_string(8,48,"blank show");     //�հ׻���
   tft180_show_string(8,64,"return");
}

void fun_b21()
{
    shield_key=0;
    tft180_show_string(0,16,">");
    tft180_show_string(8,16,"speed adjust");            //�������ٶȵ���
    tft180_show_string(8,32,"break road steer");                   //���pd����
    tft180_show_string(8,48,"L_charge_tres");             //��Բ����pd����
    tft180_show_string(8,64,"return");
}

void fun_b22()
{
    shield_key=0;
    tft180_show_string(0,32,">");
    tft180_show_string(8,16,"speed adjust");            //�������ٶȵ���
    tft180_show_string(8,32,"break road steer");                   //���pd����
    tft180_show_string(8,48,"L_charge_tres");             //��Բ����pd����
    tft180_show_string(8,64,"return");
}

void fun_b23()
{
    shield_key=0;
    tft180_show_string(0,48,">");
    tft180_show_string(8,16,"speed adjust");            //�������ٶȵ���
    tft180_show_string(8,32,"break road steer");      //���pd����
    tft180_show_string(8,48,"L_charge_tres");             //��Բ����pd����
    tft180_show_string(8,64,"return");
}

void fun_b24()
{
    shield_key=0;
    tft180_show_string(0,64,">");
    tft180_show_string(8,16,"speed adjust");            //�������ٶȵ���
    tft180_show_string(8,32,"break road steer");           //���pd����
    tft180_show_string(8,48,"L_charge_tres");             //��Բ����pd����
    tft180_show_string(8,64,"return");
}

void fun_c21()
{
    shield_key=0;
    tft180_show_string(0,16,">");
    tft180_show_string(8,16,"R_charge_tres");                    //��Բ��pd����
    tft180_show_string(8,32,"B_road_thers");                   //��·��ֵ����ֵ����
    tft180_show_string(8,48,"slope cross");                   //���������հ�ҳ
    tft180_show_string(8,64,"return");
}

void fun_c22()
{
    shield_key=0;
    tft180_show_string(0,32,">");
    tft180_show_string(8,16,"R_charge_tres");                    //��Բ��pd����
    tft180_show_string(8,32,"B_road_thers");                   //��·��ֵ����ֵ����
    tft180_show_string(8,48,"slope cross");                   //���������հ�ҳ
    tft180_show_string(8,64,"return");
}

void fun_c23()
{
    shield_key=0;
    tft180_show_string(0,48,">");
    tft180_show_string(8,16,"R_charge_tres");                    //��Բ��pd����
    tft180_show_string(8,32,"B_road_thers");                   //��·��ֵ����ֵ����
    tft180_show_string(8,48,"slope cross");                   //���������հ�ҳ
    tft180_show_string(8,64,"return");
}

void fun_c24()
{
    shield_key=0;
    tft180_show_string(0,64,">");
    tft180_show_string(8,16,"R_charge_tres");                    //��Բ��pd����
    tft180_show_string(8,32,"B_road_thers");                   //��·��ֵ����ֵ����
    tft180_show_string(8,48,"slope cross");                   //���������հ�ҳ
    tft180_show_string(8,64,"return");
}
/*********��3��***********/
//��С��ģʽ����ѡ��
bool cut_dode=0;
void fun_a31()
{

    //����Ԫ�ؽ׶�Ϊ101
    //ǰ����������������Ϊ100
    //˫���ܳ�Ϊ110
    shield_key=1;
    if(!gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
     {
         system_delay_ms(10);//����
         //Out_door_typeĬ��1�������
         if(!gpio_get_level(KEY4))
         {
             if(cut_dode==0)
             Out_door_type=2;
             else
             Out_door_type=1;
             while(!gpio_get_level(KEY4));//���ּ��
         }
         //OFF_Charge_ResueĬ��1�����о�Ԯ
         if(!gpio_get_level(KEY5))
         {
             if(cut_dode==0)
             OFF_Charge_Resue=0;
             else
             OFF_Charge_Resue=1;
             while(!gpio_get_level(KEY5));//���ּ��
         }
         //OFF_Element_testĬ��0��������Ԫ�ز���
         if(!gpio_get_level(KEY1))
         {
             if(cut_dode==0)
             OFF_Element_test=1;
             else
             OFF_Element_test=0;
             while(!gpio_get_level(KEY1));//���ּ��
         }
         if( !gpio_get_level(KEY2))
          {
             cut_dode=!cut_dode;   //ȡ��
            while(!gpio_get_level(KEY2));//���ּ��
          }
     }
     tft180_show_string(0,0,"Out_door_type");
     tft180_show_uint(108, 0, Out_door_type, 1);

     tft180_show_string(0,16,"OFF_Charge_Resue");
     tft180_show_uint(135,16, OFF_Charge_Resue, 1);

     tft180_show_string(0,32,"OFF_Element_test");
     tft180_show_uint(135,32, OFF_Element_test, 1);
     if(Out_door_type==1)
     {
       tft180_show_string(0,48,"out left door");
     }
     else if(Out_door_type==2)
     {
       tft180_show_string(0,48,"out right door");
     }

     if(OFF_Charge_Resue==1)
     {
       tft180_show_string(0,64,"begin charge");
     }
     else if(OFF_Charge_Resue==0)
     {
       tft180_show_string(0,64,"not charge");
     }

     if(OFF_Element_test==1)
     {
       tft180_show_string(0,80,"open element_test");
     }
     else if(OFF_Element_test==0)
     {
       tft180_show_string(0,80,"not element_test");
     }

     if(Out_door_type==1 && OFF_Charge_Resue==1 && OFF_Element_test==0)
     {
       tft180_show_string(0,96,"Left car chrage");
     }
     else if(Out_door_type==2 && OFF_Charge_Resue==1 && OFF_Element_test==0)
     {
         tft180_show_string(0,96,"Right car chrage");
     }
     else if(Out_door_type==2 && OFF_Charge_Resue==0 && OFF_Element_test==0)
     {
         tft180_show_string(0,96,"Right car go    ");
     }
     else if(Out_door_type==1 && OFF_Charge_Resue==0 && OFF_Element_test==0)
     {
         tft180_show_string(0,96,"Left car go     ");
     }
     else if(OFF_Element_test==1)
     {
         tft180_show_string(0,96,"only element_test");
     }
}

void fun_a32()
{
    shield_key=1;
    if( !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
      {
          system_delay_ms(10);//����
          //���Ƽ�С�����ٶȣ��������������ٶ�
          if(!gpio_get_level(KEY4))
          {
              Break_road_speed+=0.05;
              while(!gpio_get_level(KEY4));//���ּ��
          }
          if(!gpio_get_level(KEY5))
          {
              Break_road_speed-=0.05;
              while(!gpio_get_level(KEY5));//���ּ��
          }
      }

    //��������ʾ�Ļ���
    draw_point_red(46, 57);
    tft180_show_string(0,60,"Left_Car");
    tft180_show_float(80,60,Left_Car_Speed, 1,2);
    tft180_show_string(0,76,"Right_Car");
    tft180_show_float(80,76,Right_Car_Speed,1,2);
    tft180_show_int(100,0,Moto_Out_Left,4);
    tft180_show_int(100,16,Moto_Out_Right,4);
    tft180_show_int(100,32,streeing_duty,3);
    tft180_show_float(100,48,final_err,2,2);
    tft180_show_int(0, 92, Inductor_Value[0], 3);
    tft180_show_int(32,92, Inductor_Value[1], 3);
    tft180_show_int(64,92, Inductor_Value[2], 3);
    tft180_show_int(96,92, Inductor_Value[3], 3);
    tft180_show_float(0, 108,Level_Deviation, 1,1);
    tft180_show_uint(48, 108,hightest, 2);
    tft180_show_uint(72,108,dl1a_distance_mm,4);
    tft180_show_float(110,108,Break_road_speed,1,2);
    draw_line(User_image); //�����߼��߽�
    tft180_show_gray_image(0, 0, (const uint8 *)User_image, 94, 60, 94, 60,0);            //��ʾ��ֵ��ͼ��
}

void fun_a33()
{
    //�հ�ҳ��
   //    tft180_show_string(94,0,"L");
    //    tft180_show_uint(104,0,L_CirCle_State,1);
    //    tft180_show_string(94,16,"R");
      //    tft180_show_uint(104,16,R_CirCle_State,1);
          //    draw_line(User_image); //�����߼��߽�
       //    tft180_show_gray_image(0, 0,(const uint8 *)User_image, 94, 60, 94, 60,0);            //��ʾ��ֵ��ͼ��
//    tft180_show_float(102,32,Ring_add_angle,3,2);
//    tft180_show_float(102,48,ADD_angle,3,2);
}
//�޸ĳ���
void fun_b31()
{
  shield_key=1;
  if( !gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
   {
       system_delay_ms(10);//����
       //���Ƽ�С�����ٶȣ��������������ٶ�
       if(!gpio_get_level(KEY4))
       {
           straightaway_speed+=0.05;
           while(!gpio_get_level(KEY4));//���ּ��
       }
       if(!gpio_get_level(KEY5))
       {
           straightaway_speed-=0.05;
           while(!gpio_get_level(KEY5));//���ּ��
       }
      //���Ƽ�С�����ٶȣ��������������ٶ�
       if( !gpio_get_level(KEY1))
       {
           curve_speed+=0.05;
           while(!gpio_get_level(KEY1));//���ּ��
       }

       if( !gpio_get_level(KEY2))
       {
           curve_speed-=0.05;
           while(!gpio_get_level(KEY2));//���ּ��
       }
   }
     tft180_show_string(0,16,"zhi dao=");
     tft180_show_string(0,48,"wan dao=");

     tft180_show_float (70, 16, straightaway_speed, 1, 3);  //ֱ���ٶ�
     tft180_show_float (70, 48, curve_speed, 1, 3);        //����ٶ�
}
//�޸Ķ�·pid
void fun_b32()
{
     shield_key=1;
     if( !gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
      {
          system_delay_ms(10);//����
       if( !gpio_get_level(KEY4))
       {
           use_inductor_pd[0]+=0.25;
           while(!gpio_get_level(KEY4));//���ּ��
       }

       if( !gpio_get_level(KEY5))
       {
           use_inductor_pd[0]-=0.25;
           while(!gpio_get_level(KEY5));//���ּ��
       }

   //���Ƽ�С���d���������Ӷ��d
       if( !gpio_get_level(KEY1))
       {
           use_inductor_pd[1]+=0.25;
           while(!gpio_get_level(KEY1));//���ּ��
       }

       if( !gpio_get_level(KEY2))
       {
           use_inductor_pd[1]-=0.25;
           while(!gpio_get_level(KEY2));//���ּ��
       }
     }
     tft180_show_string(0,16,"in_P=");
     tft180_show_string(0,48,"in_D=");
     tft180_show_float (50,16, use_inductor_pd[0], 2, 3);
     tft180_show_float (50,48, use_inductor_pd[1], 2, 3);
}
//�޸����Ԯ��ֵ
void fun_b33()
{
     shield_key=1;
     if( !gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
      {
          system_delay_ms(10);//����

       if( !gpio_get_level(KEY4))
       {
           L_charge_big_thes+=1;
           while(!gpio_get_level(KEY4));//���ּ��
       }

       if( !gpio_get_level(KEY5))
       {
           L_charge_big_thes-=1;
           while(!gpio_get_level(KEY5));//���ּ��
       }
      //���Ƽ�С����������
       if( !gpio_get_level(KEY1))
       {
           L_charge_small_thes+=1;
           while(!gpio_get_level(KEY1));//���ּ��
       }

       if( !gpio_get_level(KEY2))
       {
           L_charge_small_thes-=1;
           while(!gpio_get_level(KEY2));//���ּ��
       }
     }

     tft180_show_int(0, 80, Inductor_Value[0], 3);
     tft180_show_int(32, 80, Inductor_Value[1], 3);
     tft180_show_int(64, 80,  Inductor_Value[2], 3);
     tft180_show_int(96, 80,  Inductor_Value[3], 3);

     tft180_show_string(0,16,"L_big=");
     tft180_show_string(0,48,"L_sml=");
     tft180_show_int(58, 16, L_charge_big_thes, 2);
     tft180_show_int(58, 48, L_charge_small_thes, 2);
}
//�޸��Ҿ�Ԯ��ֵ
void fun_c31()
{
     shield_key=1;

     if( !gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
      {
          system_delay_ms(10);//����
       if( !gpio_get_level(KEY4))
       {
           R_charge_big_thes+=1;
           while(!gpio_get_level(KEY4));//���ּ��
       }

       if( !gpio_get_level(KEY5))
       {
           R_charge_big_thes-=1;
           while(!gpio_get_level(KEY5));//���ּ��
       }

       if( !gpio_get_level(KEY1))
       {
           R_charge_small_thes+=1;
           while(!gpio_get_level(KEY1));//���ּ��
       }

       if( !gpio_get_level(KEY2))
       {
           R_charge_small_thes-=1;
           while(!gpio_get_level(KEY2));//���ּ��
       }
     }

     tft180_show_int(0, 80, Inductor_Value[0], 3);
     tft180_show_int(32, 80, Inductor_Value[1], 3);
     tft180_show_int(64, 80,  Inductor_Value[2], 3);
     tft180_show_int(96, 80,  Inductor_Value[3], 3);

     tft180_show_string(0,16,"R_big=");
     tft180_show_string(0,48,"R_sml=");
     tft180_show_int(58, 16, R_charge_big_thes, 2);
     tft180_show_int(58, 48, R_charge_small_thes, 2);
}
//�޸Ķ�·��ֵ����
void fun_c32()
{
      shield_key=1;
     //���Ƽ�С��·��ֵ����ֵ���������Ӷ�·��ֵ����ֵ
     if(!gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
      {
          system_delay_ms(10);//����
       if( !gpio_get_level(KEY4))
       {
           Break_road_add_thres+=2;
           while(!gpio_get_level(KEY4));//���ּ��
       }

       if( !gpio_get_level(KEY5))
       {
          Break_road_add_thres-=2;
           while(!gpio_get_level(KEY5));//���ּ��
       }
      //���Ƽ�С��·��Ⱥ���ֵ���������Ӷ�·��Ⱥ���ֵ
       if( !gpio_get_level(KEY1))
       {
           bottom_gray_threa+=2;
           while(!gpio_get_level(KEY1));//���ּ��
       }
       if( !gpio_get_level(KEY2))
       {
           bottom_gray_threa-=2;
           while(!gpio_get_level(KEY2));//���ּ��
       }
     }
     tft180_show_string(0,16,"Break_road");
     tft180_show_string(0,48,"bottom_gray");

     tft180_show_uint (96, 16, Break_road_add_thres, 2);
     tft180_show_uint (96, 48,  bottom_gray_threa, 2);
}
//�޸�б��ʮ�ֵ���ֵ
void fun_c33()
{
//    extern float use_slope_cross[2];  //��б��ʮ��5.8,3.9
//    extern float use_slope_cross_l[2];  //��б��ʮ��
    shield_key=1;
    //���Ƽ�С��·��ֵ����ֵ���������Ӷ�·��ֵ����ֵ
    if(!gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
     {
         system_delay_ms(10);//����
      if( !gpio_get_level(KEY4))
      {
          use_slope_cross[0]+=0.2;
          use_slope_cross_l[0]+=0.2;
          while(!gpio_get_level(KEY4));//���ּ��
      }

      if( !gpio_get_level(KEY5))
      {
          use_slope_cross[0]-=0.2;
          use_slope_cross_l[0]-=0.2;
          while(!gpio_get_level(KEY5));//���ּ��
      }

  //���Ƽ�С��·��Ⱥ���ֵ���������Ӷ�·��Ⱥ���ֵ
      if( !gpio_get_level(KEY1))
      {
          use_slope_cross[1]+=0.2;
          use_slope_cross_l[1]+=0.2;
          while(!gpio_get_level(KEY1));//���ּ��
      }
      if( !gpio_get_level(KEY2))
      {
          use_slope_cross[1]-=0.4;
          use_slope_cross_l[1]-=0.4;
          while(!gpio_get_level(KEY2));//���ּ��
      }
    }

    tft180_show_string(0,0,"r_cross_p=");
    tft180_show_string(0,16,"r_cross_d=");

    tft180_show_string(0,48,"l_cross_p=");
    tft180_show_string(0,64,"l_cross_d=");

    tft180_show_float (85, 0, use_slope_cross[0], 2,2);
    tft180_show_float (85, 16, use_slope_cross[1], 2,2);

    tft180_show_float (85, 48, use_slope_cross_l[0], 2,2);
    tft180_show_float (85, 64, use_slope_cross_l[1], 2,2);
}


key_table table[30]=
{
    //  current;        //��ǰ������
    //  up;            //���Ϸ�������
    //  down;         //���·�������
    //  enter;       //ȷ��������
    //  void (*current_operation)();            //����ָ��ִ�к�����ָ�뺯��
    //��0��
    {0,0,0,1,(*fun_0)},                   //�м�����Ϊ0˵�����Ϸ������·���Ч

    //��1��
    {1,4,2, 5,(*fun_a1)},
    {2,1,3, 9,(*fun_b1)},
    {3,2,4,13,(*fun_c1)},
    {4,3,1, 0,(*fun_d1)},

    //��2��
    {5,8,6,17,(*fun_a21)},
    {6,5,7,18,(*fun_a22)},
    {7,6,8,19,(*fun_a23)},
    {8,7,5, 1,(*fun_a24)},

    { 9,12,10,20,(*fun_b21)},
    {10, 9,11,21,(*fun_b22)},
    {11,10,12,22,(*fun_b23)},
    {12,11, 9, 2,(*fun_b24)},

    {13,16,14,23,(*fun_c21)},
    {14,13,15,24,(*fun_c22)},
    {15,14,16,25,(*fun_c23)},
    {16,15,13, 3,(*fun_c24)},

    //��3��
    {17,17,17,5,(*fun_a31)},
    {18,18,18,6,(*fun_a32)},
    {19,19,19,7,(*fun_a33)},

    {20,20,20, 9,(*fun_b31)},
    {21,21,21,10,(*fun_b32)},
    {22,22,22,11,(*fun_b33)},

    {23,23,23,13,(*fun_c31)},
    {24,24,24,14,(*fun_c32)},
    {25,25,25,15,(*fun_c33)},

};

void (*current_operation_index)();            //����һ������ָ��
void KEY()                                   //ʹ�ò˵��Ļ���ֱ����while����øú���
 {
     static uint8 func_index = 0; //��ʼ��ʾ��ӭ����
     if( !gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY3) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
     {
       system_delay_ms(10);//����
       if( !gpio_get_level(KEY1) && shield_key==0)
       {
           func_index = table[func_index].up;    //���Ϸ�
           tft180_clear ();
           while(!gpio_get_level(KEY1));//���ּ��
       }
       if( !gpio_get_level(KEY2) && shield_key==0)
       {
           func_index = table[func_index].down;    //���·�
           tft180_clear ();
           while(!gpio_get_level(KEY2));
       }
       if( !gpio_get_level(KEY3))
       {
           func_index = table[func_index].enter;    //ȷ��,�����м�����ȥ
           tft180_clear ();
           while(!gpio_get_level(KEY3));
       }
       //KEY4��δ����ڲ������ٶȻ����Ե�ʱ�����ע�͵����Ͼ�������ʱ�򼸺�û��ʱ����ٶȻ�
//        if( !gpio_get_level(KEY4))
//        {
////            if(OFF_Speed_test)      //�����ٶȻ�����
////            {
////              Speed_PID_debug=1;
////              Speed_PID_count=0;
////            }
//            tft180_clear ();
//            while(!gpio_get_level(KEY4));
//        }
   }
       //����������ִ�е�ָ��
       current_operation_index = table[func_index].current_operation;
       (*current_operation_index)();//ִ�е�ǰ��������
 }


