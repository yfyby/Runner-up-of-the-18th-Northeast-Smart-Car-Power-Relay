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
 * 采用数组轮询方法来切换菜单界面 8*16字体大小（160*128）
 ********************************************************************************************************************/
#define KEY1                    (A8 )       //五向摁键向前推
#define KEY2                    (B10 )     //五向摁键往后推
#define KEY3                    (D8 )     //五向摁键向中间摁下
#define KEY4                    (B12 )   //五向摁键向左推
#define KEY5                    (B11 )  //五向摁键向右推

void Key_init()
 {
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);     // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);    // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);   // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);  // 初始化 KEY4 输入 默认高电平 上拉输入
    gpio_init(KEY5, GPI, GPIO_HIGH, GPI_PULL_UP); // 初始化 KEY5 输入 默认高电平 上拉输入
 }

 /*********第0层***********/
void fun_0()
{
    shield_key=0;
    tft180_show_string(0,0,"USTH");
    if(flag_offset==1)  //陀螺仪去零漂成功成功
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
/*********第1层***********/
void fun_a1()
{
   shield_key=0;
   tft180_show_string(0,16,">");
   tft180_show_string(8,16,"show and mode");             //显示
   tft180_show_string(8,32,"para_adjust1");    //调参数1
   tft180_show_string(8,48,"para_adjust2");   //调参数2
   tft180_show_string(8,64,"return");
}

void fun_b1()
{
   shield_key=0;
   tft180_show_string(0,32,">");
   tft180_show_string(8,16,"show and mode");             //显示
   tft180_show_string(8,32,"para_adjust1");    //调参数1
   tft180_show_string(8,48,"para_adjust2");   //调参数2
   tft180_show_string(8,64,"return");
}

void fun_c1()
{
   shield_key=0;
   tft180_show_string(0,48,">");
   tft180_show_string(8,16,"show and mode");             //显示
   tft180_show_string(8,32,"para_adjust1");    //调参数1
   tft180_show_string(8,48,"para_adjust2");   //调参数2
   tft180_show_string(8,64,"return");
}

void fun_d1()
{
   shield_key=0;
   tft180_show_string(0,64,">");
   tft180_show_string(8,16,"show and mode");             //显示
   tft180_show_string(8,32,"para_adjust1");    //调参数1
   tft180_show_string(8,48,"para_adjust2");   //调参数2
   tft180_show_string(8,64,"return");
}

/*********第2层***********/
void fun_a21()
{
   shield_key=0;
   tft180_show_string(0,16,">");
   tft180_show_string(8,16,"mode select");       //模式选择
   tft180_show_string(8,32,"user show");        //显示使用的数据
   tft180_show_string(8,48,"blank show");     //空白画面
   tft180_show_string(8,64,"return");
}

void fun_a22()
{
   shield_key=0;
   tft180_show_string(0,32,">");
   tft180_show_string(8,16,"mode select");   //模式选择
   tft180_show_string(8,32,"user show");    //显示使用的数据
   tft180_show_string(8,48,"blank show"); //空白画面
   tft180_show_string(8,64,"return");
}

void fun_a23()
{
   shield_key=0;
   tft180_show_string(0,48,">");
   tft180_show_string(8,16,"mode select");     //模式选择
   tft180_show_string(8,32,"user show");    //显示经常调试时显示的图像和数据1
   tft180_show_string(8,48,"blank show");     //空白画面
   tft180_show_string(8,64,"return");
}

void fun_a24()
{
   shield_key=0;
   tft180_show_string(0,64,">");
   tft180_show_string(8,16,"mode select");     //模式选择
   tft180_show_string(8,32,"user show");    //显示经常调试时显示的图像和数据1
   tft180_show_string(8,48,"blank show");     //空白画面
   tft180_show_string(8,64,"return");
}

void fun_b21()
{
    shield_key=0;
    tft180_show_string(0,16,">");
    tft180_show_string(8,16,"speed adjust");            //左右轮速度调整
    tft180_show_string(8,32,"break road steer");                   //舵机pd调整
    tft180_show_string(8,48,"L_charge_tres");             //左圆环内pd调整
    tft180_show_string(8,64,"return");
}

void fun_b22()
{
    shield_key=0;
    tft180_show_string(0,32,">");
    tft180_show_string(8,16,"speed adjust");            //左右轮速度调整
    tft180_show_string(8,32,"break road steer");                   //舵机pd调整
    tft180_show_string(8,48,"L_charge_tres");             //左圆环内pd调整
    tft180_show_string(8,64,"return");
}

void fun_b23()
{
    shield_key=0;
    tft180_show_string(0,48,">");
    tft180_show_string(8,16,"speed adjust");            //左右轮速度调整
    tft180_show_string(8,32,"break road steer");      //舵机pd调整
    tft180_show_string(8,48,"L_charge_tres");             //左圆环内pd调整
    tft180_show_string(8,64,"return");
}

void fun_b24()
{
    shield_key=0;
    tft180_show_string(0,64,">");
    tft180_show_string(8,16,"speed adjust");            //左右轮速度调整
    tft180_show_string(8,32,"break road steer");           //舵机pd调整
    tft180_show_string(8,48,"L_charge_tres");             //左圆环内pd调整
    tft180_show_string(8,64,"return");
}

void fun_c21()
{
    shield_key=0;
    tft180_show_string(0,16,">");
    tft180_show_string(8,16,"R_charge_tres");                    //右圆环pd调整
    tft180_show_string(8,32,"B_road_thers");                   //断路二值化阈值调整
    tft180_show_string(8,48,"slope cross");                   //启动发车空白页
    tft180_show_string(8,64,"return");
}

void fun_c22()
{
    shield_key=0;
    tft180_show_string(0,32,">");
    tft180_show_string(8,16,"R_charge_tres");                    //右圆环pd调整
    tft180_show_string(8,32,"B_road_thers");                   //断路二值化阈值调整
    tft180_show_string(8,48,"slope cross");                   //启动发车空白页
    tft180_show_string(8,64,"return");
}

void fun_c23()
{
    shield_key=0;
    tft180_show_string(0,48,">");
    tft180_show_string(8,16,"R_charge_tres");                    //右圆环pd调整
    tft180_show_string(8,32,"B_road_thers");                   //断路二值化阈值调整
    tft180_show_string(8,48,"slope cross");                   //启动发车空白页
    tft180_show_string(8,64,"return");
}

void fun_c24()
{
    shield_key=0;
    tft180_show_string(0,64,">");
    tft180_show_string(8,16,"R_charge_tres");                    //右圆环pd调整
    tft180_show_string(8,32,"B_road_thers");                   //断路二值化阈值调整
    tft180_show_string(8,48,"slope cross");                   //启动发车空白页
    tft180_show_string(8,64,"return");
}
/*********第3层***********/
//对小车模式进行选择
bool cut_dode=0;
void fun_a31()
{

    //测试元素阶段为101
    //前车单独跑赛道测试为100
    //双车跑车为110
    shield_key=1;
    if(!gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
     {
         system_delay_ms(10);//消抖
         //Out_door_type默认1，出左库
         if(!gpio_get_level(KEY4))
         {
             if(cut_dode==0)
             Out_door_type=2;
             else
             Out_door_type=1;
             while(!gpio_get_level(KEY4));//松手检测
         }
         //OFF_Charge_Resue默认1，进行救援
         if(!gpio_get_level(KEY5))
         {
             if(cut_dode==0)
             OFF_Charge_Resue=0;
             else
             OFF_Charge_Resue=1;
             while(!gpio_get_level(KEY5));//松手检测
         }
         //OFF_Element_test默认0，不进行元素测速
         if(!gpio_get_level(KEY1))
         {
             if(cut_dode==0)
             OFF_Element_test=1;
             else
             OFF_Element_test=0;
             while(!gpio_get_level(KEY1));//松手检测
         }
         if( !gpio_get_level(KEY2))
          {
             cut_dode=!cut_dode;   //取反
            while(!gpio_get_level(KEY2));//松手检测
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
          system_delay_ms(10);//消抖
          //左推减小左轮速度，右推增加左轮速度
          if(!gpio_get_level(KEY4))
          {
              Break_road_speed+=0.05;
              while(!gpio_get_level(KEY4));//松手检测
          }
          if(!gpio_get_level(KEY5))
          {
              Break_road_speed-=0.05;
              while(!gpio_get_level(KEY5));//松手检测
          }
      }

    //调试用显示的画面
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
    draw_line(User_image); //画中线及边界
    tft180_show_gray_image(0, 0, (const uint8 *)User_image, 94, 60, 94, 60,0);            //显示二值化图像
}

void fun_a33()
{
    //空白页面
   //    tft180_show_string(94,0,"L");
    //    tft180_show_uint(104,0,L_CirCle_State,1);
    //    tft180_show_string(94,16,"R");
      //    tft180_show_uint(104,16,R_CirCle_State,1);
          //    draw_line(User_image); //画中线及边界
       //    tft180_show_gray_image(0, 0,(const uint8 *)User_image, 94, 60, 94, 60,0);            //显示二值化图像
//    tft180_show_float(102,32,Ring_add_angle,3,2);
//    tft180_show_float(102,48,ADD_angle,3,2);
}
//修改车速
void fun_b31()
{
  shield_key=1;
  if( !gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
   {
       system_delay_ms(10);//消抖
       //左推减小左轮速度，右推增加左轮速度
       if(!gpio_get_level(KEY4))
       {
           straightaway_speed+=0.05;
           while(!gpio_get_level(KEY4));//松手检测
       }
       if(!gpio_get_level(KEY5))
       {
           straightaway_speed-=0.05;
           while(!gpio_get_level(KEY5));//松手检测
       }
      //上推减小右轮速度，下推增加右轮速度
       if( !gpio_get_level(KEY1))
       {
           curve_speed+=0.05;
           while(!gpio_get_level(KEY1));//松手检测
       }

       if( !gpio_get_level(KEY2))
       {
           curve_speed-=0.05;
           while(!gpio_get_level(KEY2));//松手检测
       }
   }
     tft180_show_string(0,16,"zhi dao=");
     tft180_show_string(0,48,"wan dao=");

     tft180_show_float (70, 16, straightaway_speed, 1, 3);  //直道速度
     tft180_show_float (70, 48, curve_speed, 1, 3);        //弯道速度
}
//修改断路pid
void fun_b32()
{
     shield_key=1;
     if( !gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
      {
          system_delay_ms(10);//消抖
       if( !gpio_get_level(KEY4))
       {
           use_inductor_pd[0]+=0.25;
           while(!gpio_get_level(KEY4));//松手检测
       }

       if( !gpio_get_level(KEY5))
       {
           use_inductor_pd[0]-=0.25;
           while(!gpio_get_level(KEY5));//松手检测
       }

   //上推减小舵机d，下推增加舵机d
       if( !gpio_get_level(KEY1))
       {
           use_inductor_pd[1]+=0.25;
           while(!gpio_get_level(KEY1));//松手检测
       }

       if( !gpio_get_level(KEY2))
       {
           use_inductor_pd[1]-=0.25;
           while(!gpio_get_level(KEY2));//松手检测
       }
     }
     tft180_show_string(0,16,"in_P=");
     tft180_show_string(0,48,"in_D=");
     tft180_show_float (50,16, use_inductor_pd[0], 2, 3);
     tft180_show_float (50,48, use_inductor_pd[1], 2, 3);
}
//修改左救援阈值
void fun_b33()
{
     shield_key=1;
     if( !gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
      {
          system_delay_ms(10);//消抖

       if( !gpio_get_level(KEY4))
       {
           L_charge_big_thes+=1;
           while(!gpio_get_level(KEY4));//松手检测
       }

       if( !gpio_get_level(KEY5))
       {
           L_charge_big_thes-=1;
           while(!gpio_get_level(KEY5));//松手检测
       }
      //上推减小，下推增加
       if( !gpio_get_level(KEY1))
       {
           L_charge_small_thes+=1;
           while(!gpio_get_level(KEY1));//松手检测
       }

       if( !gpio_get_level(KEY2))
       {
           L_charge_small_thes-=1;
           while(!gpio_get_level(KEY2));//松手检测
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
//修改右救援阈值
void fun_c31()
{
     shield_key=1;

     if( !gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
      {
          system_delay_ms(10);//消抖
       if( !gpio_get_level(KEY4))
       {
           R_charge_big_thes+=1;
           while(!gpio_get_level(KEY4));//松手检测
       }

       if( !gpio_get_level(KEY5))
       {
           R_charge_big_thes-=1;
           while(!gpio_get_level(KEY5));//松手检测
       }

       if( !gpio_get_level(KEY1))
       {
           R_charge_small_thes+=1;
           while(!gpio_get_level(KEY1));//松手检测
       }

       if( !gpio_get_level(KEY2))
       {
           R_charge_small_thes-=1;
           while(!gpio_get_level(KEY2));//松手检测
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
//修改断路阈值处理
void fun_c32()
{
      shield_key=1;
     //左推减小断路二值化阈值，右推增加断路二值化阈值
     if(!gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
      {
          system_delay_ms(10);//消抖
       if( !gpio_get_level(KEY4))
       {
           Break_road_add_thres+=2;
           while(!gpio_get_level(KEY4));//松手检测
       }

       if( !gpio_get_level(KEY5))
       {
          Break_road_add_thres-=2;
           while(!gpio_get_level(KEY5));//松手检测
       }
      //上推减小断路差比和阈值，下推增加断路差比和阈值
       if( !gpio_get_level(KEY1))
       {
           bottom_gray_threa+=2;
           while(!gpio_get_level(KEY1));//松手检测
       }
       if( !gpio_get_level(KEY2))
       {
           bottom_gray_threa-=2;
           while(!gpio_get_level(KEY2));//松手检测
       }
     }
     tft180_show_string(0,16,"Break_road");
     tft180_show_string(0,48,"bottom_gray");

     tft180_show_uint (96, 16, Break_road_add_thres, 2);
     tft180_show_uint (96, 48,  bottom_gray_threa, 2);
}
//修改斜入十字的阈值
void fun_c33()
{
//    extern float use_slope_cross[2];  //右斜入十字5.8,3.9
//    extern float use_slope_cross_l[2];  //左斜入十字
    shield_key=1;
    //左推减小断路二值化阈值，右推增加断路二值化阈值
    if(!gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
     {
         system_delay_ms(10);//消抖
      if( !gpio_get_level(KEY4))
      {
          use_slope_cross[0]+=0.2;
          use_slope_cross_l[0]+=0.2;
          while(!gpio_get_level(KEY4));//松手检测
      }

      if( !gpio_get_level(KEY5))
      {
          use_slope_cross[0]-=0.2;
          use_slope_cross_l[0]-=0.2;
          while(!gpio_get_level(KEY5));//松手检测
      }

  //上推减小断路差比和阈值，下推增加断路差比和阈值
      if( !gpio_get_level(KEY1))
      {
          use_slope_cross[1]+=0.2;
          use_slope_cross_l[1]+=0.2;
          while(!gpio_get_level(KEY1));//松手检测
      }
      if( !gpio_get_level(KEY2))
      {
          use_slope_cross[1]-=0.4;
          use_slope_cross_l[1]-=0.4;
          while(!gpio_get_level(KEY2));//松手检测
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
    //  current;        //当前索引号
    //  up;            //向上翻索引号
    //  down;         //向下翻索引号
    //  enter;       //确认索引号
    //  void (*current_operation)();            //用于指向执行函数的指针函数
    //第0层
    {0,0,0,1,(*fun_0)},                   //中间两个为0说明向上翻和向下翻无效

    //第1层
    {1,4,2, 5,(*fun_a1)},
    {2,1,3, 9,(*fun_b1)},
    {3,2,4,13,(*fun_c1)},
    {4,3,1, 0,(*fun_d1)},

    //第2层
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

    //第3层
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

void (*current_operation_index)();            //定义一个函数指针
void KEY()                                   //使用菜单的话，直接在while里调用该函数
 {
     static uint8 func_index = 0; //初始显示欢迎界面
     if( !gpio_get_level(KEY1) || !gpio_get_level(KEY2) ||  !gpio_get_level(KEY3) ||  !gpio_get_level(KEY4) ||  !gpio_get_level(KEY5))
     {
       system_delay_ms(10);//消抖
       if( !gpio_get_level(KEY1) && shield_key==0)
       {
           func_index = table[func_index].up;    //向上翻
           tft180_clear ();
           while(!gpio_get_level(KEY1));//松手检测
       }
       if( !gpio_get_level(KEY2) && shield_key==0)
       {
           func_index = table[func_index].down;    //向下翻
           tft180_clear ();
           while(!gpio_get_level(KEY2));
       }
       if( !gpio_get_level(KEY3))
       {
           func_index = table[func_index].enter;    //确认,无向中间摁下去
           tft180_clear ();
           while(!gpio_get_level(KEY3));
       }
       //KEY4这段代码在不进行速度环调试的时候可以注释掉，毕竟比赛的时候几乎没有时间调速度环
//        if( !gpio_get_level(KEY4))
//        {
////            if(OFF_Speed_test)      //开启速度环测试
////            {
////              Speed_PID_debug=1;
////              Speed_PID_count=0;
////            }
//            tft180_clear ();
//            while(!gpio_get_level(KEY4));
//        }
   }
       //索引出函数执行的指针
       current_operation_index = table[func_index].current_operation;
       (*current_operation_index)();//执行当前操作函数
 }


