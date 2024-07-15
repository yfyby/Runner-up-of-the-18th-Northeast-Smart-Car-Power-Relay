/*
 * special_contral.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "special_contral.h"
#include "inductor.h"
#include "math.h"
#include "wheel.h"
#include "system.h"
/*
 *  一般情况下：用水平电感偏差
 *  在环岛时中：用垂直电感偏差
 *  V1.0 Inductor_Value[0]                    Inductor_Value[2]                      Inductor_Value[4]
 *              (水平左电感)                           (中间电感)                                （水平右电感）
 *                              Inductor_Value[1]                          Inductor_Value[3]
 *                                 （垂直左电感）                            （垂直右电感）
 */
/*********************************************************************************************************************
 * @Autor       黑科技刀哥
 * @file        电感圆环判断
 * @date            2022
 * @user        元素阈值都为最小值，保证识别准确
 ********************************************************************************************************************/
/*********圆环状态********/
#define Fine_round              1
#define Come_round              2
#define Ready_ready_round       3
#define Ready_round             4
#define Ready_Out_round_L       5
#define Ready_Out_round_R       6
#define Out_round               7
uint8 Flag_Round = Out_round;                 //环岛标志
uint8 Flag_L_or_R=0;                    //左右圆环标志位
/*********丢线处理********/
uint8 loss_line=0;                 //丢线指示，1左丢，2右丢，0不丢
uint8 loss_line_lock=0;            //丢线锁，丢线后自动上锁，1为上锁
void DirectionControl()
{
     static float  temp_Direction;
   static int16  F_round_count=0;
     float  temp_dic;
     static uint8 F_round = 0;                       //防止环岛再次误判标志
   if(F_round==1 && Flag_Round ==Out_round)                                                                                        //出圆环之后过一段时间可以再次判断
     {
      F_round_count++;
          if(F_round_count>100)
            {
                F_round=0;
                F_round_count=0;
            }
     }
        Get_ADC();                                                                                                                      //获取电感值与误差
     if(F_round==0){                                                                                                                   //防止圆环再次进入
             if(((Inductor_Value[0]+Inductor_Value[4])/2>36)&&((Inductor_Value[1]<10)||(Inductor_Value[3]<10))&&(Flag_Round!=Fine_round))   //识别到前面是圆环，36为两边横电感接近圆环中心才具有的平均值
             {
                    Flag_Round =Fine_round;
                    F_round=1;
                    }
        }
         //左入环
     if((Inductor_Value[1]>=25)&&(Flag_Round ==Fine_round)){             //左入环,左竖直电感在2位置值比较大
                    Flag_Round =Come_round;
                    Flag_L_or_R=1;             //左环
            }

     if((Inductor_Value[1]<=13)&&(Flag_Round ==Come_round)&&(Flag_L_or_R==1)){    //左入环,左竖直电感在快到达3位置
                Flag_Round=Ready_ready_round;
             }

     if((Inductor_Value[1]>=26)&&(Flag_Round ==Ready_ready_round)&&(Flag_L_or_R==1)){             //到达入环点
               Flag_Round =Ready_round;
         }

         //右入环，和左入环同理
    if((Inductor_Value[3]>=25)&&(Flag_Round ==Fine_round)){
                Flag_Round =Come_round;
                Flag_L_or_R=0;
         }

         if((Inductor_Value[3]<=13)&&(Flag_Round ==Come_round)&&(Flag_L_or_R==0)){
                Flag_Round=Ready_ready_round;
         }

         if((Inductor_Value[3]>=25)&&(Flag_Round ==Ready_ready_round)&&(Flag_L_or_R==0)){             //右入环
                Flag_Round =Ready_round;
         }

        //准备出环
         if((Inductor_Value[1]>=27)&&(Inductor_Value[3]>=20)&&(Flag_Round ==Ready_round)&&(Flag_L_or_R==1))        //准备出环
            {
                Flag_Round = Ready_Out_round_L;//出环标志位
            }
         if((Inductor_Value[1]>=20)&&(Inductor_Value[3]>=27)&&(Flag_Round ==Ready_round)&&(Flag_L_or_R==0))        //准备出环
         {
                Flag_Round = Ready_Out_round_R;//出环标志位
         }
            //真正出环位置
        if((Inductor_Value[1]<=13) && (Inductor_Value[2]>=35) && (Inductor_Value[3]<=13)&&
             ((Flag_Round ==Ready_Out_round_L)||(Flag_Round ==Ready_Out_round_R))){
                Flag_Round = Out_round;
         }
/******正常循迹*******/
 if((Level_Deviation>0.35)&&(Vert_Deviation>1.2)){     //判断为大弯道
            Level_Deviation=(Level_Deviation+Vert_Deviation*0.3);
 }
/******圆环循迹*******/
    //切换竖直电感循迹模式,使用于进入圆环
    if(Flag_Round==Ready_round){
          temp_dic=1.5/fabs(Vert_Deviation);                      //1.5为偏差极限，temp_dic为偏差增量
       if(fabs(Vert_Deviation)<1.2){                                    //左右环偏差修正
          if(Vert_Deviation>0.0){                                 //左入环
                  if(fabs(Vert_Deviation)>0.8){                //偏差还不算太小
                    temp_Direction=Vert_Deviation+temp_dic;           //记录入环修改之后的偏差
                   }else{
                                temp_Direction+=0.2;                           //一般来说速度不快会经过>0.8的阶段，这里防止temp_Direction为0
                                temp_Direction*=1.5;
                              if(fabs(temp_Direction)<0.9)
                                      temp_Direction=0.9+temp_Direction*0.5+temp_dic;
                             }
                    }else{                                                         //右入环
                if(fabs(Vert_Deviation)>0.8){                            //偏差还不算太小
                    temp_Direction=Vert_Deviation-temp_dic;           //记录入环修改之后的偏差
                   }else{
                                temp_Direction-=0.2;
                                temp_Direction*=1.5;
                              if(fabs(temp_Direction)<0.9)
                                      temp_Direction=-0.9+temp_Direction*0.5+temp_dic;
                             }
                    }
        }
    }
    //出圆环，修改误差，让车顺利出环
    if(Flag_Round == Ready_Out_round_L){//出左环
        if(fabs(Vert_Deviation)<0.80){
        Vert_Deviation=(sqrt(Inductor_Value[1]+Inductor_Value[3]) - sqrt(Inductor_Value[3]))/(Inductor_Value[1]+Inductor_Value[3]+ Inductor_Value[3])*20;//水平电感的差比和作为偏差
        }
    }
    if(Flag_Round == Ready_Out_round_R){  //出右环
        if(fabs(Vert_Deviation)<0.80){
        Vert_Deviation=(sqrt(Inductor_Value[1]) - sqrt(Inductor_Value[1]+Inductor_Value[3]))/(Inductor_Value[1]+Inductor_Value[1]+Inductor_Value[3])*20;//水平电感的差比和作为偏差
        }
    }
}
/*********************************************************
 *  @brief         丢线处理
 *  @since         v1.0
 *  sensor_value   传感器值
 *　position       偏差值
*********************************************************/
void loss_line_deal(int16 * sensor_value , float position ,float vert_position ,uint16 left_lose_threshold ,uint16 right_lose_threshold)
{
        static int32 error_add[3];
        int32 error_total = 0;
        uint16 sensor_value_total = 0;
        static uint8 loss_line_inc;

        if(Flag_Round==Out_round){

        for(uint8 i = 2 ; i>0 ; i--)
        {
            error_add[i] = error_add[i-1];              //数组循环左移
        }
        error_add[0] = (int)(position * 100);           //将当次的偏差放到数组头部
        error_total = error_add[0]  + error_add[1] + error_add[2];      //最近三次偏差求和，用于判断是哪边丢线

             }else{

                for(uint8 i = 2 ; i>0 ; i--)
        {
            error_add[i] = error_add[i-1];              //数组循环左移
        }
        error_add[0] = (int)(vert_position * 100);           //将当次的偏差放到数组头部
        error_total = error_add[0]  + error_add[1] + error_add[2];      //最近三次偏差求和，用于判断是哪边丢线
            }

        sensor_value_total = sensor_value[0] + sensor_value[2] + sensor_value[4];                        //3个传感器值相加

        if((sensor_value_total < left_lose_threshold) || (sensor_value_total < right_lose_threshold))            //根据6个传感器的相加值判断丢线
        {
            if((error_total > 0) && (loss_line == 0) && (sensor_value_total < left_lose_threshold))
            {
              loss_line = 1;                     //左丢线时返回1
            }
            else if((error_total < 0) && (loss_line == 0) && (sensor_value_total < right_lose_threshold))
            {
              loss_line = 2;                     //右丢线时返回2
            }
            else
            {;;}
        }
        else
        {
            if(loss_line_lock == 1)             //防止丢线检测中的误报，丢线后上锁
            {
                loss_line_inc ++;
                if(loss_line_inc > 7)
                {
                    loss_line_lock = 0;
                    loss_line_inc =0;
                }
            }
            if(loss_line_lock == 0)
            {
                loss_line = 0;
            }
        }
          switch(loss_line)
      {
          case 0:
             loss_line_lock=0;
              break;

          case 1:          //左丢线
              if(Flag_Round!=Out_round)
              {

                      Vert_Deviation = -1.8;
                      streeing_duty=140;
                            }else
              {
                      loss_line_lock = 1;
                      Level_Deviation=-1.8;
                      streeing_duty=140;
              }
              break;

          case 2:        //右丢线
              if(Flag_Round!=Out_round)
              {
                      Vert_Deviation = 1.8;
                      streeing_duty=-140;
              }
              else
              {
                     loss_line_lock = 1;
                     Level_Deviation=1.8;
                     streeing_duty=-140;
              }
              break;
      }
}
/*********************************************************
 *  @brief         出赛道停车处理
 *  @since         v1.0
*********************************************************/
void Inductor_unnormal_deal()
{
    uint8 check=0;
    check=Is_InductorValue_Normal();
    if(check==1 && Flag_Round!=Out_round && Barrier_state!=0)
    {
      wheel_speed(0,0);
    }
}

