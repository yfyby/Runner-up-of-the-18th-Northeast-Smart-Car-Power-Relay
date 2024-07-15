/*
 * speed_mode.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "zf_common_headfile.h"
Time_Count_Type_Def_element element_count_ms;             //元素判断
//元素计数器初始化
void ELEMENT_INIT()
{
    element_count_ms.cross_ms=0;            //十字计时器
    element_count_ms.l_slope_cross_ms=0;            //左斜入十字计时器
    element_count_ms.r_slope_cross_ms=0;            //右斜入十字计时器
    element_count_ms.ring_ms=0;            //圆环计数器
    element_count_ms.R_ring_ms=0;            //圆环计数器
    element_count_ms.garage_ms=0;         //车库计数器
    element_count_ms.barrier_ms=0;       //障碍物计数器
    element_count_ms.break_road_ms=0;   //断路计数器
    element_count_ms.ramp_ms=0;        //坡道计数器
    element_count_ms.stop_car_ms=0;      //刹车计数
    element_count_ms.out_road_ms=0;    //出赛道计数
}
//元素计数器运行(50ms定时器运行)
void ELEMENT_COUNT()
{
    static uint8 last_L_CirCle_State=0;    //左圆环
    static uint8 last_R_CirCle_State=0;  //右圆环
    static uint8 last_Indooor_state=0;    //车库
    static uint8 last_Cross_state=0;     //十字
    static uint8 last_l_slope_cross_state=0; //左斜入十字
    static uint8 last_r_slope_cross_state=0; //右斜入十字
    static uint8 last_break_road_state=0;  //断路
    static uint8 last_Barrier_state=0;      //障碍物
    static uint8 last_Ramp_state=0;         //坡道

    if(Charge_Resue_state==3 && Charge_Resue_count<=30)     //等待充电计时
    {
        Charge_Resue_count++;
    }

    if(Barrier_out_count==1)                  //过障碍物之后适当减速
    {
        element_count_ms.stop_car_ms++;
        if(element_count_ms.stop_car_ms>=10)
        {
            Barrier_out_count=0;
            element_count_ms.stop_car_ms=0;
        }

    }

    if(inductor_check==0 && inductor_check_count==1 && Barrier_state==0  && Break_road_state==0)//出赛道1s计时，inductor_check==0代表电感在赛道外
                                                       //inductor_check_count==1为未满1s计时
    {
        element_count_ms.out_road_ms++;
        if(element_count_ms.out_road_ms>10)
        {
            inductor_check_count=0;
            element_count_ms.out_road_ms=0;
        }
    }else {
        element_count_ms.out_road_ms=0;
    }
    //左圆环
    if(L_CirCle_State>=1 && L_CirCle_State<=5 && last_L_CirCle_State==0)  //进行圆环元素状态记录,圆环状态5之后的状态没必要防误判
    {
        last_L_CirCle_State=L_CirCle_State;
        element_count_ms.ring_ms=0;
    }
    if(last_L_CirCle_State!=0)    //进行圆环元素计数
    {
        element_count_ms.ring_ms++;         //圆环计数
    }
    if(  element_count_ms.ring_ms>=20 && L_CirCle_State==last_L_CirCle_State)     //过了1s,如果圆环状态没有改变
    {
        L_CirCle_State=0;                         //清空
        last_L_CirCle_State=0;
        element_count_ms.ring_ms=0;
        Beep_set(0);
    }else if(element_count_ms.ring_ms>=20 && L_CirCle_State!=last_L_CirCle_State)
    {
        element_count_ms.ring_ms=0;
        last_L_CirCle_State=0;
    }
    //右圆环
    if(R_CirCle_State>=1 && R_CirCle_State<=5 && last_R_CirCle_State==0)  //进行圆环元素状态记录,圆环状态5之后的状态没必要防误判
    {
        last_R_CirCle_State=R_CirCle_State;
        element_count_ms.R_ring_ms=0;
    }
    if(last_R_CirCle_State!=0)                    //进行圆环元素计数
    {
        element_count_ms.R_ring_ms++;         //圆环计数
    }
    if( element_count_ms.R_ring_ms>=20 && R_CirCle_State==last_R_CirCle_State)     //过了1s,如果圆环状态没有改变
    {
        R_CirCle_State=0;                         //清空
        last_R_CirCle_State=0;
        element_count_ms.R_ring_ms=0;
        Beep_set(0);
    }else if( element_count_ms.R_ring_ms>=20 && R_CirCle_State!=last_R_CirCle_State)
    {
        element_count_ms.R_ring_ms=0;
        last_R_CirCle_State=0;
    }

    //车库
    if(Indooor_state!=0 && last_Indooor_state==0)       //车库处理状态之间不会超过1s,直接！=0即可判断
    {
        last_Indooor_state=Indooor_state;
        element_count_ms.garage_ms=0;
    }
    if(last_Indooor_state!=0)                    //进行车库元素计数
    {
        element_count_ms.garage_ms++;         //车库计数
    }
    if(  element_count_ms.garage_ms>=20 && Indooor_state==last_Indooor_state)     //过了1s,如果车库状态没有改变
    {
        Indooor_state=0;                         //清空
        last_Indooor_state=0;
        element_count_ms.garage_ms=0;
        Beep_set(0);
    }else if( element_count_ms.garage_ms>=20 && Indooor_state!=last_Indooor_state)  //过了1.5s,如果车库状态改变了
    {
        element_count_ms.garage_ms=0;
        last_Indooor_state=0;
    }
    //正入十字
    if(Flag_Cross!=0 && last_Cross_state==0) //十字处理状态之间不会超过1s,直接！=0即可判断
    {
        last_Cross_state=Flag_Cross;
        element_count_ms.cross_ms=0;
    }
    if(last_Cross_state!=0)                    //进行车库元素计数
    {
        element_count_ms.cross_ms++;         //车库计数
    }

    if(element_count_ms.cross_ms>=20 && Flag_Cross==last_Cross_state)     //过了1s,如果车库状态没有改变
    {
        Flag_Cross=0;                         //清空
        last_Cross_state=0;
        element_count_ms.cross_ms=0;
        Beep_set(0);
    }else if( element_count_ms.cross_ms>=20 && Flag_Cross!=last_Cross_state)  //改变了清空
    {
        element_count_ms.cross_ms=0;
        last_Cross_state=0;
    }

    //左斜入十字
    if(L_slope_cross_state!=0 && last_l_slope_cross_state==0)   //斜入十字处理状态之间不会超过1s,直接！=0即可判断
    {
        last_l_slope_cross_state=L_slope_cross_state;
        element_count_ms.l_slope_cross_ms=0;
    }
    if(last_l_slope_cross_state!=0)
    {
        element_count_ms.l_slope_cross_ms++;
    }
    if(element_count_ms.l_slope_cross_ms>=20 && L_slope_cross_state==last_l_slope_cross_state)
    {
        L_slope_cross_state=0;  //清空
        last_l_slope_cross_state=0;
        element_count_ms.l_slope_cross_ms=0;
        Beep_set(0);
    }else if( element_count_ms.l_slope_cross_ms>=20 && L_slope_cross_state!=last_l_slope_cross_state)
    {
        element_count_ms.l_slope_cross_ms=0;
        last_l_slope_cross_state=0;
    }

     //右斜入十字
    if(R_slope_cross_state!=0 && last_r_slope_cross_state==0)  //斜入十字处理状态之间不会超过1s,直接！=0即可判断
    {
        last_r_slope_cross_state=R_slope_cross_state;
        element_count_ms.r_slope_cross_ms=0;
    }
    if(last_r_slope_cross_state!=0)
    {
        element_count_ms.r_slope_cross_ms++;
    }
    if(element_count_ms.r_slope_cross_ms>=20 && R_slope_cross_state==last_r_slope_cross_state)
    {
        R_slope_cross_state=0;  //清空
        last_r_slope_cross_state=0;
        element_count_ms.r_slope_cross_ms=0;
        Beep_set(0);
    }else if( element_count_ms.r_slope_cross_ms>=20 && R_slope_cross_state!=last_r_slope_cross_state)
    {
        element_count_ms.r_slope_cross_ms=0;
        last_r_slope_cross_state=0;
    }
     //断路
    if(Break_road_state==1&& last_break_road_state==0)    //断路状态1处理状态之间不会超过1s,直接！=0即可判断
    {
        last_break_road_state=Break_road_state;
        element_count_ms.break_road_ms=0;
    }
    if(last_break_road_state!=0)
    {
        element_count_ms.break_road_ms++;
    }

    if(element_count_ms.break_road_ms>=20 && Break_road_state==last_break_road_state)
    {
        Break_road_state=0;
        last_break_road_state=0;
        element_count_ms.break_road_ms=0;
        Beep_set(0);
    }else if( element_count_ms.break_road_ms>=20 && Break_road_state!=last_break_road_state)
    {
        element_count_ms.break_road_ms=0;
        last_break_road_state=0;
    }

    if(Barrier_state==1&& last_Barrier_state==0)    //障碍物状态1处理状态之间不会超过1s,直接！=0即可判断
    {
        last_Barrier_state=Barrier_state;
        element_count_ms.barrier_ms=0;
    }
    if(last_Barrier_state!=0)
    {
        element_count_ms.barrier_ms++;
    }

    if(element_count_ms.barrier_ms>=20 && Barrier_state==last_Barrier_state)
    {
        Barrier_state=0;
        last_Barrier_state=0;
        element_count_ms.barrier_ms=0;
        Beep_set(0);
    }else if( element_count_ms.barrier_ms>=20 && Barrier_state!=last_Barrier_state)
    {
        element_count_ms.barrier_ms=0;
        last_Barrier_state=0;
    }

    if(Ramp_state && last_Ramp_state==0)         //坡道1s计时
      {
          last_Ramp_state=Ramp_state;
          element_count_ms.ramp_ms=0;
      }
        if(last_Ramp_state!=0)
        {
            element_count_ms.ramp_ms++;
        }

        if(element_count_ms.ramp_ms>=20 && Ramp_state==last_Ramp_state)
        {
            Ramp_state=0;
            last_Ramp_state=0;
            element_count_ms.ramp_ms=0;
            Beep_set(0);
        }else if( element_count_ms.ramp_ms>=20 && Ramp_state!=last_Ramp_state)
        {
            element_count_ms.ramp_ms=0;
            last_Ramp_state=0;
        }

}
//开启一个5ms定时器
void control_mode(Time_Count_Type_Def *time_count){
                        //Car_Protect();           //停车判断
                         time_count->get_speed_conut++;             //速度采集计数器
                         time_count->control_speed_count++;        //速度外环控制计数器
                         time_count->turn_streeing_count++;      //舵机转向环计数器
                         time_count->count_ms++;      //舵机转向环计数器
                         if (time_count->count_ms >= 2)
                         {
                            time_count->count_ms = 0;
//                             L_speed=(int)Left_Car_Speed*1000;
//                             R_speed=(int)Right_Car_Speed*1000;
//                            virtual_oscilloscope_data_conversion(L_speed,R_speed,0,0);
//                            wireless_uart_send_buff(virtual_oscilloscope_data, 10);
                         }
                         /////////////转向环/////////////
                         //////////////20ms/////////////
                         if (time_count->turn_streeing_count >= 4)
                         {
                            time_count->turn_streeing_count = 0;
                            steering_angl(streeing_duty);
                         }
                        /////////////测速/////////////
                        ///////////////5ms///////////
                        if (time_count->get_speed_conut >= 1)                 //20ms进行一次测速
                         {
                            time_count->get_speed_conut = 0;
                            Get_Speed();
                          }
                        /////////////速度环/////////////
                        //////////////10ms///////////
                       if (time_count->control_speed_count >=2)      //速度外环控制
                           {
                              time_count->control_speed_count = 0;
                              // SPEED_PID_DUBGE();     //速度环测试开启
                             Template_motor();
                           }
                       /////////////以下为电机输出/////////////
                             Template_motor_out();
                           //  Moto_Out(); //速度环测试开启

            }
//双车通信函数
void Car_Communication()
{
  static uint8 Send_Flag=0;
  uint8 count=5;
  if(Send_Flag==0)
  {
      if(Out_door_state==1)
      {
          while(count--)
          {
              wireless_uart_send_byte(0x12);
          }
          Send_Flag=1;
      }
  }
}
