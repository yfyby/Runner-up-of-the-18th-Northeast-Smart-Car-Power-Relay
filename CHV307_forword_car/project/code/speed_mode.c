/*
 * speed_mode.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
#include "zf_common_headfile.h"
Time_Count_Type_Def_element element_count_ms;             //Ԫ���ж�
//Ԫ�ؼ�������ʼ��
void ELEMENT_INIT()
{
    element_count_ms.cross_ms=0;            //ʮ�ּ�ʱ��
    element_count_ms.l_slope_cross_ms=0;            //��б��ʮ�ּ�ʱ��
    element_count_ms.r_slope_cross_ms=0;            //��б��ʮ�ּ�ʱ��
    element_count_ms.ring_ms=0;            //Բ��������
    element_count_ms.R_ring_ms=0;            //Բ��������
    element_count_ms.garage_ms=0;         //���������
    element_count_ms.barrier_ms=0;       //�ϰ��������
    element_count_ms.break_road_ms=0;   //��·������
    element_count_ms.ramp_ms=0;        //�µ�������
    element_count_ms.stop_car_ms=0;      //ɲ������
    element_count_ms.out_road_ms=0;    //����������
}
//Ԫ�ؼ���������(50ms��ʱ������)
void ELEMENT_COUNT()
{
    static uint8 last_L_CirCle_State=0;    //��Բ��
    static uint8 last_R_CirCle_State=0;  //��Բ��
    static uint8 last_Indooor_state=0;    //����
    static uint8 last_Cross_state=0;     //ʮ��
    static uint8 last_l_slope_cross_state=0; //��б��ʮ��
    static uint8 last_r_slope_cross_state=0; //��б��ʮ��
    static uint8 last_break_road_state=0;  //��·
    static uint8 last_Barrier_state=0;      //�ϰ���
    static uint8 last_Ramp_state=0;         //�µ�

    if(Charge_Resue_state==3 && Charge_Resue_count<=30)     //�ȴ�����ʱ
    {
        Charge_Resue_count++;
    }

    if(Barrier_out_count==1)                  //���ϰ���֮���ʵ�����
    {
        element_count_ms.stop_car_ms++;
        if(element_count_ms.stop_car_ms>=10)
        {
            Barrier_out_count=0;
            element_count_ms.stop_car_ms=0;
        }

    }

    if(inductor_check==0 && inductor_check_count==1 && Barrier_state==0  && Break_road_state==0)//������1s��ʱ��inductor_check==0��������������
                                                       //inductor_check_count==1Ϊδ��1s��ʱ
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
    //��Բ��
    if(L_CirCle_State>=1 && L_CirCle_State<=5 && last_L_CirCle_State==0)  //����Բ��Ԫ��״̬��¼,Բ��״̬5֮���״̬û��Ҫ������
    {
        last_L_CirCle_State=L_CirCle_State;
        element_count_ms.ring_ms=0;
    }
    if(last_L_CirCle_State!=0)    //����Բ��Ԫ�ؼ���
    {
        element_count_ms.ring_ms++;         //Բ������
    }
    if(  element_count_ms.ring_ms>=20 && L_CirCle_State==last_L_CirCle_State)     //����1s,���Բ��״̬û�иı�
    {
        L_CirCle_State=0;                         //���
        last_L_CirCle_State=0;
        element_count_ms.ring_ms=0;
        Beep_set(0);
    }else if(element_count_ms.ring_ms>=20 && L_CirCle_State!=last_L_CirCle_State)
    {
        element_count_ms.ring_ms=0;
        last_L_CirCle_State=0;
    }
    //��Բ��
    if(R_CirCle_State>=1 && R_CirCle_State<=5 && last_R_CirCle_State==0)  //����Բ��Ԫ��״̬��¼,Բ��״̬5֮���״̬û��Ҫ������
    {
        last_R_CirCle_State=R_CirCle_State;
        element_count_ms.R_ring_ms=0;
    }
    if(last_R_CirCle_State!=0)                    //����Բ��Ԫ�ؼ���
    {
        element_count_ms.R_ring_ms++;         //Բ������
    }
    if( element_count_ms.R_ring_ms>=20 && R_CirCle_State==last_R_CirCle_State)     //����1s,���Բ��״̬û�иı�
    {
        R_CirCle_State=0;                         //���
        last_R_CirCle_State=0;
        element_count_ms.R_ring_ms=0;
        Beep_set(0);
    }else if( element_count_ms.R_ring_ms>=20 && R_CirCle_State!=last_R_CirCle_State)
    {
        element_count_ms.R_ring_ms=0;
        last_R_CirCle_State=0;
    }

    //����
    if(Indooor_state!=0 && last_Indooor_state==0)       //���⴦��״̬֮�䲻�ᳬ��1s,ֱ�ӣ�=0�����ж�
    {
        last_Indooor_state=Indooor_state;
        element_count_ms.garage_ms=0;
    }
    if(last_Indooor_state!=0)                    //���г���Ԫ�ؼ���
    {
        element_count_ms.garage_ms++;         //�������
    }
    if(  element_count_ms.garage_ms>=20 && Indooor_state==last_Indooor_state)     //����1s,�������״̬û�иı�
    {
        Indooor_state=0;                         //���
        last_Indooor_state=0;
        element_count_ms.garage_ms=0;
        Beep_set(0);
    }else if( element_count_ms.garage_ms>=20 && Indooor_state!=last_Indooor_state)  //����1.5s,�������״̬�ı���
    {
        element_count_ms.garage_ms=0;
        last_Indooor_state=0;
    }
    //����ʮ��
    if(Flag_Cross!=0 && last_Cross_state==0) //ʮ�ִ���״̬֮�䲻�ᳬ��1s,ֱ�ӣ�=0�����ж�
    {
        last_Cross_state=Flag_Cross;
        element_count_ms.cross_ms=0;
    }
    if(last_Cross_state!=0)                    //���г���Ԫ�ؼ���
    {
        element_count_ms.cross_ms++;         //�������
    }

    if(element_count_ms.cross_ms>=20 && Flag_Cross==last_Cross_state)     //����1s,�������״̬û�иı�
    {
        Flag_Cross=0;                         //���
        last_Cross_state=0;
        element_count_ms.cross_ms=0;
        Beep_set(0);
    }else if( element_count_ms.cross_ms>=20 && Flag_Cross!=last_Cross_state)  //�ı������
    {
        element_count_ms.cross_ms=0;
        last_Cross_state=0;
    }

    //��б��ʮ��
    if(L_slope_cross_state!=0 && last_l_slope_cross_state==0)   //б��ʮ�ִ���״̬֮�䲻�ᳬ��1s,ֱ�ӣ�=0�����ж�
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
        L_slope_cross_state=0;  //���
        last_l_slope_cross_state=0;
        element_count_ms.l_slope_cross_ms=0;
        Beep_set(0);
    }else if( element_count_ms.l_slope_cross_ms>=20 && L_slope_cross_state!=last_l_slope_cross_state)
    {
        element_count_ms.l_slope_cross_ms=0;
        last_l_slope_cross_state=0;
    }

     //��б��ʮ��
    if(R_slope_cross_state!=0 && last_r_slope_cross_state==0)  //б��ʮ�ִ���״̬֮�䲻�ᳬ��1s,ֱ�ӣ�=0�����ж�
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
        R_slope_cross_state=0;  //���
        last_r_slope_cross_state=0;
        element_count_ms.r_slope_cross_ms=0;
        Beep_set(0);
    }else if( element_count_ms.r_slope_cross_ms>=20 && R_slope_cross_state!=last_r_slope_cross_state)
    {
        element_count_ms.r_slope_cross_ms=0;
        last_r_slope_cross_state=0;
    }
     //��·
    if(Break_road_state==1&& last_break_road_state==0)    //��·״̬1����״̬֮�䲻�ᳬ��1s,ֱ�ӣ�=0�����ж�
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

    if(Barrier_state==1&& last_Barrier_state==0)    //�ϰ���״̬1����״̬֮�䲻�ᳬ��1s,ֱ�ӣ�=0�����ж�
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

    if(Ramp_state && last_Ramp_state==0)         //�µ�1s��ʱ
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
//����һ��5ms��ʱ��
void control_mode(Time_Count_Type_Def *time_count){
                        //Car_Protect();           //ͣ���ж�
                         time_count->get_speed_conut++;             //�ٶȲɼ�������
                         time_count->control_speed_count++;        //�ٶ��⻷���Ƽ�����
                         time_count->turn_streeing_count++;      //���ת�򻷼�����
                         time_count->count_ms++;      //���ת�򻷼�����
                         if (time_count->count_ms >= 2)
                         {
                            time_count->count_ms = 0;
//                             L_speed=(int)Left_Car_Speed*1000;
//                             R_speed=(int)Right_Car_Speed*1000;
//                            virtual_oscilloscope_data_conversion(L_speed,R_speed,0,0);
//                            wireless_uart_send_buff(virtual_oscilloscope_data, 10);
                         }
                         /////////////ת��/////////////
                         //////////////20ms/////////////
                         if (time_count->turn_streeing_count >= 4)
                         {
                            time_count->turn_streeing_count = 0;
                            steering_angl(streeing_duty);
                         }
                        /////////////����/////////////
                        ///////////////5ms///////////
                        if (time_count->get_speed_conut >= 1)                 //20ms����һ�β���
                         {
                            time_count->get_speed_conut = 0;
                            Get_Speed();
                          }
                        /////////////�ٶȻ�/////////////
                        //////////////10ms///////////
                       if (time_count->control_speed_count >=2)      //�ٶ��⻷����
                           {
                              time_count->control_speed_count = 0;
                              // SPEED_PID_DUBGE();     //�ٶȻ����Կ���
                             Template_motor();
                           }
                       /////////////����Ϊ������/////////////
                             Template_motor_out();
                           //  Moto_Out(); //�ٶȻ����Կ���

            }
//˫��ͨ�ź���
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
