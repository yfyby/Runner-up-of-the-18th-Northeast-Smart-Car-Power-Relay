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
 *  һ������£���ˮƽ���ƫ��
 *  �ڻ���ʱ�У��ô�ֱ���ƫ��
 *  V1.0 Inductor_Value[0]                    Inductor_Value[2]                      Inductor_Value[4]
 *              (ˮƽ����)                           (�м���)                                ��ˮƽ�ҵ�У�
 *                              Inductor_Value[1]                          Inductor_Value[3]
 *                                 ����ֱ���У�                            ����ֱ�ҵ�У�
 */
/*********************************************************************************************************************
 * @Autor       �ڿƼ�����
 * @file        ���Բ���ж�
 * @date            2022
 * @user        Ԫ����ֵ��Ϊ��Сֵ����֤ʶ��׼ȷ
 ********************************************************************************************************************/
/*********Բ��״̬********/
#define Fine_round              1
#define Come_round              2
#define Ready_ready_round       3
#define Ready_round             4
#define Ready_Out_round_L       5
#define Ready_Out_round_R       6
#define Out_round               7
uint8 Flag_Round = Out_round;                 //������־
uint8 Flag_L_or_R=0;                    //����Բ����־λ
/*********���ߴ���********/
uint8 loss_line=0;                 //����ָʾ��1�󶪣�2�Ҷ���0����
uint8 loss_line_lock=0;            //�����������ߺ��Զ�������1Ϊ����
void DirectionControl()
{
     static float  temp_Direction;
   static int16  F_round_count=0;
     float  temp_dic;
     static uint8 F_round = 0;                       //��ֹ�����ٴ����б�־
   if(F_round==1 && Flag_Round ==Out_round)                                                                                        //��Բ��֮���һ��ʱ������ٴ��ж�
     {
      F_round_count++;
          if(F_round_count>100)
            {
                F_round=0;
                F_round_count=0;
            }
     }
        Get_ADC();                                                                                                                      //��ȡ���ֵ�����
     if(F_round==0){                                                                                                                   //��ֹԲ���ٴν���
             if(((Inductor_Value[0]+Inductor_Value[4])/2>36)&&((Inductor_Value[1]<10)||(Inductor_Value[3]<10))&&(Flag_Round!=Fine_round))   //ʶ��ǰ����Բ����36Ϊ���ߺ��нӽ�Բ�����Ĳž��е�ƽ��ֵ
             {
                    Flag_Round =Fine_round;
                    F_round=1;
                    }
        }
         //���뻷
     if((Inductor_Value[1]>=25)&&(Flag_Round ==Fine_round)){             //���뻷,����ֱ�����2λ��ֵ�Ƚϴ�
                    Flag_Round =Come_round;
                    Flag_L_or_R=1;             //��
            }

     if((Inductor_Value[1]<=13)&&(Flag_Round ==Come_round)&&(Flag_L_or_R==1)){    //���뻷,����ֱ����ڿ쵽��3λ��
                Flag_Round=Ready_ready_round;
             }

     if((Inductor_Value[1]>=26)&&(Flag_Round ==Ready_ready_round)&&(Flag_L_or_R==1)){             //�����뻷��
               Flag_Round =Ready_round;
         }

         //���뻷�������뻷ͬ��
    if((Inductor_Value[3]>=25)&&(Flag_Round ==Fine_round)){
                Flag_Round =Come_round;
                Flag_L_or_R=0;
         }

         if((Inductor_Value[3]<=13)&&(Flag_Round ==Come_round)&&(Flag_L_or_R==0)){
                Flag_Round=Ready_ready_round;
         }

         if((Inductor_Value[3]>=25)&&(Flag_Round ==Ready_ready_round)&&(Flag_L_or_R==0)){             //���뻷
                Flag_Round =Ready_round;
         }

        //׼������
         if((Inductor_Value[1]>=27)&&(Inductor_Value[3]>=20)&&(Flag_Round ==Ready_round)&&(Flag_L_or_R==1))        //׼������
            {
                Flag_Round = Ready_Out_round_L;//������־λ
            }
         if((Inductor_Value[1]>=20)&&(Inductor_Value[3]>=27)&&(Flag_Round ==Ready_round)&&(Flag_L_or_R==0))        //׼������
         {
                Flag_Round = Ready_Out_round_R;//������־λ
         }
            //��������λ��
        if((Inductor_Value[1]<=13) && (Inductor_Value[2]>=35) && (Inductor_Value[3]<=13)&&
             ((Flag_Round ==Ready_Out_round_L)||(Flag_Round ==Ready_Out_round_R))){
                Flag_Round = Out_round;
         }
/******����ѭ��*******/
 if((Level_Deviation>0.35)&&(Vert_Deviation>1.2)){     //�ж�Ϊ�����
            Level_Deviation=(Level_Deviation+Vert_Deviation*0.3);
 }
/******Բ��ѭ��*******/
    //�л���ֱ���ѭ��ģʽ,ʹ���ڽ���Բ��
    if(Flag_Round==Ready_round){
          temp_dic=1.5/fabs(Vert_Deviation);                      //1.5Ϊƫ��ޣ�temp_dicΪƫ������
       if(fabs(Vert_Deviation)<1.2){                                    //���һ�ƫ������
          if(Vert_Deviation>0.0){                                 //���뻷
                  if(fabs(Vert_Deviation)>0.8){                //ƫ�����̫С
                    temp_Direction=Vert_Deviation+temp_dic;           //��¼�뻷�޸�֮���ƫ��
                   }else{
                                temp_Direction+=0.2;                           //һ����˵�ٶȲ���ᾭ��>0.8�Ľ׶Σ������ֹtemp_DirectionΪ0
                                temp_Direction*=1.5;
                              if(fabs(temp_Direction)<0.9)
                                      temp_Direction=0.9+temp_Direction*0.5+temp_dic;
                             }
                    }else{                                                         //���뻷
                if(fabs(Vert_Deviation)>0.8){                            //ƫ�����̫С
                    temp_Direction=Vert_Deviation-temp_dic;           //��¼�뻷�޸�֮���ƫ��
                   }else{
                                temp_Direction-=0.2;
                                temp_Direction*=1.5;
                              if(fabs(temp_Direction)<0.9)
                                      temp_Direction=-0.9+temp_Direction*0.5+temp_dic;
                             }
                    }
        }
    }
    //��Բ�����޸����ó�˳������
    if(Flag_Round == Ready_Out_round_L){//����
        if(fabs(Vert_Deviation)<0.80){
        Vert_Deviation=(sqrt(Inductor_Value[1]+Inductor_Value[3]) - sqrt(Inductor_Value[3]))/(Inductor_Value[1]+Inductor_Value[3]+ Inductor_Value[3])*20;//ˮƽ��еĲ�Ⱥ���Ϊƫ��
        }
    }
    if(Flag_Round == Ready_Out_round_R){  //���һ�
        if(fabs(Vert_Deviation)<0.80){
        Vert_Deviation=(sqrt(Inductor_Value[1]) - sqrt(Inductor_Value[1]+Inductor_Value[3]))/(Inductor_Value[1]+Inductor_Value[1]+Inductor_Value[3])*20;//ˮƽ��еĲ�Ⱥ���Ϊƫ��
        }
    }
}
/*********************************************************
 *  @brief         ���ߴ���
 *  @since         v1.0
 *  sensor_value   ������ֵ
 *��position       ƫ��ֵ
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
            error_add[i] = error_add[i-1];              //����ѭ������
        }
        error_add[0] = (int)(position * 100);           //�����ε�ƫ��ŵ�����ͷ��
        error_total = error_add[0]  + error_add[1] + error_add[2];      //�������ƫ����ͣ������ж����ı߶���

             }else{

                for(uint8 i = 2 ; i>0 ; i--)
        {
            error_add[i] = error_add[i-1];              //����ѭ������
        }
        error_add[0] = (int)(vert_position * 100);           //�����ε�ƫ��ŵ�����ͷ��
        error_total = error_add[0]  + error_add[1] + error_add[2];      //�������ƫ����ͣ������ж����ı߶���
            }

        sensor_value_total = sensor_value[0] + sensor_value[2] + sensor_value[4];                        //3��������ֵ���

        if((sensor_value_total < left_lose_threshold) || (sensor_value_total < right_lose_threshold))            //����6�������������ֵ�ж϶���
        {
            if((error_total > 0) && (loss_line == 0) && (sensor_value_total < left_lose_threshold))
            {
              loss_line = 1;                     //����ʱ����1
            }
            else if((error_total < 0) && (loss_line == 0) && (sensor_value_total < right_lose_threshold))
            {
              loss_line = 2;                     //�Ҷ���ʱ����2
            }
            else
            {;;}
        }
        else
        {
            if(loss_line_lock == 1)             //��ֹ���߼���е��󱨣����ߺ�����
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

          case 1:          //����
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

          case 2:        //�Ҷ���
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
 *  @brief         ������ͣ������
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

