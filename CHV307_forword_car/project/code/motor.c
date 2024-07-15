/*
 * motor.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
/*********************************************************************************************************************
 * �������Դ�ļ�
 ********************************************************************************************************************/
#include "some_algorithm.h"
#include "wheel.h"
#include "motor.h"
#include "zf_driver_encoder.h"
#include "math.h"
#include "system.h"
#include "streeing.h"
#define Radian          0.0175f                       // 1/180*3.14159
#define MOTOMAX         5000                         //������ռ�ձ�
#define MOTOMIN        -5000
#define CHANGE_RATE_THRESHOLD         1000      //�������������ٱ仯����ֵ
#define WHEEL_SPEED_THRESHOLD         2000       //����������������ֵ
int16 Moto_Out_Left = 0;                       //�����������ռ�ձ�
int16 Moto_Out_Right = 0;                     //�ҵ���������ռ�ձ�
float Car_Speed = 0;                            //����m/s
float Left_Car_Speed=0;                          //�����ٶ�m/s
float Right_Car_Speed=0;                         //�����ٶ�m/s
float Distance = 0;                             //ȫ���ۼӵľ���
float Current_Left = 0;                         //���ֱ�����ֵ
float Current_Right = 0;                       //���ֱ�����ֵ
float Original_current_Left=0;                     //ԭʼ������ֵ
float Original_current_Right=0;                     //ԭʼ������ֵ
float LR_Speed_Err = 0;                       //���ֳ��ٵĲ�ֵ
float user_distance=0;                    //��¼��·��
float Break_road_speed=0.5;           //��·�ٶ�
float straightaway_speed=1.9;  //ֱ���ٶ�
float curve_speed=1.8;        //����ٶ�
//�ٶȿ������ͱ���
PID_Type_Def Pid_LRSpeed;                             //���ٲ�����(�µ�ʹ��)
PID_Type_Def Pid_Left_Speed;                         //�����ٶȻ�
PID_Type_Def Pid_Right_Speed;                       //�����ٶȻ�
Control_Speed_Type_Def Control_Left_Speed;          //�����ٶȽṹ
Control_Speed_Type_Def Control_Right_Speed;        //�����ٶȽṹ
Time_Count_Type_Def Time_Count;                   //�������ṹ
//pid����
float Car_KI=58.5;  //58.5
float Car_KD=0;      //200,���˺���Ч�����
float Car_KP=3050;
int16 bangbang_duty=100;
bool Stop_car_flag=0;       //ͣ����־λ
/***************************************************
*                  ��ʼ����������
*  ���ͣ�����
*  ����ֵ����
***************************************************/
void Init_contral(){

    //Ĭ���ٶ���ز���
    Control_Left_Speed.stright =straightaway_speed;                              //����ֱ���ٶ�2.3
    Control_Left_Speed.cruve = curve_speed;                             //��������ٶ�1.8
    Control_Left_Speed.user_speed = 0;
    Control_Left_Speed.element=2;                            //Ԫ���ٶ�
    Control_Left_Speed.barrier_speed=0;                     //���ϰ�����ٶ�//Ĭ�ϳ���
    Control_Left_Speed.rescue_speed=-0.5;                    //�����ٶ�
    Control_Left_Speed.err = 0;                             //���
    Control_Left_Speed.Last_err = 0;                        //�ϴ����
    Control_Left_Speed.Previous_Error=0;                   //���ϴ����
    Control_Left_Speed.integral = 0;                       //�ٶȻ���
    Control_Left_Speed.Speed_Duty=0;                      //ռ�ձ�
    Control_Left_Speed.Speed_Old_Duty=0;                 //��һ��ռ�ձ�

    Control_Right_Speed.stright =straightaway_speed;      //����ֱ���ٶ�
    Control_Right_Speed.cruve = curve_speed;                      //��������ٶ�
    Control_Right_Speed.user_speed = 0;
    Control_Right_Speed.element=2;                       //Ԫ���ٶ�
    Control_Right_Speed.barrier_speed=1.5;                //���ϰ�����ٶ�//Ĭ�ϳ���
    Control_Right_Speed.rescue_speed=-0.5;                  //�����ٶ�
    Control_Right_Speed.err = 0;                            //���
    Control_Right_Speed.Last_err = 0;                      //�ϴ����
    Control_Right_Speed.Previous_Error=0;                 //���ϴ����
    Control_Right_Speed.integral = 0;                    //�ٶȻ���
    Control_Right_Speed.Speed_Duty=0;                   //ռ�ձ�
    Control_Right_Speed.Speed_Old_Duty=0;              //��һ��ռ�ձ�
    //�ٶ��⻷pid
    Pid_Left_Speed.P = Car_KP;
    Pid_Left_Speed.I = Car_KI;
    Pid_Left_Speed.D = Car_KD;
    Pid_Left_Speed.Out =0.0;

    Pid_Right_Speed.P =Car_KP;
    Pid_Right_Speed.I =Car_KI;
    Pid_Right_Speed.D = Car_KD;
    Pid_Right_Speed.Out =0.0;
    //�µ����ٲ����pid
    Pid_LRSpeed.P = 300.0;
    Pid_LRSpeed.D = 0.0;
    Pid_LRSpeed.Out = 0.0;
    //��ʱ���ṹ
    Time_Count.control_speed_count = 0;
    Time_Count.get_speed_conut = 0;
    Time_Count.count_ms = 0;
    Time_Count.turn_streeing_count = 0;
  }
/***************************************************
*                  ��ʼ��������
*  ���ͣ�����
*  ����ֵ����
***************************************************/
void Init_encoder()
{
    encoder_dir_init(TIM1_ENCOEDER, TIM1_ENCOEDER_MAP3_CH1_E9, TIM1_ENCOEDER_MAP3_CH2_E11);
    encoder_clear_count(TIM1_ENCOEDER);
    encoder_dir_init(TIM8_ENCOEDER, TIM8_ENCOEDER_MAP0_CH1_C6, TIM8_ENCOEDER_MAP0_CH2_C7);
    encoder_clear_count(TIM8_ENCOEDER);
}
/***************************************************
*                ��ȡ�����ٶȲ���(5ms����һ��)
*  ���ͣ�����
*  ����ֵ����
*  ���ܣ��ɼ��ٶȣ��Ϻõؽ����������̧�֡��������µ���
*        ���ص�Ӱ��
***************************************************/
void Get_Speed()
{
    int16 temp_l, temp_r;
    Para_Type type;  //�������˲�����
    static float  Last_Left = 0.0f, Last_Right = 0.0f;
    static Wheel_State Left_State = Normal, Right_State = Normal;  //����״̬��ʼ������
    static float LRSpeed_Record[2][5] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};     //�������ټ�¼����
    float left_speed_slope = 0.0f, right_speed_slope = 0.0f;    //�������ٱ仯��

    if(Stop_car_flag==0) //����ͣ���͵���
    {
    temp_l = abs(encoder_get_count(TIM1_ENCOEDER));//��ߵı���������
        encoder_clear_count(TIM1_ENCOEDER);
    temp_r = abs(encoder_get_count(TIM8_ENCOEDER));//�ұߵı���������
        encoder_clear_count(TIM8_ENCOEDER);
    }
    else                  //ͣ���͵�����Ҫ����
    {
    temp_l = (encoder_get_count(TIM1_ENCOEDER));//��ߵı���������
            encoder_clear_count(TIM1_ENCOEDER);
    temp_r = -(encoder_get_count(TIM8_ENCOEDER));//�ұߵı���������
            encoder_clear_count(TIM8_ENCOEDER);
    }

    Original_current_Left=temp_l;
    Original_current_Right=temp_r;

    Last_Left = Current_Left;
    Last_Right = Current_Right;
    //�˲�
    if(Stop_car_flag==0)
    {
    type = Left_Speed;
    Current_Left = First_Order_KalmanFilter(temp_l, type,2.0f, 5.0f);
    type = Right_Speed;
    Current_Right = First_Order_KalmanFilter(temp_r, type,2.0f, 5.0f);
    }
    else
    {
    Current_Left=(temp_l*5+ LRSpeed_Record[0][4]*3+LRSpeed_Record[0][3]*2)/10;
    Current_Right=(temp_r*5+ LRSpeed_Record[1][4]*3+LRSpeed_Record[1][3]*2)/10;
    }

    //�������ټ�¼
    for (uint8 i = 0; i < 4; i++)
        LRSpeed_Record[0][i] = LRSpeed_Record[0][i + 1];
    LRSpeed_Record[0][4] = temp_l;
    //�������ټ�¼
    for (uint8 i = 0; i < 4; i++)
        LRSpeed_Record[1][i] = LRSpeed_Record[1][i + 1];
    LRSpeed_Record[1][4] = temp_r;

    //��С���˹������������ٱ仯��
    left_speed_slope = Slope_Calculate(0, 4, LRSpeed_Record[0]);
    right_speed_slope = Slope_Calculate(0, 4, LRSpeed_Record[1]);

    if (Distance > 1.5f)
    {
        //�������ת�����
        if (fabs(left_speed_slope) > CHANGE_RATE_THRESHOLD)   //�������ٱ仯���쳣
            Left_State = Unusual;
        if (fabs(Current_Left) > WHEEL_SPEED_THRESHOLD)     //���ַ�ת
            Left_State = Crazy;
        if (Current_Left <0)     //���ַ�ת
            Left_State = Reverse;
        if (Left_State != Normal && fabs(left_speed_slope) <= CHANGE_RATE_THRESHOLD
            && fabs(Current_Left) <= WHEEL_SPEED_THRESHOLD)     //���ָֻ�����
            Left_State = Normal;
        //�������ת�����
        if (fabs(right_speed_slope) > CHANGE_RATE_THRESHOLD) //�������ٱ仯���쳣
            Right_State = Unusual;
        if (fabs(Current_Right) > WHEEL_SPEED_THRESHOLD)     //���ַ�ת
            Right_State = Crazy;
        if (Current_Right < 0)     //���ַ�ת
            Right_State = Reverse;
        if (Right_State != Normal && fabs(right_speed_slope) <= CHANGE_RATE_THRESHOLD
           && fabs(Current_Right) <= WHEEL_SPEED_THRESHOLD)     //���ָֻ�����
            Right_State = Normal;
    }
    //����״̬����
    if ((Left_State == Unusual || Left_State == Crazy) && (Right_State == Unusual || Right_State == Crazy))
    {
        Current_Left = Last_Left;     //�������ֽ��쳣����ǰ���������ϴγ���
        Current_Right = Last_Right;
        LRSpeed_Record[0][4] = Last_Left;
        LRSpeed_Record[1][4] = Last_Right;
    }
    else if ((Left_State != Normal)&&(Right_State == Normal))
    {
            Current_Left = Last_Left;     //�����쳣����������
            LRSpeed_Record[0][4] = Last_Left;
    }
    else if ((Right_State != Normal)&&(Left_State == Normal))
    {
            Current_Right = Last_Right;     //�����쳣����������
            LRSpeed_Record[1][4] = Last_Right;
    }

    LR_Speed_Err = (Current_Left - Current_Right)*0.0173f;     //��ǰ���ٲ�
    Left_Car_Speed=Current_Left* 0.0173f;
    Right_Car_Speed=Current_Right* 0.0173f;                          //V = N/1024*��30/68*20.1/100*20�� = N*0.0017 m/s
    Car_Speed =0.5f * (Left_Car_Speed + Right_Car_Speed);      //�������ת��ΪM/S  N/1024 *��30/68*20.1cm/50ms�� = Vm/s,30Ϊ���������֣�68Ϊ�������֣�20.1Ϊ�����ܳ�����������50ms
    Distance += 0.005 * Car_Speed;                         //ȫ�̼�¼·�� S=Vt
    user_distance=Distance;
}
/***************************************************
*                 �ٶ�Kpģ��
*  ���ͣ�����
*  ����ֵ����
*  ���ܣ��ٶ�������Kp��
***************************************************/
float Update_kp_Speed(int error, int absmin, int absmax,float kp)
{
    if(abs(error) < absmin) return kp;
    else if(abs(error) > absmax) return 0;
    else return kp*(absmax-abs(error))/(absmax-absmin);
}
/***************************************************
*                 ���ְ�����
*  ���ͣ�����
*  ����ֵ����
*  ���ܣ��������٣����ݶ�������ת��Ϊ��Ҫ���ٵ��ٶ�
 Increase =  P * (Current_Error - Last_Error) +
 I * Current_Error  +
 D  * (Current_Error  - 2 * Last_Error + Previous_Error);   //����ʽPID��ʽ
***************************************************/
void Ackerman_Left(float current_speed)
{
    if(L_straightaway==1 && R_straightaway==1 && Barrier_state==0 && Charge_Resue_state==0 && Barrier_out_count==0)   //ֱ���ͷ��ϰ��Ǿ�Ԯ����
    {
        Control_Left_Speed.user_speed=straightaway_speed+straightaway_speed*Ackerman(streeing_duty);
        Stop_car_flag=0;
    }
    else
    {
      if(Barrier_state!=0)              //�ϰ�����߽׶�
       {
            if(Barrier_state==1)     //Ԥ�жϽ׶�
            {
            Control_Left_Speed.user_speed=0.2;
            Stop_car_flag=0;
            }
            else if(Barrier_state==2) //���д�ǳ������׶�
            {
            Control_Left_Speed.user_speed=0;
            Stop_car_flag=0;
            }
            else if(Barrier_state==3) //���д�ǻ������׶�
            {
            Control_Left_Speed.user_speed=1.6;
            Stop_car_flag=0;
            }
       }
      else if(Charge_Resue_state==1) //��Ԯǰ��
       {
        Control_Left_Speed.user_speed=0.5;
        Stop_car_flag=0;
       }
      else if(Charge_Resue_state==2) //��Ԯ����
       {
        Control_Left_Speed.user_speed= Control_Left_Speed.rescue_speed;
        Stop_car_flag=1;
       }
      else if(Charge_Resue_state==3) //���ͣ��
       {
        Control_Left_Speed.user_speed=0;
        Stop_car_flag=1;
       }
      else if(Break_road_state)    //��·���ٽ׶�
      {
        Control_Left_Speed.user_speed=Break_road_speed+Break_road_speed*Ackerman(streeing_duty);
        Stop_car_flag=0;
      }
      else if(Ramp_state==2)   //���½׶μ���
      {
        Control_Left_Speed.user_speed=1.4+1.4*Ackerman(streeing_duty);
        Stop_car_flag=0;
      }
      else if(Barrier_out_count==1)   //���ϰ���֮����м��٣���ֹͻȻ������̬����
      {
        Control_Left_Speed.user_speed=1.5+1.5*Ackerman(streeing_duty);
        Stop_car_flag=0;
      }
      else                 //��Ԫ�ط�ֱ���׶�
       {
        Control_Left_Speed.user_speed=curve_speed+curve_speed*Ackerman(streeing_duty);
        Stop_car_flag=0;
       }

    }
    Control_Left_Speed.err = Control_Left_Speed.user_speed- current_speed;
    Control_Left_Speed.integral = Pid_Left_Speed.I * Control_Left_Speed.err;
    Control_Left_Speed.Speed_Duty += Pid_Left_Speed.P * (Control_Left_Speed.err- Control_Left_Speed.Last_err) + Control_Left_Speed.integral
                    + Pid_Left_Speed.D*(Control_Left_Speed.err  - 2 *  Control_Left_Speed.Last_err +  Control_Left_Speed.Previous_Error);

     //��������
     if((Control_Left_Speed.err) >= 0.5)
     {
      Control_Left_Speed.Speed_Duty +=bangbang_duty;
     }
     if((Control_Left_Speed.err)<= -0.5)
     {
      Control_Left_Speed.Speed_Duty-=bangbang_duty;
     }

    if(Charge_Resue_state>=2 || Barrier_state)         //��Ԯ�׶��Լ��ϰ���׶β�����������
      Control_Left_Speed.Speed_Duty = LIMIT( Control_Left_Speed.Speed_Duty, MOTOMAX,-2000);
    else if(Ramp_state)
    {
      Control_Left_Speed.Speed_Duty = LIMIT( Control_Left_Speed.Speed_Duty, 2900,-2980);
    }
    else
      Control_Left_Speed.Speed_Duty = LIMIT( Control_Left_Speed.Speed_Duty, MOTOMAX,980);


    Control_Left_Speed.Speed_Old_Duty = Control_Left_Speed.Speed_Duty;  //�ٶȸ���
    Control_Left_Speed.Previous_Error = Control_Left_Speed.Last_err;   //���ϴ�������
    Control_Left_Speed.Last_err = Control_Left_Speed.err;               //������

    if(Distance<=0.3)//��ֹ��������
    Control_Left_Speed.Speed_Duty = LIMIT( Control_Left_Speed.Speed_Duty, 1800,-3000);
}
/***************************************************
*                 ���ְ�����
*  ���ͣ�����
*  ����ֵ����
*  ���ܣ��������٣����ݶ�������ת��Ϊ��Ҫ���ٵ��ٶ�
 Increase =  P * (Current_Error - Last_Error) +
 I * Current_Error  +
 D  * (Current_Error  - 2 * Last_Error + Previous_Error);   //����ʽPID��ʽ
***************************************************/
void Ackerman_Right(float current_speed)
{
    if(L_straightaway==1 && R_straightaway==1 && Barrier_state==0 && Charge_Resue_state==0 && Barrier_out_count==0)      //ֱ������
    {
        Stop_car_flag=0;
        Control_Right_Speed.user_speed=straightaway_speed-straightaway_speed*Ackerman(streeing_duty);
    }
    else
    {
        if(Barrier_state!=0)              //�ϰ�����߽׶�
         {
              if(Barrier_state==1)
              {
                  Control_Right_Speed.user_speed=0.2; //Ԥ�жϽ׶�
                  Stop_car_flag=0;
              }
              else if(Barrier_state==2)
              {
                  Control_Right_Speed.user_speed=1.6; //��ǳ������׶�
                  Stop_car_flag=0;
              }
              else if(Barrier_state==3)
              {
                  Control_Right_Speed.user_speed=0; //��ǻ������׶�
                  Stop_car_flag=0;
              }
         }
        else if(Charge_Resue_state==1) //��Ԯǰ��
         {
            Control_Right_Speed.user_speed =0.5;
            Stop_car_flag=0;
         }
        else if(Charge_Resue_state==2) //��Ԯ����
         {
            Control_Right_Speed.user_speed = Control_Right_Speed.rescue_speed;
            Stop_car_flag=1;
         }
        else if(Charge_Resue_state==3) //���ͣ��
         {
            Control_Right_Speed.user_speed=0;
            Stop_car_flag=1;
         }
        else if(Break_road_state)   //��·�ٶ�
        {
            Control_Right_Speed.user_speed=Break_road_speed-Break_road_speed*Ackerman(streeing_duty);
            Stop_car_flag=0;
        }
        else if(Ramp_state==2)   //���½׶μ���
        {
          Control_Left_Speed.user_speed=1.4-1.4*Ackerman(streeing_duty);
          Stop_car_flag=0;
        }
        else if(Barrier_out_count==1)     //���ϰ���֮����м��٣���ֹͻȻ������̬����
        {
          Control_Right_Speed.user_speed=1.5-1.5*Ackerman(streeing_duty);
          Stop_car_flag=0;
        }
        else
        {
            Control_Right_Speed.user_speed=curve_speed-curve_speed*Ackerman(streeing_duty);
            Stop_car_flag=0;
        }
    }

    Control_Right_Speed.err =Control_Right_Speed.user_speed- current_speed;

    Control_Right_Speed.integral = Pid_Right_Speed.I * Control_Right_Speed.err;
    Control_Right_Speed.Speed_Duty += Pid_Right_Speed.P * (Control_Right_Speed.err-Control_Right_Speed.Last_err) + Control_Right_Speed.integral
                 + Pid_Right_Speed.D*(Control_Right_Speed.err  - 2 *  Control_Right_Speed.Last_err +  Control_Right_Speed.Previous_Error);

    //��������
    if((Control_Right_Speed.err) >= 0.5)
    {
     Control_Right_Speed.Speed_Duty +=bangbang_duty;
    }
    if((Control_Right_Speed.err)<= -0.5)
    {
     Control_Right_Speed.Speed_Duty-=bangbang_duty;
    }

    if(Charge_Resue_state>=2 || Barrier_state)  //�ϰ���׶κ;�Ԯ�׶β�������������
       Control_Right_Speed.Speed_Duty = LIMIT( Control_Right_Speed.Speed_Duty, MOTOMAX,-2000);
    else if(Ramp_state)
    {
       Control_Right_Speed.Speed_Duty = LIMIT( Control_Right_Speed.Speed_Duty, 2900,-2980);
    }
    else
       Control_Right_Speed.Speed_Duty = LIMIT( Control_Right_Speed.Speed_Duty, MOTOMAX,980);

    Control_Right_Speed.Speed_Old_Duty = Control_Right_Speed.Speed_Duty;  //�ٶȸ���
    Control_Right_Speed.Previous_Error = Control_Right_Speed.Last_err;   //���ϴ�������
    Control_Right_Speed.Last_err = Control_Right_Speed.err;               //������

    if(Distance<=0.3)
    Control_Right_Speed.Speed_Duty = LIMIT( Control_Right_Speed.Speed_Duty, 1800,-3000);                                //��ֹ��������
}
/***************************************************
*                 �ٶ��⻷����
*  ���ͣ�����
*  ����ֵ����
*  ���ܣ�|�⻷|���ٶȿ���
*pwm=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
***************************************************/
void Speed_Control_Change(Control_Speed_Type_Def *control_speed, PID_Type_Def *pid, float current_speed)
{
    control_speed->err =  control_speed->element - current_speed;
    control_speed->Speed_Duty += pid->P *  (control_speed->err-control_speed->Last_err) +  control_speed->integral
                 + pid->D*( control_speed->err  - 2 *   control_speed->Last_err +   control_speed->Previous_Error);
   //��������
    if((control_speed->err) >= 0.5)
    {
     control_speed->Speed_Duty+=bangbang_duty;
    }

    if((control_speed->err)<= -0.5)
    {
     control_speed->Speed_Duty-=bangbang_duty;
    }

    control_speed->Speed_Duty = LIMIT( control_speed->Speed_Duty, MOTOMAX,-1980);

    control_speed->Speed_Old_Duty = control_speed->Speed_Duty;  //�ٶȸ���
    control_speed->Previous_Error = control_speed->Last_err;   //���ϴ�������
    control_speed->Last_err=control_speed->err;                 //������                            //��ֹ��������
}
/***************************************************
*                 ��������������ģ��
*  ���ͣ�����
*  ��ڲ��������ռ�ձ�
*  ����ֵ������ֵ
*  ���ܣ���������
***************************************************/
float Ackerman(float duty)
{
    float angle=0;
    float diff=0;
    angle=duty/10.0;
    diff=0.3214*tan(angle*2.45*Radian);//ϸ������Ϊ0.4333
    return diff;
}
/***************************************************
*  ���ٲ����(�µ���ʹ��)��
*  ���ͣ�����
*  ��ڲ�������ǰ���ٲ�
*  ����ֵ����
*  ���ܣ��µ�ת������,ʹ����ʱ����ת��ƽ��
***************************************************/
void Wheel_Speed_Err_Control(float err)
{
    static float LRSpeedErrOld = 0.0f, LRSpeedErrNew = 0.0f;

    LRSpeedErrOld = LRSpeedErrNew;
    LRSpeedErrNew = 0.3 * LRSpeedErrOld + 0.7 * err;

    Pid_LRSpeed.Pout = Pid_LRSpeed.P * LRSpeedErrNew;
    Pid_LRSpeed.Dout = Pid_LRSpeed.D * (LRSpeedErrNew - LRSpeedErrOld);

    Pid_LRSpeed.Out = Pid_LRSpeed.Pout + Pid_LRSpeed.Dout;
    Pid_LRSpeed.Out = LIMIT(Pid_LRSpeed.Out, 1000, -1000);
}
/***************************************************
*                ͣ������
*  ���ͣ�����
*  ����ֵ����
*  ���ܣ�ͣ��
***************************************************/
void Car_Protect(float current_left,float current_Right)
{
  float Left_duty=0,Right_duty=0;
  Left_duty=Car_Stop_PID(&L_car_stop_pid,&L_car_stop_err,current_left);
  Right_duty=Car_Stop_PID(&R_car_stop_pid,&R_car_stop_err,current_Right);
  wheel_speed(Left_duty, Right_duty);
}
/***************************************************
*                 ������
*  ���ͣ�����
*  ����ֵ����
*  ���ܣ�������
***************************************************/
void Moto_Out()
{
    if (Ramp_state>1)                     //�µ�
    {
        Moto_Out_Left = Control_Left_Speed.Speed_Duty- Pid_LRSpeed.Out;   //������ʱ���
        Moto_Out_Right = Control_Right_Speed.Speed_Duty + Pid_LRSpeed.Out;    //�ҵ����ʱ���
    }
    else
    {
        Moto_Out_Left = Control_Left_Speed.Speed_Duty;    //������ʱ���
        Moto_Out_Right =Control_Right_Speed.Speed_Duty;    //�ҵ����ʱ���
    }
    Moto_Out_Left = LIMIT(Moto_Out_Left, MOTOMAX, MOTOMIN);    //��������޷�
    Moto_Out_Right = LIMIT(Moto_Out_Right, MOTOMAX, MOTOMIN);    //�ҵ������޷�
    wheel_speed(Moto_Out_Left, Moto_Out_Right);
}

/***************************************************
*                 ���ģ��ƥ��
*  ���ͣ�����
*  ����ֵ����
***************************************************/
void Template_motor()
{
    if((inductor_check_count==0 && Barrier_state==0 && Distance>1) || Door_count==track_num)//������ͣ��,����ʶ�𵽳���ͣ��
    {
      Stop_car_flag=1;
      Car_Protect(Left_Car_Speed,Right_Car_Speed);
    }else {
      Ackerman_Left(Left_Car_Speed);
      Ackerman_Right(Right_Car_Speed);
    }
}
/***************************************************
*                 ������ƥ��
*  ���ͣ�����
*  ����ֵ����
*  ���ܣ�������λ�ý�����Ϊ�ı������
***************************************************/
void Template_motor_out()
{
    if(Door_count<track_num)                                    //�������
     {
        if(inductor_check_count || Barrier_state!=0)         //������������������ϰ��ﴦ��������
          {
              Moto_Out();
          }
     }
}

/***************************************************
*                 ��pidר��
*  ���ͣ�����
*  ����ֵ����
*  ��������
*  ���ܣ����е�pid
*  ʹ�ã��ڶ�ʱ���е��ã�2s���٣�1s���٣�1s����
***************************************************/
void SPEED_PID_DUBGE()
{
   if(Speed_PID_debug)
       Speed_PID_count++;
   if(Speed_PID_debug==1)
   {
     if(Speed_PID_count<=200)
     {
     // Stop_car_flag=0; //������������
      Control_Left_Speed.element=2.1;
      Control_Right_Speed.element=2.1;
      Speed_Control_Change(&Control_Left_Speed,&Pid_Left_Speed,Left_Car_Speed);
      Speed_Control_Change(&Control_Right_Speed,&Pid_Right_Speed,Right_Car_Speed);
     }
     else if(Speed_PID_count<=300 && Speed_PID_count>200)
   {
     Stop_car_flag=1;  //������������
     Control_Left_Speed.element=0;
     Control_Right_Speed.element=0;
     Speed_Control_Change(&Control_Left_Speed,&Pid_Left_Speed,Left_Car_Speed);
     Speed_Control_Change(&Control_Right_Speed,&Pid_Right_Speed,Right_Car_Speed);
   }
     else {            //��գ��ȴ��´���������
      Speed_PID_debug=0;
      Speed_PID_count=0;
      Control_Left_Speed.Speed_Duty=0;
      Control_Right_Speed.Speed_Duty=0;
   }
  }
}


