/*
 * motor.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef MOTOR_H_
#define MOTOR_H_
#include "zf_common_typedef.h"
//����״̬ö��
typedef enum wheel_state {
    Normal = 0,
    Crazy,
    Reverse,
    Unusual,
} Wheel_State;
//PID��������
typedef struct pid_tag{
    float P;            //����ϵ��
    float I;            //����ϵ��
    float D;            //΢��ϵ��
    float Pout;         //���������
    float Iout;         //���������
    float Dout;         //΢�������
    float Out;          //�����
} PID_Type_Def;

//�ٶȿ�������
typedef struct control_speed_tag {
    float stright;          //ֱ���ٶ�
    float cruve;           //����ٶ�
    float user_speed;     //����ʹ���ٶ�
    float element;       //����Ԫ���ٶ�
    float rescue_speed;    //��Ԯʱ���ٶ�
    float barrier_speed;    //���ϰ�����ٶ�
    float err;          //���
    float Last_err;    //�ϴ����
    float Previous_Error;      //���ϴ����
    float integral;           //����
    float Speed_Duty;        //ռ�ձ�
    float Speed_Old_Duty;   //��һ��ռ�ձ�
} Control_Speed_Type_Def;

//����������
typedef struct time_count_tag {
    uint16 count_ms;                   //ms��ʱ��
    uint8 get_speed_conut;            //�ٶȲɼ�������
    uint8 control_speed_count;       //�ٶ��⻷���Ƽ�����
    uint8 turn_count;               //���ٶȻ����Ƽ�����
    uint8 turn_acc_count;          //�ǶȻ����Ƽ�����
    uint8 turn_streeing_count;    //���ת�򻷼�����
    uint8 element_deal;          //Ԫ�ش��������
} Time_Count_Type_Def;

void Init_contral(void);
void Init_encoder(void);
void Get_Speed(void);
void Speed_Control_Change(Control_Speed_Type_Def *control_speed, PID_Type_Def *pid, float current_speed);
void Wheel_Speed_Err_Control(float err);
void Ackerman_Left(float current_speed);
void Ackerman_Right(float current_speed);
float Ackerman(float duty);
void Car_Protect(float current_left,float current_Right);
void Moto_Out(void);
void Template_motor(void);
void Template_motor_out(void);
void SPEED_PID_DUBGE(void);
#endif /* MOTOR_H_ */
