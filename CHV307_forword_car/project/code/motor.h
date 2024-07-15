/*
 * motor.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef MOTOR_H_
#define MOTOR_H_
#include "zf_common_typedef.h"
//车轮状态枚举
typedef enum wheel_state {
    Normal = 0,
    Crazy,
    Reverse,
    Unusual,
} Wheel_State;
//PID控制类型
typedef struct pid_tag{
    float P;            //比例系数
    float I;            //积分系数
    float D;            //微分系数
    float Pout;         //比例项输出
    float Iout;         //积分项输出
    float Dout;         //微分项输出
    float Out;          //总输出
} PID_Type_Def;

//速度控制类型
typedef struct control_speed_tag {
    float stright;          //直道速度
    float cruve;           //弯道速度
    float user_speed;     //最终使用速度
    float element;       //特殊元素速度
    float rescue_speed;    //救援时候速度
    float barrier_speed;    //过障碍物的速度
    float err;          //误差
    float Last_err;    //上次误差
    float Previous_Error;      //上上次误差
    float integral;           //积分
    float Speed_Duty;        //占空比
    float Speed_Old_Duty;   //上一次占空比
} Control_Speed_Type_Def;

//计数器类型
typedef struct time_count_tag {
    uint16 count_ms;                   //ms计时器
    uint8 get_speed_conut;            //速度采集计数器
    uint8 control_speed_count;       //速度外环控制计数器
    uint8 turn_count;               //角速度环控制计数器
    uint8 turn_acc_count;          //角度环控制计数器
    uint8 turn_streeing_count;    //舵机转向环计数器
    uint8 element_deal;          //元素处理计数器
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
