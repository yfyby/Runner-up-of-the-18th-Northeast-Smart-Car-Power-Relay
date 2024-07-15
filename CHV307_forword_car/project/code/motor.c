/*
 * motor.c
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */
/*********************************************************************************************************************
 * 电机控制源文件
 ********************************************************************************************************************/
#include "some_algorithm.h"
#include "wheel.h"
#include "motor.h"
#include "zf_driver_encoder.h"
#include "math.h"
#include "system.h"
#include "streeing.h"
#define Radian          0.0175f                       // 1/180*3.14159
#define MOTOMAX         5000                         //电机最大占空比
#define MOTOMIN        -5000
#define CHANGE_RATE_THRESHOLD         1000      //（编码器）轮速变化率阈值
#define WHEEL_SPEED_THRESHOLD         2000       //（编码器）轮速阈值
int16 Moto_Out_Left = 0;                       //左电机最终输出占空比
int16 Moto_Out_Right = 0;                     //右电机最终输出占空比
float Car_Speed = 0;                            //车速m/s
float Left_Car_Speed=0;                          //左轮速度m/s
float Right_Car_Speed=0;                         //右轮速度m/s
float Distance = 0;                             //全程累加的距离
float Current_Left = 0;                         //左轮编码器值
float Current_Right = 0;                       //右轮编码器值
float Original_current_Left=0;                     //原始编码器值
float Original_current_Right=0;                     //原始编码器值
float LR_Speed_Err = 0;                       //两轮车速的差值
float user_distance=0;                    //记录的路程
float Break_road_speed=0.5;           //断路速度
float straightaway_speed=1.9;  //直道速度
float curve_speed=1.8;        //弯道速度
//速度控制类型变量
PID_Type_Def Pid_LRSpeed;                             //轮速差抑制(坡道使用)
PID_Type_Def Pid_Left_Speed;                         //左轮速度环
PID_Type_Def Pid_Right_Speed;                       //右轮速度环
Control_Speed_Type_Def Control_Left_Speed;          //左轮速度结构
Control_Speed_Type_Def Control_Right_Speed;        //右轮速度结构
Time_Count_Type_Def Time_Count;                   //计数器结构
//pid参数
float Car_KI=58.5;  //58.5
float Car_KD=0;      //200,给了好像效果差不多
float Car_KP=3050;
int16 bangbang_duty=100;
bool Stop_car_flag=0;       //停车标志位
/***************************************************
*                  初始化参数数据
*  类型：公有
*  返回值：无
***************************************************/
void Init_contral(){

    //默认速度相关参数
    Control_Left_Speed.stright =straightaway_speed;                              //基础直道速度2.3
    Control_Left_Speed.cruve = curve_speed;                             //基础弯道速度1.8
    Control_Left_Speed.user_speed = 0;
    Control_Left_Speed.element=2;                            //元素速度
    Control_Left_Speed.barrier_speed=0;                     //出障碍物的速度//默认出左
    Control_Left_Speed.rescue_speed=-0.5;                    //倒车速度
    Control_Left_Speed.err = 0;                             //误差
    Control_Left_Speed.Last_err = 0;                        //上次误差
    Control_Left_Speed.Previous_Error=0;                   //上上次误差
    Control_Left_Speed.integral = 0;                       //速度积分
    Control_Left_Speed.Speed_Duty=0;                      //占空比
    Control_Left_Speed.Speed_Old_Duty=0;                 //上一次占空比

    Control_Right_Speed.stright =straightaway_speed;      //基础直道速度
    Control_Right_Speed.cruve = curve_speed;                      //基础弯道速度
    Control_Right_Speed.user_speed = 0;
    Control_Right_Speed.element=2;                       //元素速度
    Control_Right_Speed.barrier_speed=1.5;                //出障碍物的速度//默认出左
    Control_Right_Speed.rescue_speed=-0.5;                  //倒车速度
    Control_Right_Speed.err = 0;                            //误差
    Control_Right_Speed.Last_err = 0;                      //上次误差
    Control_Right_Speed.Previous_Error=0;                 //上上次误差
    Control_Right_Speed.integral = 0;                    //速度积分
    Control_Right_Speed.Speed_Duty=0;                   //占空比
    Control_Right_Speed.Speed_Old_Duty=0;              //上一次占空比
    //速度外环pid
    Pid_Left_Speed.P = Car_KP;
    Pid_Left_Speed.I = Car_KI;
    Pid_Left_Speed.D = Car_KD;
    Pid_Left_Speed.Out =0.0;

    Pid_Right_Speed.P =Car_KP;
    Pid_Right_Speed.I =Car_KI;
    Pid_Right_Speed.D = Car_KD;
    Pid_Right_Speed.Out =0.0;
    //坡道轮速差控制pid
    Pid_LRSpeed.P = 300.0;
    Pid_LRSpeed.D = 0.0;
    Pid_LRSpeed.Out = 0.0;
    //计时器结构
    Time_Count.control_speed_count = 0;
    Time_Count.get_speed_conut = 0;
    Time_Count.count_ms = 0;
    Time_Count.turn_streeing_count = 0;
  }
/***************************************************
*                  初始化编码器
*  类型：公有
*  返回值：无
***************************************************/
void Init_encoder()
{
    encoder_dir_init(TIM1_ENCOEDER, TIM1_ENCOEDER_MAP3_CH1_E9, TIM1_ENCOEDER_MAP3_CH2_E11);
    encoder_clear_count(TIM1_ENCOEDER);
    encoder_dir_init(TIM8_ENCOEDER, TIM8_ENCOEDER_MAP0_CH1_C6, TIM8_ENCOEDER_MAP0_CH2_C7);
    encoder_clear_count(TIM8_ENCOEDER);
}
/***************************************************
*                获取车身速度参数(5ms进行一次)
*  类型：公有
*  返回值：无
*  功能：采集速度，较好地解决了轮速因抬轮、颠簸、坡道等
*        因素的影响
***************************************************/
void Get_Speed()
{
    int16 temp_l, temp_r;
    Para_Type type;  //卡尔曼滤波参数
    static float  Last_Left = 0.0f, Last_Right = 0.0f;
    static Wheel_State Left_State = Normal, Right_State = Normal;  //车轮状态初始化正常
    static float LRSpeed_Record[2][5] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};     //左右轮速记录数组
    float left_speed_slope = 0.0f, right_speed_slope = 0.0f;    //左右轮速变化率

    if(Stop_car_flag==0) //不是停车和倒车
    {
    temp_l = abs(encoder_get_count(TIM1_ENCOEDER));//左边的编码器读数
        encoder_clear_count(TIM1_ENCOEDER);
    temp_r = abs(encoder_get_count(TIM8_ENCOEDER));//右边的编码器读数
        encoder_clear_count(TIM8_ENCOEDER);
    }
    else                  //停车和倒车需要极性
    {
    temp_l = (encoder_get_count(TIM1_ENCOEDER));//左边的编码器读数
            encoder_clear_count(TIM1_ENCOEDER);
    temp_r = -(encoder_get_count(TIM8_ENCOEDER));//右边的编码器读数
            encoder_clear_count(TIM8_ENCOEDER);
    }

    Original_current_Left=temp_l;
    Original_current_Right=temp_r;

    Last_Left = Current_Left;
    Last_Right = Current_Right;
    //滤波
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

    //左轮轮速记录
    for (uint8 i = 0; i < 4; i++)
        LRSpeed_Record[0][i] = LRSpeed_Record[0][i + 1];
    LRSpeed_Record[0][4] = temp_l;
    //右轮轮速记录
    for (uint8 i = 0; i < 4; i++)
        LRSpeed_Record[1][i] = LRSpeed_Record[1][i + 1];
    LRSpeed_Record[1][4] = temp_r;

    //最小二乘估计左右轮轮速变化率
    left_speed_slope = Slope_Calculate(0, 4, LRSpeed_Record[0]);
    right_speed_slope = Slope_Calculate(0, 4, LRSpeed_Record[1]);

    if (Distance > 1.5f)
    {
        //监测左轮转动情况
        if (fabs(left_speed_slope) > CHANGE_RATE_THRESHOLD)   //左轮轮速变化率异常
            Left_State = Unusual;
        if (fabs(Current_Left) > WHEEL_SPEED_THRESHOLD)     //左轮疯转
            Left_State = Crazy;
        if (Current_Left <0)     //左轮反转
            Left_State = Reverse;
        if (Left_State != Normal && fabs(left_speed_slope) <= CHANGE_RATE_THRESHOLD
            && fabs(Current_Left) <= WHEEL_SPEED_THRESHOLD)     //左轮恢复正常
            Left_State = Normal;
        //监测右轮转动情况
        if (fabs(right_speed_slope) > CHANGE_RATE_THRESHOLD) //右轮轮速变化率异常
            Right_State = Unusual;
        if (fabs(Current_Right) > WHEEL_SPEED_THRESHOLD)     //右轮疯转
            Right_State = Crazy;
        if (Current_Right < 0)     //右轮反转
            Right_State = Reverse;
        if (Right_State != Normal && fabs(right_speed_slope) <= CHANGE_RATE_THRESHOLD
           && fabs(Current_Right) <= WHEEL_SPEED_THRESHOLD)     //左轮恢复正常
            Right_State = Normal;
    }
    //车轮状态处理
    if ((Left_State == Unusual || Left_State == Crazy) && (Right_State == Unusual || Right_State == Crazy))
    {
        Current_Left = Last_Left;     //左右两轮皆异常，当前车速沿用上次车速
        Current_Right = Last_Right;
        LRSpeed_Record[0][4] = Last_Left;
        LRSpeed_Record[1][4] = Last_Right;
    }
    else if ((Left_State != Normal)&&(Right_State == Normal))
    {
            Current_Left = Last_Left;     //左轮异常，右轮正常
            LRSpeed_Record[0][4] = Last_Left;
    }
    else if ((Right_State != Normal)&&(Left_State == Normal))
    {
            Current_Right = Last_Right;     //右轮异常，左轮正常
            LRSpeed_Record[1][4] = Last_Right;
    }

    LR_Speed_Err = (Current_Left - Current_Right)*0.0173f;     //当前轮速差
    Left_Car_Speed=Current_Left* 0.0173f;
    Right_Car_Speed=Current_Right* 0.0173f;                          //V = N/1024*（30/68*20.1/100*20） = N*0.0017 m/s
    Car_Speed =0.5f * (Left_Car_Speed + Right_Car_Speed);      //求出车速转换为M/S  N/1024 *（30/68*20.1cm/50ms） = Vm/s,30为编码器齿轮，68为传动齿轮，20.1为轮子周长，控制周期50ms
    Distance += 0.005 * Car_Speed;                         //全程记录路程 S=Vt
    user_distance=Distance;
}
/***************************************************
*                 速度Kp模糊
*  类型：公有
*  返回值：无
*  功能：速度误差决定Kp项
***************************************************/
float Update_kp_Speed(int error, int absmin, int absmax,float kp)
{
    if(abs(error) < absmin) return kp;
    else if(abs(error) > absmax) return 0;
    else return kp*(absmax-abs(error))/(absmax-absmin);
}
/***************************************************
*                 左轮阿克曼
*  类型：公有
*  返回值：无
*  功能：主动差速，根据舵机打角来转换为需要差速的速度
 Increase =  P * (Current_Error - Last_Error) +
 I * Current_Error  +
 D  * (Current_Error  - 2 * Last_Error + Previous_Error);   //增量式PID公式
***************************************************/
void Ackerman_Left(float current_speed)
{
    if(L_straightaway==1 && R_straightaway==1 && Barrier_state==0 && Charge_Resue_state==0 && Barrier_out_count==0)   //直道和非障碍非救援加速
    {
        Control_Left_Speed.user_speed=straightaway_speed+straightaway_speed*Ackerman(streeing_duty);
        Stop_car_flag=0;
    }
    else
    {
      if(Barrier_state!=0)              //障碍物决策阶段
       {
            if(Barrier_state==1)     //预判断阶段
            {
            Control_Left_Speed.user_speed=0.2;
            Stop_car_flag=0;
            }
            else if(Barrier_state==2) //进行打角出赛道阶段
            {
            Control_Left_Speed.user_speed=0;
            Stop_car_flag=0;
            }
            else if(Barrier_state==3) //进行打角回赛道阶段
            {
            Control_Left_Speed.user_speed=1.6;
            Stop_car_flag=0;
            }
       }
      else if(Charge_Resue_state==1) //救援前进
       {
        Control_Left_Speed.user_speed=0.5;
        Stop_car_flag=0;
       }
      else if(Charge_Resue_state==2) //救援倒车
       {
        Control_Left_Speed.user_speed= Control_Left_Speed.rescue_speed;
        Stop_car_flag=1;
       }
      else if(Charge_Resue_state==3) //充电停车
       {
        Control_Left_Speed.user_speed=0;
        Stop_car_flag=1;
       }
      else if(Break_road_state)    //断路减速阶段
      {
        Control_Left_Speed.user_speed=Break_road_speed+Break_road_speed*Ackerman(streeing_duty);
        Stop_car_flag=0;
      }
      else if(Ramp_state==2)   //下坡阶段减速
      {
        Control_Left_Speed.user_speed=1.4+1.4*Ackerman(streeing_duty);
        Stop_car_flag=0;
      }
      else if(Barrier_out_count==1)   //过障碍物之后进行减速，防止突然加速姿态不对
      {
        Control_Left_Speed.user_speed=1.5+1.5*Ackerman(streeing_duty);
        Stop_car_flag=0;
      }
      else                 //非元素非直道阶段
       {
        Control_Left_Speed.user_speed=curve_speed+curve_speed*Ackerman(streeing_duty);
        Stop_car_flag=0;
       }

    }
    Control_Left_Speed.err = Control_Left_Speed.user_speed- current_speed;
    Control_Left_Speed.integral = Pid_Left_Speed.I * Control_Left_Speed.err;
    Control_Left_Speed.Speed_Duty += Pid_Left_Speed.P * (Control_Left_Speed.err- Control_Left_Speed.Last_err) + Control_Left_Speed.integral
                    + Pid_Left_Speed.D*(Control_Left_Speed.err  - 2 *  Control_Left_Speed.Last_err +  Control_Left_Speed.Previous_Error);

     //棒棒控制
     if((Control_Left_Speed.err) >= 0.5)
     {
      Control_Left_Speed.Speed_Duty +=bangbang_duty;
     }
     if((Control_Left_Speed.err)<= -0.5)
     {
      Control_Left_Speed.Speed_Duty-=bangbang_duty;
     }

    if(Charge_Resue_state>=2 || Barrier_state)         //救援阶段以及障碍物阶段不做死区限制
      Control_Left_Speed.Speed_Duty = LIMIT( Control_Left_Speed.Speed_Duty, MOTOMAX,-2000);
    else if(Ramp_state)
    {
      Control_Left_Speed.Speed_Duty = LIMIT( Control_Left_Speed.Speed_Duty, 2900,-2980);
    }
    else
      Control_Left_Speed.Speed_Duty = LIMIT( Control_Left_Speed.Speed_Duty, MOTOMAX,980);


    Control_Left_Speed.Speed_Old_Duty = Control_Left_Speed.Speed_Duty;  //速度更新
    Control_Left_Speed.Previous_Error = Control_Left_Speed.Last_err;   //上上次误差更新
    Control_Left_Speed.Last_err = Control_Left_Speed.err;               //误差更新

    if(Distance<=0.3)//防止弹射起跑
    Control_Left_Speed.Speed_Duty = LIMIT( Control_Left_Speed.Speed_Duty, 1800,-3000);
}
/***************************************************
*                 右轮阿克曼
*  类型：公有
*  返回值：无
*  功能：主动差速，根据舵机打角来转换为需要差速的速度
 Increase =  P * (Current_Error - Last_Error) +
 I * Current_Error  +
 D  * (Current_Error  - 2 * Last_Error + Previous_Error);   //增量式PID公式
***************************************************/
void Ackerman_Right(float current_speed)
{
    if(L_straightaway==1 && R_straightaway==1 && Barrier_state==0 && Charge_Resue_state==0 && Barrier_out_count==0)      //直道加速
    {
        Stop_car_flag=0;
        Control_Right_Speed.user_speed=straightaway_speed-straightaway_speed*Ackerman(streeing_duty);
    }
    else
    {
        if(Barrier_state!=0)              //障碍物决策阶段
         {
              if(Barrier_state==1)
              {
                  Control_Right_Speed.user_speed=0.2; //预判断阶段
                  Stop_car_flag=0;
              }
              else if(Barrier_state==2)
              {
                  Control_Right_Speed.user_speed=1.6; //打角出赛道阶段
                  Stop_car_flag=0;
              }
              else if(Barrier_state==3)
              {
                  Control_Right_Speed.user_speed=0; //打角回赛道阶段
                  Stop_car_flag=0;
              }
         }
        else if(Charge_Resue_state==1) //救援前进
         {
            Control_Right_Speed.user_speed =0.5;
            Stop_car_flag=0;
         }
        else if(Charge_Resue_state==2) //救援倒车
         {
            Control_Right_Speed.user_speed = Control_Right_Speed.rescue_speed;
            Stop_car_flag=1;
         }
        else if(Charge_Resue_state==3) //充电停车
         {
            Control_Right_Speed.user_speed=0;
            Stop_car_flag=1;
         }
        else if(Break_road_state)   //断路速度
        {
            Control_Right_Speed.user_speed=Break_road_speed-Break_road_speed*Ackerman(streeing_duty);
            Stop_car_flag=0;
        }
        else if(Ramp_state==2)   //下坡阶段减速
        {
          Control_Left_Speed.user_speed=1.4-1.4*Ackerman(streeing_duty);
          Stop_car_flag=0;
        }
        else if(Barrier_out_count==1)     //过障碍物之后进行减速，防止突然加速姿态不对
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

    //棒棒控制
    if((Control_Right_Speed.err) >= 0.5)
    {
     Control_Right_Speed.Speed_Duty +=bangbang_duty;
    }
    if((Control_Right_Speed.err)<= -0.5)
    {
     Control_Right_Speed.Speed_Duty-=bangbang_duty;
    }

    if(Charge_Resue_state>=2 || Barrier_state)  //障碍物阶段和救援阶段不进行死区限制
       Control_Right_Speed.Speed_Duty = LIMIT( Control_Right_Speed.Speed_Duty, MOTOMAX,-2000);
    else if(Ramp_state)
    {
       Control_Right_Speed.Speed_Duty = LIMIT( Control_Right_Speed.Speed_Duty, 2900,-2980);
    }
    else
       Control_Right_Speed.Speed_Duty = LIMIT( Control_Right_Speed.Speed_Duty, MOTOMAX,980);

    Control_Right_Speed.Speed_Old_Duty = Control_Right_Speed.Speed_Duty;  //速度更新
    Control_Right_Speed.Previous_Error = Control_Right_Speed.Last_err;   //上上次误差更新
    Control_Right_Speed.Last_err = Control_Right_Speed.err;               //误差更新

    if(Distance<=0.3)
    Control_Right_Speed.Speed_Duty = LIMIT( Control_Right_Speed.Speed_Duty, 1800,-3000);                                //防止弹射起跑
}
/***************************************************
*                 速度外环控制
*  类型：公有
*  返回值：无
*  功能：|外环|：速度控制
*pwm=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
***************************************************/
void Speed_Control_Change(Control_Speed_Type_Def *control_speed, PID_Type_Def *pid, float current_speed)
{
    control_speed->err =  control_speed->element - current_speed;
    control_speed->Speed_Duty += pid->P *  (control_speed->err-control_speed->Last_err) +  control_speed->integral
                 + pid->D*( control_speed->err  - 2 *   control_speed->Last_err +   control_speed->Previous_Error);
   //棒棒控制
    if((control_speed->err) >= 0.5)
    {
     control_speed->Speed_Duty+=bangbang_duty;
    }

    if((control_speed->err)<= -0.5)
    {
     control_speed->Speed_Duty-=bangbang_duty;
    }

    control_speed->Speed_Duty = LIMIT( control_speed->Speed_Duty, MOTOMAX,-1980);

    control_speed->Speed_Old_Duty = control_speed->Speed_Duty;  //速度更新
    control_speed->Previous_Error = control_speed->Last_err;   //上上次误差更新
    control_speed->Last_err=control_speed->err;                 //误差更新                            //防止弹射起跑
}
/***************************************************
*                 阿克曼主动差速模型
*  类型：公有
*  入口参数：舵机占空比
*  返回值：差速值
*  功能：主动差速
***************************************************/
float Ackerman(float duty)
{
    float angle=0;
    float diff=0;
    angle=duty/10.0;
    diff=0.3214*tan(angle*2.45*Radian);//细狗车的为0.4333
    return diff;
}
/***************************************************
*  轮速差控制(坡道上使用)·
*  类型：公有
*  入口参数：当前轮速差
*  返回值：无
*  功能：坡道转向抑制,使过坡时车体转向平稳
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
*                停车控制
*  类型：公有
*  返回值：无
*  功能：停车
***************************************************/
void Car_Protect(float current_left,float current_Right)
{
  float Left_duty=0,Right_duty=0;
  Left_duty=Car_Stop_PID(&L_car_stop_pid,&L_car_stop_err,current_left);
  Right_duty=Car_Stop_PID(&R_car_stop_pid,&R_car_stop_err,current_Right);
  wheel_speed(Left_duty, Right_duty);
}
/***************************************************
*                 电机输出
*  类型：公有
*  返回值：无
*  功能：电机输出
***************************************************/
void Moto_Out()
{
    if (Ramp_state>1)                     //坡道
    {
        Moto_Out_Left = Control_Left_Speed.Speed_Duty- Pid_LRSpeed.Out;   //左电机临时输出
        Moto_Out_Right = Control_Right_Speed.Speed_Duty + Pid_LRSpeed.Out;    //右电机临时输出
    }
    else
    {
        Moto_Out_Left = Control_Left_Speed.Speed_Duty;    //左电机临时输出
        Moto_Out_Right =Control_Right_Speed.Speed_Duty;    //右电机临时输出
    }
    Moto_Out_Left = LIMIT(Moto_Out_Left, MOTOMAX, MOTOMIN);    //左电机输出限辐
    Moto_Out_Right = LIMIT(Moto_Out_Right, MOTOMAX, MOTOMIN);    //右电机输出限辐
    wheel_speed(Moto_Out_Left, Moto_Out_Right);
}

/***************************************************
*                 电机模板匹配
*  类型：公有
*  返回值：无
***************************************************/
void Template_motor()
{
    if((inductor_check_count==0 && Barrier_state==0 && Distance>1) || Door_count==track_num)//出赛道停车,或者识别到车库停车
    {
      Stop_car_flag=1;
      Car_Protect(Left_Car_Speed,Right_Car_Speed);
    }else {
      Ackerman_Left(Left_Car_Speed);
      Ackerman_Right(Right_Car_Speed);
    }
}
/***************************************************
*                 电机输出匹配
*  类型：公有
*  返回值：无
*  功能：在特殊位置进行人为改变电机输出
***************************************************/
void Template_motor_out()
{
    if(Door_count<track_num)                                    //不是入库
     {
        if(inductor_check_count || Barrier_state!=0)         //电感正常而或者是在障碍物处理可以输出
          {
              Moto_Out();
          }
     }
}

/***************************************************
*                 调pid专用
*  类型：公有
*  返回值：无
*  参数：无
*  功能：进行调pid
*  使用：在定时器中调用，2s加速，1s变速，1s减速
***************************************************/
void SPEED_PID_DUBGE()
{
   if(Speed_PID_debug)
       Speed_PID_count++;
   if(Speed_PID_debug==1)
   {
     if(Speed_PID_count<=200)
     {
     // Stop_car_flag=0; //编码器无正负
      Control_Left_Speed.element=2.1;
      Control_Right_Speed.element=2.1;
      Speed_Control_Change(&Control_Left_Speed,&Pid_Left_Speed,Left_Car_Speed);
      Speed_Control_Change(&Control_Right_Speed,&Pid_Right_Speed,Right_Car_Speed);
     }
     else if(Speed_PID_count<=300 && Speed_PID_count>200)
   {
     Stop_car_flag=1;  //编码器有正负
     Control_Left_Speed.element=0;
     Control_Right_Speed.element=0;
     Speed_Control_Change(&Control_Left_Speed,&Pid_Left_Speed,Left_Car_Speed);
     Speed_Control_Change(&Control_Right_Speed,&Pid_Right_Speed,Right_Car_Speed);
   }
     else {            //清空，等待下次摁键启动
      Speed_PID_debug=0;
      Speed_PID_count=0;
      Control_Left_Speed.Speed_Duty=0;
      Control_Right_Speed.Speed_Duty=0;
   }
  }
}


