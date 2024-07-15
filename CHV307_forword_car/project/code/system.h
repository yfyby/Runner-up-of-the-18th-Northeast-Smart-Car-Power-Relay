/*
 * system.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_
#include "zf_common_typedef.h"
#include "mypid.h"
#include "motor.h"
#include "communication.h"
void Init_All(void);
extern uint8 track_num;    //圈数
extern bool Speed_PID_debug;          //速度环调速
extern uint16 Speed_PID_count;         //调速计数
extern float ADD_angle;                                             //累加的角度;
extern uint8 start_line_point_row;                    //二值化起点从58行开始
extern float slope_to_angle;                     //斜率转化为角度
extern float parameterB;    //斜率
extern float parameterA;    //截距
extern uint8 Charge_Resue_state;        //救援状态
extern float add_gyro_x;      //x轴角速度积分
extern bool Barrier_dir;         //避障方向，1为左，0为右
extern bool OFF_Element_test;    //开启元素测试
extern bool OFF_Charge_Resue;  //1为开启充电救援，0为不开启
extern uint8 Out_door_type;
extern float straightaway_speed;  //直道速度
extern float curve_speed;        //弯道速度
extern float Break_road_speed;           //断路速度
extern int16 L_charge_big_thes,L_charge_small_thes,R_charge_big_thes,R_charge_small_thes;//可以停止开始倒车的电感阈值
//出车库需要累加的距离m
/***************重要变量*******************/
extern uint8 Charge_Resue_count;        //充电使能发射
extern bool Barrier_out_count;   //过障碍物进行减速
extern uint8 bottom_gray_threa;     //底部边界判断差比和阈值
extern uint8 Break_road_add_thres;     //断路增强阈值
extern bool l_gray_point,r_gray_point;
extern bool vert_gray_point;  //竖直差比和标志位，专用于判断断路
extern uint8 l_hightest;//左最高点
extern uint8 r_hightest;//右最高点
extern uint8 hightest_x; //最高点x坐标
extern uint8 Width_straight[60];    //直道宽度
extern bool OFF_Speed_test;  //开启速度环测试
extern uint8 Left_Line[60];
extern uint8 Left_Line_flag[60];
extern uint8 Right_Line[60];
extern uint8 Right_Line_flag[60];
extern uint8 Mid_Line[60];
extern uint8 Width_track[60];
extern uint8 start_point_l[2];//左边起点的x，y值
extern uint8 start_point_r[2];//右边起点的x，y值
extern uint8 labyrinth_start_point_l[2];//迷宫左边起点的x，y值
extern uint8 labyrinth_start_point_r[2];//迷宫右边起点的x，y值
extern uint8 hightest;//最高点
extern uint8 points_l[110][2];//左线
extern uint8 points_r[110][2];//右线
extern float  rpts_l[110][2];      //透视变换左线
extern float  rpts_r[110][2];      //透视变换右线
extern float  rpts_l_dist[110][2];      //等距采样透视变换左线
extern float  rpts_r_dist[110][2];      //等距采样透视变换右线
extern float  rpts_langle[110];      //左边界角度
extern float  rpts_rangle[110];      //右边界角度
extern float  rpts_langle_nms[110];  //进行极大抑制之后的左边界角度
extern float  rpts_rangle_nms[110];  //进行极大抑制之后的右边界角度
extern float  rpts_l_fitter[110][2];      //滤波后透视变换左线
extern float  rpts_r_fitter[110][2];      //滤波后透视变换右线
extern uint16 data_stastics_l;//统计左边找到点的个数
extern uint16 data_stastics_r0;//统计右边找到点的个数
extern uint8 inductor_check;      //检测电感是否正常标志位
extern uint8 Cross_Among;                                                         //在十字之间标志位
extern int16 Black_Point_Num;                                    //黑点数量
extern float Angle_ACC;                                  //累加的角度
extern uint8 Get_image[60][94];
extern uint8 User_image[60][94];                //最终使用的二值化图像
extern float Ring_add_angle;                         //圆环累加的角度
extern uint16 dl1a_distance_mm;                       //tof测距距离
extern int16 streeing_duty;                            //舵机占空比
extern float final_err;                            //循迹用误差
extern float tof_distance;                        //tof测距
extern float Barrier_Add_Angle;                 //累加的角度
extern float ADD_angle;                                             //累加的角度
extern float Ring_add_distance;            //累加的距离
extern uint8 inductor_check_count; //出赛道计数
/*****************图像相关标志位**********************/
extern uint8 REPIRE_LINE;                   //补线标志位
extern bool s_state[2];              //小S,中S,大S
extern bool Barrier_brake;                  //障碍物刹车标志位
extern bool L_straightaway; //左直道
extern bool R_straightaway; //右直道
extern uint8 slope_cross;               //斜入标志位
extern uint8 L_slope_cross_state;        //左斜入标志位
extern uint8 R_slope_cross_state;  //右斜入标志位
extern uint8 Mid_Line_Turncation;                                         //中线拐点环
extern uint8 Loss_L_line,Loss_R_line;                                 //左右丢线行数
extern uint8 Flag_Cross;                                           //十字路口状态位
extern uint8 Flag_L_CirCle;                                     //左圆环标志位
extern uint8 Flag_R_CirCle;                                    //右圆环标志位
extern uint8 L_CirCle_State;                                  //左圆环状态标志位
extern uint8 R_CirCle_State;                                //右圆环状态标志位
extern uint8 Indooor_state;                             //斑马线识别标志位
extern uint8 Door_count;                                //遇到斑马线次数,入库成功后值置成3，然后停车
extern uint8 Ramp_state;                               //坡道
extern uint8 Break_road_state;                        //断路状态标志位
extern uint8 Barrier_state;                          //障碍物状态
extern uint8 Out_door_state;                        //出车库状态
/**************电感相关*******************/
extern float Level_Deviation;                   //水平电感误差
extern float Vert_Deviation;                      //垂直电感误差
extern int16 Inductor_Value[4];                  //水平 垂直 水平  垂直  水平
extern int16 Inductor_Threshold[4];             //电感阈值采集
extern uint8 Flag_Round;                       //环岛标志
extern uint8 Flag_L_or_R;                    //左右圆环标志位
extern uint8 loss_line;                 //丢线指示，1左丢，2右丢，0不丢
extern uint8 loss_line_lock;            //丢线锁，丢线后自动上锁，1为上锁
/**************时间计数**************/
extern Time_Count_Type_Def Time_Count;                   //计数器结构
/**************pid相关标志位**************/
extern PID steer_pid,L_car_stop_pid,R_car_stop_pid;                  //舵机pid,停车pid
extern ERR L_car_stop_err,R_car_stop_err;
extern PID_Type_Def Pid_LRSpeed;                                  //轮速差抑制
extern PID_Type_Def Pid_Left_Speed;                              //左轮速度环
extern PID_Type_Def Pid_Right_Speed;                            //右轮速度环
extern PID_Type_Def Pid_bunch_turn;                           //串级角速度
/**********左右电机速度环参数************/
extern Control_Speed_Type_Def Control_Left_Speed;          //左轮速度结构
extern Control_Speed_Type_Def Control_Right_Speed;        //右轮速度结构
/**********电机相关参数************/
extern int16 Moto_Out_Left;                       //左电机最终输出占空比
extern int16 Moto_Out_Right;                     //右电机最终输出占空比
extern float Car_Speed;                            //车速m/s
extern float Left_Car_Speed;                          //左轮速度m/s
extern float Right_Car_Speed;                         //右轮速度m/s
extern float Distance;                             //全程累加的距离
extern float Current_Left;                         //左轮编码器值
extern float Current_Right;                         //右轮编码器值
extern float LR_Speed_Err;                         //两轮编码器的差值
extern float user_distance;                      //记录的路程
/**********陀螺仪相关参数************/
extern float gyrox,gyroy,gyroz;    //陀螺仪原始数据
extern uint8 flag_offset;     //角速度调0完成标志位
extern float Angle_X_Final;         //X最终倾斜角度//俯仰角
extern float Angle_Y_Final;
extern float Turn_speed;       //角速度,用做轮速差使用，舵机控制

extern communication_state car_state;
extern bool state;
extern bool car_stop;

/***摁键调参*****/
extern float serve_normal_pd[2];          //二次函数pd基数
extern float use_ring_pd[2];    //左圆环pid
extern float use_ring_pd_r[2];     //右圆环pid
extern float serve_pd[2];              //直道舵机不抖动的PD参数
extern float use_inductor_pd[2];         //(断路)电感pid,动态p基数
extern float use_slope_cross[2];  //右斜入十字5.8,3.9
extern float use_slope_cross_l[2];  //左斜入十字
/***上位机调试*****/
extern float Original_current_Left;                     //原始编码器值
extern float Original_current_Right;                     //原始编码器值


#endif /* SYSTEM_H_ */
