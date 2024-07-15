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
extern uint8 track_num;    //Ȧ��
extern bool Speed_PID_debug;          //�ٶȻ�����
extern uint16 Speed_PID_count;         //���ټ���
extern float ADD_angle;                                             //�ۼӵĽǶ�;
extern uint8 start_line_point_row;                    //��ֵ������58�п�ʼ
extern float slope_to_angle;                     //б��ת��Ϊ�Ƕ�
extern float parameterB;    //б��
extern float parameterA;    //�ؾ�
extern uint8 Charge_Resue_state;        //��Ԯ״̬
extern float add_gyro_x;      //x����ٶȻ���
extern bool Barrier_dir;         //���Ϸ���1Ϊ��0Ϊ��
extern bool OFF_Element_test;    //����Ԫ�ز���
extern bool OFF_Charge_Resue;  //1Ϊ��������Ԯ��0Ϊ������
extern uint8 Out_door_type;
extern float straightaway_speed;  //ֱ���ٶ�
extern float curve_speed;        //����ٶ�
extern float Break_road_speed;           //��·�ٶ�
extern int16 L_charge_big_thes,L_charge_small_thes,R_charge_big_thes,R_charge_small_thes;//����ֹͣ��ʼ�����ĵ����ֵ
//��������Ҫ�ۼӵľ���m
/***************��Ҫ����*******************/
extern uint8 Charge_Resue_count;        //���ʹ�ܷ���
extern bool Barrier_out_count;   //���ϰ�����м���
extern uint8 bottom_gray_threa;     //�ײ��߽��жϲ�Ⱥ���ֵ
extern uint8 Break_road_add_thres;     //��·��ǿ��ֵ
extern bool l_gray_point,r_gray_point;
extern bool vert_gray_point;  //��ֱ��Ⱥͱ�־λ��ר�����ж϶�·
extern uint8 l_hightest;//����ߵ�
extern uint8 r_hightest;//����ߵ�
extern uint8 hightest_x; //��ߵ�x����
extern uint8 Width_straight[60];    //ֱ�����
extern bool OFF_Speed_test;  //�����ٶȻ�����
extern uint8 Left_Line[60];
extern uint8 Left_Line_flag[60];
extern uint8 Right_Line[60];
extern uint8 Right_Line_flag[60];
extern uint8 Mid_Line[60];
extern uint8 Width_track[60];
extern uint8 start_point_l[2];//�������x��yֵ
extern uint8 start_point_r[2];//�ұ�����x��yֵ
extern uint8 labyrinth_start_point_l[2];//�Թ��������x��yֵ
extern uint8 labyrinth_start_point_r[2];//�Թ��ұ�����x��yֵ
extern uint8 hightest;//��ߵ�
extern uint8 points_l[110][2];//����
extern uint8 points_r[110][2];//����
extern float  rpts_l[110][2];      //͸�ӱ任����
extern float  rpts_r[110][2];      //͸�ӱ任����
extern float  rpts_l_dist[110][2];      //�Ⱦ����͸�ӱ任����
extern float  rpts_r_dist[110][2];      //�Ⱦ����͸�ӱ任����
extern float  rpts_langle[110];      //��߽�Ƕ�
extern float  rpts_rangle[110];      //�ұ߽�Ƕ�
extern float  rpts_langle_nms[110];  //���м�������֮�����߽�Ƕ�
extern float  rpts_rangle_nms[110];  //���м�������֮����ұ߽�Ƕ�
extern float  rpts_l_fitter[110][2];      //�˲���͸�ӱ任����
extern float  rpts_r_fitter[110][2];      //�˲���͸�ӱ任����
extern uint16 data_stastics_l;//ͳ������ҵ���ĸ���
extern uint16 data_stastics_r0;//ͳ���ұ��ҵ���ĸ���
extern uint8 inductor_check;      //������Ƿ�������־λ
extern uint8 Cross_Among;                                                         //��ʮ��֮���־λ
extern int16 Black_Point_Num;                                    //�ڵ�����
extern float Angle_ACC;                                  //�ۼӵĽǶ�
extern uint8 Get_image[60][94];
extern uint8 User_image[60][94];                //����ʹ�õĶ�ֵ��ͼ��
extern float Ring_add_angle;                         //Բ���ۼӵĽǶ�
extern uint16 dl1a_distance_mm;                       //tof������
extern int16 streeing_duty;                            //���ռ�ձ�
extern float final_err;                            //ѭ�������
extern float tof_distance;                        //tof���
extern float Barrier_Add_Angle;                 //�ۼӵĽǶ�
extern float ADD_angle;                                             //�ۼӵĽǶ�
extern float Ring_add_distance;            //�ۼӵľ���
extern uint8 inductor_check_count; //����������
/*****************ͼ����ر�־λ**********************/
extern uint8 REPIRE_LINE;                   //���߱�־λ
extern bool s_state[2];              //СS,��S,��S
extern bool Barrier_brake;                  //�ϰ���ɲ����־λ
extern bool L_straightaway; //��ֱ��
extern bool R_straightaway; //��ֱ��
extern uint8 slope_cross;               //б���־λ
extern uint8 L_slope_cross_state;        //��б���־λ
extern uint8 R_slope_cross_state;  //��б���־λ
extern uint8 Mid_Line_Turncation;                                         //���߹յ㻷
extern uint8 Loss_L_line,Loss_R_line;                                 //���Ҷ�������
extern uint8 Flag_Cross;                                           //ʮ��·��״̬λ
extern uint8 Flag_L_CirCle;                                     //��Բ����־λ
extern uint8 Flag_R_CirCle;                                    //��Բ����־λ
extern uint8 L_CirCle_State;                                  //��Բ��״̬��־λ
extern uint8 R_CirCle_State;                                //��Բ��״̬��־λ
extern uint8 Indooor_state;                             //������ʶ���־λ
extern uint8 Door_count;                                //���������ߴ���,���ɹ���ֵ�ó�3��Ȼ��ͣ��
extern uint8 Ramp_state;                               //�µ�
extern uint8 Break_road_state;                        //��·״̬��־λ
extern uint8 Barrier_state;                          //�ϰ���״̬
extern uint8 Out_door_state;                        //������״̬
/**************������*******************/
extern float Level_Deviation;                   //ˮƽ������
extern float Vert_Deviation;                      //��ֱ������
extern int16 Inductor_Value[4];                  //ˮƽ ��ֱ ˮƽ  ��ֱ  ˮƽ
extern int16 Inductor_Threshold[4];             //�����ֵ�ɼ�
extern uint8 Flag_Round;                       //������־
extern uint8 Flag_L_or_R;                    //����Բ����־λ
extern uint8 loss_line;                 //����ָʾ��1�󶪣�2�Ҷ���0����
extern uint8 loss_line_lock;            //�����������ߺ��Զ�������1Ϊ����
/**************ʱ�����**************/
extern Time_Count_Type_Def Time_Count;                   //�������ṹ
/**************pid��ر�־λ**************/
extern PID steer_pid,L_car_stop_pid,R_car_stop_pid;                  //���pid,ͣ��pid
extern ERR L_car_stop_err,R_car_stop_err;
extern PID_Type_Def Pid_LRSpeed;                                  //���ٲ�����
extern PID_Type_Def Pid_Left_Speed;                              //�����ٶȻ�
extern PID_Type_Def Pid_Right_Speed;                            //�����ٶȻ�
extern PID_Type_Def Pid_bunch_turn;                           //�������ٶ�
/**********���ҵ���ٶȻ�����************/
extern Control_Speed_Type_Def Control_Left_Speed;          //�����ٶȽṹ
extern Control_Speed_Type_Def Control_Right_Speed;        //�����ٶȽṹ
/**********�����ز���************/
extern int16 Moto_Out_Left;                       //�����������ռ�ձ�
extern int16 Moto_Out_Right;                     //�ҵ���������ռ�ձ�
extern float Car_Speed;                            //����m/s
extern float Left_Car_Speed;                          //�����ٶ�m/s
extern float Right_Car_Speed;                         //�����ٶ�m/s
extern float Distance;                             //ȫ���ۼӵľ���
extern float Current_Left;                         //���ֱ�����ֵ
extern float Current_Right;                         //���ֱ�����ֵ
extern float LR_Speed_Err;                         //���ֱ������Ĳ�ֵ
extern float user_distance;                      //��¼��·��
/**********��������ز���************/
extern float gyrox,gyroy,gyroz;    //������ԭʼ����
extern uint8 flag_offset;     //���ٶȵ�0��ɱ�־λ
extern float Angle_X_Final;         //X������б�Ƕ�//������
extern float Angle_Y_Final;
extern float Turn_speed;       //���ٶ�,�������ٲ�ʹ�ã��������

extern communication_state car_state;
extern bool state;
extern bool car_stop;

/***��������*****/
extern float serve_normal_pd[2];          //���κ���pd����
extern float use_ring_pd[2];    //��Բ��pid
extern float use_ring_pd_r[2];     //��Բ��pid
extern float serve_pd[2];              //ֱ�������������PD����
extern float use_inductor_pd[2];         //(��·)���pid,��̬p����
extern float use_slope_cross[2];  //��б��ʮ��5.8,3.9
extern float use_slope_cross_l[2];  //��б��ʮ��
/***��λ������*****/
extern float Original_current_Left;                     //ԭʼ������ֵ
extern float Original_current_Right;                     //ԭʼ������ֵ


#endif /* SYSTEM_H_ */
