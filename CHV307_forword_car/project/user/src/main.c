/*********************************************************************************************************************
                                                 ÿ��һ�� �ù�ʡ��             ����ΰ� �ȹ�����
 \\ \\ \\ \\ \\ \\ \\ || || || || || || // // // // // // // //
\\ \\ \\ \\ \\ \\ \\        _ooOoo_          // // // // // // //
\\ \\ \\ \\ \\ \\          o8888888o            // // // // // //
\\ \\ \\ \\ \\             88" . "88               // // // // //
\\ \\ \\ \\                (| -_- |)                  // // // //
\\ \\ \\                   O\  =  /O                     // // //
\\ \\                   ____/`---'\____                     // //
\\                    .'  \\|     |//  `.                      //
==                   /  \\|||  :  |||//  \                     ==
==                  /  _||||| -:- |||||-  \                    ==
==                  |   | \\\  -  /// |   |                    ==
==                  | \_|  ''\---/''  |   |                    ==
==                  \  .-\__  `-`  ___/-. /                    ==
==                ___`. .'  /--.--\  `. . ___                  ==
==              ."" '<  `.___\_<|>_/___.'  >'"".               ==
==            | | :  `- \`.;`\ _ /`;.`/ - ` : | |              \\
//            \  \ `-.   \_ __\ /__ _/   .-` /  /              \\
//      ========`-.____`-.___\_____/___.-`____.-'========      \\
//                           `=---='                           \\
// //   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  \\ \\
// // //      ���汣��      ����BUG      �����޸�        \\ \\ \\
// // // // // // || || || || || || || || || || \\ \\ \\ \\ \\
*�޸ļ�¼    2023/5/31
*����            ���ܽ������������
********************************************************************************************************************/
#include "zf_common_headfile.h"
bool OFF_Charge_Resue=1;  //1Ϊ��������Ԯ��0Ϊ������
bool OFF_Element_test=0; //����Ԫ�ز���,1Ϊֱ�ӽ���Ԫ�ز��ԣ�������һ��Ԫ�ؾ�ͣ���������µ���0ΪĬ�ϳ�絹��

bool OFF_Speed_test=0;  //�����ٶȻ�����
bool Barrier_dir=1;    //���Ϸ���1Ϊ��0Ϊ��
uint8 track_num=2;    //��������ͣ����2Ϊ����1�ξ�ͣ����������Ȧ��4Ϊ����Ȧ
uint8 communication_count=0; //ͨ�ż���
uint8 speed_set_count=0;     //���ٷ��ͼ���
//extern uint8 Out_door_type;
int main (void)
{
    Init_All();
    while(1)
    {
            if(mt9v03x_finish_flag)
            {
               Get_ADC();                                 //��ȡ���ֵ
               Image_Deal(mt9v03x_image,User_image,1,13);//��ֵ������ȡһ��ͼ��
               labyrinth_image_process(User_image,94,60); //�Թ�Ѳ��
               Fine_element_angle_point(User_image);      //�ҽǵ�
               Find_angle_point_grow_dir();                //�жϽǵ���������
               Final_deal(User_image);                     //���մ�����
               communication_count++;
               if(communication_count==2)  //40ms����һ��
               {
                if(car_state==gogogo && speed_set_count<=10)   //����10���ٶ���Ϣ�Լ���������
                {
                    Data_Send1((Control_Left_Speed.stright*10)+Out_door_type*100);
                    speed_set_count++;
                }
                if(car_state==run_ok && state==0)  //β���Լ�ͨ����ǰ����ǰ��һ�ξ���
                {
                    gpio_set_level(B14,0); //�رճ�緢��
                    if(Distance<=0.1)
                    {
                        Charge_Resue_state=1;//0.6m/s
                    }
                    if(Distance>0.1)
                    {
                       Charge_Resue_state=3; //����������
                    }
                }
                else if(car_state==led_isfind && state==0)//���ҵ����ˣ�����ǰ��
                {
                    state=1;
                    car_state=gogogo;
                    Charge_Resue_state=0;        //����ǰ����
                }
                else if((Door_count==track_num) ||
                (inductor_check_count==0 &&
                 Barrier_state==0 && Distance>1))              //���ͣ����������ͣ��
               {
                   state=0;
                   car_state=stop;
                   state_send(car_state);
               }
                communication_count=0;
            }
                mt9v03x_finish_flag = 0;
         }
            KEY();
    }
}

