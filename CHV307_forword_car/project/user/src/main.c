/*********************************************************************************************************************
                                                 每日一拜 好过省赛             天天参拜 稳过国赛
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
// // //      佛祖保佑      永无BUG      永不修改        \\ \\ \\
// // // // // // || || || || || || || || || || \\ \\ \\ \\ \\
*修改记录    2023/5/31
*作者            核能接力——杨锋勇
********************************************************************************************************************/
#include "zf_common_headfile.h"
bool OFF_Charge_Resue=1;  //1为开启充电救援，0为不开启
bool OFF_Element_test=0; //开启元素测试,1为直接进行元素测试，过任意一个元素就停车，除了坡道，0为默认充电倒车

bool OFF_Speed_test=0;  //开启速度环测试
bool Barrier_dir=1;    //避障方向，1为左，0为右
uint8 track_num=2;    //遇到车库停车，2为遇到1次就停车，即跑两圈，4为跑两圈
uint8 communication_count=0; //通信计数
uint8 speed_set_count=0;     //车速发送计数
//extern uint8 Out_door_type;
int main (void)
{
    Init_All();
    while(1)
    {
            if(mt9v03x_finish_flag)
            {
               Get_ADC();                                 //获取电感值
               Image_Deal(mt9v03x_image,User_image,1,13);//二值化和提取一半图像
               labyrinth_image_process(User_image,94,60); //迷宫巡线
               Fine_element_angle_point(User_image);      //找角点
               Find_angle_point_grow_dir();                //判断角点生长方向
               Final_deal(User_image);                     //最终处理函数
               communication_count++;
               if(communication_count==2)  //40ms接收一次
               {
                if(car_state==gogogo && speed_set_count<=10)   //发送10次速度信息以及发车方向
                {
                    Data_Send1((Control_Left_Speed.stright*10)+Out_door_type*100);
                    speed_set_count++;
                }
                if(car_state==run_ok && state==0)  //尾车自检通过，前车向前走一段距离
                {
                    gpio_set_level(B14,0); //关闭充电发射
                    if(Distance<=0.1)
                    {
                        Charge_Resue_state=1;//0.6m/s
                    }
                    if(Distance>0.1)
                    {
                       Charge_Resue_state=3; //不可以走了
                    }
                }
                else if(car_state==led_isfind && state==0)//后车找到灯了，可以前进
                {
                    state=1;
                    car_state=gogogo;
                    Charge_Resue_state=0;        //可以前进了
                }
                else if((Door_count==track_num) ||
                (inductor_check_count==0 &&
                 Barrier_state==0 && Distance>1))              //入库停车，出赛道停车
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

