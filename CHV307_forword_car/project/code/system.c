/*
 * system.c
 *
 *  Created on: Mar 22, 2023
 *      Author: linjias
 */
#include "zf_common_headfile.h"
int16 streeing_duty=0;                               //舵机占空比
float final_err=0;                                  //循迹用误差
bool Speed_PID_debug=0;          //速度环调速
uint16 Speed_PID_count=0;         //调速计数
uint8 User_image[60][94]={0};
int16 Black_Point_Num=0;                           //黑点数量
uint8 charge_enable=0;
uint8 start_line_point_row=58;                    //二值化起点从58行开始
void Init_All()
{
    clock_init(SYSTEM_CLOCK_144M);                                              // 初始化芯片时钟 工作频率为 144MHz
    debug_init();                                                               // 初始化默认 Debug UART
    wheel_init(17000,0,0);                                                     //电机初始化
    steering_init(100, 0);                                                     //舵机初始化
    tft180_clear();                                                             //屏幕清空
    Init_contral();                                                            //电机控制相关参数设置
    tft180_set_dir(TFT180_CROSSWISE);                                           //需要先横屏 不然显示不下
    tft180_init();                                                             //屏幕初始化
    Init_encoder();                                                             //编码器初始化
    Key_init();                                                                 //摁键初始化
    Mypid_Init();                                                                //pid初始化
   //Beep_Init();                                                                 //蜂鸣器初始化
    Init_Magenet_AD();                                                         //电感初始化
    ELEMENT_INIT();
    while(1)                                                             //tof测距初始化
    {
         if(dl1a_init())
         {
             tft180_show_string(0, 32, "DL1A init error."); // DL1A 初始化失败
         }
         else
        {
            break;
        }
      system_delay_ms(1000);
    }
   gpio_init(B14, GPO, 0, GPIO_PIN_CONFIG);//发射端使能io
   tft180_clear();
   tft180_show_string(0, 0, "mt9v03x init.");
    while(1)
    {
       if(mt9v03x_init())
           tft180_show_string(0, 16, "mt9v03x reinit.");
           else
           break;
           system_delay_ms(1000);        // 闪灯表示异常
    }
   tft180_show_string(0, 16, "init success.");
   tft180_clear();
   icm20602_init();        //陀螺仪初始化
   wireless_uart_init();    //无线转串口初始化
   gyroOffset_init();     //去零漂
   pit_ms_init(TIM3_PIT, 5);                            //5ms定时器(控制时序用)
   pit_ms_init(TIM6_PIT, 20);                          //20ms定时器(陀螺仪角度解算用)
   pit_ms_init(TIM7_PIT, 5);                          //5ms定时器(tof测距和无mcu超声波用)
   pit_ms_init(TIM9_PIT, 50);                        //50ms定时器(元素计数用)
}

