/*
 * system.c
 *
 *  Created on: Mar 22, 2023
 *      Author: linjias
 */
#include "zf_common_headfile.h"
int16 streeing_duty=0;                               //���ռ�ձ�
float final_err=0;                                  //ѭ�������
bool Speed_PID_debug=0;          //�ٶȻ�����
uint16 Speed_PID_count=0;         //���ټ���
uint8 User_image[60][94]={0};
int16 Black_Point_Num=0;                           //�ڵ�����
uint8 charge_enable=0;
uint8 start_line_point_row=58;                    //��ֵ������58�п�ʼ
void Init_All()
{
    clock_init(SYSTEM_CLOCK_144M);                                              // ��ʼ��оƬʱ�� ����Ƶ��Ϊ 144MHz
    debug_init();                                                               // ��ʼ��Ĭ�� Debug UART
    wheel_init(17000,0,0);                                                     //�����ʼ��
    steering_init(100, 0);                                                     //�����ʼ��
    tft180_clear();                                                             //��Ļ���
    Init_contral();                                                            //���������ز�������
    tft180_set_dir(TFT180_CROSSWISE);                                           //��Ҫ�Ⱥ��� ��Ȼ��ʾ����
    tft180_init();                                                             //��Ļ��ʼ��
    Init_encoder();                                                             //��������ʼ��
    Key_init();                                                                 //������ʼ��
    Mypid_Init();                                                                //pid��ʼ��
   //Beep_Init();                                                                 //��������ʼ��
    Init_Magenet_AD();                                                         //��г�ʼ��
    ELEMENT_INIT();
    while(1)                                                             //tof����ʼ��
    {
         if(dl1a_init())
         {
             tft180_show_string(0, 32, "DL1A init error."); // DL1A ��ʼ��ʧ��
         }
         else
        {
            break;
        }
      system_delay_ms(1000);
    }
   gpio_init(B14, GPO, 0, GPIO_PIN_CONFIG);//�����ʹ��io
   tft180_clear();
   tft180_show_string(0, 0, "mt9v03x init.");
    while(1)
    {
       if(mt9v03x_init())
           tft180_show_string(0, 16, "mt9v03x reinit.");
           else
           break;
           system_delay_ms(1000);        // ���Ʊ�ʾ�쳣
    }
   tft180_show_string(0, 16, "init success.");
   tft180_clear();
   icm20602_init();        //�����ǳ�ʼ��
   wireless_uart_init();    //����ת���ڳ�ʼ��
   gyroOffset_init();     //ȥ��Ư
   pit_ms_init(TIM3_PIT, 5);                            //5ms��ʱ��(����ʱ����)
   pit_ms_init(TIM6_PIT, 20);                          //20ms��ʱ��(�����ǽǶȽ�����)
   pit_ms_init(TIM7_PIT, 5);                          //5ms��ʱ��(tof������mcu��������)
   pit_ms_init(TIM9_PIT, 50);                        //50ms��ʱ��(Ԫ�ؼ�����)
}

