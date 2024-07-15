/*********************************************************************************************************************
* CH32V307VCT6 Opensourec Library ����CH32V307VCT6 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ���CH32V307VCT6 ��Դ���һ����
*
* CH32V307VCT6 ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          zf_device_dm1xa
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          MounRiver Studio V1.8.1
* ����ƽ̨          CH32V307VCT6
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����                                      ����                             ��ע
* 2023-03-18        ��W            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* ���߶��壺
*                   ------------------------------------
*                   ��ȥģ��(��MCU�汾 ģ���ʶ�ͺ�<DM1TA>)
*                   ģ��ܽ�            ��Ƭ���ܽ�
*                   FB                  �鿴 zf_device_dm1xa.h �� DM1XA_FB_PIN �궨��
*                   EN                  �鿴 zf_device_dm1xa.h �� DM1XA_EN_PIN �궨��
*                   5V                  5V ��Դ
*                   GND                 ��Դ��
*                   ------------------------------------
*                   ------------------------------------
*                   ����ģ��(��MCU�汾 ģ���ʶ�ͺ�<DM1RA>)
*                   ģ��ܽ�            ��Ƭ���ܽ�
*                   S                   �鿴 zf_device_dm1xa.h �� DM1XA_S_PIN  �궨��
*                   L                   �鿴 zf_device_dm1xa.h �� DM1XA_L_PIN  �궨��
*                   5V                  5V ��Դ
*                   GND                 ��Դ��
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_exti.h"
#include "zf_driver_timer.h"

#include "zf_device_dm1xa.h"

static uint16                   dm1xa_distance_mm   = 6800;
static uint32                   dm1xa_plus_count    = 0;
static dm1xa_type_enum          dm1xa_type          = DM1XA_NO_INIT;
static dm1xa_ranging_state_enum dm1xa_ranging_state = DM1XA_RECEIVER_RANGING_NO_SIGNAL;

//-------------------------------------------------------------------------------------------------------------------
// �������     DM1XA ���ź� / �����ź� �ⲿ�жϻص�����
// ����˵��     void        ��
// ����˵��     void        ��
// ʹ��ʾ��     dm1xa_sound_callback();
// ��ע��Ϣ     ���������Ҫ���� DM1XA_FB_PIN / DM1XA_S_PIN ��Ӧ���ⲿ�жϷ�������
//-------------------------------------------------------------------------------------------------------------------
void dm1xa_sound_callback (void)
{
    switch(dm1xa_type)
    {
        case DM1XA_NO_INIT:                                                     // δ��ʼ�� �˳�
        {
        }break;
        case DM1XA_CHECK_TYPE:                                                  // ��ʼ���׶�
        {
            dm1xa_plus_count ++;                                                // �� FB ���� sound �źż���
        }break;
        case DM1XA_TRANSMITTER:                                                 // DM1TA ģ�鷢����
        {
            dm1xa_plus_count ++;                                                // �� FB �źż���
            if(DM1XA_FB_SEND <= dm1xa_plus_count)                               // �ﵽ�涨�� DM1XA_FB_SEND ����
            {
                gpio_low(DM1XA_EN_PIN);                                         // ֹͣ���Ͳ���ź�
                dm1xa_plus_count = 0;                                           // ��ռ���
            }
        }break;
        case DM1XA_RECEIVER:                                                    // DM1RA ��ȡ����ź�
        {
            if(DM1XA_RECEIVER_RANGING_WAIT_SOUND == dm1xa_ranging_state)        // �Ѿ���ȡ���ź� ֤�������ź���Ч
            {
                if(gpio_get_level(DM1XA_S_PIN))                                 // ���ź�Ϊ�� ��һ����������
                {
                    if(150 < timer_get(DM1XA_TIM_INDEX) - dm1xa_plus_count)     // �ж�������ź������Ƿ��ǵ��� 150us �ĸ�������
                    {
                        timer_clear(DM1XA_TIM_INDEX);                           // ���ʱ��
                        dm1xa_distance_mm = (uint16)((float)dm1xa_plus_count * 0.3432); // �������ֵ ���׵�λ
                        dm1xa_ranging_state = DM1XA_RECEIVER_RANGING_SUCCESS;   // �����Ϣ����Ϊ��ɲ��
                    }
                }
                else                                                            // ���ź�Ϊ�� ֤����������ʼ
                {
                    dm1xa_plus_count = timer_get(DM1XA_TIM_INDEX);              // ��¼����ʱ���
                }
            }
        }break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     DM1XA ���ź� �ⲿ�жϻص�����
// ����˵��     void        ��
// ����˵��     void        ��
// ʹ��ʾ��     dm1xa_light_callback();
// ��ע��Ϣ     ���������Ҫ���� DM1XA_EN_PIN / DM1XA_L_PIN ��Ӧ���ⲿ�жϷ�������
//-------------------------------------------------------------------------------------------------------------------
void dm1xa_light_callback (void)
{
    switch(dm1xa_type)
    {
        case DM1XA_NO_INIT:                                                     // δ��ʼ�� �˳�
        case DM1XA_CHECK_TYPE:                                                  // ��ʼ���׶�
        case DM1XA_TRANSMITTER:                                                 // DM1TA ģ�鷢����
        {
        }break;
        case DM1XA_RECEIVER:                                                    // DM1RA ��ȡ����ź�
        {
            timer_clear(DM1XA_TIM_INDEX);                                       // ���ʱ�� ׼����ȡ����ʱ���
            dm1xa_ranging_state = DM1XA_RECEIVER_RANGING_WAIT_SOUND;            // ��ǻ�ȡ�����ź�
        }break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     DM1RA ģ�� ��ȡ���������
// ����˵��     void        ��
// ���ز���     uint16      ������Ϣ 6800 Ϊ��ʱĬ�Ͼ���
// ʹ��ʾ��     uint16 distance_mm = dm1xa_receiver_ranging();
// ��ע��Ϣ     ���¾�����Ϣ
//
//              ��Ҫע�� dm1xa_receiver_ranging �ĵ���������ؿ����� 10-20ms �������
//              �������ھ������������� ���㹫ʽ�������� period * 343.2 mm
//              ��ô 10-20ms �ĵ������������Ӧ 3432-6864mm ������෶Χ
//              ��� dm1xa_receiver_ranging �ĵ������ڲ��������Χ
//              ��ô���ܳ��ֱ������Ĳ����Ϣ�쳣
//-------------------------------------------------------------------------------------------------------------------
uint16 dm1xa_receiver_ranging (void)
{
    switch(dm1xa_ranging_state)
    {
        case DM1XA_RECEIVER_RANGING_NO_SIGNAL:                                  // �޲���ź�
        case DM1XA_RECEIVER_RANGING_WAIT_SOUND:                                 // ����ȡ����ź�
        {
            if(DM1XA_RECEIVER_TIMEROUT_US <= timer_get(DM1XA_TIM_INDEX))        // ��������ϴι��źŲ����� 30ms
            {
                dm1xa_distance_mm = 6800;                                       // �ָ�Ĭ��������ֵ
            }
        }break;
        case DM1XA_RECEIVER_RANGING_SUCCESS:                                    // ��ɲ����Ϣ
        {
            dm1xa_ranging_state = DM1XA_RECEIVER_RANGING_NO_SIGNAL;             // ��ǲ���źŻָ�Ĭ��
        }break;
        default:
        {
        }break;
    }
    return dm1xa_distance_mm;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     DM1TA ģ�� ����һ�����ź�
// ����˵��     void        ��
// ���ز���     void        ��
// ʹ��ʾ��     dm1xa_transmitter_ranging();
// ��ע��Ϣ     ������ɺ������Լ�ͨ����ʱ������
//              �����������ʹ�� DM1TA ����һ�β��
//              ��� DM1RA ���Զ�����һ�β����¾�����Ϣ
//
//              ��Ҫע�� dm1xa_transmitter_ranging �ĵ���������ؿ����� 10-20ms �������
//              �������ھ������������� ���㹫ʽ�������� period * 343.2 mm
//              ��ô 10-20ms �ĵ������������Ӧ 3432-6864mm ������෶Χ
//              ��� dm1xa_transmitter_ranging �ĵ������ڲ��������Χ
//              ��ô���ܳ��ֱ������Ĳ����Ϣ�쳣
//-------------------------------------------------------------------------------------------------------------------
void dm1xa_transmitter_ranging (void)
{
    gpio_high(DM1XA_EN_PIN);                                                    // ���� EN ����һ�β����Ϣ����
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ�� ������ ģ��
// ����˵��     void                    ��
// ���ز���     dm1xa_error_code_enum   DM1XA_NO_ERROR-��ʼ���ɹ�
// ʹ��ʾ��     dm1xa_init();
// ��ע��Ϣ     ���Զ�ʶ���� DM1TA ģ�黹�� DM1RA ģ��
//-------------------------------------------------------------------------------------------------------------------
dm1xa_error_code_enum dm1xa_init (void)
{
    dm1xa_error_code_enum return_state = DM1XA_NO_ERROR;

    do
    {
        dm1xa_distance_mm = 0;
        dm1xa_type = DM1XA_CHECK_TYPE;                                          // ģ��״̬���Ϊ����ȷ��ģʽ

        gpio_init(DM1XA_S_PIN, GPI, GPIO_LOW, GPI_PULL_DOWN);                   // ������������Ϊ��������ģʽ
        gpio_init(DM1XA_L_PIN, GPI, GPIO_LOW, GPI_PULL_DOWN);                   // ������������Ϊ��������ģʽ

        int16 i = DM1XA_INIT_MAX_COUNT;
        while(i --)
        {
            if(gpio_get_level(DM1XA_S_PIN) && gpio_get_level(DM1XA_L_PIN))      // �������Ǹߵ�ƽ ��ô������ DM1RA ģ��
            {
                dm1xa_type = DM1XA_RECEIVER;
                dm1xa_ranging_state = DM1XA_RECEIVER_RANGING_NO_SIGNAL;
                exti_init(DM1XA_S_PIN, EXTI_TRIGGER_BOTH);                      // ���ų�ʼ��Ϊ�ⲿ�ж�����
                exti_init(DM1XA_L_PIN, EXTI_TRIGGER_FALLING);                   // ���ų�ʼ��Ϊ�ⲿ�ж�����
                timer_init(DM1XA_TIM_INDEX, TIMER_US);                          // ΢���ʱ
                timer_clear(DM1XA_TIM_INDEX);                                   // ��ռ���
                timer_start(DM1XA_TIM_INDEX);                                   // ������ʱ��
                break;
            }
            system_delay_us(100);
        }
        if(0 > i)
        {
            exti_init(DM1XA_FB_PIN, EXTI_TRIGGER_FALLING);
            gpio_init(DM1XA_EN_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
            dm1xa_plus_count = 0;
            gpio_high(DM1XA_EN_PIN);
            system_delay_us(210);
            gpio_low(DM1XA_EN_PIN);
            if(6 < dm1xa_plus_count && 10 > dm1xa_plus_count)
            {
                dm1xa_type = DM1XA_TRANSMITTER;
                dm1xa_plus_count = 0;
            }
            else
            {
                dm1xa_type = DM1XA_NO_INIT;
                return_state = DM1XA_TYPE_ERROR;
                break;
            }
        }
    }while(0);

    return return_state;
}
