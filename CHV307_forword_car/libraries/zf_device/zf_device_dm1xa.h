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

#ifndef _ZF_DEVICE_DM1XA_H_
#define _ZF_DEVICE_DM1XA_H_

#include "zf_common_typedef.h"

// ��Ҫע�� dm1xa_transmitter_ranging / dm1xa_receiver_ranging �ĵ�������
// ��ؿ����� 10-20ms �������
//
// �������ھ������������� ���㹫ʽ�������� period * 343.2 mm
// ��ô 10-20ms �ĵ������������Ӧ 3432-6864mm ������෶Χ
//
// ��� dm1xa_transmitter_ranging / dm1xa_receiver_ranging
// �ĵ������ڲ��� 10-20ms �������
// ��ô���ܳ��ֱ������Ĳ����Ϣ�쳣

// DM1TA ģ�� ���Ŷ�Ӧ
#define DM1XA_FB_PIN                        ( E15 )
#define DM1XA_EN_PIN                        ( E14 )

// DM1RA ģ�� ���Ŷ�Ӧ
#define DM1XA_S_PIN                         ( E15 )
#define DM1XA_L_PIN                         ( E14 )

// �̶�ʹ��һ����ʱ��
#define DM1XA_TIM_INDEX                     ( TIM_7 )

// ÿ�β����ز��� ��С 6 ��� 100
// ��ôÿ�β�� EN ������ʱ��Ϊ DM1XA_FB_SEND * 1000 / 38 ΢��
#define DM1XA_FB_SEND                       ( 10 )

#if (DM1XA_FB_SEND < 6 || DM1XA_FB_SEND > 100)
#error "DM1XA_FB_SEND error,  it must be between 6 and 100"
#endif

// ��ʼ�����Դ���
#define DM1XA_INIT_MAX_COUNT                ( 100 )

// ��ʱ���� ���ﲻ�����û��޸�
#define DM1XA_RECEIVER_TIMEROUT_US          ( 30000 )

// DM1XA ģ�����ʶ���� �û����������
typedef enum
{
    DM1XA_NO_ERROR,
    DM1XA_TYPE_ERROR,
}dm1xa_error_code_enum;

// DM1XA ģ�����ʶ���� �û����������
typedef enum
{
    DM1XA_RECEIVER_RANGING_NO_SIGNAL,
    DM1XA_RECEIVER_RANGING_WAIT_SOUND,
    DM1XA_RECEIVER_RANGING_SUCCESS,
}dm1xa_ranging_state_enum;

// DM1XA ģ������ �û����������
typedef enum
{
    DM1XA_NO_INIT,
    DM1XA_CHECK_TYPE,
    DM1XA_TRANSMITTER,
    DM1XA_RECEIVER,
}dm1xa_type_enum;

void                    dm1xa_sound_callback        (void);
void                    dm1xa_light_callback        (void);

uint16                  dm1xa_receiver_ranging      (void);
void                    dm1xa_transmitter_ranging   (void);
dm1xa_error_code_enum   dm1xa_init                  (void);

#endif
