/*********************************************************************************************************************
* CH32V307VCT6 Opensourec Library ����CH32V307VCT6 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ���CH32V307VCT6 ��Դ���һ����
*
* CH32V307VCT6 ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
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
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          zf_device_aht20
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          MounRiver Studio V1.8.1
* ����ƽ̨          CH32V307VCT6
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����                                      ����                             ��ע
* 2022-09-15        ��W            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* ���߶���:
*                   ------------------------------------
*                   ģ��ܽ�            ��Ƭ���ܽ�
*                   ���� IIC ͨ�����Ŷ�Ӧ��ϵ
*                   SCL                 �鿴 zf_device_aht20.h �� AHT20_SOFT_IIC_SCL �궨��
*                   SDA                 �鿴 zf_device_aht20.h �� AHT20_SOFT_IIC_SDA �궨��
*                   VCC                 3.3V��Դ
*                   GND                 ��Դ��
*                   ������������
*
*                   Ӳ�� IIC ͨ�����Ŷ�Ӧ��ϵ
*                   SCL                 �鿴 zf_device_aht20.h �� AHT20_IIC_SCL �궨��
*                   SDA                 �鿴 zf_device_aht20.h �� AHT20_IIC_SDA �궨��
*                   VCC                 3.3V��Դ
*                   GND                 ��Դ��
*                   ������������
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_aht20_h_
#define _zf_device_aht20_h_

#include "zf_common_clock.h"
#include "zf_common_debug.h"

#include "zf_driver_delay.h"

#include "zf_driver_soft_iic.h"

#define AHT20_USE_SOFT_IIC          (1)                                         // Ĭ��ʹ������ IIC ��ʽ���� ����ʹ������ IIC ��ʽ
#if AHT20_USE_SOFT_IIC                                                          // ������ ��ɫ�����Ĳ�����ȷ�� ��ɫ�ҵľ���û���õ�
//====================================================���� IIC ����====================================================
#define AHT20_SOFT_IIC_DELAY        (10 )                                       // ���� IIC ��ʱ����ʱ���� ��ֵԽС IIC ͨ������Խ��
#define AHT20_SCL_PIN               (B10)                                       // ���� IIC SCL ���� ���� MPU6050 �� SCL ����
#define AHT20_SDA_PIN               (B11)                                       // ���� IIC SDA ���� ���� MPU6050 �� SDA ����
//====================================================���� IIC ����====================================================
#else
//====================================================Ӳ�� IIC ����====================================================
#define AHT20_IIC_SPEED             (400000     )                               // Ӳ�� IIC ͨ������ ��� 400KHz ��������� 40KHz
#define AHT20_IIC                   (IIC_2      )                               // Ӳ�� IIC SCL ���� ���� MPU6050 �� SCL ����
#define AHT20_SCL_PIN               (IIC2_SCL_B10)                              // Ӳ�� IIC SCL ���� ���� MPU6050 �� SCL ����
#define AHT20_SDA_PIN               (IIC2_SDA_B11)                              // Ӳ�� IIC SDA ���� ���� MPU6050 �� SDA ����
//====================================================Ӳ�� IIC ����====================================================
#endif

#define AHT20_TIMEOUT_COUNT         (0x001F)                                    // MPU6050 ��ʱ����

//================================================���� AHT20 �ڲ���ַ================================================
#define	AHT20_DEV_ADDR              (0x38)

#define	AHT20_READ_STATE            (0x71)
#define	AHT20_CAL_ENABLE            (0x08)
#define	AHT20_STATE_BUSY            (0x80)

#define	AHT20_MEASURE_CMD           (0xAC)

#define	AHT20_SELF_INIT             (0xBE)
//================================================���� AHT20 �ڲ���ַ================================================

extern float aht_temperature, aht_humidity;

void    aht20_read_data     (void);
uint8   aht20_init          (void);

#endif