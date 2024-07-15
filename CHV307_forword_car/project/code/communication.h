/*
 * communication.h
 *
 *  Created on: 2023��7��2��
 *      Author: linjias
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_
#include "zf_common_typedef.h"
#include "zf_driver_uart.h"
#include "zf_device_wireless_uart.h"

typedef enum  {
    run_nok=0x10,//�ȴ����Լ�
    run_ok ,//�Լ�ɹ������Կ���
    gogogo , //����
    led_isfind ,//�ҵ��ƣ��ڸ�����
    led_warning,//��̫Զ��ǰ������
    led_nofind,//û�ҵ����ܵ��
    stop//ͣ��
}communication_state;

void state_send(communication_state state);
uint8 state_receive(void);
uint8 state_receive1(void);
void Data_Send1(int32 A);
void Data_Send2(uint8 A);
void data_acceptance(uint8 *A,uint8 *B);

#endif /* COMMUNICATION_H_ */
