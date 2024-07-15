/*
 * communication.c
 *
 *  Created on: 2023��7��2��
 *      Author: linjias
 */
#include "communication.h"

//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define byte0(DwTemp)       ( *( (char *)(&DwTemp)) )
#define byte1(DwTemp)       ( *( (char *)(&DwTemp) + 1) )
#define byte2(DwTemp)       ( *( (char *)(&DwTemp) + 2) )
#define byte3(DwTemp)       ( *( (char *)(&DwTemp) + 3) )

#define transition_int32(b0,b1,b2,b3)  ((int32_t)(b3) << 24) | ((int32_t)(b2) << 16) | ((int32_t)(b1) << 8) | (int32_t)(b0)
#define transition_uint32(b0,b1,b2,b3)  ((uint32_t)(b3) << 24) | ((uint32_t)(b2) << 16) | ((uint32_t)(b1) << 8) | (uint32_t)(b0)
#define transition_int16(b0,b1) ((int16_t)(b1) << 8) | (int16_t)(b0)
#define transition_uint16(b0,b1) ((uint16_t)(b1) << 8) | (uint16_t)(b0)
#define transition_int8(b0) (int8_t)(b0)
#define transition_uint8(b0) (uint8_t)(b0)

uint8 state_buff[3];
uint8 state_buffer[3];

uint8 communication_buff[30];
uint8 data_buffer[30];

communication_state car_state=run_nok;
bool state=0;
bool car_stop;

void state_send(communication_state state)
{
    uint8 _cnt=0;
    state_buff[_cnt++]=0xAA;//֡ͷ
    state_buff[_cnt++]=state;//����
    state_buff[_cnt++]=0xBB;//֡β
    uart_write_buffer(WIRELESS_UART_INDEX,state_buff, _cnt);
}

uint8 state_receive(void)
{


       uint8 data_len=wireless_uart_read_buff(state_buffer,3);
       static uint8 last_state=run_nok;
       if(data_len)
       {
           if(state_buffer[0]==0xAA&&state_buffer[2]==0xBB)
           {
               last_state=state_buffer[1];
               return state_buffer[1];
           }

       }

       return last_state;
}


//���Է��������ݵ�Э��
void Data_Send1(int32 A)
{
        uint8 _cnt=0;
        communication_buff[_cnt++]=0xCC;//֡ͷ
        communication_buff[_cnt++]=4;//���ݳ���
        communication_buff[_cnt++]=byte0(A);//��������,С��ģʽ����λ��ǰ
        communication_buff[_cnt++]=byte1(A);//��������,С��ģʽ����λ��ǰ
        communication_buff[_cnt++]=byte2(A);//��������,С��ģʽ����λ��ǰ
        communication_buff[_cnt++]=byte3(A);//��������,С��ģʽ����λ��ǰ
        uart_write_buffer(WIRELESS_UART_INDEX,communication_buff, _cnt);
}

uint8 state_receive1(void)
{
       static uint8 last_state=run_nok;
       static uint8 _cnt=0;
       uint8 dat;
       if(uart_query_byte(UART_7,&dat))
       state_buffer[_cnt++]=dat;
       if(state_buffer[0]!=0xAA)
           _cnt=0;
       if(state_buffer[2]!=0xBB&&_cnt==3)
           _cnt=0;
       if(state_buffer[0]==0xAA&&state_buffer[2]==0xBB&&_cnt==3)
       {
           _cnt=0;
           last_state=state_buffer[1];
           return state_buffer[1];
       }

       return last_state;
}

void data_acceptance(uint8 *A,uint8 *B)
{
    uint8 cnt=0;
    uint8 data_len=wireless_uart_read_buff(data_buffer,30);
    if(data_len)
    {
        if(data_buffer[cnt++]==0xAA&&data_buffer[cnt++]==1)
        {

            *A=data_buffer[cnt++];
        }
    }

}








