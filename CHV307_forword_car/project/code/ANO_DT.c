/*
 * ANO_DT.c
 *
 *  Created on: 2023��6��4��
 *      Author: linjias
 */
//#include "ANO_DT.h"
#include "zf_common_headfile.h"
//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
uint8 BUFF[30];
void sent_data(int32 A,int32 B,int32 C,int32 D)
{
    int i;
    uint8 sumcheck = 0;
    uint8 addcheck = 0;
    uint8 _cnt=0;
    BUFF[_cnt++]=0xAA;//֡ͷ
    BUFF[_cnt++]=0xFF;//Ŀ���ַ
    BUFF[_cnt++]=0XF1;//������
    BUFF[_cnt++]=0x10;//�β������ܳ��� ������һ��16�ֽڣ�16���Ʊ�ʾΪ0x10
    BUFF[_cnt++]=BYTE0(A);//��������,С��ģʽ����λ��ǰ��һ��С��Ϊ1���ֽ�
    BUFF[_cnt++]=BYTE1(A);//��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
    BUFF[_cnt++]=BYTE2(A);
    BUFF[_cnt++]=BYTE3(A);
    BUFF[_cnt++]=BYTE0(B);//��������,С��ģʽ����λ��ǰ
    BUFF[_cnt++]=BYTE1(B);//��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
    BUFF[_cnt++]=BYTE2(B);
    BUFF[_cnt++]=BYTE3(B);
    BUFF[_cnt++]=BYTE0(C);//��������,С��ģʽ����λ��ǰ
    BUFF[_cnt++]=BYTE1(C);//��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
    BUFF[_cnt++]=BYTE2(C);
    BUFF[_cnt++]=BYTE3(C);
    BUFF[_cnt++]=BYTE0(D);//��������,С��ģʽ����λ��ǰ
    BUFF[_cnt++]=BYTE1(D);//��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
    BUFF[_cnt++]=BYTE2(D);
    BUFF[_cnt++]=BYTE3(D);
    //SC��AC��У��ֱ�ӳ�������������ļ���
    for(i=0;i<BUFF[3]+4;i++)
    {
        sumcheck+=BUFF[i];
        addcheck+=sumcheck;
    }
    BUFF[_cnt++]=sumcheck;
    BUFF[_cnt++]=addcheck;
    ANO_DT_Send_Data(BUFF, _cnt);
}


/*Send_Data������Э�������з������ݹ���ʹ�õ��ķ��ͺ���*/
//��ֲʱ���û�Ӧ��������Ӧ�õ����������ʹ�õ�ͨ�ŷ�ʽ��ʵ�ִ˺���������Ͳ����������ӣ�����������2��

void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length)
{
    uart_write_buffer(WIRELESS_UART_INDEX, dataToSend, length);
}





