/*
 * ANO_DT.c
 *
 *  Created on: 2023年6月4日
 *      Author: linjias
 */
//#include "ANO_DT.h"
#include "zf_common_headfile.h"
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
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
    BUFF[_cnt++]=0xAA;//帧头
    BUFF[_cnt++]=0xFF;//目标地址
    BUFF[_cnt++]=0XF1;//功能码
    BUFF[_cnt++]=0x10;//形参数据总长度 ，上面一共16字节，16进制表示为0x10
    BUFF[_cnt++]=BYTE0(A);//数据内容,小段模式，低位在前，一个小段为1个字节
    BUFF[_cnt++]=BYTE1(A);//需要将字节进行拆分，调用上面的宏定义即可。
    BUFF[_cnt++]=BYTE2(A);
    BUFF[_cnt++]=BYTE3(A);
    BUFF[_cnt++]=BYTE0(B);//数据内容,小段模式，低位在前
    BUFF[_cnt++]=BYTE1(B);//需要将字节进行拆分，调用上面的宏定义即可。
    BUFF[_cnt++]=BYTE2(B);
    BUFF[_cnt++]=BYTE3(B);
    BUFF[_cnt++]=BYTE0(C);//数据内容,小段模式，低位在前
    BUFF[_cnt++]=BYTE1(C);//需要将字节进行拆分，调用上面的宏定义即可。
    BUFF[_cnt++]=BYTE2(C);
    BUFF[_cnt++]=BYTE3(C);
    BUFF[_cnt++]=BYTE0(D);//数据内容,小段模式，低位在前
    BUFF[_cnt++]=BYTE1(D);//需要将字节进行拆分，调用上面的宏定义即可。
    BUFF[_cnt++]=BYTE2(D);
    BUFF[_cnt++]=BYTE3(D);
    //SC和AC的校验直接抄最上面上面简介的即可
    for(i=0;i<BUFF[3]+4;i++)
    {
        sumcheck+=BUFF[i];
        addcheck+=sumcheck;
    }
    BUFF[_cnt++]=sumcheck;
    BUFF[_cnt++]=addcheck;
    ANO_DT_Send_Data(BUFF, _cnt);
}


/*Send_Data函数是协议中所有发送数据功能使用到的发送函数*/
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数，这里就采用有线连接，发送至串口2了

void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length)
{
    uart_write_buffer(WIRELESS_UART_INDEX, dataToSend, length);
}





