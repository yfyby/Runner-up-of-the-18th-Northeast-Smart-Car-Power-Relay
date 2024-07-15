/*
 * communication.h
 *
 *  Created on: 2023年7月2日
 *      Author: linjias
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_
#include "zf_common_typedef.h"
#include "zf_driver_uart.h"
#include "zf_device_wireless_uart.h"

typedef enum  {
    run_nok=0x10,//等待后车自检
    run_ok ,//自检成功，可以开跑
    gogogo , //开跑
    led_isfind ,//找到灯，在跟灯跑
    led_warning,//灯太远，前车减速
    led_nofind,//没找到灯跑电磁
    stop//停车
}communication_state;

void state_send(communication_state state);
uint8 state_receive(void);
uint8 state_receive1(void);
void Data_Send1(int32 A);
void Data_Send2(uint8 A);
void data_acceptance(uint8 *A,uint8 *B);

#endif /* COMMUNICATION_H_ */
