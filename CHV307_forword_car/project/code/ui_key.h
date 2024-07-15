/*
 * ui_key.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef UI_KEY_H_
#define UI_KEY_H_
#include "zf_common_typedef.h"
typedef struct
{
    uint8 current;          //当前索引号
    uint8 up;            //向上翻索引号
    uint8 down;         //向下翻索引号
    uint8 enter;          //确认索引号
  void (*current_operation)();            //用于指向执行函数的指针函数
} key_table;

extern key_table table[30];
extern void (*current_operation_index)();

 void KEY(void);
 void Key_init(void);

 void fun_a1(void);
 void fun_b1(void);
 void fun_c1(void);
 void fun_d1(void);

 void fun_a21(void);
 void fun_a22(void);
 void fun_a23(void);
 void fun_a24(void);

 void fun_b21(void);
 void fun_b22(void);
 void fun_b23(void);
 void fun_b24(void);

 void fun_c21(void);
 void fun_c22(void);
 void fun_c23(void);
 void fun_c24(void);

 void fun_a31(void);
 void fun_a32(void);
 void fun_a33(void);

 void fun_b31(void);
 void fun_b32(void);
 void fun_b33(void);

 void fun_c31(void);
 void fun_c32(void);
 void fun_c33(void);

 void fun_0(void);


//void key_scan(void);



#endif /* UI_KEY_H_ */
