/*
 * inductor.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef INDUCTOR_H_
#define INDUCTOR_H_
#include "zf_common_typedef.h"
typedef struct
{
  float fitter_value[4];
}Fitter_err;
void Init_Voltage_AD(void);
float Get_Voltage_value(void);
void Init_Magenet_AD(void);
void Get_InductValue(int16 value[]);
uint8 Is_InductorValue_Normal(void);
void InductorVal_Filter_Kalman(int16 value[],float q,float r);
void Normalize_Induct(void);
void Calculate_Deviation(void);
void Calculate_Vert_Deviation(void);
float Cal_Mid_Val_Slope(int16 mid_val);
float Position_Filter(Fitter_err *para,float err);
void Position_Record(float queue[], float para,uint8 n);
void Get_ADC(void);


#endif /* INDUCTOR_H_ */
