/*
 * mypid.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef MYPID_H_
#define MYPID_H_
#include "zf_common_typedef.h"
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float out;
    float max;
    float min;
}PID;

typedef struct
{
    float SumError;
    float LastError;
    float dError;
    float Error;
}ERR;
void Mypid_Init(void);
float Car_Stop_PID(PID *pid,ERR *err,float current);
float Steering_gear_PID(float Error);
float Inductor_user_PID(float Error);
float slopetoangle_user_PID(float Error);
#endif /* MYPID_H_ */
