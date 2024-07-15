/*
 * wheel.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef WHEEL_H_
#define WHEEL_H_
#include "zf_common_typedef.h"

void wheel_init(uint32 freq, int16 Vl, int16 Vr);
void wheel_speed(int16 Vl, int16 Vr);


#endif /* WHEEL_H_ */
