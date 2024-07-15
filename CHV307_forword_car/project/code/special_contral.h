/*
 * special_contral.h
 *
 *  Created on: Mar 10, 2023
 *      Author: linjias
 */

#ifndef SPECIAL_CONTRAL_H_
#define SPECIAL_CONTRAL_H_
#include "zf_common_typedef.h"
void loss_line_deal(int16 * sensor_value , float position , float vert_position,uint16 left_lose_threshold ,uint16 right_lose_threshold);
void DirectionControl(void);
void Inductor_unnormal_deal(void);


#endif /* SPECIAL_CONTRAL_H_ */
