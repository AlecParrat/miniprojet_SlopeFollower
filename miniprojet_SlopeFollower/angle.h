/*
 * angle.h
 *
 *  Created on: 21 avr. 2022
 *      Author: thibault.schaller
 */

#ifndef ANGLE_H_
#define ANGLE_H_

int16_t average(int16_t new_value, int32_t* sum, int16_t* values, int16_t* counter, int16_t size);
int16_t compute_angle(void);
int16_t get_angle(void);
bool get_slope(void);
void compute_angle_thd_start(void);

#endif /* ANGLE_H_ */
