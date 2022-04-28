/*
 * prox.h
 *
 *  Created on: 25 avr. 2022
 *      Author: alecp
 */

#ifndef PROX_H_
#define PROX_H_

//allows to get the value in an other file
int get_proximity(int sensor_number);

//allows to  get the value of the proximity alert
int8_t get_prox_alert(void);

void prox_sensors_start(void);

#endif /* PROX_H_ */
