/*
 * prox.h
 *
 *  Created on: 25 avr. 2022
 *      Author: alecp
 */

#ifndef PROX_H_
#define PROX_H_

// dénomination des alertes
#define R_SIDE 1
#define R_CENTER 2
#define R_FRONT 3
#define L_FRONT 4
#define L_CENTER 5
#define L_SIDE 6

int get_proximity(int sensor_number);
int8_t get_prox_alert(void);
void prox_sensors_start(void);

#endif /* PROX_H_ */
