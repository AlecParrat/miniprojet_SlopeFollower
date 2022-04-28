/*
 * regulation.h
 *
 *  Created on: 20 avr. 2022
 *      Author: alecp
 */

#ifndef REGULATION_H_
#define REGULATION_H_

// r�gulateur PID
int16_t regulator(int16_t angle, int16_t angle_consigne);

int16_t esquive(int8_t alert_number);

//d�marrage du thread de r�gulation
void regulator_start(void);

#endif /* REGULATION_H_ */
