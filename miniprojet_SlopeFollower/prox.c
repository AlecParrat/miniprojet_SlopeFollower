/*
 * prox.c
 *
 *  Created on: 25 avr. 2022
 *      Author: alecp
 */

#include <main.h>
#include <prox.h>
#include <msgbus/messagebus.h>
#include <sensors/proximity.h>

int get_proximity(int sensor_number) {
	return get_prox(sensor_number);
}

void prox_sensors_start(void) {
	proximity_start();
}
