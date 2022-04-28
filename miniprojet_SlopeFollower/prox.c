/*
 * prox.c
 *
 *  Created on: 25 avr. 2022
 *      Author: alecp
 */

#define GET_PROXIMITY_PERIOD 100

#include <main.h>
#include <prox.h>
#include <msgbus/messagebus.h>
#include <sensors/proximity.h>
#include <chprintf.h>

int get_proximity(int sensor_number) {
	return get_prox(sensor_number);
}

//thread dedicated to the acquisition of the proximity with the 4 sensors at the front of the robot (IR 1, 2, 7, 8)
static THD_WORKING_AREA(get_proximity_thd_wa, 1024);
static THD_FUNCTION(get_proximity_thd, arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	int16_t proxy_1 = 0;
	int16_t proxy_2 = 0;
	int16_t proxy_7 = 0;
	int16_t proxy_8 = 0;

	while(1){

		proxy_1=get_proximity(0);
		proxy_2=get_proximity(1);
		proxy_7=get_proximity(6);
		proxy_8=get_proximity(7);

		chprintf((BaseSequentialStream *)&SD3, "Droite = %4d   Devant droite = %4d   Devant gauche = %4d   Gauche = %4d\r\n"
		    									, proxy_2, proxy_1, proxy_8, proxy_7);

		chThdSleepUntilWindowed(time, time + MS2ST(GET_PROXIMITY_PERIOD));

	}
}

void prox_sensors_start(void) {
	proximity_start();
	chThdCreateStatic(get_proximity_thd_wa, sizeof(get_proximity_thd_wa), NORMALPRIO, get_proximity_thd, NULL);
}
