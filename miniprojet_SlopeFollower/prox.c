/*
 * prox.c
 *
 *  Created on: 25 avr. 2022
 *      Author: alecp
 */

// Sensor numbers definition
#define RIGHT_3 2		// IR3 on the body
#define RIGHT_2 1		// IR2 on the body
#define RIGHT_1 0		// IR1 on the body
#define LEFT_1 7		// IR8 on the body
#define LEFT_2 6		// IR7 on the body
#define LEFT_3 5		// IR6 on the body

// Proximity treshold
#define PROXIMITY_TRESHOLD 1500

#define GET_PROXIMITY_PERIOD 100

#include <main.h>
#include <prox.h>
#include <msgbus/messagebus.h>
#include <sensors/proximity.h>
#include <chprintf.h>
#include <stdbool.h>

int get_proximity(int sensor_number) {
	//sans calibration
	//return get_prox(sensor_number);

	//avec calibration
	return get_calibrated_prox(sensor_number);
}

//thread dedicated to the acquisition of the proximity with the 4 sensors at the front of the robot (IR 1, 2, 7, 8)
static THD_WORKING_AREA(get_proximity_thd_wa, 1024);
static THD_FUNCTION(get_proximity_thd, arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	// Proximity variables
	int16_t proxy_right_3 = 0;
	int16_t proxy_right_2 = 0;
	int16_t proxy_right_1 = 0;
	int16_t proxy_left_1 = 0;
	int16_t proxy_left_2 = 0;
	int16_t proxy_left_3 = 0;

	// Proximity alert
	int8_t proximity_alert = 0;

	// Alert values :

	// 0 : no alert
	// 1 : right_1 alert
	// 2 : right_2 alert
	// 3 : right_3 alert
	// 4 : left_1 alert
	// 5 : left_2 alert
	// 6 : left_3 alert

	while(1){

		proxy_right_3 = get_proximity(RIGHT_3);
		proxy_right_2 = get_proximity(RIGHT_2);
		proxy_right_1 = get_proximity(RIGHT_1);
		proxy_left_1 = get_proximity(LEFT_1);
		proxy_left_2 = get_proximity(LEFT_2);
		proxy_left_3 = get_proximity(LEFT_3);

		// alert on the right_3 :
		if (proxy_right_3 > PROXIMITY_TRESHOLD && proxy_right_3 > proxy_right_2){
			proximity_alert = 1;
		}
		// alert on the right_2 :
		else if (proxy_right_2 > PROXIMITY_TRESHOLD && proxy_right_2 > proxy_right_1 && proxy_right_2 > proxy_left_1 && proxy_right_2 > proxy_left_2){
			proximity_alert = 2;
		}
		// alert on the right_1 :
		else if (proxy_right_1 > PROXIMITY_TRESHOLD && proxy_right_1 > proxy_right_2 && proxy_right_1 > proxy_left_1 && proxy_right_1 > proxy_left_2){
			proximity_alert = 3;
		}
		// alert on the left_1 :
		else if (proxy_left_1 > PROXIMITY_TRESHOLD && proxy_left_1 > proxy_right_2 && proxy_left_1 > proxy_right_1 && proxy_left_1 > proxy_left_2){
			proximity_alert = 4;
		}
		// alert on the left_2 :
		else if (proxy_left_2 > PROXIMITY_TRESHOLD && proxy_left_2 > proxy_right_2 && proxy_left_2 > proxy_right_1 && proxy_left_2 > proxy_left_1){
			proximity_alert = 5;
		}
		// alert on theleft_3 :
		else if(proxy_left_3 > PROXIMITY_TRESHOLD && proxy_left_3 > proxy_left_2){
			proximity_alert = 6;
		}
		else{
			proximity_alert = 0;
		}

		chprintf((BaseSequentialStream *)&SD3, "Alerte : %d\r\n", proximity_alert);


//		chprintf((BaseSequentialStream *)&SD3, "Droite = %4d   Devant droite = %4d   Devant gauche = %4d   Gauche = %4d\r\n"
//		    									, proxy_right, proxy_front_right, proxy_front_left, proxy_left);

		chThdSleepUntilWindowed(time, time + MS2ST(GET_PROXIMITY_PERIOD));

	}
}

void prox_sensors_start(void) {
	proximity_start();
	calibrate_ir();
	chThdCreateStatic(get_proximity_thd_wa, sizeof(get_proximity_thd_wa), NORMALPRIO, get_proximity_thd, NULL);
}
