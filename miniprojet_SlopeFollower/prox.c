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
#include <chprintf.h>
#include <stdbool.h>

#define PROX_ACTIVATED true // true to activate proximity alerts

// Proximity threshold : above this proximity value, a proximity alert is enabled
#define PROXIMITY_TRESHOLD 800

#define PROXIMITY_PERIOD 100 // period of the proximity thread (in ms)

// Sensors numbers definition
#define RIGHT_3 2		// IR3 on the body
#define RIGHT_2 1		// IR2 on the body
#define RIGHT_1 0		// IR1 on the body
#define LEFT_1 7		// IR8 on the body
#define LEFT_2 6		// IR7 on the body
#define LEFT_3 5		// IR6 on the body

// Proximity alert variable
static int8_t proximity_alert = 0;

/*Alert values*/
// 0 : no alert
// 1 : right_1 alert
// 2 : right_2 alert
// 3 : right_3 alert
// 4 : left_1 alert
// 5 : left_2 alert
// 6 : left_3 alert

/*
 * test function, unused
 * allows to  get the value of the proximity in an other file
 *
 * \param sensor_number		number of the sensor to read
 *
 * \return		value mesured by the sensor
 */
int get_proximity(int sensor_number) {
	// without calibration
	// return get_prox(sensor_number);

	// with calibration
	return get_calibrated_prox(sensor_number);
}

/*
 * allows to  get the value of the proximity alert in an other file
 *
 * \return number of current proximity alert
 */
int8_t get_prox_alert(void){
	return proximity_alert;
}

/*
 * thread dedicated to the acquisition of the proximity with the 6 sensors at the front of the robot (IR 1, 2, 3, 6, 7, 8)
 */
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

	while(1){

		time = chVTGetSystemTime();

		proxy_right_3 = get_proximity(RIGHT_3);
		proxy_right_2 = get_proximity(RIGHT_2);
		proxy_right_1 = get_proximity(RIGHT_1);
		proxy_left_1 = get_proximity(LEFT_1);
		proxy_left_2 = get_proximity(LEFT_2);
		proxy_left_3 = get_proximity(LEFT_3);

		// alert on the right_3 :
		if (proxy_right_3 > PROXIMITY_TRESHOLD && proxy_right_3 > proxy_right_2){
			proximity_alert = R_SIDE;
		}
		// alert on the right_2 :
		else if (proxy_right_2 > PROXIMITY_TRESHOLD && proxy_right_2 > proxy_right_3 && proxy_right_2 > proxy_right_1){
			proximity_alert = R_CENTER;
		}
		// alert on the right_1 :
		else if (proxy_right_1 > PROXIMITY_TRESHOLD && proxy_right_1 > proxy_right_2 && proxy_right_1 > proxy_left_1){
			proximity_alert = R_FRONT;
		}
		// alert on the left_1 :
		else if (proxy_left_1 > PROXIMITY_TRESHOLD && proxy_left_1 > proxy_left_2 && proxy_left_1 > proxy_right_1){
			proximity_alert = L_FRONT;
		}
		// alert on the left_2 :
		else if (proxy_left_2 > PROXIMITY_TRESHOLD && proxy_left_2 > proxy_left_1 && proxy_left_2 > proxy_left_3){
			proximity_alert = L_CENTER;
		}
		// alert on the left_3 :
		else if(proxy_left_3 > PROXIMITY_TRESHOLD && proxy_left_3 > proxy_left_2){
			proximity_alert = L_SIDE;
		}
		else{
			proximity_alert = 0;
		}

		// possibility to deactivate proximity alerts
		if (PROX_ACTIVATED == false) {
			proximity_alert = 0;
		}

		/*uncomment to print the alert value*/
		// chprintf((BaseSequentialStream *)&SD3, "Alerte : %d\r\n", proximity_alert);

		/*uncomment to print the proximity sensors value*/
		// chprintf((BaseSequentialStream *)&SD3, "right_3 = %d right_2 = %d right_1 = %d left_1 = %d left_2 = %d left_3 = %d\r\n",
		// proxy_right_3, proxy_right_2, proxy_right_1, proxy_left_1, proxy_left_2, proxy_left_3);

		chThdSleepUntilWindowed(time, time + MS2ST(PROXIMITY_PERIOD));
	}
}

/*
 * Initializes and calibrates the proximity sensors, starts the thread dedicated to the alert of proximity
 */
void prox_sensors_start(void) {
	proximity_start();
	calibrate_ir();
	chThdCreateStatic(get_proximity_thd_wa, sizeof(get_proximity_thd_wa), NORMALPRIO, get_proximity_thd, NULL);
}
