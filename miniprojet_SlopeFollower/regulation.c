/*
 * regulation.c
 *
 *  Created on: 20 avr. 2022
 *      Author: alecp
 */

#include <main.h>
#include <motors.h>
#include <regulation.h>
#include <angle.h>
#include <chprintf.h>
#include <prox.h>
#include <leds.h>

// customizable parameters
#define PRINT 0 // 1 to print the regulator variable in the serial, 0 to stop the printing
#define MOTORS_ON 1 // 1 to turn the motors on, 0 to turn them of
#define ARW true // true to activate the Anti Reset Windup

// operating modes
#define NORMAL false	// standart
#define ESCAPING true	// proximity alert : escape maneuver

// wheels maximum speed [step/s]
#define SPEED_MAX  1000

#define SPEED_MOY (SPEED_MAX/2) // wheel average speed during the normal operations

// percentage of turn to do during escape maneuvers
#define PERCENT_FRONT 50
#define PERCENT_MIDDLE 38
#define PERCENT_SIDE 25

// number of steps to do a 360° turn
#define STEPS_TURN 1320

// measured time to execute thread content : 12 us
// period of the regulation thread [ms]
#define REGUL_PERIOD 10 // 10 ms -> 100 Hz

// regulator constants
#define KP 5 //5
#define KI 0.02 //0.1

#define AVERAGE_SIZE_SPEED 10

/*
 * PI regulator
 * input : slope direction (angle) relative to the front of the robot
 * output : speed difference to apply to the motors
 * A variable speed diference signify controllable turns
 *
 * \param mesured_angle : slope angle measured by the angle thread
 *
 * \param angle_to_reach : angle to reach (always 0 here)
 *
 * \param reset : if true : reset of the regulator variables
 *
 * \return	speed difference to apply to the motors
 */
int16_t regulator(int16_t mesured_angle, int16_t angle_to_reach, bool reset){
	int16_t err = 0; // angle error
	float prop = 0; // proportional term, float because order depends on the KP
	float integr = 0; // integral term, float because order depends on the KI
	static float integr_last = 0; // last value of the integral term
	static int16_t delta_speed = 0;// output to compute (to keep for the ARW)
	static int16_t delta_speed_ini = 0; // delta speed before limit check (to keep for the ARW)

	err = mesured_angle - angle_to_reach;

	prop = KP * (float)err;
	if(KI != 0) { // useless if KI = 0
		integr = integr_last + KI * (float)err;
	}

	// integral term to reset if needed
	if(reset || err == 0) {
		integr = 0;
		integr_last = 0;
	}

	delta_speed_ini = prop + integr; // command computation

	// limits management
	if (delta_speed_ini > SPEED_MAX) {
		delta_speed = SPEED_MAX;
	} else if (delta_speed_ini < -SPEED_MAX) {
		delta_speed = -SPEED_MAX;
	} else {
		delta_speed = delta_speed_ini;
	}

	// ARW management, useless if KI = 0
	if (KI != 0) {
		// if ARW activated AND there is saturation AND integration term would get bigger
		if((ARW == true) && (delta_speed != delta_speed_ini) && abs(integr) >  abs(integr_last)) {
			integr = integr_last; // integral term, can't get bigger
		} else {
			integr_last = integr; // normally stock integral term in the last one
		}
	}

	if(PRINT) {
//		int16_t pourcent_P = 0;
//		int16_t pourcent_I = 0;
//		pourcent_P = 100*prop/delta_speed_ini;
//		pourcent_I = 100 - pourcent_P;

		chprintf((BaseSequentialStream *)&SD3, "Angle : %4d     ", mesured_angle);
		chprintf((BaseSequentialStream *)&SD3, "Propo : %6.1f     ", prop);
//		chprintf((BaseSequentialStream *)&SD3, "%d%%  |  ", pourcent_P);
		chprintf((BaseSequentialStream *)&SD3, "Integ : %6.1f \n\r", integr);
//		chprintf((BaseSequentialStream *)&SD3, "%d%%  |  ", pourcent_I);
	}

	return delta_speed;
}

/*
 * Escape maneuvers function
 * Defines motors sense (robot is rotating without advancing)
 * Defines the duration of turn
 *
 * \param alert_number : Position of proximity alert
 *
 * \retun : number of steps to do to finish the escape maneuver
 */
int32_t escape(int8_t alert_number) {
	int32_t steps_to_do = 0;
	int16_t speed = SPEED_MAX;

	switch (alert_number) {

	// choice of movement to do
	case R_SIDE : // -90°
		set_led(LED3, 1);
		steps_to_do = STEPS_TURN * PERCENT_SIDE / 100;
		speed = -SPEED_MAX;
		break;

	case R_CENTER : // ~ -135°
		set_led(LED1, 1);
		set_led(LED3, 1);
		steps_to_do = STEPS_TURN * PERCENT_MIDDLE / 100;
		speed = -SPEED_MAX;
		break;

	case R_FRONT : // -180°
		set_led(LED1, 1);
		steps_to_do = STEPS_TURN * PERCENT_FRONT / 100;
		speed = -SPEED_MAX;
		break;

	case L_FRONT : // 180°
		set_led(LED1, 1);
		steps_to_do = STEPS_TURN * PERCENT_FRONT / 100;
		speed = SPEED_MAX;
		break;

	case L_CENTER : // ~135°
		set_led(LED1, 1);
		set_led(LED7, 1);
		steps_to_do = STEPS_TURN * PERCENT_MIDDLE / 100;
		speed = SPEED_MAX;
		break;

	case L_SIDE : // 180°
		set_led(LED7, 1);
		steps_to_do = STEPS_TURN * PERCENT_SIDE / 100;
		speed = SPEED_MAX;
		break;
	}

	// motor command
	left_motor_set_speed(speed);
	right_motor_set_speed(-speed);
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	return abs(steps_to_do);
}

/*
 * movement command thread
 * defines movement mode (normal / escaping)
 * calls the PI regulator in normal mode
 * enters the speed for each motor
 * calls the escape function if a wall is close
 * controls the escape maneuvers duration
 * important that the regulator runs at a precise frequency : high priority
 */
static THD_WORKING_AREA(waRegulator, 256);
static THD_FUNCTION(Regulator, arg) {

	//chRegSetThreadName(__FUNCITON__);
	(void)arg;

	systime_t time;

	bool mode_fonc = NORMAL; // movement mode
	int8_t prox_alert = 0; // alerts returned by the proximity sensors
	int16_t steps_to_do = 0; // steps to do to finish an escape maneuver
	int16_t delta_speed = 0; // speed difference between the motors in normal mode
	int16_t delta_speed_mean = 0;
	int16_t angle_consigne = 0; // angle desired between the slope and the front of the robot

	static int32_t sum_dSpeed = 0;
	static int16_t values_dSpeed[AVERAGE_SIZE_SPEED] = {0};
	static int16_t counter_dSpeed = 0;

	while(1) {
		time = chVTGetSystemTime();

		//set_front_led(1);

		// see if there is a proximity alert
		if(mode_fonc == NORMAL) {
			prox_alert = get_prox_alert();
		}

		// functioning modes control
		if ((mode_fonc == NORMAL) && (prox_alert == 0)) { // normal mode
			// call of the PI regulator, with the last computed angle
			// if the slope is small, resets the integral term of the regulator
			delta_speed = regulator(get_angle(), angle_consigne, false);
			delta_speed_mean = average(delta_speed, &sum_dSpeed, values_dSpeed, &counter_dSpeed, AVERAGE_SIZE_SPEED);
			// motors command with the regulated value
			if(MOTORS_ON) {
				right_motor_set_speed(SPEED_MOY - delta_speed_mean);
				left_motor_set_speed(SPEED_MOY + delta_speed_mean);
			}

		} else if ((mode_fonc == NORMAL) && (prox_alert != 0)) { // escape maneuver begins
			mode_fonc = ESCAPING;
			steps_to_do = escape(prox_alert); // start of the maneuver

		} else if ((mode_fonc == ESCAPING) && (abs(left_motor_get_pos()) >= steps_to_do)) { // escape maneuver ends
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			mode_fonc = NORMAL;
			delta_speed = regulator(get_angle(), angle_consigne, true); // call the regulator and resets its variable
			if(MOTORS_ON) {
				right_motor_set_speed(SPEED_MOY - delta_speed);
				left_motor_set_speed(SPEED_MOY + delta_speed);
			}
			clear_leds();
		}

		//set_front_led(0);

		chThdSleepUntilWindowed(time, time + MS2ST(REGUL_PERIOD));
	}
}

/*
 * regulator thread starts
 */
void regulator_start(void){
    motors_init();
	chThdCreateStatic(waRegulator, sizeof(waRegulator), NORMALPRIO + 1, Regulator, NULL);
}
