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

//faudra include imu.h pour obtenir l'angle directement avec une fonction
// int16_t get_angle(void); qui retourne l'angle mesuré

#define PRINT 0 //1 pour afficher les variables du régulateur, 0 pour pas afficher

#define MOTORS_ON 0 // 1 pour allumer les moteurs, 0 pour les éteindre

//vitesse rectiligne d'avance du robot [step/s]
#define SPEED_MOY  500 //500

//période d'appel du régulateur [ms]
#define REGUL_PERIOD 10 // 10 ms -> 100 Hz

//constantes du régulateur
#define KP 5
#define KI 0.01
#define KD 0

// régulateur PID
int16_t regulator(int16_t angle_pente, int16_t angle_consigne){
	int16_t err = 0; //error relativ to consigne
	float prop = 0; // proportional term, float because can be small

	static int16_t delta_speed = 0;//to compute (to keep for the ARW)
	static int16_t delta_speed_ini = 0; // delta speed before limit check (to keep for the ARW)
	static float integr = 0; // integral term, float because can be small
	static int16_t err_pre = 0; //previous error for the differential term

	err = angle_pente - angle_consigne;

	prop = KP * (float)err;

	if(KI != 0) { // inutile de faire ce calcul si KI = 0
		//intégrateur avec anti-reset windup commenté
		integr += KI * (float)err; //*(prop - (delta_speed_ini-delta_speed));
	}

	//calcul de la commande, transformation en float pour plus de précision avant la somme
	delta_speed_ini = prop + integr + KD * (float)(err - err_pre);
	err_pre = err;

	//gestion des limites
	if (delta_speed_ini > (2*SPEED_MOY)) {
		delta_speed = (2*SPEED_MOY);
	} else if (delta_speed_ini < -(2*SPEED_MOY)) {
		delta_speed = -(2*SPEED_MOY);
	} else {
		delta_speed = delta_speed_ini;
	}

	if(PRINT) {
//		int16_t pourcent_P = 0;
//		int16_t pourcent_I = 0;
//		pourcent_P = 100*prop/delta_speed_ini;
//		pourcent_I = 100 - pourcent_P;

		chprintf((BaseSequentialStream *)&SD3, "Angle : %4d     ", angle_pente);
		chprintf((BaseSequentialStream *)&SD3, "Propo : %6.1f     ", prop);
//		chprintf((BaseSequentialStream *)&SD3, "%d%%  |  ", pourcent_P);
		chprintf((BaseSequentialStream *)&SD3, "Integ : %6.1f     ", integr);
//		chprintf((BaseSequentialStream *)&SD3, "%d%%  |  ", pourcent_I);
		chprintf((BaseSequentialStream *)&SD3, "D_spe : %4d  \r\n", delta_speed_ini);
	}

	return delta_speed;
}

// thread de gestion du PID pour la commande des moteurs
// définit la différence de vitesse entre les deux roues (virage)
static THD_WORKING_AREA(waRegulator, 256);
static THD_FUNCTION(Regulator, arg) {

	//chRegSetThreadName(__FUNCITON__);
	(void)arg;

	systime_t time;

	// différence de vitesse entre les roues
	int16_t delta_speed = 0;

	// consigne d'angle à obtenir entre l'inclinaison et le sens de marche
	int16_t angle_consigne = 0;

	while(1) {
		time = chVTGetSystemTime();

		// appel du régulateur, la fct get_angle va chercher l'angle mesuré
		delta_speed = regulator(get_angle(), angle_consigne);

		// calcul de la vitesse de rotation à transmettre à chaque moteur
		// envoi de la consigne aux moteurs
		if(MOTORS_ON) {
			right_motor_set_speed(SPEED_MOY - delta_speed);
			left_motor_set_speed(SPEED_MOY + delta_speed);
		}

		//frequence du thread
		chThdSleepUntilWindowed(time, time + MS2ST(REGUL_PERIOD));
	}
}

//démarrage du thread de régulation
void regulator_start(void){
    motors_init();
	chThdCreateStatic(waRegulator, sizeof(waRegulator), NORMALPRIO, Regulator, NULL);
}
