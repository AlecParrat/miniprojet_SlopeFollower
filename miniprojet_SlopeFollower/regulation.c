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

// modes de fonctionnement
#define NORMAL false		// fonctionnement normal
#define ESCAPING true	// echappement d'un mur

#define PRINT 0 //1 pour afficher les variables du régulateur, 0 pour pas afficher

#define MOTORS_ON 0 // 1 pour allumer les moteurs, 0 pour les éteindre

//vitesse max du robot [step/s]
#define SPEED_MAX  1000

#define SPEED_MOY (SPEED_MAX/2) //vitesse moyenne du robot

// pourcentage de tour à faire pour chaque manoeuvre d'esquive
#define POURCENT_AVANT 50
#define POURCENT_MILIEU 38
#define POURCENT_COTE 25

// Nombre de steps pour 1 tour sur lui-meme
#define STEPS_TOUR 1320

//période d'appel du régulateur [ms]
#define REGUL_PERIOD 10 // 10 ms -> 100 Hz

//constantes du régulateur
#define KP 5
#define KI 0.01
#define KD 0

// régulateur PID
int16_t regulator(int16_t angle_pente, int16_t angle_consigne, bool reset){
	int16_t err = 0; //error relativ to consigne
	float prop = 0; // proportional term, float because can be small

	static int16_t delta_speed = 0;//to compute (to keep for the ARW)
	static int16_t delta_speed_ini = 0; // delta speed before limit check (to keep for the ARW)
	static float integr = 0; // integral term, float because can be small
	static int16_t err_pre = 0; //previous error for the differential term

	// Variables à remettre à 0 après une pause de régualtion
	if(reset) {
		integr = 0;
		err_pre = 0;
	}

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

// fonction d'esquive d'obstacles
// retuourne le nombre de step à effectuer
int32_t esquive(int8_t alert_number) {
	int32_t steps_to_do = 0;
	int16_t speed = SPEED_MAX;

	switch (alert_number) {

	//définition du mouvement à effectuer
	case R_SIDE :
		steps_to_do = STEPS_TOUR * POURCENT_COTE / 100;
		speed = -SPEED_MAX;
		break;

	case R_CENTER :
		steps_to_do = STEPS_TOUR * POURCENT_MILIEU / 100;
		speed = -SPEED_MAX;
		break;

	case R_FRONT :
		steps_to_do = STEPS_TOUR * POURCENT_AVANT / 100;
		speed = -SPEED_MAX;
		break;

	case L_FRONT :
		steps_to_do = STEPS_TOUR * POURCENT_AVANT / 100;
		speed = SPEED_MAX;
		break;

	case L_CENTER :
		steps_to_do = STEPS_TOUR * POURCENT_MILIEU / 100;
		speed = SPEED_MAX;
		break;

	case L_SIDE :
		steps_to_do = STEPS_TOUR * POURCENT_COTE / 100;
		speed = SPEED_MAX;
		break;
	}

	//gestion des moteurs
	left_motor_set_speed(speed);
	right_motor_set_speed(-speed);
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	return abs(steps_to_do);
}

// thread de gestion du déplacement
// définit le mode de déplacement
// gestion du PID pour la commande des moteurs
// définit la différence de vitesse entre les deux roues (virage)
// gestion des manoeuvres d'esquive
static THD_WORKING_AREA(waRegulator, 256);
static THD_FUNCTION(Regulator, arg) {

	//chRegSetThreadName(__FUNCITON__);
	(void)arg;

	systime_t time;

	bool mode_fonc = NORMAL; // mode de fonctionnement
	int8_t prox_alert = 0; //variable controlée par le thread des capteurs de proximité
	int16_t steps_to_do = 0; //step à faire pour l'esquive

	// différence de vitesse entre les roues
	int16_t delta_speed = 0;

	// consigne d'angle à obtenir entre l'inclinaison et le sens de marche
	int16_t angle_consigne = 0;

	while(1) {
		time = chVTGetSystemTime();

		//récupération des alertes de proximité (inutile si en cours d'estquive
		if(mode_fonc == NORMAL) {
			prox_alert = get_prox_alert();
		}

		//gestion du mode de fonctionnement
		if ((mode_fonc == NORMAL) && (prox_alert == 0)) { // mode normal
			// appel du régulateur, la fct get_angle va chercher l'angle mesuré
			delta_speed = regulator(get_angle(), angle_consigne, false);

			// calcul de la vitesse de rotation à transmettre à chaque moteur
			// envoi de la consigne aux moteurs
			if(MOTORS_ON) {
				right_motor_set_speed(SPEED_MOY - delta_speed);
				left_motor_set_speed(SPEED_MOY + delta_speed);
			}

		} else if ((mode_fonc == NORMAL) && (prox_alert != 0)) { // manoeuvre d'esquive
			mode_fonc = ESCAPING;
			steps_to_do = esquive(prox_alert); // démarrage de la manoeuvre d'esquive

		} else if ((mode_fonc == ESCAPING) && (abs(left_motor_get_pos()) >= steps_to_do)) { // fin de l'esquive
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			mode_fonc = NORMAL;
			delta_speed = regulator(get_angle(), angle_consigne, true); //appel du régulateur et reset des variable
			if(MOTORS_ON) {
				right_motor_set_speed(SPEED_MOY - delta_speed);
				left_motor_set_speed(SPEED_MOY + delta_speed);
			}
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
