#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <arm_math.h>
#include <math.h>
#include <prox.h>
#include <regulation.h>

#include <sensors/imu.h>
#include <sensors/mpu9250.h>
#include <i2c_bus.h>
#include <angle.h>
#include <leds.h>

//inclure main.h dans les fichiers où on utilise les bus
//inits the message bus, the mutexe and the conditionnal variable used for the communication with the IMU
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//inits the serial communication
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

//inits the timer 12
static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

/*
 * fonction de gestion des LEDs indicatives de début/fin des calibrations
 *
 * \param etat	etat que l'on souhaite afficher		1 : calibration begin	2: calibration end
 */
void leds_calibration(unsigned int etat) {
//    set_led(LED3, etat);
//    set_led(LED7, etat);
    set_rgb_led(LED2, 100*etat, 100*etat, 100*etat);
    set_rgb_led(LED4, 100*etat, 100*etat, 100*etat);
    set_rgb_led(LED6, 100*etat, 100*etat, 100*etat);
    set_rgb_led(LED8, 100*etat, 100*etat, 100*etat);
    if (etat == 0) {
    	set_body_led(1);
    }
}

int main(void)
{
	/* Initialisations */

	//inits the inter-process communication bus
	messagebus_init(&bus, &bus_lock, &bus_condvar);

    halInit();
    chSysInit();

    //starts the serial communication
    serial_start();
    //starts timer 12
    timer12_start();

    //démarre la communication SPI pour l'utilisation des LEDs RGB
    spi_comm_start();

    // LEDs rouge signalant que le robot commence les calibrations
    leds_calibration(1);

    // démarrage des capteurs de proximité et de l'IMU, calibrations
    // le robot doit rester à plat et loin des murs
    compute_angle_thd_start(); //starts the thread dedicated to the computation of the angle
    prox_sensors_start(); // démarrage des capteurs de proximité

    //petit temps d'attente par sécurité : 2s
    //wait(MS2ST(2000)); //bonne fonction ?


    //fin des calibrations, le robot peut bouger
    // LEDs vertes signalant la fin des calibrations
    leds_calibration(0);

    // démarrage de la régulation et des moteurs
    regulator_start();

    /* Infinite loop. */
    while (1) {

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
