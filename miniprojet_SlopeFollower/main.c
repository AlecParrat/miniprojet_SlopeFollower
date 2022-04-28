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

#include <regulation.h>

#include <sensors/imu.h>
#include <sensors/mpu9250.h>
#include <i2c_bus.h>
#include <angle.h>

#include <sensors/proximity.h>

//inits a message bus, a mutexe and a conditionnal variable used for the communication
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

int main(void)
{
	/* Initialisations */

    halInit();
    chSysInit();

    //starts the serial communication
    serial_start();
    //starts timer 12
    timer12_start();
    //inits the motors
    motors_init();

    //starts the IMU
    imu_start();
    //calibrates the IMU
    calibrate_acc();

    //starts the thread dedicated to the computation of the angle
    //compute_angle_thd_start();

    // démarrage de la régulation
    //regulator_start();

    proximity_start();

    unsigned int prox_1=0;

    /* Infinite loop. */
    while (1) {
    	prox_1=get_prox(1);
    	chprintf((BaseSequentialStream *)&SD3, "%Prox_1 =%4d\r\n", prox_1);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
