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

// inits the message bus, the mutexe and the conditionnal variable used for the communication with the IMU and the proximity sensors
// It is necessary to include main.h in the files where the bus is used
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
 * function controlling the LEDs
 * Two red LEDs turn on when the calibration starts
 * When the calibration ends, the red LEDs turn off and the body LEDs turn on
 *
 * \param state		state that we want to display		1 : calibration begin	2: calibration end
 */
void leds_calibration(unsigned int state) {
    set_led(LED3, state);
    set_led(LED7, state);
    if (state == 0) {
    	set_body_led(1);
    }
}

int main(void)
{
	/* Initializations */

	//inits the inter-process communication bus
	messagebus_init(&bus, &bus_lock, &bus_condvar);

    halInit();
    chSysInit();

    //starts the serial communication
    serial_start();
    //starts timer 12
    timer12_start();

    // red LEDs turn on
    leds_calibration(1);

    // sensors calibration and threads start
    // the robot should stay on a flat surface and far from the walls
    compute_angle_thd_start(); // calibrates the IMU and starts the thread dedicated to the computation of the angle
    prox_sensors_start(); // calibrates the proximity sensors and starts the thread dedicated to the proximity sensors

    // end of calibrations
    // bodyLEDs turn on
    leds_calibration(0);

    // starts the thread dedicated to the regulation and motors control
    regulator_start();

    /* Infinite loop. */
    while (1) {
    	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
