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

#include <sensors/imu.h>
#include <sensors/mpu9250.h>
#include <i2c_bus.h>

//inits the messagebus with a mutexe and a condvar in order to transfer data from the IMU
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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
    halInit();							//Hardware Abstraction Layer
    chSysInit();						//RTOS Chibi OS

    //starts the serial communication
    serial_start();
    //starts timer 12
    timer12_start();
    //inits the motors
    motors_init();

    //starts the I2C communication bus
    i2c_start();
    //starts the IMU
    imu_start();

    //inits the inter-process communication bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;



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
