#include <math.h>

#include <sensors/imu.h>
#include <sensors/mpu9250.h>
#include <i2c_bus.h>
#include <msgbus/messagebus.h>
#include <angle.h>
#include <chprintf.h>

#define PI 3.14
#define COMPUTE_ANGLE_PERIOD 10				//waiting time (in ms) of the thread that computes the angle

static int16_t angle = 0; 					//measured angle
static int16_t acc_x = 0;					//acceleration on the X axis
static int16_t acc_y = 0;					//acceleration on the Y axis

//inits the message bus, the mutexe and the conditionnal variable used for the communication with the IMU
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//computes and returns the angle according to the defined convention (left : [-180°, 0°[ ; right : ]0°, +180°]
int16_t compute_angle(void){

	acc_x = get_acc(0) - get_acc_offset(0);		//acquires the acceleration on the X axis and retires the offset from the calibration.

	acc_y = get_acc(1) - get_acc_offset(1);		//acquires the acceleration on the Y axis and retires the offset from the calibration.

	angle=(180/PI)*atan(((float)acc_y)/((float)acc_x));	//computes the angle and converts it in degrees

	//corrects the angle value according to the orientation of the accelerometer (see axis printed on the body)

	// dial 1
	if(acc_x > 0 && acc_y > 0){
		angle = -angle-+90;
	}

	// dial 2
	if(acc_x < 0 && acc_y > 0){
		angle = -angle+90;
	}

	// dial 3
	if(acc_x < 0 && acc_y < 0){
		angle = -angle+90;
	}

	// dial 4
	if(acc_x > 0 && acc_y <0){
		angle = -angle-90;
	}

	//uncomment to print the computed angle value in RealTerm
	//chprintf((BaseSequentialStream *)&SD3, "%Angle_x=%-7d\r\n", angle);

	return(angle);
}

//allows to get the measured angle value from another file
int16_t get_angle(void) {
	return angle;
}


//thread dedicated to the computation of the angle
static THD_WORKING_AREA(compute_angle_thd_wa, 1024);
static THD_FUNCTION(compute_angle_thd, arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	while(1){

    	angle=compute_angle();

		chThdSleepUntilWindowed(time, time + MS2ST(COMPUTE_ANGLE_PERIOD));
	}
}

//inits and calibrates the IMU, starts the thread that computes the angle
void compute_angle_thd_start(){

	//inits the inter-process communication bus
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	//starts the IMU
	imu_start();
    //calibrates the IMU
    calibrate_acc();
	//starts the thread dedicated to the computation of the angle
	chThdCreateStatic(compute_angle_thd_wa, sizeof(compute_angle_thd_wa), NORMALPRIO, compute_angle_thd, NULL);
}
