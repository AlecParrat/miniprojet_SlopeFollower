#include <math.h>

#include <sensors/imu.h>
#include <sensors/mpu9250.h>
#include <i2c_bus.h>
#include <msgbus/messagebus.h>
#include <angle.h>
#include <chprintf.h>
#include <main.h>

#define PI 3.14
#define COMPUTE_ANGLE_PERIOD 20 // period (in ms) of the thread that computes the angle

// accelerometer axis
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define INCL_LIMIT 300 // inclination threshold, if the slope isn't sufficient, the angle is 0

#define AVERAGE_SIZE 10 // number of value to use to compute the average

int16_t average(int16_t new_value, int16_t* sum, int16_t values[AVERAGE_SIZE], int16_t* counter) {

	(*sum) -= values[*counter]; //remove the oldest value form the sum
	values[*counter] = new_value; //stock the new value
	(*sum) += values[*counter]; //add the new value to the sum

	(*counter) ++;
	if(*counter == AVERAGE_SIZE) {
		*counter = 0; // cycle the counter
	}

	return *sum / AVERAGE_SIZE;
}

static int16_t angle = 0; // computed angle (value to regulate)
static bool flat = true; // true if the slope is small (useful for the regulator)

/*
 * computes and returns the slope angle according to the defined convention (left : [-180°, 0°[ ; right : ]0°, +180°])
 *
 *         BACK
 *         ####
 *      #180/-180#
 *    #            #
 * R # 90  TOP  -90 # L
 *    #    VIEW    #
 *      #   0    #
 *         ####
 *         FRONT
 *
 * The slope angle is the direction of descending slope in an inclined plane
 * In a flat surface, it is undefined, in that there is an inclination threshold and it is put to 0
 *
 * \return	computed angle
 */
int16_t compute_angle(void){

	int16_t acc_x = 0;					// acceleration on the X axis
	int16_t acc_y = 0;					// acceleration on the Y axis
	int16_t acc_z = 0;					// acceleration on the Z axis
	int16_t acc_z_mean = 0;				// mean of acceleration Z

	// values to keep for the average computing
	static int16_t sum_angle = 0;
	static int16_t values_angle[AVERAGE_SIZE] = {0};
	static int16_t counter_angle = 0;
	static int16_t sum_slope = 0;
	static int16_t values_slope[AVERAGE_SIZE] = {0};
	static int16_t counter_slope = 0;

	acc_z = get_acc(Z_AXIS) - get_acc_offset(Z_AXIS);

	acc_z_mean = average(acc_z, &sum_slope, values_slope, &counter_slope);

	if(acc_z_mean > INCL_LIMIT) {
		flat =  false; // slope is sufficient to start regulation
		acc_x = get_acc(X_AXIS) - get_acc_offset(X_AXIS); // acquires the acceleration on the X axis and retires the offset from the calibration.

		acc_y = get_acc(Y_AXIS) - get_acc_offset(Y_AXIS); // acquires the acceleration on the Y axis and retires the offset from the calibration.

		angle=(180/PI)*atan(((float)acc_y)/((float)acc_x));	// computes the angle and converts it in degrees

		// corrects the angle value according to the orientation of the accelerometer (see axis printed on the body)

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
	} else {
		angle = 0;
		flat = true; // slope isn't sufficient to start regulation
	}

	// uncomment to print the computed angle value in RealTerm
	// chprintf((BaseSequentialStream *)&SD3, "%Angle_x=%-7d\r\n", angle);

	// uncomment to print z acceleration with offset
	chprintf((BaseSequentialStream *)&SD3, "%acc en z =%-7d       ", acc_z);
	chprintf((BaseSequentialStream *)&SD3, "%acc en z moy =%-7d\r\n", acc_z_mean);

	return(angle);
}

/*
 * allows to get the last computed angle value from another file
 *
 * \return	computed angle
 */
int16_t get_angle(void) {
	return angle;
}

/*
 * return the state of the surface in another file
 *
 * \return	slope (high or low)
 */
bool get_slope(void) {
	return flat;
}

/*
 * thread dedicated to the timing of the slope angle computation
 */
static THD_WORKING_AREA(compute_angle_thd_wa, 1024);
static THD_FUNCTION(compute_angle_thd, arg){

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	systime_t time;

	while(1){
		time = chVTGetSystemTime();

    	angle = compute_angle(); // angle computation function

		chThdSleepUntilWindowed(time, time + MS2ST(COMPUTE_ANGLE_PERIOD));
	}
}

/*
 * inits and calibrates the IMU, starts the thread that computes the angle
 */
void compute_angle_thd_start(){

	// starts the IMU
	imu_start();
    // calibrates the IMU
    calibrate_acc();
	// starts the thread dedicated to the computation of the angle
	chThdCreateStatic(compute_angle_thd_wa, sizeof(compute_angle_thd_wa), NORMALPRIO, compute_angle_thd, NULL);
}
