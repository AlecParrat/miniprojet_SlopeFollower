/*
 * average.c
 *
 *  Created on: 15 mai 2022
 *      Author: alecp
 */

#include <main.h>

/*
 * Simple moving average
 * sum is on 32 bits to be protected against too high values
 *
 * \param new_value		Last value to add to the averaging
 *
 * \param sum			Moving sum to be modified
 *
 * \param values		Table of last values to be modified
 *
 * \param counter		Knows the position of the last value, to be modified
 *
 * \param size			Number of values to average
 *
 * \return				Last average value
 */
int16_t average(int16_t new_value, int32_t* sum, int16_t* values, int16_t* counter, int16_t size) {

	*sum -= values[*counter]; //remove the oldest value form the sum
	*sum += new_value; //add the new value to the sum
	values[*counter] = new_value; //stock the new value

	*counter = (*counter + 1) % size; //cycle the counter

	return *sum / size;
}
