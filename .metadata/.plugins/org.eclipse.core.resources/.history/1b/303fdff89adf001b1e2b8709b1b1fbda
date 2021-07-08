/*
 * pid.c
 *
 *  Created on: Jul. 7, 2021
 *      Author: bcimring
 */

#include "pid.h"

void init_pid(struct PID *controller, double K_p, double K_i, double K_d, int max, int min) {
	controller->K_p = K_p;
	controller->K_i = K_i;
	controller->K_d = K_d;
	controller->e = 0.0;
	controller->u = 0.0;
	controller->i = 0.0;
	controller->max = max;
	controller->min = min;
}

double set_pulse_PID(struct PID *pid, double sp, double pv) {
	double e_prev = pid->e;
	pid->e = sp - pv;
	double tmp_i = pid->i + pid->e;

	double out = pid->K_p *(pid->e)  +  pid->K_i * (tmp_i)  +  pid->K_d *(e_prev - pid->e);

	if (( out < pid->max ) && ( out > pid->min )) {
		pid->i = tmp_i;
	} else if ( out >= pid->max ) {
		out = pid->max;
	} else if ( out <= pid->min ) {
		out = pid->min;
	}

	return out;
}
