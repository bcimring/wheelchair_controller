/*
 * pid.h
 *
 *  Created on: Jul. 7, 2021
 *      Author: bcimring
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

#include "main.h"

struct PID{
	  double K_i;
	  double K_p;
	  double K_d;
	  double e;
	  double u;
	  double i;
	  double max;
	  double min;
};

void init_pid(struct PID *controller, double K_p, double K_i, double K_d, int max, int min);
double set_pulse_PID(struct PID *pid, double sp, double pv);

#endif /* SRC_PID_H_ */
