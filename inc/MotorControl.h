/*
 * MotorControl.h
 *
 *  Created on: Feb 7, 2015
 *      Author: bmarohn
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <stdint.h>

namespace MotorControl
{
	void setheight(double target_height);
	void unset_height(void);
	void setbasespeed(int16_t speed);
	int16_t getbasespeed(void);
	bool is_height_set(void);

	void enablepid(void);
	void disablepid(void);

	void update(void);
	void getparams(double k_out[3], double *perf_out);
};


#endif /* MOTORCONTROL_H_ */
