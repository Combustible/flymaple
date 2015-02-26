/*
 * MotorControl.h
 *
 *  Created on: Feb 7, 2015
 *      Author: bmarohn
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <stdint.h>


#define MOTOR_CONTROL_LEVEL_MAX_DIFF 400
#define MOTOR_CONTROL_MIN_BASE_SPEED 2700
#define MOTOR_CONTROL_MAX_BASE_SPEED 4200

#define NUM_HIDDEN_LAYER_NODES 6

static const double alpha = 0.4;
static const double beta = 0.6;
static const double epsilon = 0.01;

static const double alpha_actor = 0.013;
static const double alpha_critic = 0.01;
static const double discount_factor = 0.98;
static const double eta_u = 0.025; // Center
static const double eta_sigma = 0.015; // Width



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
	void getparams(double k_out[3], double phi_out[NUM_HIDDEN_LAYER_NODES], double *perf_out);
};


#endif /* MOTORCONTROL_H_ */
