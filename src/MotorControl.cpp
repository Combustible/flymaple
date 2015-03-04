
// This includes must preceed wirish.h
#include <stdlib.h>

#ifdef REINFORCEMENT_SIMULATION
#include "MotorControl.h"
#include "Motor.h"
#include <stdint.h>
#include <math.h>
#include <iostream>

using namespace std;
#else

#include "wirish.h"
#include "MotorControl.h"
#include "Motor.h"
#include "GlobalXYZ.h"
#include "MapleFreeRTOS.h"
#include "flymaple_utils.h"

#endif


/********************** Locally Accessible **********************/

bool learning_enabled = true;

static double target_height = 0;
static bool target_height_set = false;
static int16_t base_speed = 0;

static int32_t x_output = 0;
static int32_t y_output = 0;
static bool pid_enabled = false;

double phi_x[NUM_HIDDEN_LAYER_NODES];
double phi_y[NUM_HIDDEN_LAYER_NODES];

double sigma_x[NUM_HIDDEN_LAYER_NODES] = {0};  // Hidden layer weight
double w_x[NUM_HIDDEN_LAYER_NODES][3] = {{0}}; // Actor weights
double v_x[NUM_HIDDEN_LAYER_NODES] = {0};      // Critic weights

double perf_index_x = 0;
double perf_index_y = 0;

double k_x[3] = {0, 0, 0};


#ifdef REINFORCEMENT_SIMULATION
#define TWO_PI      6.283185307179586476925286766559
static int16_t motor[4];
#endif

#ifdef abs
#undef abs
#endif
#define abs(X) ((X) > 0 ? (X) : (X) * (-1))

// Equation 2
static inline double reward(double err, double prev_err)
{
	return ((alpha * (abs(err) <= epsilon ? 0.0 : -0.5)) +
	        (beta * (abs(err) <= (abs(prev_err)) ? 0.0 : -0.5)));
}

double generateGaussianNoise(const double mu, const double sigma)
{
	static bool haveSpare = false;
	static double rand1, rand2;

	if (haveSpare) {
		haveSpare = false;
		return (sigma * sqrt(rand1) * sin(rand2)) + mu;
	}

	haveSpare = true;

	rand1 = rand() / ((double) RAND_MAX);
	if (rand1 < 1e-100) {
		rand1 = 1e-100;
	}
	rand1 = -2 * log(rand1);
	rand2 = (rand() / ((double) RAND_MAX)) * TWO_PI;

	return (sigma * sqrt(rand1) * cos(rand2)) + mu;
}

/**
 * @param [inout] err
 *    err[0] = x(t)
 *    err[1] = delta x(t)
 *    err[2] = delta^2 x(t)
 * @param [inout] err_prev
 *    x (t - 1)
 * @param [inout] err_prev_prev
 *    x (t - 2)
 * @param [inout] u
 *    Hidden layer center vector
 * @param [inout] phi
 *    Weight of each hidden node
 * @param [inout] sigma
 *    Hidden layer weight
 * @param [inout] w
 *    Actor weights
 * @param [inout] v
 *    Critic weights
 * @param [inout] prev_critic
 *    Previous critic value
 * @param [out] perf_index
 *    Performance index out
 * @param [inout] k
 *    Control weights
 * @param [inout] needsinit
 *    true if parameters need to be initialized, then set to false
 * @param [in] angle
 *    Angle for the particular axis
 * @param [inout] output
 *    Previous output value in, next output value out
 */
static void reinforcement_learning(double err[3],
                                   double *err_prev,
                                   double *err_prev_prev,
                                   double u[NUM_HIDDEN_LAYER_NODES][3],
                                   double phi[NUM_HIDDEN_LAYER_NODES],
                                   double sigma[NUM_HIDDEN_LAYER_NODES],
                                   double w[NUM_HIDDEN_LAYER_NODES][3],
                                   double v[NUM_HIDDEN_LAYER_NODES],
                                   double *prev_critic,
								   double *perf_index,
                                   double k[3],
                                   bool *needsinit,
                                   double angle,
                                   int32_t *output
                                  )
{
	// Initialize everything one time to values of 1
	if ((*needsinit) == true) {
		for (int i = 0; i < NUM_HIDDEN_LAYER_NODES; i++) {
			for (int j = 0; j < 3; j++) {
				u[i][j] = generateGaussianNoise(1, 0.5);
				w[i][j] = generateGaussianNoise(1, 0.5);
			}
			sigma[i] = generateGaussianNoise(1, 0.5);
			v[i] = generateGaussianNoise(1, 0.5);
		}
		*needsinit = false;
	}

	*err_prev_prev = (*err_prev);
	*err_prev = err[0];
	err[0] = angle;
	err[1] = angle - (*err_prev);
	err[2] = angle - (2 * (*err_prev)) + (*err_prev_prev);

	// Compute the output differential value to apply
	*output = 10 * ((k[0] * err[0]) + (k[1] * err[1]) + (k[2] * err[2]));
	if ((*output) > MOTOR_CONTROL_LEVEL_MAX_DIFF) {
		*output = MOTOR_CONTROL_LEVEL_MAX_DIFF;
	} else if ((*output) < -MOTOR_CONTROL_LEVEL_MAX_DIFF) {
		*output = -MOTOR_CONTROL_LEVEL_MAX_DIFF;
	}

	// Equation 3
	for (int i = 0; i < NUM_HIDDEN_LAYER_NODES; i++) {
		phi[i] = exp((-1.0) * (pow(err[0] - u[i][0], 2) +
		                       pow(err[1] - u[i][1], 2) +
		                       pow(err[2] - u[i][2], 2)) /
		             (2.0 * pow(sigma[i], 2)));
	}

	// Equation 5
	double critic = 0;
	for (int i = 0; i < NUM_HIDDEN_LAYER_NODES; i++) {
		critic += v[i] * phi[i];
	}

	// Rather than one noise term per output parameter, one per network link
	double noise[NUM_HIDDEN_LAYER_NODES][3];
	{
		double sigma_v = (1.0 / (1.0 + exp(2 * critic))) / 3.0;
		for (int i = 0; i < NUM_HIDDEN_LAYER_NODES; i++) {
			for (int j = 0; j < 3; j++) {
				noise[i][j] = generateGaussianNoise(0, sigma_v);
			}
		}
	}

	// Equation 4
	for (int j = 0; j < 3; j++) {
		k[j] = 0;
		for (int i = 0; i < NUM_HIDDEN_LAYER_NODES; i++) {
			k[j] += (w[i][j] * phi[i]) + noise[i][j];
		}
	}

	// Equation 6 no longer exists because noise is pre-added

	// Equation 7
	// TODO: This might still be wrong
	double td_err = reward(err[0], (*err_prev)) + (discount_factor * critic) - (*prev_critic);

	// Equation 8
	*perf_index = 0.5 * pow(td_err, 2);

	if (learning_enabled == true) {

		{
			double sigma_next[NUM_HIDDEN_LAYER_NODES] = {0};
			for (int i = 0; i < NUM_HIDDEN_LAYER_NODES; i++) {
				// Equation 12
				sigma_next[i] = sigma[i] + eta_sigma * td_err * v[i] * phi[i] *
								(pow(err[0] - u[i][0], 2) +
								 pow(err[1] - u[i][1], 2) +
								 pow(err[2] - u[i][2], 2)) / pow(sigma[i], 3);

				// Equation 11
				for (int j = 0; j < 3; j++) {
					u[i][j] = u[i][j] + eta_u * td_err * v[i] * phi[i] * (err[j] - u[i][j]) / pow(sigma[i], 2);
				}

				sigma[i] = sigma_next[i];
			}
		}

		for (int i = 0; i < NUM_HIDDEN_LAYER_NODES; i++) {
			// Equation 9
			for (int j = 0; j < 3; j++) {
				w[i][j] = w[i][j] + alpha_actor * td_err * (noise[i][j]) * phi[i] / sigma[i];
			}

			// Equation 10
			v[i] = v[i] + alpha_critic * td_err * sigma[i];
		}
	}

	*prev_critic = critic;
}




void update_level_flight()
{
	double up_ref[3];

	int16_t local_base_speed;

	// If PID is not enabled, set all motors to the base speed
	if (!pid_enabled) {
#ifndef REINFORCEMENT_SIMULATION
		Motor::update(base_speed);
#endif
		return;
	}

#ifdef REINFORCEMENT_SIMULATION
	up_ref[0] = generateGaussianNoise(0, 0.5);
	up_ref[1] = 0.00;
	up_ref[2] = sqrt(1 - pow(up_ref[1], 2));

	local_base_speed = base_speed;
#else
	// Need to  get all the sensor data at once
	taskENTER_CRITICAL();

	up_ref[0] = GlobalXYZ::up[0];
	up_ref[1] = GlobalXYZ::up[1];
	up_ref[2] = GlobalXYZ::up[2];

	local_base_speed = base_speed;

	taskEXIT_CRITICAL();
#endif

	// X axis
	{
		/**
		 * err[0] = x(t)
		 * err[1] = delta x(t)
		 * err[2] = delta^2 x(t)
		 */
		static double err[3] = {0, 0, 0};

		static double err_prev = 0; // x (t - 1)
		static double err_prev_prev = 0; // x (t - 2)

		static double u[NUM_HIDDEN_LAYER_NODES][3] = {{0}}; // Hidden layer center vector
		static double prev_critic = 0;

		static bool needsinit = true;

		reinforcement_learning(err,
		                       &err_prev,
		                       &err_prev_prev,
		                       u,
		                       phi_x,
		                       sigma_x,
		                       w_x,
		                       v_x,
		                       &prev_critic,
							   &perf_index_x,
		                       k_x,
		                       &needsinit,
		                       atan2(up_ref[0], up_ref[2]),
		                       &x_output);

#ifdef REINFORCEMENT_SIMULATION
		motor[0] = MOTOR_COMPUTE_NEW_SPEED(local_base_speed, (-1) * x_output);
		motor[2] = MOTOR_COMPUTE_NEW_SPEED(local_base_speed,  x_output);
#else
		Motor::update(MOTOR_COMPUTE_NEW_SPEED(local_base_speed, (-1) * x_output),
		              FLYMAPLE_MOTOR_0);
		Motor::update(MOTOR_COMPUTE_NEW_SPEED(local_base_speed,  x_output),
		              FLYMAPLE_MOTOR_2);
#endif
	}

	// Y axis
	{
		static double err[3] = {0, 0, 0};

		static double err_prev = 0;
		static double err_prev_prev = 0;

		static double u[NUM_HIDDEN_LAYER_NODES][3] = {{0}};
		static double sigma[NUM_HIDDEN_LAYER_NODES] = {0};
		static double w[NUM_HIDDEN_LAYER_NODES][3] = {{0}};
		static double v[NUM_HIDDEN_LAYER_NODES] = {0};
		static double prev_critic = 0;

		static double k[3] = {0, 0, 0};

		static bool needsinit = true;

		reinforcement_learning(err,
		                       &err_prev,
		                       &err_prev_prev,
		                       u,
		                       phi_y,
		                       sigma,
		                       w,
		                       v,
		                       &prev_critic,
							   &perf_index_y,
		                       k,
		                       &needsinit,
		                       atan2(up_ref[1], up_ref[2]),
		                       &y_output);

#ifdef REINFORCEMENT_SIMULATION
		motor[1] = MOTOR_COMPUTE_NEW_SPEED(local_base_speed, (-1) * y_output);
		motor[3] = MOTOR_COMPUTE_NEW_SPEED(local_base_speed,  y_output);
#else
		Motor::update(MOTOR_COMPUTE_NEW_SPEED(local_base_speed, (-1) * y_output),
		              FLYMAPLE_MOTOR_1);
		Motor::update(MOTOR_COMPUTE_NEW_SPEED(local_base_speed,  y_output),
		              FLYMAPLE_MOTOR_3);
#endif
	}

#ifdef REINFORCEMENT_SIMULATION
		cout  << "phi_x" << endl;
		for (int i = 0; i < 6; i ++) {
			cout << phi_x[i] << endl;
		}
		cout  << "phi_y" << endl;
		for (int i = 0; i < 6; i ++) {
			cout << phi_y[i] << endl;
		}
		cout << "perf_index_x: " << perf_index_x << endl;
		cout << "perf_index_y: " << perf_index_y << endl;
#endif
}

void update_height_base_speed(void)
{
#ifndef REINFORCEMENT_SIMULATION
	if (target_height_set == true) {
		taskENTER_CRITICAL();

		if (GlobalXYZ::rel_height > target_height) {
			base_speed --;
		} else {
			base_speed ++;
		}
		if (base_speed < MOTOR_CONTROL_MIN_BASE_SPEED) {
			base_speed = MOTOR_CONTROL_MIN_BASE_SPEED;
		}
		if (base_speed > MOTOR_CONTROL_MAX_BASE_SPEED) {
			base_speed = MOTOR_CONTROL_MAX_BASE_SPEED;
		}

		taskEXIT_CRITICAL();
	}
#endif
}

/********************** Globally Accessible **********************/


void MotorControl::enablepid(void)
{
	pid_enabled = true;
	x_output = 0;
	y_output = 0;
}

void MotorControl::disablepid(void)
{
	pid_enabled = false;
	x_output = 0;
	y_output = 0;
}

void MotorControl::setheight(double target_height_in)
{
	target_height = target_height_in;
	target_height_set = true;
}

bool MotorControl::is_height_set(void)
{
	return target_height_set;
}

void MotorControl::unset_height(void)
{
	target_height_set = false;
}

void MotorControl::setbasespeed(int16_t speed)
{
	base_speed = speed;
}

int16_t MotorControl::getbasespeed()
{
	return base_speed;
}

void MotorControl::update()
{
	// update_height_base_speed();
	update_level_flight();
}




#ifdef REINFORCEMENT_SIMULATION
main(int arc, char **argv)
{
	srand(23489872);
	MotorControl::enablepid();
	MotorControl::setbasespeed(2000);

	while (1) {
		MotorControl::update();
	}

	return 0;
}
#endif
