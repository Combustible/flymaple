
// This includes must preceed wirish.h
#include <stdlib.h>

#include "wirish.h"
#include "MotorControl.h"
#include "Motor.h"
#include "GlobalXYZ.h"
#include "MapleFreeRTOS.h"
#include "flymaple_utils.h"

/********************** Locally Accessible **********************/

#define MOTOR_CONTROL_LEVEL_MAX_DIFF 1200
#define MOTOR_CONTROL_MIN_BASE_SPEED 2700
#define MOTOR_CONTROL_MAX_BASE_SPEED 4200

static double target_height = 0;
static bool target_height_set = false;
static int16_t base_speed = 0;

static int32_t x_output = 0;
static int32_t y_output = 0;
static bool pid_enabled = false;

static double k[3] = {0, 0, 0}; // Weights. Effectively ki, kp, kd respectively.
static double perf_index = 0;

static const double alpha = 0.4;
static const double beta = 0.6;
static const double epsilon = 0.01;

static const double alpha_actor = 0.013;
static const double alpha_critic = 0.01;
static const double discount_factor = 0.98;
static const double eta_u = 0.025; // Center
static const double eta_sigma = 0.015; // Width

#define NUM_HIDDEN_LAYER_NODES 6

// Equation 2
static inline double reward(double err, double prev_err)
{
	return ((alpha * ((abs(err) <= epsilon) ? 0.0 : -0.5)) +
	        (beta * ((abs(err) <= (abs(prev_err))) ? 0.0 : -0.5)));
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

void update_level_flight()
{
	double up_ref[3];

	int16_t local_base_speed;

	// If PID is not enabled, set all motors to the base speed
	if (!pid_enabled) {
		Motor::update(base_speed);
		return;
	}

	// Need to  get all the sensor data at once
	taskENTER_CRITICAL();

	up_ref[0] = GlobalXYZ::up[0];
	up_ref[1] = GlobalXYZ::up[1];
	up_ref[2] = GlobalXYZ::up[2];

	local_base_speed = base_speed;

	taskEXIT_CRITICAL();

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


		static double u[NUM_HIDDEN_LAYER_NODES][3] = {{0}};
		static double sigma[NUM_HIDDEN_LAYER_NODES] = {0};
		static double w[NUM_HIDDEN_LAYER_NODES][3] = {{0}};
		static double v[NUM_HIDDEN_LAYER_NODES] = {0};
		static double prev_critic = 0;

		{
			// Compute the angle, which is linear
			double angle = atan2(up_ref[0], up_ref[2]);

			err_prev_prev = err_prev;
			err_prev = err[0];
			err[0] = angle;
			err[1] = angle - err[1];
			err[2] = angle - (2 * err[1]) + err[2];
		}

		{
			// Compute the output differential value to apply
			int32_t new_output = x_output + (k[0] * err[0]) +
			                     (k[1] * err[1]) + (k[2] * err[2]);

			if (new_output > MOTOR_CONTROL_LEVEL_MAX_DIFF) {
				new_output = MOTOR_CONTROL_LEVEL_MAX_DIFF;
			} else if (new_output < -MOTOR_CONTROL_LEVEL_MAX_DIFF) {
				new_output = -MOTOR_CONTROL_LEVEL_MAX_DIFF;
			}

			Motor::update(local_base_speed - new_output, FLYMAPLE_MOTOR_0);
			Motor::update(local_base_speed + new_output, FLYMAPLE_MOTOR_2);

			x_output = new_output;
		}

		// Equation 3
		double phi[NUM_HIDDEN_LAYER_NODES];
		for (int i = 0; i < NUM_HIDDEN_LAYER_NODES; i++) {
			phi[i] = exp((-1.0) * (pow(err[0] - u[i][0], 2) +
			                       pow(err[1] - u[i][1], 2) +
			                       pow(err[2] - u[i][2], 2)) /
			             (2.0 * pow(sigma[i], 2)));
		}

		// Equation 4
		double k_actor[3] = {0};
		for (int i = 0; i < NUM_HIDDEN_LAYER_NODES; i++) {
			for (int j = 0; j < 3; j++) {
				k_actor[j] += w[i][j] * phi[i];
			}
		}

		// Equation 5
		double critic = 0;
		for (int i = 0; i < NUM_HIDDEN_LAYER_NODES; i++) {
			critic += v[i] * phi[i];
		}

		// Equation 6
		{
			double sigma_v = 1.0 / (1.0 + exp(2 * critic));
			for (int j = 0; j < 3; j++) {
				k[j] = k_actor[j] + generateGaussianNoise(0, sigma_v);
			}
		}

		// Equation 7
		// TODO: This might still be wrong
		double td_err = reward(err[0], err_prev) + (discount_factor * critic) - prev_critic;

		// Equation 8
		perf_index = 0.5 * pow(td_err, 2);

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

				w[i][j] = w[i][j] + alpha_actor * td_err * (k[j] - k_actor[j]) * phi[i] / sigma[i];
			}

			// Equation 10
			v[i] = v[i] + alpha_critic * td_err * sigma[i];
		}

		prev_critic = critic;
	}

	// Y axis
	{
		/**
		 * err[0] = x(t)
		 * err[1] = delta x(t)
		 * err[2] = delta^2 x(t)
		 */
		static double err[3] = {0, 0, 0};

		static double err_prev = 0; // x (t - 1)
		static double err_prev_prev = 0; // x (t - 2)


		{
			// Compute the angle, which is linear
			double angle = atan2(up_ref[1], up_ref[2]);

			err_prev_prev = err_prev;
			err_prev = err[0];
			err[0] = angle;
			err[1] = angle - err[1];
			err[2] = angle - (2 * err[1]) + err[2];
		}

		{
			// Compute the output differential value to apply
			int32_t new_output = y_output + (k[0] * err[0]) +
			                     (k[1] * err[1]) + (k[2] * err[2]);

			if (new_output > MOTOR_CONTROL_LEVEL_MAX_DIFF) {
				new_output = MOTOR_CONTROL_LEVEL_MAX_DIFF;
			} else if (new_output < -MOTOR_CONTROL_LEVEL_MAX_DIFF) {
				new_output = -MOTOR_CONTROL_LEVEL_MAX_DIFF;
			}

			Motor::update(local_base_speed - new_output, FLYMAPLE_MOTOR_1);
			Motor::update(local_base_speed + new_output, FLYMAPLE_MOTOR_3);

			y_output = new_output;
		}


	}
}

void update_height_base_speed(void)
{
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

void MotorControl::getparams(double k_out[3], double *perf_out)
{
	k_out[0] = k_out[0];
	k_out[1] = k_out[1];
	k_out[2] = k_out[2];
	*perf_out = perf_index;
}
