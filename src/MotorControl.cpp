#include "wirish.h"
#include "MotorControl.h"
#include "Motor.h"
#include "GlobalXYZ.h"
#include "MapleFreeRTOS.h"
#include "flymaple_utils.h"

/********************** Locally Accessible **********************/

#define MOTOR_CONTROL_LEVEL_MAX_DIFF 500

static int16_t base_speed = 0;
static double kp = 60, ki = 30, kd = 60;

void update_level_flight()
{
	double up_ref[3];

	int16_t local_base_speed;

	// Last time this function was called
	static portTickType last_time = xTaskGetTickCount();

	// Current time
	portTickType cur_time = xTaskGetTickCount();

	// Need to  get all the sensor data at once
	taskENTER_CRITICAL();

	up_ref[0] = GlobalXYZ::up[0];
	up_ref[1] = GlobalXYZ::up[1];
	up_ref[2] = GlobalXYZ::up[2];

	local_base_speed = base_speed;

	taskEXIT_CRITICAL();

	// X axis
	{
		static double i_term = 0;
		static double last_angle = 0;

		// Compute the angle, which is linear
		double angle = atan2(up_ref[0], up_ref[2]);

		// Compute the integral component, and keep it within the limits
		i_term += (ki * angle);
		if (i_term > MOTOR_CONTROL_LEVEL_MAX_DIFF) {
			i_term = MOTOR_CONTROL_LEVEL_MAX_DIFF;
		} else if (i_term < -MOTOR_CONTROL_LEVEL_MAX_DIFF) {
			i_term = -MOTOR_CONTROL_LEVEL_MAX_DIFF;
		}

		// Compute the output differential value to apply
		double out_diff = (kp * angle) + i_term + (kd * (angle - last_angle));
		if (out_diff > MOTOR_CONTROL_LEVEL_MAX_DIFF) {
			out_diff = MOTOR_CONTROL_LEVEL_MAX_DIFF;
		} else if (out_diff < -MOTOR_CONTROL_LEVEL_MAX_DIFF) {
			out_diff = -MOTOR_CONTROL_LEVEL_MAX_DIFF;
		}

		Motor::update(local_base_speed - out_diff, FLYMAPLE_MOTOR_0);
		Motor::update(local_base_speed + out_diff, FLYMAPLE_MOTOR_2);

		last_angle = angle;
	}

	// Y axis
	{
		static double i_term = 0;
		static double last_angle = 0;

		// Compute the angle, which is linear
		double angle = atan2(up_ref[1], up_ref[2]);

		// Compute the integral component, and keep it within the limits
		i_term += (ki * angle);
		if (i_term > MOTOR_CONTROL_LEVEL_MAX_DIFF) {
			i_term = MOTOR_CONTROL_LEVEL_MAX_DIFF;
		} else if (i_term < -MOTOR_CONTROL_LEVEL_MAX_DIFF) {
			i_term = -MOTOR_CONTROL_LEVEL_MAX_DIFF;
		}

		// Compute the output differential value to apply
		double out_diff = (kp * angle) + i_term + (kd * (angle - last_angle));
		if (out_diff > MOTOR_CONTROL_LEVEL_MAX_DIFF) {
			out_diff = MOTOR_CONTROL_LEVEL_MAX_DIFF;
		} else if (out_diff < -MOTOR_CONTROL_LEVEL_MAX_DIFF) {
			out_diff = -MOTOR_CONTROL_LEVEL_MAX_DIFF;
		}

		Motor::update(local_base_speed - out_diff, FLYMAPLE_MOTOR_1);
		Motor::update(local_base_speed + out_diff, FLYMAPLE_MOTOR_3);

		last_angle = angle;
	}


	last_time = cur_time;
}

/********************** Globally Accessible **********************/


void MotorControl::setspeed(int16_t speed)
{
	base_speed = speed;
}

int16_t MotorControl::getspeed()
{
	return base_speed;
}

void MotorControl::update()
{

	// Update base speed here, height PID
	update_level_flight();

}
