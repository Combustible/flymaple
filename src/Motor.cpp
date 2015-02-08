#include "wirish.h"
#include "Motor.h"
#include "MapleFreeRTOS.h"
#include "flymaple_utils.h"

/********************** Locally Accessible **********************/

static bool gIsInit = false;
static bool gIsEnabled = false;

static HardwareTimer timer3(3);

static uint16_t speed[4] = {0, 0, 0, 0};

static uint16_t compare_min = 0;
static uint16_t compare_max = 0;
static uint16_t compare_divider = 0;

/********************** Globally Accessible **********************/

void Motor::init()
{
	if (gIsInit != true) {
		// Just in case, re-set speeds to 0
		speed[0] = 0;
		speed[1] = 0;
		speed[2] = 0;
		speed[3] = 0;

		// Configure pins as PWM
		pinMode(D12, PWM);
		pinMode(D11, PWM);
		pinMode(D27, PWM);
		pinMode(D28, PWM);

		// Disable counting + output for now
		timer3.pause();
		timer3.setCount(0);

		// Set the device to overflow after 20ms
		timer3.setPeriod(20000);

		// Set the pin mode of each channel to PWM
		timer3.setMode(TIMER_CH1, TIMER_PWM);
		timer3.setMode(TIMER_CH2, TIMER_PWM);
		timer3.setMode(TIMER_CH3, TIMER_PWM);
		timer3.setMode(TIMER_CH4, TIMER_PWM);

		// Compute how many counts it will take to overflow (at 20ms)
		uint16_t overflow = timer3.getOverflow();

		// Minimum for PPM control is 1ms, max is 2ms
		compare_min = overflow / 20;
		compare_max = overflow / 10;

		// Compute a rough integer divider to convert input values (0 - 10000) to
		// valid compare values
		// @TODO - Revisit how much it would hurt performance to make this a float
		compare_divider = 10000 / (compare_max - compare_min);

		FLY_PRINT("overflow: ");
		FLY_PRINTLN(overflow);

		FLY_PRINT("compare_min,compare_max: ");
		FLY_PRINT(compare_min);
		FLY_PRINT(",");
		FLY_PRINTLN(compare_max);

		// Motors start at minimum speed (off, hopefully)
		timer3.setCompare(TIMER_CH1, compare_min);
		timer3.setCompare(TIMER_CH2, compare_min);
		timer3.setCompare(TIMER_CH3, compare_min);
		timer3.setCompare(TIMER_CH4, compare_min);

		// Re-enable the timer
		timer3.refresh();
		timer3.resume();

		gIsInit = true;
	}
}

void Motor::update(uint16_t newspeed[4])
{
	if (gIsInit && gIsEnabled) {
		// If any channels out of range, do nothing
		for (int i = 0; i < 4; i++) {
			if (newspeed[i] > 10000) {
				FLY_PRINT("WARNING! speed value out of range:");
				FLY_PRINTLN(newspeed[i]);
				return;
			}
		}

		taskENTER_CRITICAL();

		speed[0] = newspeed[0];
		speed[1] = newspeed[1];
		speed[2] = newspeed[2];
		speed[3] = newspeed[3];

		timer3.setCompare(TIMER_CH1, compare_min + (newspeed[0] / compare_divider));
		timer3.setCompare(TIMER_CH2, compare_min + (newspeed[1] / compare_divider));
		timer3.setCompare(TIMER_CH3, compare_min + (newspeed[2] / compare_divider));
		timer3.setCompare(TIMER_CH4, compare_min + (newspeed[3] / compare_divider));

		taskEXIT_CRITICAL();
	}
}

void Motor::update(uint16_t newspeed)
{
	if (gIsInit && gIsEnabled) {
		if (newspeed > 10000) {
			FLY_PRINT("WARNING! speed value out of range:");
			FLY_PRINTLN(newspeed);
			return;
		}

		taskENTER_CRITICAL();

		speed[0] = newspeed;
		speed[1] = newspeed;
		speed[2] = newspeed;
		speed[3] = newspeed;

		uint16_t compare = compare_min + (newspeed / compare_divider);

		timer3.setCompare(TIMER_CH1, compare);
		timer3.setCompare(TIMER_CH2, compare);
		timer3.setCompare(TIMER_CH3, compare);
		timer3.setCompare(TIMER_CH4, compare);

		taskEXIT_CRITICAL();
	}
}

void Motor::update(uint16_t newspeed, enum flymaple_motor_channel channel)
{
	if (channel < 4 && gIsInit && gIsEnabled) {
		if (newspeed > 10000) {
			FLY_PRINT("WARNING! speed value out of range:");
			FLY_PRINTLN(newspeed);
			return;
		}

		taskENTER_CRITICAL();

		speed[channel] = newspeed;

		uint16_t compare = compare_min + (newspeed / compare_divider);

		switch (channel) {
		case 0:
			timer3.setCompare(TIMER_CH1, compare);
			break;
		case 1:
			timer3.setCompare(TIMER_CH2, compare);
			break;
		case 2:
			timer3.setCompare(TIMER_CH3, compare);
			break;
		case 3:
			timer3.setCompare(TIMER_CH4, compare);
			break;
		}

		taskEXIT_CRITICAL();
	}
}

void Motor::getspeed(uint16_t currentspeed[4])
{
	taskENTER_CRITICAL();

	currentspeed[0] = speed[0];
	currentspeed[1] = speed[1];
	currentspeed[2] = speed[2];
	currentspeed[3] = speed[3];

	taskEXIT_CRITICAL();
}

uint16_t Motor::getspeed(enum flymaple_motor_channel channel)
{
	if (channel < 4) {
		return speed[channel];
	}

	// Not sure what else to do here
	return 0;
}


void Motor::disable()
{
	taskENTER_CRITICAL();

	gIsEnabled = false;

	stop();

	taskEXIT_CRITICAL();
}

void Motor::enable()
{
	// Don't do anything if already enabled or not initialized
	if (gIsInit && !gIsEnabled) {
		taskENTER_CRITICAL();

		gIsEnabled = true;

		stop();

		taskEXIT_CRITICAL();
	}
}

void Motor::stop()
{
	taskENTER_CRITICAL();

	speed[0] = 0;
	speed[1] = 0;
	speed[2] = 0;
	speed[3] = 0;

	// Only set the compare values if initialized
	if (gIsInit) {
		timer3.setCompare(TIMER_CH1, compare_min);
		timer3.setCompare(TIMER_CH2, compare_min);
		timer3.setCompare(TIMER_CH3, compare_min);
		timer3.setCompare(TIMER_CH4, compare_min);
	}

	taskEXIT_CRITICAL();
}


