#ifndef MOTOR_H
#define MOTOR_H

#define MOTOR_COMPUTE_NEW_SPEED(current, delta) \
	((((int32_t)current + (int32_t)delta) > 10000) ? 10000 : \
	 ((int32_t)current + (int32_t)delta) < 0 ? 0 : \
	 ((uint16_t)((int32_t)current + (int32_t)delta)))

namespace Motor
{
	/**
	 * Initialize the motor timer subsystem and start sending out minimum speed
	 */
	void init();

	/**
	 * Update all motors at once, valid values 0 - 10000
	 */
	void update(uint16_t newspeed[4]);

	/**
	 * Update all motors at once to the same speed, valid values 0 - 10000. Atomic
	 */
	void update(uint16_t newspeed);

	/**
	 * Update just one motor, valid values 0 - 10000. Atomic
	 */
	void update(uint16_t newspeed, uint8_t channel);

	/**
	 * Read all motors at once, values range from 0 - 10000. Atomic
	 */
	void getspeed(uint16_t currentspeed[4]);

	/**
	 * Read just one motor, values range from 0 - 10000. Atomic
	 */
	uint16_t getspeed(uint8_t channel);

	/**
	 * Disable the motors (send out minimum) until re-enabled. Atomic
	 */
	void disable();

	/**
	 * Re-enable the motors. Automatically sets motor_speed values to 0. Atomic
	 */
	void enable();

	/**
	 * Set motor speed to 0 without disabling the subsystem. Atomic
	 */
	void stop();
}


#endif
