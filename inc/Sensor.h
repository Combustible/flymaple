/**
 * @file   Sensor.h
 * @author breadbread1984 <breadbread1984@163.com>
 * @date   Sat Jul 21 15:12:00 2012
 *
 * Last updated by: Byron Marohn <combustible@live.com>
 *                  August 3, 2014
 *
 * @brief       I2C sensor read/write utilities
 *
 * Before using read or write, caller should call Sensor::init() to ensure the
 * I2C bus is set up correctly.
 *
 * @copyright GPLv3
 */

#ifndef SENSOR_H
#define SENSOR_H


namespace Sensor
{

	/**
	 * Initialize I2C subsystem if not done already
	 */
	void init();

	/**
	 * Read data from a specific register on a designated device.
	 *
	 * @param dev_addr device address on the i2c bus.
	 * @param read_addr register identifier.
	 * @param read_length the length of data read from the device.
	 * @param buffer a preallocated place where the data will be stored.
	 */
	void read(unsigned char dev_addr, unsigned char read_addr,
	          unsigned char read_length, unsigned char *buffer);

	/**
	 * Write one byte to a specific register on a designated device.
	 *
	 * @param dev_addr device address on the i2c bus.
	 * @param write_addr register identifier.
	 * @param value one byte data that will be write to the device.
	 */
	void write(unsigned char dev_addr, unsigned char write_addr, unsigned char value);

};


#endif // SENSOR_H
