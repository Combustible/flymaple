/**
 * @file        Pressure.h
 * @author      breadbread1984 <breadbread1984@163.com>
 * @date        Fri Sep 7 10:01:00 2012
 *
 * Last updated by: Byron Marohn <combustible@live.com>
 *                  August 3, 2014
 *
 * @brief  The namespace for manipulating the pressure sensor.
 *
 * Caller should call Pressure::init() before otherwise accessing pressure routines.
 *
 * @copyright GPLv3
 */

#include "Error.h"
#include "Sensor.h"
#include "Vector.h"

#ifndef PRESSURE_H
#define PRESSURE_H


#define BMP085_I2C_ADDR (0x77)
#define BMP085_I2C_REG_EEPROM (0xAA)
#define BMP085_I2C_REG_EEPROM_LEN (22)

#define BIG_ENDIAN_INT16_FROM_PTR(ptr) \
	(((int16_t)(*((uint8_t *)ptr)) << 8) | ((int16_t)(*(((uint8_t *)ptr) + 1))))
#define BIG_ENDIAN_UINT16_FROM_PTR(ptr) \
	(((uint16_t)(*((uint8_t *)ptr)) << 8) | ((uint16_t)(*(((uint8_t *)ptr) + 1))))

namespace Pressure
{

	/**
	 * Initialize Pressure subsystem if not done already
	 */
	void init();

	/**
	 * Get the pressure, temperature, altitude (in centimeter) of the quadcopter.
	 *
	 * @return the elements in the returned vector are pressure (of int type in Pa), temperature (of short type in 0.1 degree C) and altitude (of double type in meters) in sequence.
	 */
	Vector<double> getReading();

};


#endif
