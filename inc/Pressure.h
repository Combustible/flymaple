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


#define BMP085_I2C_ADDR                 (0x77)
#define BMP085_I2C_REG_EEPROM           (0xAA)
#define BMP085_I2C_REG_EEPROM_LEN       (22)
#define BMP085_I2C_REG_CONTROL          (0xf4)
#define BMP085_I2C_REG_CONTROL_OUTPUT   (0xf6)
#define BMP085_CMD_READ_TEMPERATURE     (0x2e)
#define BMP085_CMD_READ_PRESSURE        (0x34)
#define PRESSURE_AT_SEALEVEL_IN_PA      (101325)

#define BIG_ENDIAN_INT16_FROM_PTR(ptr) \
	(((int16_t)(*((int8_t *)ptr)) << 8) | ((int16_t)(*(((int8_t *)ptr) + 1))))
#define BIG_ENDIAN_UINT16_FROM_PTR(ptr) \
	(((uint16_t)(*((uint8_t *)ptr)) << 8) | ((uint16_t)(*(((uint8_t *)ptr) + 1))))

namespace Pressure
{

	/** Temperature in .1 degree C */
	extern uint32_t temperature;
	/** Pressure in Pa */
	extern uint32_t pressure;

	/**
	 * Initialize Pressure subsystem if not done already
	 */
	void init();

	/**
	 * Read the temperature and pressure from the pressure sensor and update the global
	 * values 'temperature' and 'pressure', respectively.
	 */
	status getReading();

	/**
	 * Floating point operating to compute the absolute altitude from the previously obtained
	 * pressure measurement.
	 */
	float computeAltitude();

	int32_t debug_get_ut();
	int32_t debug_get_up();
};


#endif
