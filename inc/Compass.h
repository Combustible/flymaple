/**
 * @file   Compass.h
 * @author breadbread1984 <breadbread1984@163.com>
 * @date   Sat Jul 21 15:12:00 2012
 *
 * Last updated by: Byron Marohn <combustible@live.com>
 *                  August 3, 2014
 *
 * @brief  The namespace for manipulating the compass.
 *
 * Caller should call Compass::init() before otherwise accessing compass routines.
 *
 * @copyright GPLv3
 */

#include "Error.h"
#include "Sensor.h"

#ifndef COMPASS_H
#define COMPASS_H


#define COMPASS_I2C_ADDR                    (0x1E)
#define COMPASS_I2C_REG_CONFIG_A            (0x00)
#define COMPASS_I2C_REG_CONFIG_B            (0x01)
#define COMPASS_I2C_REG_MODE                (0x02)
#define COMPASS_I2C_REG_DATA_OUT            (0x03)
#define COMPASS_I2C_REG_STATUS              (0x09)

#define COMPASS_CONFIG_A_AVG_SAMPLES_1      (0x00)
#define COMPASS_CONFIG_A_AVG_SAMPLES_2      (0x20)
#define COMPASS_CONFIG_A_AVG_SAMPLES_4      (0x40)
#define COMPASS_CONFIG_A_AVG_SAMPLES_8      (0x60)

#define COMPASS_CONFIG_A_DATA_OUT_HZ_0PT75  (0x00)
#define COMPASS_CONFIG_A_DATA_OUT_HZ_1PT5   (0x04)
#define COMPASS_CONFIG_A_DATA_OUT_HZ_3      (0x08)
#define COMPASS_CONFIG_A_DATA_OUT_HZ_7PT5   (0x0C)
#define COMPASS_CONFIG_A_DATA_OUT_HZ_15     (0x10)
#define COMPASS_CONFIG_A_DATA_OUT_HZ_30     (0x14)
#define COMPASS_CONFIG_A_DATA_OUT_HZ_75     (0x18)

#define COMPASS_CONFIG_A_MEAS_MODE_NORMAL   (0x00)
#define COMPASS_CONFIG_A_MEAS_MODE_POS_BIAS (0x01)
#define COMPASS_CONFIG_A_MEAS_MODE_NEG_BIAS (0x02)

#define COMPASS_CONFIG_B_GAIN_VALUE(gain)   ((gain & 0x07) << 5)

#define COMPASS_MODE_I2C_REGULAR_SPEED      (0x00)
#define COMPASS_MODE_I2C_HIGH_SPEED         (0x80)

#define COMPASS_MODE_CONTINUOUS             (0x00)
#define COMPASS_MODE_SINGLE_MEAS            (0x01)
#define COMPASS_MODE_IDLE                   (0x03)

#define COMPASS_DATA_OUT_MIN_VAL            (-2048)
#define COMPASS_DATA_OUT_MAX_VAL            (2047)
#define COMPASS_DATA_OUT_ERR_VAL            (-4096)

namespace Compass
{

	/**
	 * The vector the north magnetic pole. Please notice that the direction
	 * points to a point below the northern horizon in the northern hemisphere.
	 * It is not a horizontal vector.
	 */
	extern float x;
	extern float y;
	extern float z;

	/**
	 * Initialize Compass subsystem if not done already
	 */
	void init();

	/**
	 * Get the direction of the north magnetic pole into x,y,z
	 */
	status getReading();

	/**
	 * Prints the calibration information for off-line calibration. The calibration
	 * is needed only once for a specific compass. The user need to place this function
	 * in a infinite loop. When the program runs, the scalex, scaley and scalez are
	 * printed to the serial. The user need rotate the flymaple till the reading on
	 * the serial stabilizes. Then the record the readings to Compass::scale[3] in
	 * Compass.cpp. Last, recompile the program. Calibration accomplished.
	 */
	void calibrate();

};


#endif
